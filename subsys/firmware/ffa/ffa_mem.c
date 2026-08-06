/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * FF-A memory sharing: FFA_MEM_SHARE (FN64) + FFA_MEM_FRAG_TX for large
 * descriptors + FFA_MEM_RECLAIM.
 *
 * The descriptor is built directly in the shared TX buffer whenever it fits
 * (common case: 1 receiver, 1..N constituents that fill <= one TX page).  For
 * descriptors that exceed the TX buffer the initial fragment is sent via
 * FFA_FN64_MEM_SHARE (a3/a4=0 → shared RXTX) and the remaining constituents
 * are piped through FFA_MEM_FRAG_TX until the SPMC acknowledges receipt.
 *
 * Composite memory-region descriptor layout (FF-A §10.9, packed):
 *
 *   [0..23]  struct ffa_mem_region  (sender, handle, tag, attr_cnt=1)
 *   [24..39] struct ffa_mem_region_attributes[0]  (receiver, composite_off=40)
 *   [40..55] struct ffa_composite_mem_region  (total_pg, range_cnt, rsv)
 *   [56...]  struct ffa_mem_region_addr_range[0..N-1]  (address, pg_cnt, rsv)
 *
 * Offsets are fixed for the one-receiver case used here.  Fragmentation splits
 * only at constituent boundaries.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/firmware/ffa.h>
#include "ffa_internal.h"

LOG_MODULE_DECLARE(arm_ffa, CONFIG_ARM_FFA_LOG_LEVEL);

/* Fixed byte offsets within the descriptor for the one-receiver case. */
#define FFA_MEM_REGION_HDR_SZ   \
	((uint32_t)sizeof(struct ffa_mem_region))
#define FFA_MEM_ATTR_SZ         \
	((uint32_t)sizeof(struct ffa_mem_region_attributes))
#define FFA_MEM_COMPOSITE_HDR_SZ \
	((uint32_t)sizeof(struct ffa_composite_mem_region))
#define FFA_MEM_CONSTITUENT_SZ  \
	((uint32_t)sizeof(struct ffa_mem_region_addr_range))

/* Offset of the ffa_mem_region_attributes array from start of descriptor. */
#define ATTR_OFF   FFA_MEM_REGION_HDR_SZ
/* Offset of the ffa_composite_mem_region from start of descriptor.
 * composite_off is relative to the start of the ffa_mem_region header. */
#define COMPOSITE_OFF  (ATTR_OFF + FFA_MEM_ATTR_SZ)
/* Offset of the first constituent inside the composite region. */
#define CONST_OFF  (COMPOSITE_OFF + FFA_MEM_COMPOSITE_HDR_SZ)

/* How many constituents fit in the first TX fragment. */
static uint32_t first_frag_max(uint32_t tx_bytes)
{
	if (tx_bytes <= CONST_OFF) {
		return 0U;
	}
	return (tx_bytes - CONST_OFF) / FFA_MEM_CONSTITUENT_SZ;
}

/* How many constituents fit in a subsequent FFA_MEM_FRAG_TX fragment. */
static uint32_t frag_max(uint32_t tx_bytes)
{
	return tx_bytes / FFA_MEM_CONSTITUENT_SZ;
}

/*
 * Build the memory-region descriptor header + attributes into @buf, writing
 * at most @buf_sz bytes.  The constituent array is NOT written here; callers
 * copy constituents separately.
 *
 * Returns the number of bytes written (CONST_OFF = 56), or 0 if buf is too
 * small.
 */
static uint32_t ffa_mem_build_hdr(uint8_t *buf, uint32_t buf_sz,
				  uint16_t sender, uint16_t receiver,
				  uint32_t total_pg_cnt, uint32_t range_cnt)
{
	struct ffa_mem_region *hdr;
	struct ffa_mem_region_attributes *attr;
	struct ffa_composite_mem_region *comp;

	if (buf_sz < CONST_OFF) {
		return 0U;
	}

	/* Zero the header area so reserved/pad fields are clean. */
	memset(buf, 0, CONST_OFF);

	hdr = (struct ffa_mem_region *)(void *)buf;
	hdr->sender             = sender;
	/* Normal memory, Write-Back cacheable, Inner-Shareable */
	hdr->flags              = (uint8_t)(
		(FFA_MEM_TYPE_NORMAL      << FFA_MEM_TYPE_SHIFT) |
		(FFA_MEM_CACHE_WRITE_BACK << FFA_MEM_CACHE_SHIFT) |
		(FFA_MEM_SHARE_INNER      << FFA_MEM_SHARE_SHIFT));
	/* Sender view: read/write data, no execute */
	hdr->mem_access_perm    = FFA_MEM_DATA_PERM_RW;
	hdr->mem_access_attr_cnt = 1U;

	attr = (struct ffa_mem_region_attributes *)(void *)(buf + ATTR_OFF);
	attr->receiver     = receiver;
	attr->perms        = FFA_MEM_DATA_PERM_RW;
	attr->flags        = hdr->flags;
	attr->composite_off = COMPOSITE_OFF;

	comp = (struct ffa_composite_mem_region *)(void *)(buf + COMPOSITE_OFF);
	comp->total_pg_cnt  = total_pg_cnt;
	comp->addr_range_cnt = range_cnt;

	return CONST_OFF;
}

/*
 * Write @n consecutive constituents from @ranges[@start..@start+n-1] into
 * @buf.  Returns bytes written.
 */
static uint32_t ffa_mem_write_constituents(
	uint8_t *buf, uint32_t buf_sz,
	const struct ffa_mem_region_addr_range *ranges,
	uint32_t start, uint32_t n)
{
	uint32_t bytes = n * FFA_MEM_CONSTITUENT_SZ;

	if (bytes > buf_sz) {
		return 0U;
	}
	memcpy(buf, &ranges[start], bytes);
	return bytes;
}

int ffa_mem_share_impl(struct ffa_drv_state *st,
		       struct ffa_mem_ops_args *args)
{
	struct arm_smccc_1_2_regs smc_args = {0};
	struct arm_smccc_1_2_regs smc_ret  = {0};
	uint8_t *tx = (uint8_t *)st->tx_buf;
	uint32_t tx_sz = st->rxtx_pages * FFA_PAGE_SIZE;
	uint32_t total_pg = 0U;
	uint32_t written;
	uint32_t first_max;
	uint32_t frag_sent;
	uint32_t total_desc_sz;
	uint32_t first_frag_sz;

	if (tx == NULL || tx_sz == 0U) {
		return -ENOMEM;
	}
	if (args->range_cnt == 0U || args->ranges == NULL) {
		return -EINVAL;
	}

	/* Compute total page count from constituents. */
	for (uint32_t i = 0U; i < args->range_cnt; i++) {
		total_pg += args->ranges[i].pg_cnt;
	}

	first_max = first_frag_max(tx_sz);
	if (first_max == 0U) {
		/* TX buffer is impossibly small (< 56 bytes). */
		return -ENOMEM;
	}

	k_mutex_lock(&st->lock, K_FOREVER);

	/* Build fixed header (56 bytes). */
	written = ffa_mem_build_hdr(tx, tx_sz, st->vm_id, args->dst_id,
				    total_pg, args->range_cnt);
	if (written == 0U) {
		k_mutex_unlock(&st->lock);
		return -ENOMEM;
	}

	/* Append as many constituents as fit in the first fragment. */
	frag_sent = (args->range_cnt < first_max) ? args->range_cnt : first_max;
	written += ffa_mem_write_constituents(tx + written, tx_sz - written,
					      args->ranges, 0U, frag_sent);

	total_desc_sz = CONST_OFF +
			args->range_cnt * FFA_MEM_CONSTITUENT_SZ;
	first_frag_sz = CONST_OFF +
			frag_sent * FFA_MEM_CONSTITUENT_SZ;

	/* FFA_FN64_MEM_SHARE: a1=total_len, a2=frag_len, a3/a4=0 (shared buf) */
	smc_args.a0 = FFA_FN64_MEM_SHARE;
	smc_args.a1 = total_desc_sz;
	smc_args.a2 = first_frag_sz;
	smc_args.a3 = 0U;
	smc_args.a4 = 0U;
	ffa_invoke(st, &smc_args, &smc_ret);

	if ((uint32_t)smc_ret.a0 == FFA_ERROR) {
		k_mutex_unlock(&st->lock);
		return ffa_to_errno((int)smc_ret.a2);
	}

	/*
	 * SPMC accepted less than the full descriptor: drive the
	 * FFA_MEM_FRAG_TX loop until all constituents are sent.
	 */
	while ((uint32_t)smc_ret.a0 == FFA_MEM_FRAG_RX) {
		uint32_t frag_handle_lo = (uint32_t)smc_ret.a1;
		uint32_t frag_handle_hi = (uint32_t)smc_ret.a2;
		uint32_t sender_id      = (uint32_t)smc_ret.a3;
		uint32_t this_max;
		uint32_t this_n;
		uint32_t this_sz;

		(void)sender_id; /* informational only */

		if (frag_sent >= args->range_cnt) {
			/* All constituents already sent — SPMC bookkeeping
			 * error; bail to avoid an infinite loop. */
			k_mutex_unlock(&st->lock);
			return -EIO;
		}

		this_max = frag_max(tx_sz);
		this_n = args->range_cnt - frag_sent;
		if (this_n > this_max) {
			this_n = this_max;
		}
		this_sz = ffa_mem_write_constituents(tx, tx_sz, args->ranges,
						     frag_sent, this_n);

		memset(&smc_args, 0, sizeof(smc_args));
		smc_args.a0 = FFA_MEM_FRAG_TX;
		smc_args.a1 = frag_handle_lo;
		smc_args.a2 = frag_handle_hi;
		smc_args.a3 = this_sz;
		ffa_invoke(st, &smc_args, &smc_ret);

		if ((uint32_t)smc_ret.a0 == FFA_ERROR) {
			k_mutex_unlock(&st->lock);
			return ffa_to_errno((int)smc_ret.a2);
		}

		frag_sent += this_n;
	}

	k_mutex_unlock(&st->lock);

	if ((uint32_t)smc_ret.a0 != FFA_SUCCESS_64 &&
	    (uint32_t)smc_ret.a0 != FFA_SUCCESS_32) {
		return -EINVAL;
	}

	/* Global handle is packed as {a3[31:0], a2[31:0]} (64-bit). */
	args->g_handle = ((uint64_t)(uint32_t)smc_ret.a3 << 32) |
			 (uint32_t)smc_ret.a2;
	return 0;
}

int ffa_mem_reclaim_impl(struct ffa_drv_state *st,
			 uint64_t g_handle, uint32_t flags)
{
	struct arm_smccc_1_2_regs smc_args = {0};
	struct arm_smccc_1_2_regs smc_ret  = {0};

	smc_args.a0 = FFA_MEM_RECLAIM;
	smc_args.a1 = (uint32_t)(g_handle & 0xFFFFFFFFU);
	smc_args.a2 = (uint32_t)(g_handle >> 32);
	smc_args.a3 = flags;
	ffa_invoke(st, &smc_args, &smc_ret);

	if ((uint32_t)smc_ret.a0 == FFA_ERROR) {
		return ffa_to_errno((int)smc_ret.a2);
	}
	return 0;
}

int ffa_mem_share(struct ffa_mem_ops_args *args)
{
	if (!ffa_is_available()) {
		return -EAGAIN;
	}
	return ffa_mem_share_impl(&ffa_state, args);
}

int ffa_mem_reclaim(uint64_t g_handle, uint32_t flags)
{
	if (!ffa_is_available()) {
		return -EAGAIN;
	}
	return ffa_mem_reclaim_impl(&ffa_state, g_handle, flags);
}
