/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * FF-A memory sharing: FFA_FN64_MEM_SHARE + FFA_MEM_FRAG_TX loop +
 * FFA_MEM_RECLAIM.
 *
 * Descriptor layout (byte offsets, one receiver):
 *
 *   [0 .. 47]      ffa_mem_region header (48 bytes, fixed across versions)
 *   [48 .. 63]     ffa_mem_region_attributes, pre-v1.2 (16 bytes = FFA_EMAD_SIZE_V1_0)
 *   [48 .. 79]     ffa_mem_region_attributes, v1.2     (32 bytes = FFA_EMAD_SIZE_V1_2)
 *   [composite_off .. composite_off+15]  ffa_composite_mem_region header (16 bytes)
 *   [composite_off+16 ...]               ffa_mem_region_addr_range[0..N-1]
 *
 * composite_off = 48 + emad_size (64 pre-v1.2, 80 for v1.2).
 *
 * When the full descriptor (header + EMAD + composite hdr + N constituents)
 * exceeds the TX buffer, FFA_FN64_MEM_SHARE sends the initial fragment and
 * FFA_MEM_FRAG_TX carries the remaining constituents.  Only constituents are
 * fragmented; the headers always fit in the first fragment for any reasonable
 * TX buffer (>= 96 bytes required for v1.2, >= 80 for pre-v1.2).
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/firmware/ffa.h>
#include "ffa_internal.h"

LOG_MODULE_DECLARE(arm_ffa, CONFIG_ARM_FFA_LOG_LEVEL);

/* Minimum TX buffer size needed to hold the descriptor headers. */
#define FFA_MEM_HDR_SZ(version) \
	(48U + ffa_emad_size(version) + 16U)

/* Size of each constituent record. */
#define CONST_SZ ((uint32_t)sizeof(struct ffa_mem_region_addr_range))

/* Constituent capacity in the first TX fragment. */
static uint32_t first_frag_capacity(uint32_t tx_sz, uint32_t version)
{
	uint32_t hdr = FFA_MEM_HDR_SZ(version);

	if (tx_sz <= hdr) {
		return 0U;
	}
	return (tx_sz - hdr) / CONST_SZ;
}

/* Constituent capacity in a subsequent FFA_MEM_FRAG_TX fragment. */
static uint32_t frag_capacity(uint32_t tx_sz)
{
	return tx_sz / CONST_SZ;
}

/*
 * Build the fixed-size descriptor headers into @buf for one receiver.
 * Writes 48 + emad_size + 16 bytes total (all zeroed then filled).
 * Returns the number of bytes written, or 0 if buf_sz is too small.
 */
static uint32_t ffa_mem_build_hdr(uint8_t *buf, uint32_t buf_sz,
				  uint32_t version,
				  uint16_t sender, uint16_t receiver,
				  uint32_t total_pg_cnt, uint32_t range_cnt)
{
	struct ffa_mem_region *hdr;
	struct ffa_mem_region_attributes *attr;
	struct ffa_composite_mem_region *comp;
	uint32_t emad_sz  = ffa_emad_size(version);
	uint32_t hdr_sz   = FFA_MEM_HDR_SZ(version);
	uint32_t comp_off = 48U + emad_sz;

	if (buf_sz < hdr_sz) {
		return 0U;
	}
	memset(buf, 0, hdr_sz);

	/* Region header. */
	hdr = (struct ffa_mem_region *)(void *)buf;
	hdr->sender_id   = sender;
	hdr->attributes  = (uint16_t)(FFA_MEM_ATTRS_NWB_IS);
	hdr->ep_count    = 1U;
	if (version >= FFA_VERSION_1_1) {
		hdr->ep_mem_size   = emad_sz;
		hdr->ep_mem_offset = 48U;
	}

	/* Endpoint memory access descriptor (write only emad_sz bytes). */
	attr = (struct ffa_mem_region_attributes *)(void *)(buf + 48U);
	attr->receiver      = receiver;
	attr->attrs         = (uint8_t)(FFA_MEM_RW | FFA_MEM_NO_EXEC);
	attr->composite_off = comp_off;

	/* Composite memory region header. */
	comp = (struct ffa_composite_mem_region *)(void *)(buf + comp_off);
	comp->total_pg_cnt  = total_pg_cnt;
	comp->addr_range_cnt = range_cnt;

	return hdr_sz;
}

int ffa_mem_share_impl(struct ffa_drv_state *st,
		       struct ffa_mem_ops_args *args)
{
	struct arm_smccc_1_2_regs smc_args = {0};
	struct arm_smccc_1_2_regs smc_ret  = {0};
	uint8_t *tx = (uint8_t *)st->tx_buf;
	uint32_t tx_sz = (st->tx_sz != 0U) ? st->tx_sz
					    : st->rxtx_pages * FFA_PAGE_SIZE;
	uint32_t total_pg = 0U;
	uint32_t hdr_sz;
	uint32_t cap_first;
	uint32_t frag_sent;
	uint32_t total_desc_sz;
	uint32_t first_frag_sz;

	if (tx == NULL || tx_sz == 0U) {
		return -ENOMEM;
	}
	if (args->range_cnt == 0U || args->ranges == NULL) {
		return -EINVAL;
	}

	cap_first = first_frag_capacity(tx_sz, st->version);
	if (cap_first == 0U) {
		/* TX buffer too small to hold even the fixed headers. */
		return -ENOMEM;
	}

	/* Compute total page count. */
	for (uint32_t i = 0U; i < args->range_cnt; i++) {
		total_pg += args->ranges[i].pg_cnt;
	}

	hdr_sz = FFA_MEM_HDR_SZ(st->version);

	k_mutex_lock(&st->lock, K_FOREVER);

	/* Build fixed headers. */
	if (ffa_mem_build_hdr(tx, tx_sz, st->version, st->vm_id,
			      args->dst_id, total_pg,
			      args->range_cnt) == 0U) {
		k_mutex_unlock(&st->lock);
		return -ENOMEM;
	}

	/* Copy first batch of constituents. */
	frag_sent = (args->range_cnt < cap_first) ? args->range_cnt : cap_first;
	memcpy(tx + hdr_sz, args->ranges,
	       (size_t)frag_sent * sizeof(struct ffa_mem_region_addr_range));

	total_desc_sz = hdr_sz + args->range_cnt * CONST_SZ;
	first_frag_sz = hdr_sz + frag_sent * CONST_SZ;

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

	/* Drive FFA_MEM_FRAG_TX for any remaining constituents. */
	while ((uint32_t)smc_ret.a0 == FFA_MEM_FRAG_RX) {
		uint32_t frag_handle_lo = (uint32_t)smc_ret.a1;
		uint32_t frag_handle_hi = (uint32_t)smc_ret.a2;
		uint32_t cap;
		uint32_t this_n;
		uint32_t this_sz;

		if (frag_sent >= args->range_cnt) {
			/* All constituents sent — unexpected FRAG_RX. */
			k_mutex_unlock(&st->lock);
			return -EIO;
		}

		cap    = frag_capacity(tx_sz);
		this_n = args->range_cnt - frag_sent;
		if (this_n > cap) {
			this_n = cap;
		}
		this_sz = this_n * CONST_SZ;
		memcpy(tx, &args->ranges[frag_sent],
		       (size_t)this_n * sizeof(struct ffa_mem_region_addr_range));

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

	/* Global handle: lo=a2 (bits[31:0]), hi=a3 (bits[63:32]). */
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
