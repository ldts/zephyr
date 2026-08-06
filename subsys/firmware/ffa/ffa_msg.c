/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/firmware/ffa.h>
#include "ffa_internal.h"

LOG_MODULE_DECLARE(arm_ffa, CONFIG_ARM_FFA_LOG_LEVEL);

/* a4..a17 must be 14 contiguous unsigned longs for REQ2 payload marshalling. */
BUILD_ASSERT(offsetof(struct arm_smccc_1_2_regs, a17) ==
	     offsetof(struct arm_smccc_1_2_regs, a4) + 13 * sizeof(unsigned long),
	     "a4..a17 must be 14 contiguous longs for REQ2 payload marshalling");

static void ffa_msg_wait_for_completion(struct ffa_drv_state *st,
					struct arm_smccc_1_2_regs *ret)
{
	while ((uint32_t)ret->a0 == FFA_INTERRUPT ||
	       (uint32_t)ret->a0 == FFA_YIELD) {
		struct arm_smccc_1_2_regs args = {0};

		if ((uint32_t)ret->a0 == FFA_YIELD) {
			k_msleep(1);
		}
		args.a0 = FFA_RUN;
		args.a1 = ret->a1;
		ffa_invoke(st, &args, ret);
	}
}

int ffa_send_direct_req(struct ffa_drv_state *st, uint16_t dst, bool mode_32bit,
			struct ffa_send_direct_data *data)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs ret = {0};
	uint32_t req_id = mode_32bit ? FFA_MSG_SEND_DIRECT_REQ_32
				     : FFA_MSG_SEND_DIRECT_REQ_64;
	uint32_t resp_id = mode_32bit ? FFA_MSG_SEND_DIRECT_RESP_32
				      : FFA_MSG_SEND_DIRECT_RESP_64;

	args.a0 = req_id;
	args.a1 = FFA_PACK_TARGET_INFO(st->vm_id, dst);
	args.a2 = 0;
	args.a3 = data->data0;
	args.a4 = data->data1;
	args.a5 = data->data2;
	args.a6 = data->data3;
	args.a7 = data->data4;

	ffa_invoke(st, &args, &ret);
	ffa_msg_wait_for_completion(st, &ret);

	if ((uint32_t)ret.a0 == FFA_ERROR) {
		return ffa_to_errno((int)ret.a2);
	}
	if ((uint32_t)ret.a0 == resp_id) {
		data->data0 = ret.a3;
		data->data1 = ret.a4;
		data->data2 = ret.a5;
		data->data3 = ret.a6;
		data->data4 = ret.a7;
		return 0;
	}
	return -EINVAL;
}

/* Read a little-endian u64 from a byte array without unaligned pointer casts. */
static uint64_t ffa_uuid_lo(const struct ffa_uuid *u)
{
	uint64_t v;

	memcpy(&v, &u->bytes[0], sizeof(v));
	return v; /* AArch64 is little-endian; bytes[0] is the LSB */
}

static uint64_t ffa_uuid_hi(const struct ffa_uuid *u)
{
	uint64_t v;

	memcpy(&v, &u->bytes[8], sizeof(v));
	return v;
}

int ffa_send_direct_req2(struct ffa_drv_state *st, uint16_t dst,
			 const struct ffa_uuid *uuid,
			 struct ffa_send_direct_data2 *data)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs ret = {0};
	unsigned long *argp = &args.a4;
	unsigned long *retp;

	if (st->version < FFA_VERSION_1_2) {
		return -ENOTSUP;
	}

	args.a0 = FFA_MSG_SEND_DIRECT_REQ2;
	args.a1 = FFA_PACK_TARGET_INFO(st->vm_id, dst);
	args.a2 = ffa_uuid_lo(uuid);
	args.a3 = ffa_uuid_hi(uuid);
	for (int i = 0; i < 14; i++) {
		argp[i] = data->data[i];
	}

	ffa_invoke(st, &args, &ret);
	ffa_msg_wait_for_completion(st, &ret);

	if ((uint32_t)ret.a0 == FFA_ERROR) {
		return ffa_to_errno((int)ret.a2);
	}
	if ((uint32_t)ret.a0 == FFA_MSG_SEND_DIRECT_RESP2) {
		retp = &ret.a4;
		for (int i = 0; i < 14; i++) {
			data->data[i] = retp[i];
		}
		return 0;
	}
	return -EINVAL;
}

int ffa_rx_release(struct ffa_drv_state *st)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs ret = {0};

	args.a0 = FFA_RX_RELEASE;
	ffa_invoke(st, &args, &ret);
	if ((uint32_t)ret.a0 == FFA_ERROR) {
		return ffa_to_errno((int)ret.a2);
	}
	return 0;
}

int ffa_partition_info_get_rxbuf(struct ffa_drv_state *st,
				 const struct ffa_uuid *uuid,
				 struct ffa_partition_info *out, size_t *count)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs ret = {0};
	uint32_t u[4];
	uint32_t flags = 0;
	uint32_t reported, rec_sz;
	int rc = 0;
	bool count_only = (out == NULL) || (*count == 0);

	memcpy(u, uuid->bytes, sizeof(u));

	if (count_only && st->version > FFA_VERSION_1_0) {
		flags = FFA_PARTITION_INFO_GET_COUNT_ONLY;
	}

	k_mutex_lock(&st->lock, K_FOREVER);

	args.a0 = FFA_PARTITION_INFO_GET;
	args.a1 = u[0]; args.a2 = u[1]; args.a3 = u[2]; args.a4 = u[3];
	args.a5 = flags;
	ffa_invoke(st, &args, &ret);

	if ((uint32_t)ret.a0 == FFA_ERROR) {
		rc = ffa_to_errno((int)ret.a2);
		goto out_unlock;
	}

	reported = (uint32_t)ret.a2;
	rec_sz = (st->version > FFA_VERSION_1_0) ? (uint32_t)ret.a3
						 : FFA_1_0_PARTITION_INFO_SZ;

	if (out != NULL && !count_only) {
		uint32_t n = (reported < *count) ? reported : (uint32_t)*count;
		const uint8_t *base = st->rx_buf;

		for (uint32_t i = 0; i < n; i++) {
			const uint8_t *r = base + (size_t)i * rec_sz;
			uint16_t id16, exec16;
			uint32_t props32;

			memcpy(&id16, r + 0, sizeof(id16));
			memcpy(&exec16, r + 2, sizeof(exec16));
			memcpy(&props32, r + 4, sizeof(props32));
			out[i].id = id16;
			out[i].exec_ctxt = exec16;
			out[i].properties = props32;
			memset(&out[i].uuid, 0, sizeof(out[i].uuid));
			if (rec_sz > FFA_1_0_PARTITION_INFO_SZ) {
				memcpy(out[i].uuid.bytes, r + 8,
				       sizeof(out[i].uuid.bytes));
			}
		}
	}

	if (!count_only) {
		(void)ffa_rx_release(st);
	}
	*count = reported;

out_unlock:
	k_mutex_unlock(&st->lock);
	return rc;
}

/* BUILD_ASSERT: a3..a17 must be 15 contiguous longs for _REGS record packing.
 * Each record is 3 u64; up to 5 records per call = 15 words = a3..a17. */
BUILD_ASSERT(offsetof(struct arm_smccc_1_2_regs, a17) ==
	     offsetof(struct arm_smccc_1_2_regs, a3) + 14 * sizeof(unsigned long),
	     "a3..a17 must be 15 contiguous longs for _REGS record packing");

int ffa_partition_info_get_regs(struct ffa_drv_state *st,
				const struct ffa_uuid *uuid,
				struct ffa_partition_info *out, size_t *count)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs ret = {0};
	uint32_t u[4];
	uint16_t start_idx = 0;
	uint16_t tag = 0;
	size_t total_count = 0;
	bool count_only = (out == NULL) || (*count == 0);
	size_t out_idx = 0;

	memcpy(u, uuid->bytes, sizeof(u));

	do {
		uint16_t cur_idx;
		const unsigned long *regs;

		args.a0 = FFA_PARTITION_INFO_GET_REGS;
		args.a1 = ((uint64_t)u[1] << 32) | u[0];
		args.a2 = ((uint64_t)u[3] << 32) | u[2];
		args.a3 = (uint32_t)start_idx | ((uint32_t)tag << 16);

		ffa_invoke(st, &args, &ret);

		if ((uint32_t)ret.a0 == FFA_ERROR) {
			return ffa_to_errno((int)ret.a2);
		}

		/* On first iteration, extract total count from last_idx+1. */
		if (start_idx == 0) {
			total_count = (size_t)FFA_PIG_REGS_LAST_IDX((uint64_t)ret.a2) + 1;
			if (count_only) {
				*count = total_count;
				return 0;
			}
		}

		cur_idx = FFA_PIG_REGS_CUR_IDX((uint64_t)ret.a2);
		tag = FFA_PIG_REGS_TAG((uint64_t)ret.a2);

		/* Unpack partition records from a3 onward; 3 u64 per record. */
		regs = &ret.a3;
		for (uint16_t i = start_idx; i <= cur_idx && out_idx < *count; i++) {
			size_t rec_off = (size_t)(i - start_idx) * 3;
			uint64_t word0, word1, word2;

			word0 = (uint64_t)regs[rec_off];
			word1 = (uint64_t)regs[rec_off + 1];
			word2 = (uint64_t)regs[rec_off + 2];

			out[out_idx].id = FFA_PIG_REC_ID(word0);
			out[out_idx].exec_ctxt = FFA_PIG_REC_EXEC_CTXT(word0);
			out[out_idx].properties = FFA_PIG_REC_PROPS(word0);
			memcpy(&out[out_idx].uuid.bytes[0], &word1, sizeof(word1));
			memcpy(&out[out_idx].uuid.bytes[8], &word2, sizeof(word2));
			out_idx++;
		}

		start_idx = cur_idx + 1;
	} while (start_idx < total_count);

	*count = total_count;
	return 0;
}

static int ffa_partition_info_get_impl(struct ffa_drv_state *st,
				       const struct ffa_uuid *uuid,
				       struct ffa_partition_info *out,
				       size_t *count)
{
	if (st->version >= FFA_VERSION_1_2 &&
	    ffa_query_feature(st, FFA_PARTITION_INFO_GET_REGS, NULL) == 0) {
		return ffa_partition_info_get_regs(st, uuid, out, count);
	}
	return ffa_partition_info_get_rxbuf(st, uuid, out, count);
}

int ffa_partition_info_get(const struct ffa_uuid *uuid,
			   struct ffa_partition_info *out, size_t *count)
{
	if (!ffa_is_available()) {
		return -EAGAIN;
	}
	return ffa_partition_info_get_impl(&ffa_state, uuid, out, count);
}

int ffa_msg_send_direct_req(uint16_t dst, struct ffa_send_direct_data *data)
{
	if (!ffa_is_available()) {
		return -EAGAIN;
	}
	/* NS endpoint talks to a 64-bit SP by default; 32-bit selection is a
	 * per-partition property discovered via partition info (caller-driven
	 * later). SP-2b uses native 64-bit. */
	return ffa_send_direct_req(&ffa_state, dst, false, data);
}

int ffa_msg_send_direct_req2(uint16_t dst, const struct ffa_uuid *uuid,
			     struct ffa_send_direct_data2 *data)
{
	if (!ffa_is_available()) {
		return -EAGAIN;
	}
	return ffa_send_direct_req2(&ffa_state, dst, uuid, data);
}

