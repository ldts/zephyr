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
