/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/logging/log.h>
#include "ffa_internal.h"

LOG_MODULE_REGISTER(arm_ffa, CONFIG_ARM_FFA_LOG_LEVEL);

/* SP-1 deferred layout guards: the asm in smccc-call.S relies on these. */
BUILD_ASSERT(sizeof(struct arm_smccc_1_2_regs) == 18 * sizeof(unsigned long),
	     "arm_smccc_1_2_regs must be 18 contiguous longs");
BUILD_ASSERT(offsetof(struct arm_smccc_1_2_regs, a17) == 17 * sizeof(unsigned long),
	     "arm_smccc_1_2_regs a17 must be at offset 17*8");

int ffa_to_errno(int ffa_ret)
{
	switch (ffa_ret) {
	case FFA_RET_SUCCESS:            return 0;
	case FFA_RET_NOT_SUPPORTED:      return -ENOTSUP;
	case FFA_RET_INVALID_PARAMETERS: return -EINVAL;
	case FFA_RET_NO_MEMORY:          return -ENOMEM;
	case FFA_RET_BUSY:               return -EBUSY;
	case FFA_RET_INTERRUPTED:        return -EINTR;
	case FFA_RET_DENIED:             return -EACCES;
	case FFA_RET_RETRY:              return -EAGAIN;
	case FFA_RET_ABORTED:            return -ECANCELED;
	case FFA_RET_NO_DATA:            return -ENODATA;
	default:                         return -EINVAL;
	}
}

/* Default conduit set at init; overridable by tests. */
static ffa_conduit_fn_t ffa_conduit_fn;

void ffa_invoke(struct ffa_drv_state *st, struct arm_smccc_1_2_regs *args,
		struct arm_smccc_1_2_regs *res)
{
	ARG_UNUSED(st);
	__ASSERT_NO_MSG(ffa_conduit_fn != NULL);
	ffa_conduit_fn(args, res);
}

#ifdef CONFIG_ZTEST
void ffa_test_set_conduit(ffa_conduit_fn_t fn)
{
	ffa_conduit_fn = fn;
}
#endif

int ffa_get_id(struct ffa_drv_state *st)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs res = {0};

	args.a0 = FFA_ID_GET;
	ffa_invoke(st, &args, &res);

	if ((uint32_t)res.a0 == FFA_ERROR) {
		return ffa_to_errno((int)res.a2);
	}
	st->vm_id = (uint16_t)(res.a2 & 0xFFFFU);
	return 0;
}

int ffa_query_feature(struct ffa_drv_state *st, uint32_t ffa_func_id,
		      uint32_t *out)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs res = {0};

	args.a0 = FFA_FEATURES;
	args.a1 = ffa_func_id;
	ffa_invoke(st, &args, &res);

	if ((uint32_t)res.a0 == FFA_ERROR) {
		return ffa_to_errno((int)res.a2);
	}
	if (out != NULL) {
		*out = (uint32_t)res.a2;
	}
	return 0;
}

int ffa_negotiate_version(struct ffa_drv_state *st)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs res = {0};
	uint32_t ret;
	uint16_t major, minor;

	args.a0 = FFA_VERSION;
	args.a1 = FFA_VERSION_1_2;
	ffa_invoke(st, &args, &res);

	ret = (uint32_t)res.a0;
	if (ret == FFA_VERSION_NOT_SUPPORTED || (ret & 0x80000000U)) {
		LOG_ERR("FFA_VERSION not supported by SPMC");
		return -ENOTSUP;
	}

	major = FFA_VERSION_MAJOR(ret);
	minor = FFA_VERSION_MINOR(ret);
	if (major != 1) {
		LOG_ERR("Unsupported FF-A major version %u", major);
		return -ENOTSUP;
	}

	if (minor >= 2) {
		st->version = FFA_VERSION_1_2;
	} else if (minor == 1) {
		st->version = FFA_VERSION_1_1;
	} else {
		st->version = FFA_VERSION_1_0;
	}

	LOG_INF("FF-A version negotiated: 1.%u", FFA_VERSION_MINOR(st->version));
	return 0;
}

