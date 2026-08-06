/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/arm64/arm-smccc.h>
#include "ffa_internal.h"

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
