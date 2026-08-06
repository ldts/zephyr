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
#include <zephyr/init.h>
#include <zephyr/firmware/ffa.h>
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

/* RXTX buffer note: static .bss buffers are acceptable for QEMU simulation;
 * production use on hardware with non-coherent caches may require DMA-safe
 * allocation. This is revisited in SP-5 (OP-TEE transport bring-up). */
#define FFA_RXTX_BUF_SIZE (CONFIG_ARM_FFA_RXTX_PAGES * FFA_PAGE_SIZE)

static uint8_t ffa_tx_buf[FFA_RXTX_BUF_SIZE] __aligned(FFA_PAGE_SIZE);
static uint8_t ffa_rx_buf[FFA_RXTX_BUF_SIZE] __aligned(FFA_PAGE_SIZE);

struct ffa_drv_state ffa_state;

enum arm_smccc_conduit ffa_detect_conduit(void)
{
	/*
	 * SP-2a: Zephyr as the single NS endpoint on qemu_cortex_a53 uses SMC.
	 * DT-driven conduit selection (smc/hvc) arrives with the OP-TEE FF-A
	 * transport in SP-5, mirroring the existing OP-TEE SMC driver.
	 */
	return SMCCC_CONDUIT_SMC;
}

int ffa_rxtx_map(struct ffa_drv_state *st)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs res = {0};

	args.a0 = FFA_RXTX_MAP_64;
	args.a1 = (unsigned long)(uintptr_t)st->tx_buf;
	args.a2 = (unsigned long)(uintptr_t)st->rx_buf;
	args.a3 = st->rxtx_pages;
	ffa_invoke(st, &args, &res);

	if ((uint32_t)res.a0 == FFA_ERROR) {
		return ffa_to_errno((int)res.a2);
	}
	return 0;
}

bool ffa_is_available(void)
{
	return ffa_state.available;
}

int ffa_version(uint32_t *out)
{
	if (!ffa_state.available) {
		return -EAGAIN;
	}
	if (out != NULL) {
		*out = ffa_state.version;
	}
	return 0;
}

int ffa_id_get(uint16_t *vm_id)
{
	if (!ffa_state.available) {
		return -EAGAIN;
	}
	if (vm_id != NULL) {
		*vm_id = ffa_state.vm_id;
	}
	return 0;
}

static int ffa_init(void)
{
	struct ffa_drv_state *st = &ffa_state;
	int ret;

	k_mutex_init(&st->lock);
	st->tx_buf = ffa_tx_buf;
	st->rx_buf = ffa_rx_buf;
	st->rxtx_pages = CONFIG_ARM_FFA_RXTX_PAGES;
	st->conduit = ffa_detect_conduit();

#ifndef CONFIG_ZTEST
	ffa_conduit_fn = (st->conduit == SMCCC_CONDUIT_HVC)
			? arm_smccc_1_2_hvc : arm_smccc_1_2_smc;
#endif

	if (ffa_conduit_fn == NULL) {
		LOG_INF("FF-A: no conduit configured; core idle");
		return 0;
	}

	ret = ffa_negotiate_version(st);
	if (ret != 0) {
		LOG_WRN("FF-A not available: version negotiation failed (%d)", ret);
		return 0;
	}

	ret = ffa_get_id(st);
	if (ret != 0) {
		LOG_WRN("FF-A not available: ID_GET failed (%d)", ret);
		return 0;
	}

	ret = ffa_query_feature(st, FFA_RXTX_MAP_64, NULL);
	if (ret != 0) {
		LOG_INF("FFA_RXTX_MAP feature not advertised (%d)", ret);
	}

	ret = ffa_rxtx_map(st);
	if (ret != 0) {
		LOG_WRN("FF-A not available: RXTX_MAP failed (%d)", ret);
		return 0;
	}

	st->available = true;
	LOG_INF("FF-A core ready (v1.%u, id 0x%04x)",
		FFA_VERSION_MINOR(st->version), st->vm_id);
	return 0;
}

SYS_INIT(ffa_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
