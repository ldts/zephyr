/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file main.c
 * @brief End-to-end tests for the OP-TEE FF-A transport on qemu_cortex_a53.
 *
 * Uses ffa_test_set_conduit() to install a mock that simulates OP-TEE
 * secure-partition responses, then exercises the full optee_ffa driver stack:
 * probe → get_version → open_session → invoke_func → close_session →
 * shm_register → shm_unregister.
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/tee.h>
#include <zephyr/firmware/ffa.h>

#include "ffa_internal.h"   /* ffa_state, ffa_test_set_conduit, FID constants */

/* OP-TEE FFA protocol constants (duplicated from optee_ffa.c) */
#define OPTEE_FFA_GET_API_VERSION         UINT32_C(0)
#define OPTEE_FFA_GET_OS_VERSION          UINT32_C(1)
#define OPTEE_FFA_EXCHANGE_CAPABILITIES   UINT32_C(2)
#define OPTEE_FFA_UNREGISTER_SHM          UINT32_C(3)
#define OPTEE_FFA_YIELDING_CALL_WITH_ARG  (UINT32_C(0) | BIT(31))
#define OPTEE_FFA_YIELDING_CALL_RESUME    (UINT32_C(1) | BIT(31))

#define OPTEE_FFA_VERSION_MAJOR           UINT32_C(1)
#define OPTEE_FFA_SEC_CAP_ARG_OFFSET      BIT(0)

#define OPTEE_FFA_RETURN_DONE             UINT32_C(0)

/* FF-A v1.1 partition info record: id[2] exec_ctxt[2] props[4] uuid[16] = 24B */
#define OPTEE_FFA_PART_INFO_REC_SZ  24U
#define MOCK_OPTEE_ID               UINT16_C(0x8001)

/* OP-TEE OS partition UUID: 486178e0-e7f8-11e3-bc5e-0002a5d5c51b */
static const uint8_t optee_uuid_bytes[16] = {
	0x48, 0x61, 0x78, 0xe0, 0xe7, 0xf8, 0x11, 0xe3,
	0xbc, 0x5e, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
};

/* Fake RX/TX buffers (4 KiB aligned to satisfy FF-A page constraints). */
static uint8_t fake_rx[4096] __aligned(4096);
static uint8_t fake_tx[4096] __aligned(4096);

/* Re-init entry point exposed by optee_ffa.c when CONFIG_ZTEST is set. */
extern int optee_ffa_test_reinit(const struct device *dev);

/* ------------------------------------------------------------------ */
/* Mock FF-A conduit                                                   */
/* ------------------------------------------------------------------ */

static void fill_rx_with_optee_partition(void)
{
	uint8_t *r = fake_rx;
	uint16_t id    = MOCK_OPTEE_ID;
	uint16_t exec  = 1U;
	uint32_t props = (1U << 0) | (1U << 1) | (1U << 8); /* DIRECT_RECV|SEND|AARCH64 */

	memcpy(r + 0, &id,    sizeof(id));
	memcpy(r + 2, &exec,  sizeof(exec));
	memcpy(r + 4, &props, sizeof(props));
	memcpy(r + 8, optee_uuid_bytes, 16);
}

static void mock_handle_direct_req(const struct arm_smccc_1_2_regs *args,
				   struct arm_smccc_1_2_regs *res)
{
	uint32_t svc = (uint32_t)args->a3;  /* data0 = service ID */

	switch (svc) {
	case OPTEE_FFA_GET_API_VERSION:
		res->a3 = OPTEE_FFA_VERSION_MAJOR;
		res->a4 = 0U;
		break;
	case OPTEE_FFA_GET_OS_VERSION:
		res->a3 = 3U;  /* major */
		res->a4 = 17U; /* minor */
		res->a5 = 0U;  /* sha */
		break;
	case OPTEE_FFA_EXCHANGE_CAPABILITIES:
		res->a3 = 0U;  /* error = success */
		res->a5 = OPTEE_FFA_SEC_CAP_ARG_OFFSET;
		res->a6 = 10U; /* max_notif_value */
		break;
	case OPTEE_FFA_UNREGISTER_SHM:
		res->a3 = 0U;
		break;
	case OPTEE_FFA_YIELDING_CALL_WITH_ARG:
	case OPTEE_FFA_YIELDING_CALL_RESUME:
		res->a3 = 0U;                   /* error = 0 */
		res->a4 = OPTEE_FFA_RETURN_DONE;
		break;
	default:
		res->a0 = FFA_ERROR;
		res->a2 = (unsigned long)FFA_RET_NOT_SUPPORTED;
		break;
	}
}

static void mock_optee_ffa_conduit(const struct arm_smccc_1_2_regs *args,
				   struct arm_smccc_1_2_regs *res)
{
	memset(res, 0, sizeof(*res));

	switch ((uint32_t)args->a0) {
	case FFA_PARTITION_INFO_GET:
		fill_rx_with_optee_partition();
		res->a0 = FFA_SUCCESS_32;
		res->a2 = 1U;                      /* count */
		res->a3 = OPTEE_FFA_PART_INFO_REC_SZ;
		break;

	case FFA_RX_RELEASE:
		res->a0 = FFA_SUCCESS_32;
		break;

	case FFA_MSG_SEND_DIRECT_REQ_32:
	case FFA_MSG_SEND_DIRECT_REQ_64:
		/* Pre-set success resp FID; error cases inside will override */
		res->a0 = ((uint32_t)args->a0 == FFA_MSG_SEND_DIRECT_REQ_32)
			  ? FFA_MSG_SEND_DIRECT_RESP_32
			  : FFA_MSG_SEND_DIRECT_RESP_64;
		mock_handle_direct_req(args, res);
		break;

	case FFA_FN64_MEM_SHARE:
	case FFA_MEM_SHARE_32:
		res->a0 = FFA_SUCCESS_64;
		res->a2 = 0x1234UL;  /* handle_lo */
		res->a3 = 0UL;       /* handle_hi */
		break;

	case FFA_MEM_RECLAIM:
		res->a0 = FFA_SUCCESS_32;
		break;

	default:
		res->a0 = FFA_ERROR;
		res->a2 = (unsigned long)FFA_RET_NOT_SUPPORTED;
		break;
	}
}

/* ------------------------------------------------------------------ */
/* Test fixture: set up ffa_state and install mock conduit             */
/* ------------------------------------------------------------------ */

static const struct device *g_dev;

static void test_setup(void)
{
	/* Point ffa_state at our fake buffers (version 1.1: uses RX path) */
	ffa_state.rx_buf     = fake_rx;
	ffa_state.tx_buf     = fake_tx;
	ffa_state.rxtx_pages = 1U;
	ffa_state.version    = FFA_VERSION_1_1;
	ffa_state.vm_id      = 0x0001U;
	ffa_state.available  = true;

	ffa_test_set_conduit(mock_optee_ffa_conduit);

	g_dev = DEVICE_DT_GET_ONE(linaro_optee_ffa);
	zassert_not_null(g_dev, "linaro,optee-ffa device not found in DT");

	/* Re-run probe with mock conduit */
	zassert_equal(optee_ffa_test_reinit(g_dev), 0,
		      "optee_ffa probe failed");
}

/* ------------------------------------------------------------------ */
/* Test cases                                                          */
/* ------------------------------------------------------------------ */

ZTEST(optee_ffa, test_probe_succeeds)
{
	test_setup();
	/* If test_setup didn't assert, probe succeeded */
}

ZTEST(optee_ffa, test_get_version)
{
	test_setup();

	struct tee_version_info info = {0};

	zassert_equal(tee_get_version(g_dev, &info), 0, "get_version failed");
	zassert_equal(info.impl_id, 1U, "should be OP-TEE (id=1)");
	zassert_equal(info.gen_caps & TEE_GEN_CAP_GP, TEE_GEN_CAP_GP,
		      "must report GP capability");
}

ZTEST(optee_ffa, test_open_and_close_session)
{
	test_setup();

	struct tee_open_session_arg arg = {
		.uuid     = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
			      0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10 },
		.clnt_login = 0,
	};
	uint32_t session_id = 0;

	int rc = tee_open_session(g_dev, &arg, 0, NULL, &session_id);

	/* The mock always returns RETURN_DONE with error=0.  The optee_msg_arg
	 * returned has ret=0 (TEEC_SUCCESS) so open_session should succeed. */
	zassert_equal(rc, 0, "open_session failed (rc=%d)", rc);

	/* Close the session */
	zassert_equal(tee_close_session(g_dev, session_id), 0,
		      "close_session failed");
}

ZTEST(optee_ffa, test_invoke_func)
{
	test_setup();

	/* Open session first */
	struct tee_open_session_arg open_arg = {0};
	uint32_t sid = 0;

	zassert_equal(tee_open_session(g_dev, &open_arg, 0, NULL, &sid), 0,
		      "open_session for invoke test failed");

	struct tee_invoke_func_arg inv_arg = {
		.session = sid,
		.func    = 0x1000,
	};

	zassert_equal(tee_invoke_func(g_dev, &inv_arg, 0, NULL), 0,
		      "invoke_func failed");

	tee_close_session(g_dev, sid);
}

ZTEST(optee_ffa, test_shm_register_and_unregister)
{
	test_setup();

	/* Allocate a shared memory region */
	struct tee_shm *shm = NULL;
	int rc = tee_add_shm(g_dev, NULL, 0, 4096U, TEE_SHM_ALLOC, &shm);

	if (rc) {
		/* Heap too small in test config; skip gracefully */
		ztest_test_skip();
		return;
	}

	/* Register with OP-TEE (triggers ffa_mem_share) */
	const struct tee_driver_api *api = g_dev->api;

	rc = api->shm_register(g_dev, shm);
	zassert_equal(rc, 0, "shm_register failed (rc=%d)", rc);

	/* Unregister (triggers UNREGISTER_SHM + ffa_mem_reclaim) */
	rc = api->shm_unregister(g_dev, shm);
	zassert_equal(rc, 0, "shm_unregister failed (rc=%d)", rc);

	tee_rm_shm(g_dev, shm);
}

ZTEST_SUITE(optee_ffa, NULL, NULL, NULL, NULL, NULL);
