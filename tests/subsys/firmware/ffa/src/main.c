/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <errno.h>
#include "ffa_internal.h"
#include <zephyr/firmware/ffa.h>

ZTEST(ffa_core, test_to_errno_mapping)
{
	zassert_equal(ffa_to_errno(FFA_RET_SUCCESS), 0, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_NOT_SUPPORTED), -ENOTSUP, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_INVALID_PARAMETERS), -EINVAL, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_NO_MEMORY), -ENOMEM, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_BUSY), -EBUSY, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_INTERRUPTED), -EINTR, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_DENIED), -EACCES, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_RETRY), -EAGAIN, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_ABORTED), -ECANCELED, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_NO_DATA), -ENODATA, NULL);
	zassert_equal(ffa_to_errno(-12345), -EINVAL, "unknown maps to -EINVAL");
}

#include <string.h>
#include <zephyr/arch/arm64/arm-smccc.h>

/* Mock conduit: canned response + capture of the last request. */
static struct arm_smccc_1_2_regs mock_last_args;
static struct arm_smccc_1_2_regs mock_next_res;

static void mock_conduit(const struct arm_smccc_1_2_regs *args,
			 struct arm_smccc_1_2_regs *res)
{
	mock_last_args = *args;
	*res = mock_next_res;
}

static void set_mock_res_a0(unsigned long a0)
{
	memset(&mock_next_res, 0, sizeof(mock_next_res));
	mock_next_res.a0 = a0;
}

ZTEST(ffa_core, test_version_offer_and_accept_1_2)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_res_a0(FFA_VERSION_1_2);

	zassert_equal(ffa_negotiate_version(&st), 0, NULL);
	zassert_equal(st.version, FFA_VERSION_1_2, NULL);
	/* We offered FFA_VERSION with 1.2 in a1. */
	zassert_equal(mock_last_args.a0, FFA_VERSION, "function id");
	zassert_equal(mock_last_args.a1, FFA_VERSION_1_2, "offered version");
}

ZTEST(ffa_core, test_version_downgrade_to_1_1)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_res_a0(FFA_VERSION_1_1);

	zassert_equal(ffa_negotiate_version(&st), 0, NULL);
	zassert_equal(st.version, FFA_VERSION_1_1, NULL);
}

ZTEST(ffa_core, test_version_downgrade_to_1_0)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_res_a0(FFA_VERSION_1_0);

	zassert_equal(ffa_negotiate_version(&st), 0, NULL);
	zassert_equal(st.version, FFA_VERSION_1_0, NULL);
}

ZTEST(ffa_core, test_version_not_supported)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_res_a0(FFA_VERSION_NOT_SUPPORTED);

	zassert_equal(ffa_negotiate_version(&st), -ENOTSUP, NULL);
}

ZTEST(ffa_core, test_version_bad_major)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_res_a0(FFA_PACK_VERSION(2, 0));

	zassert_equal(ffa_negotiate_version(&st), -ENOTSUP, NULL);
}

/* Helper: canned FFA_SUCCESS_32 with a chosen a2. */
static void set_mock_success(unsigned long a2)
{
	memset(&mock_next_res, 0, sizeof(mock_next_res));
	mock_next_res.a0 = FFA_SUCCESS_32;
	mock_next_res.a2 = a2;
}

/* Helper: canned FFA_ERROR with a chosen error code in a2. */
static void set_mock_error(long code)
{
	memset(&mock_next_res, 0, sizeof(mock_next_res));
	mock_next_res.a0 = FFA_ERROR;
	mock_next_res.a2 = (unsigned long)code;
}

ZTEST(ffa_core, test_id_get_success)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_success(0x8001);

	zassert_equal(ffa_get_id(&st), 0, NULL);
	zassert_equal(st.vm_id, 0x8001, NULL);
	zassert_equal(mock_last_args.a0, FFA_ID_GET, "function id");
}

ZTEST(ffa_core, test_id_get_error)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_error(FFA_RET_NOT_SUPPORTED);

	zassert_equal(ffa_get_id(&st), -ENOTSUP, NULL);
}

ZTEST(ffa_core, test_features_supported)
{
	struct ffa_drv_state st = {0};
	uint32_t props = 0xdead;

	ffa_test_set_conduit(mock_conduit);
	set_mock_success(FFA_FEAT_RXTX_MIN_SZ_4K);

	zassert_equal(ffa_query_feature(&st, FFA_RXTX_MAP_64, &props), 0, NULL);
	zassert_equal(props, FFA_FEAT_RXTX_MIN_SZ_4K, NULL);
	zassert_equal(mock_last_args.a0, FFA_FEATURES, NULL);
	zassert_equal(mock_last_args.a1, FFA_RXTX_MAP_64, NULL);
}

ZTEST(ffa_core, test_features_not_supported)
{
	struct ffa_drv_state st = {0};
	uint32_t props = 0;

	ffa_test_set_conduit(mock_conduit);
	set_mock_error(FFA_RET_NOT_SUPPORTED);

	zassert_equal(ffa_query_feature(&st, FFA_RXTX_MAP_64, &props), -ENOTSUP, NULL);
}

ZTEST(ffa_core, test_rxtx_map_marshalling)
{
	static uint8_t txb[FFA_PAGE_SIZE] __aligned(FFA_PAGE_SIZE);
	static uint8_t rxb[FFA_PAGE_SIZE] __aligned(FFA_PAGE_SIZE);
	struct ffa_drv_state st = {0};

	st.tx_buf = txb;
	st.rx_buf = rxb;
	st.rxtx_pages = 1;

	ffa_test_set_conduit(mock_conduit);
	set_mock_success(0);

	zassert_equal(ffa_rxtx_map(&st), 0, NULL);
	zassert_equal(mock_last_args.a0, FFA_RXTX_MAP_64, NULL);
	zassert_equal(mock_last_args.a1, (unsigned long)(uintptr_t)txb, "tx addr");
	zassert_equal(mock_last_args.a2, (unsigned long)(uintptr_t)rxb, "rx addr");
	zassert_equal(mock_last_args.a3, 1, "page count");
}

ZTEST(ffa_core, test_rxtx_map_error)
{
	static uint8_t txb[FFA_PAGE_SIZE] __aligned(FFA_PAGE_SIZE);
	static uint8_t rxb[FFA_PAGE_SIZE] __aligned(FFA_PAGE_SIZE);
	struct ffa_drv_state st = { .tx_buf = txb, .rx_buf = rxb, .rxtx_pages = 1 };

	ffa_test_set_conduit(mock_conduit);
	set_mock_error(FFA_RET_NO_MEMORY);

	zassert_equal(ffa_rxtx_map(&st), -ENOMEM, NULL);
}

/* After boot SYS_INIT ran with the real (SMC) conduit but no responder, so
 * FF-A must report unavailable rather than crashing. */
ZTEST(ffa_core, test_public_accessors_when_unavailable)
{
	uint32_t v = 0;
	uint16_t id = 0;

	if (!ffa_is_available()) {
		zassert_equal(ffa_version(&v), -EAGAIN, NULL);
		zassert_equal(ffa_id_get(&id), -EAGAIN, NULL);
	} else {
		zassert_equal(ffa_version(&v), 0, NULL);
		zassert_equal(ffa_id_get(&id), 0, NULL);
	}
}

/* Scripted responses for multi-invoke sequences (e.g. INTERRUPT -> RUN -> RESP). */
static struct arm_smccc_1_2_regs mock_script[4];
static int mock_script_len;
static int mock_script_pos;
static struct arm_smccc_1_2_regs mock_seen[4];
static int mock_seen_n;

static void mock_script_conduit(const struct arm_smccc_1_2_regs *args,
				struct arm_smccc_1_2_regs *res)
{
	if (mock_seen_n < (int)ARRAY_SIZE(mock_seen)) {
		mock_seen[mock_seen_n++] = *args;
	}
	if (mock_script_pos < mock_script_len) {
		*res = mock_script[mock_script_pos++];
	} else {
		memset(res, 0, sizeof(*res));
	}
}

static void mock_script_reset(void)
{
	mock_script_len = 0;
	mock_script_pos = 0;
	mock_seen_n = 0;
	memset(mock_script, 0, sizeof(mock_script));
	memset(mock_seen, 0, sizeof(mock_seen));
}

ZTEST(ffa_core, test_direct_req_roundtrip_64)
{
	struct ffa_drv_state st = {0};
	struct ffa_send_direct_data d = {
		.data0 = 0x11, .data1 = 0x22, .data2 = 0x33,
		.data3 = 0x44, .data4 = 0x55,
	};

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	/* single response: RESP_64 with echoed+1 payload */
	mock_script[0] = (struct arm_smccc_1_2_regs){
		.a0 = FFA_MSG_SEND_DIRECT_RESP_64,
		.a3 = 0x111, .a4 = 0x222, .a5 = 0x333, .a6 = 0x444, .a7 = 0x555,
	};
	mock_script_len = 1;

	zassert_equal(ffa_send_direct_req(&st, 0x8001, false, &d), 0, NULL);
	/* request marshalling */
	zassert_equal(mock_seen[0].a0, FFA_MSG_SEND_DIRECT_REQ_64, NULL);
	zassert_equal(mock_seen[0].a1, FFA_PACK_TARGET_INFO(st.vm_id, 0x8001), NULL);
	zassert_equal(mock_seen[0].a2, 0, NULL);
	zassert_equal(mock_seen[0].a3, 0x11, NULL);
	zassert_equal(mock_seen[0].a7, 0x55, NULL);
	/* response unpacked */
	zassert_equal(d.data0, 0x111, NULL);
	zassert_equal(d.data4, 0x555, NULL);
}

ZTEST(ffa_core, test_direct_req_32bit_fid)
{
	struct ffa_drv_state st = {0};
	struct ffa_send_direct_data d = {0};

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0] = (struct arm_smccc_1_2_regs){ .a0 = FFA_MSG_SEND_DIRECT_RESP_32 };
	mock_script_len = 1;

	zassert_equal(ffa_send_direct_req(&st, 0x1, true, &d), 0, NULL);
	zassert_equal(mock_seen[0].a0, FFA_MSG_SEND_DIRECT_REQ_32, NULL);
}

ZTEST(ffa_core, test_direct_req_completion_loop)
{
	struct ffa_drv_state st = {0};
	struct ffa_send_direct_data d = {0};

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	/* INTERRUPT (a1 carries the target for FFA_RUN) then real RESP */
	mock_script[0] = (struct arm_smccc_1_2_regs){ .a0 = FFA_INTERRUPT, .a1 = 0xABCD };
	mock_script[1] = (struct arm_smccc_1_2_regs){ .a0 = FFA_MSG_SEND_DIRECT_RESP_64 };
	mock_script_len = 2;

	zassert_equal(ffa_send_direct_req(&st, 0x2, false, &d), 0, NULL);
	/* second invoke must be FFA_RUN with a1 from the interrupt */
	zassert_equal(mock_seen[1].a0, FFA_RUN, NULL);
	zassert_equal(mock_seen[1].a1, 0xABCD, NULL);
}

ZTEST(ffa_core, test_direct_req_error)
{
	struct ffa_drv_state st = {0};
	struct ffa_send_direct_data d = {0};

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0] = (struct arm_smccc_1_2_regs){ .a0 = FFA_ERROR, .a2 = (unsigned long)FFA_RET_BUSY };
	mock_script_len = 1;

	zassert_equal(ffa_send_direct_req(&st, 0x3, false, &d), -EBUSY, NULL);
}

ZTEST(ffa_core, test_direct_req2_roundtrip)
{
	struct ffa_drv_state st = { .version = FFA_VERSION_1_2 };
	struct ffa_uuid uuid = { .bytes = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15} };
	struct ffa_send_direct_data2 d = {0};

	for (int i = 0; i < 14; i++) {
		d.data[i] = 0x100 + i;
	}
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0].a0 = FFA_MSG_SEND_DIRECT_RESP2;
	for (int i = 0; i < 14; i++) {
		((unsigned long *)&mock_script[0].a4)[i] = 0x200 + i;
	}
	mock_script_len = 1;

	zassert_equal(ffa_send_direct_req2(&st, 0x8001, &uuid, &d), 0, NULL);
	zassert_equal(mock_seen[0].a0, FFA_MSG_SEND_DIRECT_REQ2, NULL);
	zassert_equal(mock_seen[0].a1, FFA_PACK_TARGET_INFO(st.vm_id, 0x8001), NULL);
	/* a2 = LE u64 of bytes[0..7], a3 = LE u64 of bytes[8..15] */
	zassert_equal(mock_seen[0].a2, 0x0706050403020100UL, NULL);
	zassert_equal(mock_seen[0].a3, 0x0F0E0D0C0B0A0908UL, NULL);
	/* payload x4.. carried data[0..13] */
	zassert_equal(mock_seen[0].a4, 0x100, NULL);
	zassert_equal(mock_seen[0].a17, 0x100 + 13, NULL);
	/* response unpacked */
	zassert_equal(d.data[0], 0x200, NULL);
	zassert_equal(d.data[13], 0x200 + 13, NULL);
}

ZTEST(ffa_core, test_direct_req2_requires_1_2)
{
	struct ffa_drv_state st = { .version = FFA_VERSION_1_1 };
	struct ffa_uuid uuid = {0};
	struct ffa_send_direct_data2 d = {0};

	ffa_test_set_conduit(mock_script_conduit); /* must not be invoked */
	zassert_equal(ffa_send_direct_req2(&st, 0x1, &uuid, &d), -ENOTSUP, NULL);
}

ZTEST(ffa_core, test_direct_req2_error)
{
	struct ffa_drv_state st = { .version = FFA_VERSION_1_2 };
	struct ffa_uuid uuid = {0};
	struct ffa_send_direct_data2 d = {0};

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0].a0 = FFA_ERROR;
	mock_script[0].a2 = (unsigned long)FFA_RET_DENIED;
	mock_script_len = 1;

	zassert_equal(ffa_send_direct_req2(&st, 0x1, &uuid, &d), -EACCES, NULL);
}

/* On-wire v>1.0 partition record layout. */
struct pinfo_rec_le { uint16_t id; uint16_t exec; uint32_t props; uint8_t uuid[16]; };

ZTEST(ffa_core, test_partition_info_rxbuf)
{
	static uint8_t rxbuf[256] __aligned(8);
	struct pinfo_rec_le *rec = (struct pinfo_rec_le *)rxbuf;
	struct ffa_drv_state st = { .version = FFA_VERSION_1_1, .rx_buf = rxbuf };
	struct ffa_uuid uuid = {0};
	struct ffa_partition_info out[2];
	size_t count = 2;

	k_mutex_init(&st.lock);
	rec[0].id = 0x8001; rec[0].exec = 1; rec[0].props = 0x3;
	for (int i = 0; i < 16; i++) { rec[0].uuid[i] = i; }

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0] = (struct arm_smccc_1_2_regs){
		.a0 = FFA_SUCCESS_32, .a2 = 1, .a3 = sizeof(struct pinfo_rec_le),
	};
	mock_script[1] = (struct arm_smccc_1_2_regs){ .a0 = FFA_SUCCESS_32 }; /* RX_RELEASE */
	mock_script_len = 2;

	zassert_equal(ffa_partition_info_get_rxbuf(&st, &uuid, out, &count), 0, NULL);
	zassert_equal(mock_seen[0].a0, FFA_PARTITION_INFO_GET, NULL);
	zassert_equal(count, 1, NULL);
	zassert_equal(out[0].id, 0x8001, NULL);
	zassert_equal(out[0].properties, 0x3, NULL);
	zassert_equal(out[0].uuid.bytes[15], 15, NULL);
	zassert_equal(mock_seen[1].a0, FFA_RX_RELEASE, NULL); /* released */
}

ZTEST(ffa_core, test_partition_info_count_only)
{
	static uint8_t rxbuf[64] __aligned(8);
	struct ffa_drv_state st = { .version = FFA_VERSION_1_1, .rx_buf = rxbuf };
	struct ffa_uuid uuid = {0};
	size_t count = 0;

	k_mutex_init(&st.lock);
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0] = (struct arm_smccc_1_2_regs){ .a0 = FFA_SUCCESS_32, .a2 = 3 };
	mock_script_len = 1;

	zassert_equal(ffa_partition_info_get_rxbuf(&st, &uuid, NULL, &count), 0, NULL);
	zassert_equal(count, 3, NULL);
	/* count-only sets flags bit0 and does NOT release RX */
	zassert_equal(mock_seen[0].a5, FFA_PARTITION_INFO_GET_COUNT_ONLY, NULL);
}

/* FF-A 1.0 has no count-only mode: a count-only request sends flags==0, so the
 * SPMC does a full populate of the shared RX buffer. The driver must therefore
 * release RX even though the caller only asked for a count. Regression for the
 * leaked-RX bug that wedged the next RX-buffer query. */
ZTEST(ffa_core, test_partition_info_v1_0_count_only_releases_rx)
{
	static uint8_t rxbuf[64] __aligned(8);
	struct ffa_drv_state st = { .version = FFA_VERSION_1_0, .rx_buf = rxbuf };
	struct ffa_uuid uuid = {0};
	size_t count = 0;

	k_mutex_init(&st.lock);
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0] = (struct arm_smccc_1_2_regs){ .a0 = FFA_SUCCESS_32, .a2 = 2 };
	mock_script[1] = (struct arm_smccc_1_2_regs){ .a0 = FFA_SUCCESS_32 }; /* RX_RELEASE */
	mock_script_len = 2;

	zassert_equal(ffa_partition_info_get_rxbuf(&st, &uuid, NULL, &count), 0, NULL);
	zassert_equal(count, 2, NULL);
	/* v1.0 count-only must NOT send the count-only flag ... */
	zassert_equal(mock_seen[0].a0, FFA_PARTITION_INFO_GET, NULL);
	zassert_equal(mock_seen[0].a5, 0, "v1.0 has no count-only flag");
	/* ... and MUST release the RX buffer the SPMC populated. */
	zassert_equal(mock_seen[1].a0, FFA_RX_RELEASE, "v1.0 count-only releases RX");
}


ZTEST(ffa_core, test_partition_info_error)
{
	struct ffa_drv_state st = { .version = FFA_VERSION_1_1 };
	struct ffa_uuid uuid = {0};
	size_t count = 0;

	k_mutex_init(&st.lock);
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0] = (struct arm_smccc_1_2_regs){ .a0 = FFA_ERROR, .a2 = (unsigned long)FFA_RET_INVALID_PARAMETERS };
	mock_script_len = 1;

	zassert_equal(ffa_partition_info_get_rxbuf(&st, &uuid, NULL, &count), -EINVAL, NULL);
}

ZTEST(ffa_core, test_partition_info_regs_decode)
{
	struct ffa_drv_state st = { .version = FFA_VERSION_1_2 };
	struct ffa_uuid uuid = {0};
	struct ffa_partition_info out[1];
	size_t count = 1;

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	/* a2: last_idx=0 (count=1), cur_idx=0, tag=0, size=... ; record in a3..a5 */
	struct arm_smccc_1_2_regs r = { .a0 = FFA_SUCCESS_64, .a2 = 0 /*last_idx 0 => count 1*/ };
	r.a3 = (uint64_t)0x8001 | ((uint64_t)1 << 16) | ((uint64_t)0x3 << 32); /* id, exec, props */
	r.a4 = 0x0706050403020100UL; /* uuid[0..7] */
	r.a5 = 0x0F0E0D0C0B0A0908UL; /* uuid[8..15] */
	mock_script[0] = r;
	mock_script_len = 1;

	zassert_equal(ffa_partition_info_get_regs(&st, &uuid, out, &count), 0, NULL);
	zassert_equal(mock_seen[0].a0, FFA_PARTITION_INFO_GET_REGS, NULL);
	zassert_equal(count, 1, NULL);
	zassert_equal(out[0].id, 0x8001, NULL);
	zassert_equal(out[0].exec_ctxt, 1, NULL);
	zassert_equal(out[0].properties, 0x3, NULL);
	zassert_equal(out[0].uuid.bytes[0], 0, NULL);
	zassert_equal(out[0].uuid.bytes[15], 15, NULL);
}

ZTEST(ffa_core, test_partition_info_regs_rejects_bad_window)
{
	struct ffa_drv_state st = { .version = FFA_VERSION_1_2 };
	struct ffa_uuid uuid = {0};
	struct ffa_partition_info out[8];
	size_t count = 8;

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	/* a2: last_idx=10 (count>1), cur_idx=5 => window (5-0+1)=6 > 5 records.
	 * Must be rejected before the record-decode loop reads past a17.
	 */
	struct arm_smccc_1_2_regs r = {
		.a0 = FFA_SUCCESS_64,
		.a2 = (uint64_t)10 | ((uint64_t)5 << 16),
	};
	mock_script[0] = r;
	mock_script_len = 1;

	zassert_equal(ffa_partition_info_get_regs(&st, &uuid, out, &count),
		      -EINVAL, NULL);
}

ZTEST(ffa_core, test_public_msg_unavailable)
{
	struct ffa_send_direct_data d = {0};
	struct ffa_uuid uuid = {0};
	struct ffa_send_direct_data2 d2 = {0};
	size_t count = 0;

	if (!ffa_is_available()) {
		zassert_equal(ffa_partition_info_get(&uuid, NULL, &count), -EAGAIN, NULL);
		zassert_equal(ffa_msg_send_direct_req(0x1, &d), -EAGAIN, NULL);
		zassert_equal(ffa_msg_send_direct_req2(0x1, &uuid, &d2), -EAGAIN, NULL);
	}
}

/*
 * ---- SP-3: memory sharing tests ----
 *
 * All tests use ffa_mem_share_impl / ffa_mem_reclaim_impl with a local
 * ffa_drv_state and a static TX buffer so no real SMCCC conduit is needed.
 *
 * Descriptor offsets for one receiver (v1.1, emad_size=16):
 *   [0..47]   ffa_mem_region header
 *   [48..63]  ffa_mem_region_attributes (16 bytes, no impdef_val)
 *   [64..79]  ffa_composite_mem_region header
 *   [80..]    ffa_mem_region_addr_range constituents
 */
#ifdef CONFIG_ARM_FFA_MEM_SHARE

/* TX/RX buffers are required to be page-aligned; align statically. */
static uint8_t mem_tx_buf[512] __aligned(8);

/* Helper: read a little-endian u16 from a byte buffer at a given offset. */
static uint16_t rd16(const uint8_t *buf, uint32_t off)
{
	uint16_t v;

	memcpy(&v, &buf[off], sizeof(v));
	return v;
}

/* Helper: read a little-endian u32 from a byte buffer at a given offset. */
static uint32_t rd32(const uint8_t *buf, uint32_t off)
{
	uint32_t v;

	memcpy(&v, &buf[off], sizeof(v));
	return v;
}

/* Helper: read a little-endian u64 from a byte buffer at a given offset. */
static uint64_t rd64(const uint8_t *buf, uint32_t off)
{
	uint64_t v;

	memcpy(&v, &buf[off], sizeof(v));
	return v;
}

/*
 * Basic share: one constituent, v1.1, descriptor fits in TX buffer.
 * Verify FFA_FN64_MEM_SHARE is called with correct a1/a2 and that
 * g_handle is assembled from a2 (lo) and a3 (hi) of FFA_SUCCESS_64.
 */
ZTEST(ffa_core, test_mem_share_single_region_v1_1)
{
	static const struct ffa_mem_region_addr_range range = {
		.address = 0x1000U, .pg_cnt = 1U
	};
	struct ffa_mem_ops_args args = {
		.dst_id = 0x8002U,
		.flags = 0U,
		.ranges = &range,
		.range_cnt = 1U,
	};
	struct ffa_drv_state st = {
		.version = FFA_VERSION_1_1,
		.vm_id   = 0x8001U,
		.tx_buf  = mem_tx_buf,
		.rxtx_pages = 1U,  /* 4 KiB */
	};
	/* hdr_sz = 48 + 16 + 16 = 80; total_desc = 80 + 1*16 = 96 */
	uint32_t hdr_sz = 80U;
	uint32_t total  = 96U;

	k_mutex_init(&st.lock);
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0].a0 = FFA_SUCCESS_64;
	mock_script[0].a2 = 0xDEADBEEFU;
	mock_script[0].a3 = 0xCAFEBABEU;
	mock_script_len = 1;

	zassert_equal(ffa_mem_share_impl(&st, &args), 0, NULL);

	/* Verify SMC arguments. */
	zassert_equal(mock_seen[0].a0, FFA_FN64_MEM_SHARE, NULL);
	zassert_equal(mock_seen[0].a1, total, "total_desc_sz");
	zassert_equal(mock_seen[0].a2, total, "first_frag_sz (all fit)");
	zassert_equal(mock_seen[0].a3, 0U, "a3=0 for shared RXTX buffer");
	zassert_equal(mock_seen[0].a4, 0U, "a4=0 for shared RXTX buffer");

	/* Verify g_handle is assembled from a2 (lo) and a3 (hi). */
	zassert_equal(args.g_handle,
		      ((uint64_t)0xCAFEBABEU << 32) | 0xDEADBEEFU, NULL);

	/* Spot-check descriptor: sender_id at byte 0, ep_count at byte 28. */
	zassert_equal(rd16(mem_tx_buf, 0U), 0x8001U, "sender_id");
	zassert_equal(rd32(mem_tx_buf, 28U), 1U, "ep_count");
	/* For v1.1, ep_mem_offset must be set to 48. */
	zassert_equal(rd32(mem_tx_buf, 32U), 48U, "ep_mem_offset");
	/* EMAD at [48]: receiver. */
	zassert_equal(rd16(mem_tx_buf, 48U), 0x8002U, "receiver");
	/* composite_off at [52] = 48 + 16 = 64. */
	zassert_equal(rd32(mem_tx_buf, 52U), 64U, "composite_off v1.1");
	/* Composite header at [64]: total_pg_cnt=1, range_cnt=1. */
	zassert_equal(rd32(mem_tx_buf, 64U), 1U, "total_pg_cnt");
	zassert_equal(rd32(mem_tx_buf, 68U), 1U, "addr_range_cnt");
	/* Constituent at [80]: address=0x1000, pg_cnt=1. */
	zassert_equal(rd64(mem_tx_buf, (uint32_t)hdr_sz),
		      0x1000U, "constituent address");
	zassert_equal(rd32(mem_tx_buf, (uint32_t)hdr_sz + 8U),
		      1U, "constituent pg_cnt");
}

/*
 * v1.2 share: composite_off is 80 (48+32), constituent at byte 96.
 */
ZTEST(ffa_core, test_mem_share_single_region_v1_2)
{
	static const struct ffa_mem_region_addr_range range = {
		.address = 0x5000U, .pg_cnt = 2U
	};
	struct ffa_mem_ops_args args = {
		.dst_id = 0x8003U,
		.flags = 0U,
		.ranges = &range,
		.range_cnt = 1U,
	};
	struct ffa_drv_state st = {
		.version    = FFA_VERSION_1_2,
		.vm_id      = 0x8001U,
		.tx_buf     = mem_tx_buf,
		.rxtx_pages = 1U,
	};
	/* hdr_sz = 48 + 32 + 16 = 96; total = 96 + 16 = 112 */

	k_mutex_init(&st.lock);
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0].a0 = FFA_SUCCESS_64;
	mock_script[0].a2 = 0x1234U;
	mock_script[0].a3 = 0x5678U;
	mock_script_len = 1;

	zassert_equal(ffa_mem_share_impl(&st, &args), 0, NULL);

	zassert_equal(mock_seen[0].a0, FFA_FN64_MEM_SHARE, NULL);
	zassert_equal(mock_seen[0].a1, 112U, "total_desc_sz v1.2");
	zassert_equal(mock_seen[0].a2, 112U, "first_frag_sz v1.2");

	/* composite_off at EMAD+4 = 48+4 = 52; value = 48+32 = 80 */
	zassert_equal(rd32(mem_tx_buf, 52U), 80U, "composite_off v1.2");
	/* Composite header at [80]. */
	zassert_equal(rd32(mem_tx_buf, 80U), 2U, "total_pg_cnt");
	zassert_equal(rd32(mem_tx_buf, 84U), 1U, "addr_range_cnt");
	/* Constituent at [96]. */
	zassert_equal(rd64(mem_tx_buf, 96U), 0x5000U, "constituent address v1.2");
	zassert_equal(rd32(mem_tx_buf, 104U), 2U, "constituent pg_cnt v1.2");

	zassert_equal(args.g_handle,
		      ((uint64_t)0x5678U << 32) | 0x1234U, NULL);
}

/*
 * Fragmentation test: simulate the SPMC returning FFA_MEM_FRAG_RX after the
 * initial FFA_FN64_MEM_SHARE (e.g. the SPMC needs additional FRAG_TX to
 * complete the handle assignment). The driver must issue FFA_MEM_FRAG_TX
 * carrying the partial handle from the FRAG_RX response, then loop until
 * FFA_SUCCESS_64.
 *
 * With a 4 KiB TX buffer and 3 small constituents the descriptor (128 bytes)
 * fits in one fragment. We force FRAG_RX via the mock to exercise the loop.
 */
ZTEST(ffa_core, test_mem_share_frag_tx)
{
	static const struct ffa_mem_region_addr_range ranges[3] = {
		{ .address = 0x1000U, .pg_cnt = 1U },
		{ .address = 0x2000U, .pg_cnt = 1U },
		{ .address = 0x3000U, .pg_cnt = 1U },
	};
	struct ffa_mem_ops_args args = {
		.dst_id    = 0x8002U,
		.ranges    = ranges,
		.range_cnt = 3U,
	};
	struct ffa_drv_state st = {
		.version    = FFA_VERSION_1_1,
		.vm_id      = 0x8001U,
		.tx_buf     = mem_tx_buf,
		/* TX = 96 bytes: hdr_sz(80) + 1 constituent(16) = 96.
		 * first_frag_capacity = (96-80)/16 = 1, so only 1 of the
		 * 3 constituents goes into the initial MEM_SHARE call; the
		 * remaining 2 are sent via FRAG_TX. */
		.tx_sz      = 96U,
	};

	k_mutex_init(&st.lock);
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	/* SPMC returns FRAG_RX after initial MEM_SHARE. */
	mock_script[0].a0 = FFA_MEM_FRAG_RX;
	mock_script[0].a1 = 0xAABBCCDDU;
	mock_script[0].a2 = 0xEEFF0011U;
	/* SPMC accepts FRAG_TX and returns SUCCESS_64. */
	mock_script[1].a0 = FFA_SUCCESS_64;
	mock_script[1].a2 = 0xFEEDU;
	mock_script[1].a3 = 0xBEEFU;
	mock_script_len = 2;

	zassert_equal(ffa_mem_share_impl(&st, &args), 0, NULL);

	/* First call: FFA_FN64_MEM_SHARE with total=128, frag=96 (1 constituent). */
	zassert_equal(mock_seen[0].a0, FFA_FN64_MEM_SHARE, NULL);
	zassert_equal(mock_seen[0].a1, 128U, "total_desc_sz = 80+3*16");
	zassert_equal(mock_seen[0].a2, 96U,  "first_frag_sz = 80+1*16");
	/* Second call: FFA_MEM_FRAG_TX with partial handle from FRAG_RX. */
	zassert_equal(mock_seen[1].a0, FFA_MEM_FRAG_TX, "second call is FRAG_TX");
	zassert_equal(mock_seen[1].a1, 0xAABBCCDDU, "frag handle lo");
	zassert_equal(mock_seen[1].a2, 0xEEFF0011U, "frag handle hi");
	/* FRAG_TX carries the remaining 2 constituents = 32 bytes. */
	zassert_equal((uint32_t)mock_seen[1].a3, 32U, "frag_tx sz = 2*16");
	/* g_handle from SUCCESS_64. */
	zassert_equal(args.g_handle,
		      ((uint64_t)0xBEEFU << 32) | 0xFEEDU, NULL);
}

/* Share returns -EINVAL when SPMC responds with FFA_ERROR. */
ZTEST(ffa_core, test_mem_share_error)
{
	static const struct ffa_mem_region_addr_range range = {
		.address = 0x1000U, .pg_cnt = 1U
	};
	struct ffa_mem_ops_args args = {
		.dst_id    = 0x8002U,
		.ranges    = &range,
		.range_cnt = 1U,
	};
	struct ffa_drv_state st = {
		.version    = FFA_VERSION_1_1,
		.vm_id      = 0x8001U,
		.tx_buf     = mem_tx_buf,
		.rxtx_pages = 1U,
	};

	k_mutex_init(&st.lock);
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0].a0 = FFA_ERROR;
	mock_script[0].a2 = (unsigned long)FFA_RET_NO_MEMORY;
	mock_script_len = 1;

	zassert_equal(ffa_mem_share_impl(&st, &args), -ENOMEM, NULL);
}

/* Share returns -EINVAL on NULL ranges. */
ZTEST(ffa_core, test_mem_share_null_ranges)
{
	struct ffa_mem_ops_args args = {
		.dst_id    = 0x8002U,
		.ranges    = NULL,
		.range_cnt = 1U,
	};
	struct ffa_drv_state st = {
		.version    = FFA_VERSION_1_1,
		.vm_id      = 0x8001U,
		.tx_buf     = mem_tx_buf,
		.rxtx_pages = 1U,
	};

	k_mutex_init(&st.lock);
	ffa_test_set_conduit(mock_conduit);

	zassert_equal(ffa_mem_share_impl(&st, &args), -EINVAL, NULL);
}

/* Reclaim issues FFA_MEM_RECLAIM with correct a1/a2/a3 and returns 0. */
ZTEST(ffa_core, test_mem_reclaim_success)
{
	struct ffa_drv_state st = {
		.version    = FFA_VERSION_1_1,
		.vm_id      = 0x8001U,
		.tx_buf     = mem_tx_buf,
		.rxtx_pages = 1U,
	};
	uint64_t handle = ((uint64_t)0x0000CAFEU << 32) | 0xBABE0000U;

	k_mutex_init(&st.lock);
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0].a0 = FFA_SUCCESS_32;
	mock_script_len = 1;

	zassert_equal(ffa_mem_reclaim_impl(&st, handle, FFA_MEM_RECLAIM_CLEAR), 0, NULL);

	zassert_equal(mock_seen[0].a0, FFA_MEM_RECLAIM, "function id");
	zassert_equal((uint32_t)mock_seen[0].a1,
		      (uint32_t)(handle & 0xFFFFFFFFU), "handle lo");
	zassert_equal((uint32_t)mock_seen[0].a2,
		      (uint32_t)(handle >> 32), "handle hi");
	zassert_equal((uint32_t)mock_seen[0].a3,
		      FFA_MEM_RECLAIM_CLEAR, "flags");
}

/* Reclaim propagates FFA_ERROR as -errno. */
ZTEST(ffa_core, test_mem_reclaim_error)
{
	struct ffa_drv_state st = {
		.version    = FFA_VERSION_1_1,
		.vm_id      = 0x8001U,
		.tx_buf     = mem_tx_buf,
		.rxtx_pages = 1U,
	};

	k_mutex_init(&st.lock);
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0].a0 = FFA_ERROR;
	mock_script[0].a2 = (unsigned long)FFA_RET_DENIED;
	mock_script_len = 1;

	zassert_equal(ffa_mem_reclaim_impl(&st, 0xDEADU, 0U), -EACCES, NULL);
}

/* Public wrappers return -EAGAIN when FF-A core is not available. */
ZTEST(ffa_core, test_mem_public_unavailable)
{
	struct ffa_mem_region_addr_range range = { .address = 0U, .pg_cnt = 1U };
	struct ffa_mem_ops_args args = {
		.dst_id = 0x8002U, .ranges = &range, .range_cnt = 1U
	};

	if (!ffa_is_available()) {
		zassert_equal(ffa_mem_share(&args), -EAGAIN, NULL);
		zassert_equal(ffa_mem_reclaim(0U, 0U), -EAGAIN, NULL);
	}
}

#endif /* CONFIG_ARM_FFA_MEM_SHARE */

/* ================================================================
 * SP-4: Notification tests
 * ================================================================ */
#ifdef CONFIG_ARM_FFA_NOTIF

/* Helper: set ffa_state so public notification functions pass guard checks. */
static void notif_setup_state(uint16_t vm_id)
{
	ffa_state.available     = true;
	ffa_state.notif_enabled = true;
	ffa_state.vm_id         = vm_id;
	ffa_test_set_conduit(mock_conduit);
}

static bool g_cb_called;
static int  g_cb_seen_id;

static void notif_test_cb(int notify_id, void *cb_data)
{
	g_cb_called  = true;
	g_cb_seen_id = notify_id;
}

static void notif_dummy_cb(int notify_id, void *cb_data)
{
	(void)notify_id;
	(void)cb_data;
}

ZTEST(ffa_core, test_notif_bitmap_create_ok)
{
	struct ffa_drv_state st = {
		.vm_id   = 0x0001U,
		.conduit = ARM_SMCCC_CONDUIT_SMC,
	};

	k_mutex_init(&st.lock);
	ffa_test_set_conduit(mock_conduit);
	set_mock_res_a0(FFA_SUCCESS_32);

	zassert_equal(ffa_notification_bitmap_create_impl(&st), 0, NULL);
	zassert_equal(mock_last_args.a0, FFA_NOTIFICATION_BITMAP_CREATE, NULL);
	zassert_equal(mock_last_args.a1, 0x0001U, "vm_id in a1");
}

ZTEST(ffa_core, test_notif_bitmap_create_error)
{
	struct ffa_drv_state st = {
		.vm_id   = 0x0001U,
		.conduit = ARM_SMCCC_CONDUIT_SMC,
	};

	k_mutex_init(&st.lock);
	ffa_test_set_conduit(mock_conduit);
	memset(&mock_next_res, 0, sizeof(mock_next_res));
	mock_next_res.a0 = FFA_ERROR;
	mock_next_res.a2 = (unsigned long)FFA_RET_NOT_SUPPORTED;

	zassert_equal(ffa_notification_bitmap_create_impl(&st), -ENOTSUP, NULL);
}

ZTEST(ffa_core, test_notif_bind_args)
{
	/* Sender 0x8002, our vm_id 0x0001, bitmap = 0x5 */
	notif_setup_state(0x0001U);
	set_mock_res_a0(FFA_SUCCESS_32);

	zassert_equal(ffa_notification_bind(0x8002U, 0x5ULL, 0), 0, NULL);
	zassert_equal(mock_last_args.a0, FFA_NOTIFICATION_BIND, NULL);
	/* a1 = (sender << 16) | recv_id */
	zassert_equal(mock_last_args.a1, (0x8002U << 16) | 0x0001U, NULL);
	zassert_equal(mock_last_args.a2, 0U, "flags");
	zassert_equal(mock_last_args.a3, 0x5U, "bitmap_lo");
	zassert_equal(mock_last_args.a4, 0U, "bitmap_hi");
}

ZTEST(ffa_core, test_notif_bind_error)
{
	notif_setup_state(0x0001U);
	memset(&mock_next_res, 0, sizeof(mock_next_res));
	mock_next_res.a0 = FFA_ERROR;
	mock_next_res.a2 = (unsigned long)FFA_RET_DENIED;

	zassert_equal(ffa_notification_bind(0x8002U, 0x1ULL, 0), -EACCES, NULL);
}

ZTEST(ffa_core, test_notif_get_bitmap)
{
	notif_setup_state(0x0001U);
	/* SP bitmap = 5 (a2=5, a3=0), VM bitmap = 2 (a4=2, a5=0) */
	memset(&mock_next_res, 0, sizeof(mock_next_res));
	mock_next_res.a0 = FFA_SUCCESS_32;
	mock_next_res.a2 = 5UL;
	mock_next_res.a4 = 2UL;

	uint64_t bm = 0;

	zassert_equal(ffa_notification_get(0, FFA_NOTIF_GET_ALL, &bm), 0, NULL);
	zassert_equal(bm, 7ULL, "SP(5) | VM(2) = 7");
	zassert_equal(mock_last_args.a0, FFA_NOTIFICATION_GET, NULL);
	zassert_equal(mock_last_args.a1, 0x0001U, "vcpu=0, vm_id=1");
	zassert_equal(mock_last_args.a2, FFA_NOTIF_GET_ALL, "flags");
}

ZTEST(ffa_core, test_notif_request_and_dispatch)
{
	g_cb_called  = false;
	g_cb_seen_id = -1;

	ffa_notification_unregister(3);
	zassert_equal(ffa_notification_request(3, notif_test_cb, NULL), 0, NULL);

	/* Mock GET: bit 3 set in SP bitmap */
	notif_setup_state(0x0001U);
	memset(&mock_next_res, 0, sizeof(mock_next_res));
	mock_next_res.a0 = FFA_SUCCESS_32;
	mock_next_res.a2 = BIT(3);

	zassert_equal(ffa_notification_dispatch(), 0, NULL);
	zassert_true(g_cb_called, "callback must fire for bit 3");
	zassert_equal(g_cb_seen_id, 3, NULL);

	ffa_notification_unregister(3);
}

ZTEST(ffa_core, test_notif_request_busy)
{
	ffa_notification_unregister(7);

	zassert_equal(ffa_notification_request(7, notif_dummy_cb, NULL), 0, NULL);
	zassert_equal(ffa_notification_request(7, notif_dummy_cb, NULL), -EBUSY, NULL);
	ffa_notification_unregister(7);
}

ZTEST(ffa_core, test_notif_unavailable)
{
	ffa_state.available     = true;
	ffa_state.notif_enabled = false;

	zassert_equal(ffa_notification_bind(0x8002U, 1ULL, 0), -ENOTSUP, NULL);
	zassert_equal(ffa_notification_set(0x8002U, 1ULL, 0), -ENOTSUP, NULL);

	uint64_t bm = 0;

	zassert_equal(ffa_notification_get(0, FFA_NOTIF_GET_ALL, &bm), -ENOTSUP, NULL);
}

#endif /* CONFIG_ARM_FFA_NOTIF */

ZTEST_SUITE(ffa_core, NULL, NULL, NULL, NULL, NULL);
