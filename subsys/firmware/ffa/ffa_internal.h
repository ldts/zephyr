/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal FF-A ABI constants and driver state. Values ported from
 * linux/include/linux/arm_ffa.h and cross-checked against
 * optee_os/core/arch/arm/include/ffa.h.
 */

#ifndef ZEPHYR_SUBSYS_FIRMWARE_FFA_FFA_INTERNAL_H_
#define ZEPHYR_SUBSYS_FIRMWARE_FFA_FFA_INTERNAL_H_

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/firmware/ffa.h>

/* FF-A function IDs (SMC, fast call, owner=Standard). */
#define FFA_ERROR                 0x84000060U
#define FFA_SUCCESS_32            0x84000061U
#define FFA_SUCCESS_64            0xC4000061U
#define FFA_INTERRUPT             0x84000062U
#define FFA_VERSION               0x84000063U
#define FFA_FEATURES              0x84000064U
#define FFA_RX_RELEASE            0x84000065U
#define FFA_RXTX_MAP_64           0xC4000066U
#define FFA_RXTX_UNMAP            0x84000067U
#define FFA_PARTITION_INFO_GET    0x84000068U
#define FFA_ID_GET                0x84000069U
#define FFA_YIELD                 0x8400006CU
#define FFA_RUN                   0x8400006DU
#define FFA_MSG_SEND_DIRECT_REQ_32  0x8400006FU
#define FFA_MSG_SEND_DIRECT_REQ_64  0xC400006FU
#define FFA_MSG_SEND_DIRECT_RESP_32 0x84000070U
#define FFA_MSG_SEND_DIRECT_RESP_64 0xC4000070U
#define FFA_MSG_SEND_DIRECT_REQ2    0xC400008DU
#define FFA_MSG_SEND_DIRECT_RESP2   0xC400008EU
#define FFA_PARTITION_INFO_GET_REGS 0xC400008BU

/* FF-A return/error codes (in w2 of FFA_ERROR, or negative w0). */
#define FFA_RET_SUCCESS            0
#define FFA_RET_NOT_SUPPORTED      (-1)
#define FFA_RET_INVALID_PARAMETERS (-2)
#define FFA_RET_NO_MEMORY          (-3)
#define FFA_RET_BUSY               (-4)
#define FFA_RET_INTERRUPTED        (-5)
#define FFA_RET_DENIED             (-6)
#define FFA_RET_RETRY              (-7)
#define FFA_RET_ABORTED            (-8)
#define FFA_RET_NO_DATA            (-9)

/* Version encoding: bits[30:16]=major, bits[15:0]=minor; bit31 must be 0. */
#define FFA_MAJOR_SHIFT            16
#define FFA_MAJOR_MASK             0x7FFFU
#define FFA_MINOR_MASK             0xFFFFU
#define FFA_PACK_VERSION(major, minor) \
	((((uint32_t)(major) & FFA_MAJOR_MASK) << FFA_MAJOR_SHIFT) | \
	 ((uint32_t)(minor) & FFA_MINOR_MASK))
#define FFA_VERSION_MAJOR(v)       (((v) >> FFA_MAJOR_SHIFT) & FFA_MAJOR_MASK)
#define FFA_VERSION_MINOR(v)       ((v) & FFA_MINOR_MASK)
#define FFA_VERSION_1_0            FFA_PACK_VERSION(1, 0)
#define FFA_VERSION_1_1            FFA_PACK_VERSION(1, 1)
#define FFA_VERSION_1_2            FFA_PACK_VERSION(1, 2)

/* FFA_VERSION returns a negative 32-bit value (bit31 set) on NOT_SUPPORTED. */
#define FFA_VERSION_NOT_SUPPORTED  0xFFFFFFFFU

/* Endpoint pair packing for direct messages: sender[31:16], receiver[15:0]. */
#define FFA_PACK_TARGET_INFO(src, dst) \
	((((uint32_t)(src) & 0xFFFFU) << 16) | ((uint32_t)(dst) & 0xFFFFU))

/* PARTITION_INFO_GET flags. */
#define FFA_PARTITION_INFO_GET_COUNT_ONLY  0x1U

/* FF-A 1.0 partition record is the first 8 bytes (no UUID). */
#define FFA_1_0_PARTITION_INFO_SZ          8U

/* PARTITION_INFO_GET_REGS: packed fields in a2. */
#define FFA_PIG_REGS_LAST_IDX(x)   ((uint16_t)((x) & 0xFFFFU))
#define FFA_PIG_REGS_CUR_IDX(x)    ((uint16_t)(((x) >> 16) & 0xFFFFU))
#define FFA_PIG_REGS_TAG(x)        ((uint16_t)(((x) >> 32) & 0xFFFFU))
#define FFA_PIG_REGS_SIZE(x)       ((uint16_t)(((x) >> 48) & 0xFFFFU))
/* PARTITION_INFO_GET_REGS: packed partition record word0. */
#define FFA_PIG_REC_ID(x)          ((uint16_t)((x) & 0xFFFFU))
#define FFA_PIG_REC_EXEC_CTXT(x)   ((uint16_t)(((x) >> 16) & 0xFFFFU))
#define FFA_PIG_REC_PROPS(x)       ((uint32_t)(((x) >> 32) & 0xFFFFFFFFU))

/* FF-A 4 KiB page (spec constant, not the kernel translation granule). */
#define FFA_PAGE_SIZE              0x1000U

/* FFA_FEATURES RXTX minimum-buffer-size encodings (w2 bits[1:0]). */
#define FFA_FEAT_RXTX_MIN_SZ_MASK  0x3U
#define FFA_FEAT_RXTX_MIN_SZ_4K    0
#define FFA_FEAT_RXTX_MIN_SZ_64K   1
#define FFA_FEAT_RXTX_MIN_SZ_16K   2

/* Internal singleton driver state. */
struct ffa_drv_state {
	bool available;             /* FF-A init succeeded */
	uint32_t version;           /* negotiated framework version */
	uint16_t vm_id;             /* our endpoint ID (FFA_ID_GET) */
	enum arm_smccc_conduit conduit;
	void *tx_buf;
	void *rx_buf;
	uint32_t rxtx_pages;        /* pages per buffer */
	struct k_mutex lock;        /* serializes RX buffer use */
};

/* Conduit seam: production uses arm_smccc_1_2_smc/hvc; tests install a mock. */
typedef void (*ffa_conduit_fn_t)(const struct arm_smccc_1_2_regs *args,
				 struct arm_smccc_1_2_regs *res);

int ffa_to_errno(int ffa_ret);

void ffa_invoke(struct ffa_drv_state *st, struct arm_smccc_1_2_regs *args,
		struct arm_smccc_1_2_regs *res);
int ffa_negotiate_version(struct ffa_drv_state *st);
int ffa_get_id(struct ffa_drv_state *st);
int ffa_query_feature(struct ffa_drv_state *st, uint32_t ffa_func_id,
		      uint32_t *out);

int ffa_rxtx_map(struct ffa_drv_state *st);
enum arm_smccc_conduit ffa_detect_conduit(void);

#ifdef CONFIG_ZTEST
void ffa_test_set_conduit(ffa_conduit_fn_t fn);
#endif

int ffa_send_direct_req(struct ffa_drv_state *st, uint16_t dst, bool mode_32bit,
			struct ffa_send_direct_data *data);

int ffa_send_direct_req2(struct ffa_drv_state *st, uint16_t dst,
			 const struct ffa_uuid *uuid,
			 struct ffa_send_direct_data2 *data);

int ffa_rx_release(struct ffa_drv_state *st);
int ffa_partition_info_get_rxbuf(struct ffa_drv_state *st,
				 const struct ffa_uuid *uuid,
				 struct ffa_partition_info *out, size_t *count);

int ffa_partition_info_get_regs(struct ffa_drv_state *st,
				const struct ffa_uuid *uuid,
				struct ffa_partition_info *out, size_t *count);

/* Singleton driver state — non-static so ffa_msg.c can reference it directly.
 * Buffers remain static in ffa_core.c. */
extern struct ffa_drv_state ffa_state;

#endif /* ZEPHYR_SUBSYS_FIRMWARE_FFA_FFA_INTERNAL_H_ */
