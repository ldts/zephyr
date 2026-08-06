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

/* Memory-sharing function IDs (64-bit, native/FN64 variants). */
#define FFA_MEM_SHARE_32           0x84000073U
#define FFA_FN64_MEM_SHARE         0xC4000073U
#define FFA_MEM_RETRIEVE_REQ_32    0x84000074U
#define FFA_FN64_MEM_RETRIEVE_REQ  0xC4000074U
#define FFA_MEM_RETRIEVE_RESP      0x84000075U
#define FFA_MEM_RELINQUISH         0x84000076U
#define FFA_MEM_RECLAIM            0x84000077U
#define FFA_MEM_FRAG_RX            0x8400007AU
#define FFA_MEM_FRAG_TX            0x8400007BU

/* FFA_MEM_RECLAIM flags. */
#define FFA_MEM_RECLAIM_CLEAR      (1U << 0)

/* Memory access permissions in ffa_mem_region_attributes.attrs (Linux arm_ffa.h). */
#define FFA_MEM_RW                 (1U << 1)  /* Read/write */
#define FFA_MEM_RO                 (1U << 0)  /* Read-only */
#define FFA_MEM_NO_EXEC            (1U << 2)  /* No execute */
#define FFA_MEM_EXEC               (1U << 3)  /* Execute */

/*
 * Memory region attributes packed into ffa_mem_region.attributes (lower byte).
 * From Linux arm_ffa.h: type[5]=normal/device, cache[3:2], share[1:0].
 */
#define FFA_MEM_NORMAL             (1U << 5)  /* Normal memory */
#define FFA_MEM_DEVICE             (1U << 4)  /* Device memory */
#define FFA_MEM_WRITE_BACK         (3U << 2)  /* Write-back cacheable */
#define FFA_MEM_NON_CACHEABLE      (1U << 2)  /* Non-cacheable */
#define FFA_MEM_NON_SHAREABLE      0U
#define FFA_MEM_OUTER_SHAREABLE    2U
#define FFA_MEM_INNER_SHAREABLE    3U

/* Combined: Normal, Write-Back, Inner-shareable (pre-built for RW NS→SP share) */
#define FFA_MEM_ATTRS_NWB_IS       (FFA_MEM_NORMAL | FFA_MEM_WRITE_BACK | FFA_MEM_INNER_SHAREABLE)

/* FFA_MEM_RECLAIM_CLEAR flag (also defined in public header). */
#define FFA_MEM_CLEAR_AFTER_RECLAIM  (1U << 0)

/*
 * Composite memory-region descriptor layout (FF-A spec §10.9, from Linux
 * include/linux/arm_ffa.h), only needed when CONFIG_ARM_FFA_MEM_SHARE is set.
 *
 * struct ffa_mem_region (48 bytes):
 *   sender_id, attributes (type/cache/share), flags, handle, tag,
 *   ep_mem_size, ep_count, ep_mem_offset, reserved[3]
 *
 * struct ffa_mem_region_attributes (16 bytes pre-v1.2, 32 bytes v1.2):
 *   receiver, attrs (perms), flag, composite_off, [impdef_val[16],] reserved
 *
 * struct ffa_composite_mem_region (16-byte header + constituents[]):
 *   total_pg_cnt, addr_range_cnt, reserved, constituents[]
 *
 * struct ffa_mem_region_addr_range (16 bytes, public in ffa.h):
 *   address, pg_cnt, reserved
 *
 * composite_off in each ffa_mem_region_attributes is a byte offset from the
 * start of the enclosing ffa_mem_region.  For one receiver:
 *   pre-v1.2: composite_off = 48 + 16 = 64
 *   v1.2:     composite_off = 48 + 32 = 80
 */
#ifdef CONFIG_ARM_FFA_MEM_SHARE

/** FF-A memory region descriptor header (48 bytes). */
struct ffa_mem_region {
	uint16_t sender_id;
	uint16_t attributes;      /**< memory type/cacheability/shareability */
	uint32_t flags;           /**< transfer operation flags */
	uint64_t handle;          /**< 0 on share, filled by SPMC */
	uint64_t tag;             /**< implementation-defined tag */
	uint32_t ep_mem_size;     /**< EMAD size, 0 pre-v1.1 */
	uint32_t ep_count;        /**< number of ffa_mem_region_attributes entries */
	uint32_t ep_mem_offset;   /**< byte offset to EMAD array, 0 pre-v1.1 */
	uint32_t reserved[3];
} __packed;

BUILD_ASSERT(sizeof(struct ffa_mem_region) == 48U,
	     "ffa_mem_region must be 48 bytes");

/** Per-borrower memory access and permissions descriptor.
 *  16 bytes for FF-A < 1.2; 32 bytes (with impdef_val) for FF-A >= 1.2.
 */
struct ffa_mem_region_attributes {
	uint16_t receiver;      /**< borrower endpoint ID */
	uint8_t  attrs;         /**< access permissions (FFA_MEM_RW etc.) */
	uint8_t  flag;          /**< retrieve flags (0 for share) */
	uint32_t composite_off; /**< byte offset to ffa_composite_mem_region */
	uint8_t  impdef_val[16];/**< implementation defined (v1.2 only) */
	uint64_t reserved;
} __packed;

BUILD_ASSERT(sizeof(struct ffa_mem_region_attributes) == 32U,
	     "ffa_mem_region_attributes must be 32 bytes");

/* Effective EMAD size: 16 bytes pre-v1.2, 32 bytes for v1.2+. */
#define FFA_EMAD_SIZE_V1_0  16U
#define FFA_EMAD_SIZE_V1_2  32U

static inline uint32_t ffa_emad_size(uint32_t version)
{
	return (version >= FFA_VERSION_1_2) ? FFA_EMAD_SIZE_V1_2
					    : FFA_EMAD_SIZE_V1_0;
}

/** Composite memory region descriptor (variable-length, placed in TX buf). */
struct ffa_composite_mem_region {
	uint32_t total_pg_cnt;   /**< total pages across all constituents */
	uint32_t addr_range_cnt;
	uint64_t reserved;
	struct ffa_mem_region_addr_range constituents[];
} __packed;

BUILD_ASSERT(sizeof(struct ffa_composite_mem_region) == 16U,
	     "ffa_composite_mem_region header must be 16 bytes");

#endif /* CONFIG_ARM_FFA_MEM_SHARE */

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

#ifdef CONFIG_ARM_FFA_MEM_SHARE
int ffa_mem_share_impl(struct ffa_drv_state *st,
		       struct ffa_mem_ops_args *args);
int ffa_mem_reclaim_impl(struct ffa_drv_state *st,
			 uint64_t g_handle, uint32_t flags);
#endif

#endif /* ZEPHYR_SUBSYS_FIRMWARE_FFA_FFA_INTERNAL_H_ */
