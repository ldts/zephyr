/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_FIRMWARE_FFA_H_
#define ZEPHYR_INCLUDE_FIRMWARE_FFA_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Arm FF-A (Firmware Framework for A-profile) Core API
 * @defgroup ffa_core_api Arm FF-A Core API
 * @ingroup os_services
 * @{
 */

/** A 16-byte FF-A partition UUID, little-endian byte order per the FF-A spec. */
struct ffa_uuid {
	uint8_t bytes[16];
};

/** @return true once the FF-A core has initialised and negotiated a version. */
bool ffa_is_available(void);

/**
 * @brief Get the negotiated FF-A framework version.
 * @param out receives the packed version (major<<16 | minor).
 * @retval 0 on success, -EAGAIN if FF-A is not available.
 */
int ffa_version(uint32_t *out);

/**
 * @brief Get this endpoint's FF-A ID.
 * @param vm_id receives the endpoint ID.
 * @retval 0 on success, -EAGAIN if FF-A is not available.
 */
int ffa_id_get(uint16_t *vm_id);

/** FF-A partition properties (subset relevant to a NS endpoint). */
#define FFA_PARTITION_DIRECT_RECV        (1U << 0)
#define FFA_PARTITION_DIRECT_SEND        (1U << 1)
#define FFA_PARTITION_INDIRECT_MSG       (1U << 2)
#define FFA_PARTITION_NOTIFICATION_RECV  (1U << 3)
#define FFA_PARTITION_AARCH64_EXEC       (1U << 8)
#define FFA_PARTITION_DIRECT_REQ2_RECV   (1U << 9)
#define FFA_PARTITION_DIRECT_REQ2_SEND   (1U << 10)

/** One partition descriptor returned by ffa_partition_info_get(). */
struct ffa_partition_info {
	uint16_t id;            /**< partition/endpoint ID */
	uint16_t exec_ctxt;     /**< number of execution contexts */
	uint32_t properties;    /**< FFA_PARTITION_* property bits */
	struct ffa_uuid uuid;   /**< partition UUID (zero if negotiated version is 1.0) */
};

/** Register payload for FFA_MSG_SEND_DIRECT_REQ/RESP (x3-x7). */
struct ffa_send_direct_data {
	unsigned long data0;
	unsigned long data1;
	unsigned long data2;
	unsigned long data3;
	unsigned long data4;
};

/** Register payload for FFA_MSG_SEND_DIRECT_REQ2/RESP2 (x4-x17). */
struct ffa_send_direct_data2 {
	unsigned long data[14];
};

/**
 * @brief Discover partitions matching a UUID.
 * @param uuid  UUID to match; pass the nil UUID (all-zero) to enumerate all.
 * @param out   caller buffer of @p max entries (may be NULL to count only).
 * @param count in: capacity of @p out; out: number of partitions reported.
 * @retval 0 on success, negative errno otherwise, -EAGAIN if FF-A unavailable.
 */
int ffa_partition_info_get(const struct ffa_uuid *uuid,
			   struct ffa_partition_info *out, size_t *count);

/**
 * @brief Send a direct request (FFA_MSG_SEND_DIRECT_REQ) and await the response.
 * @param dst   destination endpoint ID.
 * @param data  in/out register payload (x3-x7).
 * @retval 0 on success (response unpacked into @p data), negative errno otherwise.
 */
int ffa_msg_send_direct_req(uint16_t dst, struct ffa_send_direct_data *data);

/**
 * @brief Send a direct request2 (FFA_MSG_SEND_DIRECT_REQ2, FF-A >= 1.2).
 * @param dst   destination endpoint ID.
 * @param uuid  target service UUID.
 * @param data  in/out register payload (x4-x17).
 * @retval 0 on success, -ENOTSUP if negotiated version < 1.2, negative errno otherwise.
 */
int ffa_msg_send_direct_req2(uint16_t dst, const struct ffa_uuid *uuid,
			     struct ffa_send_direct_data2 *data);

/**
 * One physical address range passed to ffa_mem_share().
 * Always defined so that callers can reference it unconditionally.
 */
struct ffa_mem_region_addr_range {
	uint64_t address;  /**< IPA/PA base address */
	uint32_t pg_cnt;   /**< number of 4 KiB pages */
	uint32_t reserved;
};

#ifdef CONFIG_ARM_FFA_MEM_SHARE

/**
 * Arguments for ffa_mem_share().  On success g_handle is filled by the SPMC.
 */
struct ffa_mem_ops_args {
	uint64_t g_handle;  /**< OUT: global memory handle */
	uint16_t dst_id;    /**< borrower endpoint ID */
	uint32_t flags;     /**< FFA_MEM_RECLAIM_CLEAR or 0 */
	/** caller-supplied physical ranges to share */
	const struct ffa_mem_region_addr_range *ranges;
	uint32_t range_cnt;
};

/** Flag: zero memory on reclaim. */
#define FFA_MEM_RECLAIM_CLEAR  (1U << 0)

/**
 * @brief Share a memory region with a secure partition via FFA_MEM_SHARE.
 *
 * Builds the FF-A composite memory-region descriptor in the TX buffer and
 * calls FFA_FN64_MEM_SHARE, driving the FFA_MEM_FRAG_TX loop if the
 * descriptor exceeds the TX buffer size.
 *
 * @param args  caller-supplied parameters; @p args->g_handle is filled on
 *              success.
 * @retval 0 on success, negative errno otherwise, -EAGAIN if FF-A unavailable.
 */
int ffa_mem_share(struct ffa_mem_ops_args *args);

/**
 * @brief Reclaim a previously shared memory region.
 *
 * @param g_handle  global handle returned by ffa_mem_share().
 * @param flags     FFA_MEM_RECLAIM_CLEAR or 0.
 * @retval 0 on success, negative errno otherwise, -EAGAIN if FF-A unavailable.
 */
int ffa_mem_reclaim(uint64_t g_handle, uint32_t flags);

#endif /* CONFIG_ARM_FFA_MEM_SHARE */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_FIRMWARE_FFA_H_ */
