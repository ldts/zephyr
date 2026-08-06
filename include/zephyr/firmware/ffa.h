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

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_FIRMWARE_FFA_H_ */
