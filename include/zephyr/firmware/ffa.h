/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_FIRMWARE_FFA_H_
#define ZEPHYR_INCLUDE_FIRMWARE_FFA_H_

#include <stdint.h>
#include <stdbool.h>

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

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_FIRMWARE_FFA_H_ */
