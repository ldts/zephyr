/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file ffa_notif.c
 * @brief FF-A endpoint notification support (SP-4).
 *
 * Implements FFA_NOTIFICATION_BITMAP_CREATE/DESTROY, BIND/UNBIND, SET, GET,
 * per-notification-ID callback registration, and a poll-dispatch helper.
 * Uses the FFA core's conduit seam (ffa_invoke) so tests can install a mock.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/firmware/ffa.h>

#include "ffa_internal.h"

LOG_MODULE_DECLARE(arm_ffa, CONFIG_ARM_FFA_LOG_LEVEL);

#define FFA_MAX_NOTIF_IDS  64U   /* FF-A bitmap is 64 bits wide */

struct ffa_notif_entry {
	ffa_notifier_cb  cb;
	void            *data;
};

static struct ffa_notif_entry notif_table[FFA_MAX_NOTIF_IDS];

K_MUTEX_DEFINE(notif_table_lock);

/* ------------------------------------------------------------------ */
/* Internal helpers called from ffa_init()                             */
/* ------------------------------------------------------------------ */

int ffa_notification_bitmap_create_impl(struct ffa_drv_state *st)
{
	struct arm_smccc_1_2_regs args = {
		.a0 = FFA_NOTIFICATION_BITMAP_CREATE,
		.a1 = st->vm_id,
		.a2 = CONFIG_MP_MAX_NUM_CPUS,
	};
	struct arm_smccc_1_2_regs res = {0};

	ffa_invoke(st, &args, &res);
	if (res.a0 == FFA_ERROR) {
		return ffa_to_errno((int)res.a2);
	}

	return 0;
}

void ffa_notification_bitmap_destroy_impl(struct ffa_drv_state *st)
{
	struct arm_smccc_1_2_regs args = {
		.a0 = FFA_NOTIFICATION_BITMAP_DESTROY,
		.a1 = st->vm_id,
	};
	struct arm_smccc_1_2_regs res = {0};

	ffa_invoke(st, &args, &res);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int ffa_notification_bind(uint16_t sender, uint64_t bitmap, uint32_t flags)
{
	if (!ffa_is_available()) {
		return -ENODEV;
	}
	if (!ffa_state.notif_enabled) {
		return -ENOTSUP;
	}

	struct arm_smccc_1_2_regs args = {
		.a0 = FFA_NOTIFICATION_BIND,
		.a1 = FFA_NOTIF_PACK_RECV_VCPU(sender, ffa_state.vm_id),
		.a2 = flags,
		.a3 = (uint32_t)bitmap,
		.a4 = (uint32_t)(bitmap >> 32),
	};
	struct arm_smccc_1_2_regs res = {0};

	ffa_invoke(&ffa_state, &args, &res);
	if (res.a0 == FFA_ERROR) {
		return ffa_to_errno((int)res.a2);
	}

	return 0;
}

int ffa_notification_unbind(uint16_t sender, uint64_t bitmap)
{
	if (!ffa_is_available()) {
		return -ENODEV;
	}
	if (!ffa_state.notif_enabled) {
		return -ENOTSUP;
	}

	struct arm_smccc_1_2_regs args = {
		.a0 = FFA_NOTIFICATION_UNBIND,
		.a1 = FFA_NOTIF_PACK_RECV_VCPU(sender, ffa_state.vm_id),
		.a2 = 0,
		.a3 = (uint32_t)bitmap,
		.a4 = (uint32_t)(bitmap >> 32),
	};
	struct arm_smccc_1_2_regs res = {0};

	ffa_invoke(&ffa_state, &args, &res);
	if (res.a0 == FFA_ERROR) {
		return ffa_to_errno((int)res.a2);
	}

	return 0;
}

int ffa_notification_set(uint16_t receiver, uint64_t bitmap, uint32_t flags)
{
	if (!ffa_is_available()) {
		return -ENODEV;
	}
	if (!ffa_state.notif_enabled) {
		return -ENOTSUP;
	}

	struct arm_smccc_1_2_regs args = {
		.a0 = FFA_NOTIFICATION_SET,
		.a1 = FFA_NOTIF_PACK_RECV_VCPU(ffa_state.vm_id, receiver),
		.a2 = flags,
		.a3 = (uint32_t)bitmap,
		.a4 = (uint32_t)(bitmap >> 32),
	};
	struct arm_smccc_1_2_regs res = {0};

	ffa_invoke(&ffa_state, &args, &res);
	if (res.a0 == FFA_ERROR) {
		return ffa_to_errno((int)res.a2);
	}

	return 0;
}

int ffa_notification_get(uint16_t vcpu, uint32_t flags, uint64_t *bitmap)
{
	if (!ffa_is_available()) {
		return -ENODEV;
	}
	if (!ffa_state.notif_enabled) {
		return -ENOTSUP;
	}
	if (!bitmap) {
		return -EINVAL;
	}

	struct arm_smccc_1_2_regs args = {
		.a0 = FFA_NOTIFICATION_GET,
		.a1 = FFA_NOTIF_PACK_RECV_VCPU(vcpu, ffa_state.vm_id),
		.a2 = flags,
	};
	struct arm_smccc_1_2_regs res = {0};

	ffa_invoke(&ffa_state, &args, &res);
	if (res.a0 == FFA_ERROR) {
		return ffa_to_errno((int)res.a2);
	}
	if (res.a0 != FFA_SUCCESS_32 && res.a0 != FFA_SUCCESS_64) {
		return -EIO;
	}

	/*
	 * Return SP bitmap (a2/a3) OR'd with VM bitmap (a4/a5).
	 * Caller selects relevant bits via flags; we merge into one 64-bit
	 * value for simplicity.  SPM bitmap (a6) is narrow (32-bit, no hi).
	 */
	uint64_t sp_map = (uint64_t)res.a2 | ((uint64_t)res.a3 << 32);
	uint64_t vm_map = (uint64_t)res.a4 | ((uint64_t)res.a5 << 32);

	*bitmap = sp_map | vm_map;
	return 0;
}

int ffa_notification_request(int notify_id, ffa_notifier_cb cb, void *data)
{
	if (notify_id < 0 || (uint32_t)notify_id >= FFA_MAX_NOTIF_IDS) {
		return -EINVAL;
	}
	if (!cb) {
		return -EINVAL;
	}

	k_mutex_lock(&notif_table_lock, K_FOREVER);
	if (notif_table[notify_id].cb) {
		k_mutex_unlock(&notif_table_lock);
		return -EBUSY;
	}
	notif_table[notify_id].cb   = cb;
	notif_table[notify_id].data = data;
	k_mutex_unlock(&notif_table_lock);

	return 0;
}

int ffa_notification_unregister(int notify_id)
{
	if (notify_id < 0 || (uint32_t)notify_id >= FFA_MAX_NOTIF_IDS) {
		return -EINVAL;
	}

	k_mutex_lock(&notif_table_lock, K_FOREVER);
	notif_table[notify_id].cb   = NULL;
	notif_table[notify_id].data = NULL;
	k_mutex_unlock(&notif_table_lock);

	return 0;
}

int ffa_notification_dispatch(void)
{
	uint64_t bitmap = 0;
	int ret;

	ret = ffa_notification_get(0, FFA_NOTIF_GET_ALL, &bitmap);
	if (ret) {
		return ret;
	}

	k_mutex_lock(&notif_table_lock, K_FOREVER);
	uint64_t pending = bitmap;

	while (pending) {
		int bit = __builtin_ctzll(pending);

		pending &= ~BIT64(bit);
		if (notif_table[bit].cb) {
			notif_table[bit].cb(bit, notif_table[bit].data);
		}
	}
	k_mutex_unlock(&notif_table_lock);

	return 0;
}
