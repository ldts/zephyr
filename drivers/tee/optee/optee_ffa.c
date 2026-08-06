/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file optee_ffa.c
 * @brief OP-TEE Trusted OS driver using the Arm FF-A transport (SP-5).
 *
 * Discovers the OP-TEE Secure Partition via FFA_PARTITION_INFO_GET, negotiates
 * the OP-TEE protocol (GET_API_VERSION / GET_OS_VERSION /
 * EXCHANGE_CAPABILITIES), issues yielding calls with
 * OPTEE_FFA_YIELDING_CALL_WITH_ARG via FFA_MSG_SEND_DIRECT_REQ, drives the
 * RPC return loop, and manages shared-memory handles via ffa_mem_share /
 * ffa_mem_reclaim + OPTEE_FFA_UNREGISTER_SHM.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/tee.h>
#include <zephyr/firmware/ffa.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "optee_msg.h"
#include "optee_rpc_cmd.h"

LOG_MODULE_REGISTER(optee_ffa);

#define DT_DRV_COMPAT linaro_optee_ffa

#define TEE_IMPL_ID_OPTEE  1U
#define TEE_OPTEE_CAP_TZ   BIT(0)

/* OP-TEE OS partition UUID: 486178e0-e7f8-11e3-bc5e-0002a5d5c51b */
static const struct ffa_uuid optee_ffa_os_uuid = {
	.bytes = { 0x48, 0x61, 0x78, 0xe0, 0xe7, 0xf8, 0x11, 0xe3,
		   0xbc, 0x5e, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b },
};

/* OP-TEE FFA protocol definitions (from optee_os optee_ffa.h). */
#define OPTEE_FFA_BLOCKING_CALL(id)   UINT32_C(id)
#define OPTEE_FFA_YIELDING_CALL_BIT   31U
#define OPTEE_FFA_YIELDING_CALL(id)   (UINT32_C(id) | BIT(OPTEE_FFA_YIELDING_CALL_BIT))

#define OPTEE_FFA_GET_API_VERSION         OPTEE_FFA_BLOCKING_CALL(0)
#define OPTEE_FFA_GET_OS_VERSION          OPTEE_FFA_BLOCKING_CALL(1)
#define OPTEE_FFA_EXCHANGE_CAPABILITIES   OPTEE_FFA_BLOCKING_CALL(2)
#define OPTEE_FFA_UNREGISTER_SHM          OPTEE_FFA_BLOCKING_CALL(3)
#define OPTEE_FFA_ENABLE_ASYNC_NOTIF      OPTEE_FFA_BLOCKING_CALL(5)

#define OPTEE_FFA_YIELDING_CALL_WITH_ARG  OPTEE_FFA_YIELDING_CALL(0)
#define OPTEE_FFA_YIELDING_CALL_RESUME    OPTEE_FFA_YIELDING_CALL(1)

#define OPTEE_FFA_YIELDING_CALL_RETURN_DONE      0U
#define OPTEE_FFA_YIELDING_CALL_RETURN_RPC_CMD   1U
#define OPTEE_FFA_YIELDING_CALL_RETURN_INTERRUPT 2U

#define OPTEE_FFA_VERSION_MAJOR  1U
#define OPTEE_FFA_VERSION_MINOR  0U

#define OPTEE_FFA_SEC_CAP_ARG_OFFSET   BIT(0)
#define OPTEE_FFA_SEC_CAP_ASYNC_NOTIF  BIT(1)

/* ------------------------------------------------------------------ */

struct optee_ffa_supp_req {
	sys_dnode_t  link;
	uint32_t     func;
	uint32_t     ret;
	size_t       num_params;
	struct tee_param *param;
	struct k_sem complete;
};

struct optee_ffa_supp {
	struct k_mutex mutex;
	sys_dlist_t    reqs;
	struct optee_ffa_supp_req *current;
	struct k_sem   reqs_c;
};

struct optee_ffa_notify_entry {
	sys_dnode_t  node;
	uint32_t     key;
	struct k_sem wait;
};

struct optee_ffa_shm_entry {
	sys_dnode_t  node;
	const struct tee_shm *shm;
	uint64_t     handle;
};

struct optee_ffa_data {
	uint16_t optee_id;
	uint32_t sec_caps;
	unsigned int max_notif_value;
	struct k_mutex call_mutex;
	struct k_spinlock shm_lock;
	sys_dlist_t shm_list;
	struct optee_ffa_supp supp;
	sys_dlist_t notif_list;
	struct k_spinlock notif_lock;
};

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static int optee_ffa_direct_req(const struct device *dev,
				struct ffa_send_direct_data *d)
{
	struct optee_ffa_data *data = dev->data;

	return ffa_msg_send_direct_req(data->optee_id, d);
}

/* Register a TEE SHM with OP-TEE via ffa_mem_share; store handle in shm_list. */
static int optee_ffa_reg_shm(const struct device *dev, const struct tee_shm *shm,
			      uint64_t *handle_out)
{
	struct optee_ffa_data *data = dev->data;
	struct optee_ffa_shm_entry *e;
	k_spinlock_key_t key;

	/* Check if already registered */
	key = k_spin_lock(&data->shm_lock);
	SYS_DLIST_FOR_EACH_CONTAINER(&data->shm_list, e, node) {
		if (e->shm == shm) {
			*handle_out = e->handle;
			k_spin_unlock(&data->shm_lock, key);
			return 0;
		}
	}
	k_spin_unlock(&data->shm_lock, key);

	e = k_malloc(sizeof(*e));
	if (!e) {
		return -ENOMEM;
	}

	uint64_t phys = (uintptr_t)shm->addr;
	uint32_t pg_cnt = (uint32_t)(ROUND_UP(shm->size, 4096U) / 4096U);
	struct ffa_mem_region_addr_range range = { .address = phys, .pg_cnt = pg_cnt };
	struct ffa_mem_ops_args args = {
		.dst_id    = data->optee_id,
		.range_cnt = 1,
		.ranges    = &range,
	};

	int rc = ffa_mem_share(&args);

	if (rc) {
		k_free(e);
		return rc;
	}

	e->shm    = shm;
	e->handle = args.g_handle;

	key = k_spin_lock(&data->shm_lock);
	sys_dlist_append(&data->shm_list, &e->node);
	k_spin_unlock(&data->shm_lock, key);

	*handle_out = e->handle;
	return 0;
}

/* Unregister: remove from list, send UNREGISTER_SHM, reclaim. */
static int optee_ffa_unreg_shm(const struct device *dev, const struct tee_shm *shm)
{
	struct optee_ffa_data *data = dev->data;
	struct optee_ffa_shm_entry *e = NULL;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->shm_lock);
	struct optee_ffa_shm_entry *iter;

	SYS_DLIST_FOR_EACH_CONTAINER(&data->shm_list, iter, node) {
		if (iter->shm == shm) {
			e = iter;
			sys_dlist_remove(&e->node);
			break;
		}
	}
	k_spin_unlock(&data->shm_lock, key);

	if (!e) {
		return -ENOENT;
	}

	/* Tell OP-TEE to unmap the buffer */
	struct ffa_send_direct_data msg = {
		.data0 = OPTEE_FFA_UNREGISTER_SHM,
		.data1 = (uint32_t)e->handle,
		.data2 = (uint32_t)(e->handle >> 32),
	};
	(void)optee_ffa_direct_req(dev, &msg);

	ffa_mem_reclaim(e->handle, 0);
	k_free(e);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Supplicant helpers (from OP-TEE RPC path)                          */
/* ------------------------------------------------------------------ */

static uint32_t optee_ffa_call_supp(const struct device *dev, uint32_t func,
				    size_t num_params, struct tee_param *param)
{
	struct optee_ffa_data *data = dev->data;
	struct optee_ffa_supp *supp = &data->supp;
	struct optee_ffa_supp_req *req;

	req = k_malloc(sizeof(*req));
	if (!req) {
		return TEEC_ERROR_OUT_OF_MEMORY;
	}
	k_sem_init(&req->complete, 0, 1);
	req->func = func;
	req->num_params = num_params;
	req->param = param;

	k_mutex_lock(&supp->mutex, K_FOREVER);
	sys_dlist_append(&supp->reqs, &req->link);
	k_mutex_unlock(&supp->mutex);
	k_sem_give(&supp->reqs_c);
	k_sem_take(&req->complete, K_FOREVER);

	uint32_t ret = req->ret;

	k_free(req);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Yield-call driver: YIELDING_CALL_WITH_ARG + RPC loop               */
/* ------------------------------------------------------------------ */

static void optee_ffa_handle_rpc_alloc(const struct device *dev,
				       struct optee_msg_arg *arg)
{
	struct tee_shm *shm = NULL;
	uint64_t handle = 0;
	int rc;

	if (arg->num_params != 1 ||
	    arg->params[0].attr != OPTEE_MSG_ATTR_TYPE_VALUE_INPUT) {
		arg->ret = TEEC_ERROR_BAD_PARAMETERS;
		return;
	}

	switch (arg->params[0].u.value.a) {
	case OPTEE_RPC_SHM_TYPE_KERNEL:
		rc = tee_add_shm(dev, NULL, OPTEE_MSG_NONCONTIG_PAGE_SIZE,
				 arg->params[0].u.value.b, TEE_SHM_ALLOC, &shm);
		break;
	case OPTEE_RPC_SHM_TYPE_APPL:
		rc = -ENOTSUP; /* TODO: supplicant alloc */
		break;
	default:
		arg->ret = TEEC_ERROR_BAD_PARAMETERS;
		return;
	}

	if (rc || !shm) {
		arg->ret = TEEC_ERROR_OUT_OF_MEMORY;
		return;
	}

	rc = optee_ffa_reg_shm(dev, shm, &handle);
	if (rc) {
		tee_rm_shm(dev, shm);
		arg->ret = TEEC_ERROR_OUT_OF_MEMORY;
		return;
	}

	arg->params[0].attr        = OPTEE_MSG_ATTR_TYPE_RMEM_INOUT;
	arg->params[0].u.rmem.shm_ref = (uint64_t)(uintptr_t)shm;
	arg->params[0].u.rmem.size = shm->size;
	arg->params[0].u.rmem.offs = 0;
	arg->ret                   = TEEC_SUCCESS;

	/* Also store handle so OP-TEE can map it: pack into value */
	/* (Linux stores handle via a separate call; keep simple here) */
	(void)handle;
}

static void optee_ffa_handle_rpc_free(const struct device *dev,
				      struct optee_msg_arg *arg)
{
	if (arg->num_params != 1) {
		arg->ret = TEEC_ERROR_BAD_PARAMETERS;
		return;
	}

	struct tee_shm *shm = (struct tee_shm *)(uintptr_t)arg->params[0].u.rmem.shm_ref;

	optee_ffa_unreg_shm(dev, shm);
	tee_rm_shm(dev, shm);
	arg->ret = TEEC_SUCCESS;
}

static void optee_ffa_handle_rpc(const struct device *dev,
				 struct optee_msg_arg *arg)
{
	switch (arg->cmd) {
	case OPTEE_RPC_CMD_SHM_ALLOC:
		optee_ffa_handle_rpc_alloc(dev, arg);
		break;
	case OPTEE_RPC_CMD_SHM_FREE:
		optee_ffa_handle_rpc_free(dev, arg);
		break;
	case OPTEE_RPC_CMD_GET_TIME: {
		int64_t ticks = k_uptime_ticks();

		arg->params[0].u.value.a =
			ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		arg->params[0].u.value.b =
			k_ticks_to_ns_floor64(
				ticks - arg->params[0].u.value.a *
				CONFIG_SYS_CLOCK_TICKS_PER_SEC);
		arg->ret = TEEC_SUCCESS;
		break;
	}
	case OPTEE_RPC_CMD_NOTIFICATION: {
		/* Simplified: just succeed */
		arg->ret = TEEC_SUCCESS;
		break;
	}
	default:
		/* Dispatch to supplicant */
		if (arg->num_params > 0) {
			struct tee_param *params = k_malloc(
				sizeof(*params) * arg->num_params);

			if (params) {
				arg->ret = optee_ffa_call_supp(dev, arg->cmd,
							       arg->num_params,
							       params);
				k_free(params);
			} else {
				arg->ret = TEEC_ERROR_OUT_OF_MEMORY;
			}
		} else {
			arg->ret = TEEC_ERROR_NOT_IMPLEMENTED;
		}
		break;
	}
}

/*
 * Issue a yielding call to OP-TEE and drive the RPC return loop.
 *
 * @param dev      OP-TEE FF-A device
 * @param arg_shm  Shared memory holding the optee_msg_arg buffer
 * @param handle   FF-A memory handle for arg_shm
 *
 * @return 0 on TEEC_SUCCESS, negative errno on transport error, or
 *         TEEC_ERROR_* values for secure-world failures.
 */
static int optee_ffa_do_call(const struct device *dev,
			     struct tee_shm *arg_shm, uint64_t handle)
{
	struct optee_msg_arg *arg = arg_shm->addr;
	struct ffa_send_direct_data msg = {
		.data0 = OPTEE_FFA_YIELDING_CALL_WITH_ARG,
		.data1 = (uint32_t)handle,
		.data2 = (uint32_t)(handle >> 32),
		.data3 = 0,  /* offset 0 within the shared region */
		.data4 = 0,
	};
	int rc;

	for (;;) {
		rc = optee_ffa_direct_req(dev, &msg);
		if (rc) {
			return rc;
		}

		uint32_t ret_type = (uint32_t)msg.data1;
		uint32_t resume_info = (uint32_t)msg.data4;

		if (ret_type == OPTEE_FFA_YIELDING_CALL_RETURN_DONE) {
			return (msg.data0 == 0) ? 0 : -EIO;
		}

		if (ret_type == OPTEE_FFA_YIELDING_CALL_RETURN_INTERRUPT) {
			/* Just resume; no RPC work needed */
		} else if (ret_type == OPTEE_FFA_YIELDING_CALL_RETURN_RPC_CMD) {
			optee_ffa_handle_rpc(dev, arg);
		} else {
			LOG_ERR("Unknown OP-TEE FFA return type 0x%x", ret_type);
			return -EIO;
		}

		msg = (struct ffa_send_direct_data){
			.data0 = OPTEE_FFA_YIELDING_CALL_RESUME,
			.data4 = resume_info,
		};
	}
}

/* ------------------------------------------------------------------ */
/* Param helpers shared with SMC path                                  */
/* ------------------------------------------------------------------ */

static int param_to_msg(const struct tee_param *p, unsigned int n,
			struct optee_msg_param *mp)
{
	for (unsigned int i = 0; i < n; i++) {
		switch (p[i].attr) {
		case TEE_PARAM_ATTR_TYPE_NONE:
			mp[i].attr = OPTEE_MSG_ATTR_TYPE_NONE;
			memset(&mp[i].u, 0, sizeof(mp[i].u));
			break;
		case TEE_PARAM_ATTR_TYPE_VALUE_INPUT:
		case TEE_PARAM_ATTR_TYPE_VALUE_OUTPUT:
		case TEE_PARAM_ATTR_TYPE_VALUE_INOUT:
			mp[i].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT +
				     p[i].attr - TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
			mp[i].u.value.a = p[i].a;
			mp[i].u.value.b = p[i].b;
			mp[i].u.value.c = p[i].c;
			break;
		case TEE_PARAM_ATTR_TYPE_MEMREF_INPUT:
		case TEE_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
		case TEE_PARAM_ATTR_TYPE_MEMREF_INOUT:
			mp[i].attr = OPTEE_MSG_ATTR_TYPE_RMEM_INPUT +
				     p[i].attr - TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
			mp[i].u.rmem.shm_ref = p[i].c;
			mp[i].u.rmem.size    = p[i].b;
			mp[i].u.rmem.offs    = p[i].a;
			break;
		default:
			return -EINVAL;
		}
	}
	return 0;
}

static int msg_to_param(struct tee_param *p, unsigned int n,
			const struct optee_msg_param *mp)
{
	for (unsigned int i = 0; i < n; i++) {
		uint32_t attr = mp[i].attr & OPTEE_MSG_ATTR_TYPE_MASK;

		switch (attr) {
		case OPTEE_MSG_ATTR_TYPE_NONE:
			memset(&p[i], 0, sizeof(p[i]));
			p[i].attr = TEE_PARAM_ATTR_TYPE_NONE;
			break;
		case OPTEE_MSG_ATTR_TYPE_VALUE_INPUT:
		case OPTEE_MSG_ATTR_TYPE_VALUE_OUTPUT:
		case OPTEE_MSG_ATTR_TYPE_VALUE_INOUT:
			p[i].attr = TEE_PARAM_ATTR_TYPE_VALUE_INPUT +
				    attr - OPTEE_MSG_ATTR_TYPE_VALUE_INPUT;
			p[i].a = mp[i].u.value.a;
			p[i].b = mp[i].u.value.b;
			p[i].c = mp[i].u.value.c;
			break;
		case OPTEE_MSG_ATTR_TYPE_RMEM_INPUT:
		case OPTEE_MSG_ATTR_TYPE_RMEM_OUTPUT:
		case OPTEE_MSG_ATTR_TYPE_RMEM_INOUT:
			p[i].attr = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT +
				    attr - OPTEE_MSG_ATTR_TYPE_RMEM_INPUT;
			p[i].b = mp[i].u.rmem.size;
			p[i].a = mp[i].u.rmem.offs;
			p[i].c = mp[i].u.rmem.shm_ref;
			break;
		default:
			return -EINVAL;
		}
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Alloc + register an arg buffer, call OP-TEE, then free             */
/* ------------------------------------------------------------------ */

static int optee_ffa_call_with_arg(const struct device *dev,
				   struct optee_msg_arg *arg_buf,
				   size_t arg_size)
{
	struct tee_shm *shm;
	uint64_t handle;
	int rc;

	rc = tee_add_shm(dev, NULL, OPTEE_MSG_NONCONTIG_PAGE_SIZE,
			 arg_size, TEE_SHM_ALLOC, &shm);
	if (rc) {
		return rc;
	}

	memcpy(shm->addr, arg_buf, arg_size);

	rc = optee_ffa_reg_shm(dev, shm, &handle);
	if (rc) {
		goto out_free_shm;
	}

	rc = optee_ffa_do_call(dev, shm, handle);
	if (!rc) {
		/* Copy result back */
		memcpy(arg_buf, shm->addr, arg_size);
	}

	/* Reclaim memory without UNREGISTER_SHM (arg buffer is ephemeral) */
	ffa_mem_reclaim(handle, 0);
	{
		/* Remove from shm_list without unreg message */
		struct optee_ffa_data *d = dev->data;
		struct optee_ffa_shm_entry *e;
		k_spinlock_key_t key = k_spin_lock(&d->shm_lock);

		SYS_DLIST_FOR_EACH_CONTAINER(&d->shm_list, e, node) {
			if (e->shm == shm) {
				sys_dlist_remove(&e->node);
				k_free(e);
				break;
			}
		}
		k_spin_unlock(&d->shm_lock, key);
	}

out_free_shm:
	tee_rm_shm(dev, shm);
	return rc;
}

/* ------------------------------------------------------------------ */
/* TEE driver API implementation                                       */
/* ------------------------------------------------------------------ */

static int optee_ffa_get_version(const struct device *dev,
				 struct tee_version_info *info)
{
	if (!info) {
		return -EINVAL;
	}

	info->impl_id   = TEE_IMPL_ID_OPTEE;
	info->impl_caps = TEE_OPTEE_CAP_TZ;
	info->gen_caps  = TEE_GEN_CAP_GP | TEE_GEN_CAP_REG_MEM;
	return 0;
}

static int optee_ffa_open_session(const struct device *dev,
				  struct tee_open_session_arg *arg,
				  unsigned int num_param,
				  struct tee_param *param,
				  uint32_t *session_id)
{
	if (!arg || !session_id) {
		return -EINVAL;
	}

	size_t arg_size = OPTEE_MSG_GET_ARG_SIZE(num_param + 2);
	struct optee_msg_arg *marg = k_malloc(arg_size);

	if (!marg) {
		return -ENOMEM;
	}

	memset(marg, 0, arg_size);
	marg->cmd       = OPTEE_MSG_CMD_OPEN_SESSION;
	marg->num_params = num_param + 2;
	marg->params[0].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT | OPTEE_MSG_ATTR_META;
	marg->params[1].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT | OPTEE_MSG_ATTR_META;
	memcpy(&marg->params[0].u.value, arg->uuid, sizeof(arg->uuid));
	memcpy(&marg->params[1].u.value, arg->clnt_uuid, sizeof(arg->clnt_uuid));
	marg->params[1].u.value.c = arg->clnt_login;

	int rc = param_to_msg(param, num_param, marg->params + 2);

	if (!rc) {
		rc = optee_ffa_call_with_arg(dev, marg, arg_size);
	}

	if (!rc) {
		(void)msg_to_param(param, num_param, marg->params);
		*session_id      = marg->session;
		arg->ret         = marg->ret;
		arg->ret_origin  = marg->ret_origin;
	}

	k_free(marg);
	return rc;
}

static int optee_ffa_close_session(const struct device *dev, uint32_t session_id)
{
	size_t arg_size = OPTEE_MSG_GET_ARG_SIZE(0);
	struct optee_msg_arg *marg = k_malloc(arg_size);

	if (!marg) {
		return -ENOMEM;
	}

	memset(marg, 0, arg_size);
	marg->cmd       = OPTEE_MSG_CMD_CLOSE_SESSION;
	marg->session   = session_id;
	marg->num_params = 0;

	int rc = optee_ffa_call_with_arg(dev, marg, arg_size);

	k_free(marg);
	return rc;
}

static int optee_ffa_invoke_func(const struct device *dev,
				 struct tee_invoke_func_arg *arg,
				 unsigned int num_param,
				 struct tee_param *param)
{
	if (!arg) {
		return -EINVAL;
	}

	size_t arg_size = OPTEE_MSG_GET_ARG_SIZE(num_param);
	struct optee_msg_arg *marg = k_malloc(arg_size);

	if (!marg) {
		return -ENOMEM;
	}

	memset(marg, 0, arg_size);
	marg->cmd       = OPTEE_MSG_CMD_INVOKE_COMMAND;
	marg->func      = arg->func;
	marg->session   = arg->session;
	marg->num_params = num_param;

	int rc = param_to_msg(param, num_param, marg->params);

	if (!rc) {
		rc = optee_ffa_call_with_arg(dev, marg, arg_size);
	}

	if (!rc) {
		(void)msg_to_param(param, num_param, marg->params);
		arg->ret        = marg->ret;
		arg->ret_origin = marg->ret_origin;
	}

	k_free(marg);
	return rc;
}

static int optee_ffa_cancel(const struct device *dev, uint32_t session_id,
			    uint32_t cancel_id)
{
	size_t arg_size = OPTEE_MSG_GET_ARG_SIZE(0);
	struct optee_msg_arg *marg = k_malloc(arg_size);

	if (!marg) {
		return -ENOMEM;
	}

	memset(marg, 0, arg_size);
	marg->cmd       = OPTEE_MSG_CMD_CANCEL;
	marg->session   = session_id;
	marg->cancel_id = cancel_id;
	marg->num_params = 0;

	int rc = optee_ffa_call_with_arg(dev, marg, arg_size);

	k_free(marg);
	return rc;
}

static int optee_ffa_shm_register(const struct device *dev, struct tee_shm *shm)
{
	uint64_t handle;

	return optee_ffa_reg_shm(dev, shm, &handle);
}

static int optee_ffa_shm_unregister(const struct device *dev, struct tee_shm *shm)
{
	return optee_ffa_unreg_shm(dev, shm);
}

static int optee_ffa_suppl_recv(const struct device *dev, uint32_t *func,
				unsigned int *num_params,
				struct tee_param *param)
{
	struct optee_ffa_data *data = dev->data;
	struct optee_ffa_supp *supp = &data->supp;
	struct optee_ffa_supp_req *req = NULL;

	while (true) {
		k_mutex_lock(&supp->mutex, K_FOREVER);
		sys_dnode_t *head = sys_dlist_peek_head(&supp->reqs);

		req = head ? CONTAINER_OF(head, struct optee_ffa_supp_req, link)
			   : NULL;
		if (req) {
			if (*num_params < req->num_params) {
				k_mutex_unlock(&supp->mutex);
				return -EINVAL;
			}
			supp->current = req;
			sys_dlist_remove(&req->link);
		}
		k_mutex_unlock(&supp->mutex);
		if (req) {
			break;
		}
		k_sem_take(&supp->reqs_c, K_FOREVER);
	}

	*func = req->func;
	*num_params = req->num_params;
	memcpy(param, req->param, sizeof(*param) * req->num_params);
	return 0;
}

static int optee_ffa_suppl_send(const struct device *dev, unsigned int ret,
				unsigned int num_params,
				struct tee_param *param)
{
	struct optee_ffa_data *data = dev->data;
	struct optee_ffa_supp *supp = &data->supp;
	struct optee_ffa_supp_req *req;

	k_mutex_lock(&supp->mutex, K_FOREVER);
	req = supp->current;
	supp->current = NULL;
	k_mutex_unlock(&supp->mutex);

	if (!req) {
		return -EINVAL;
	}

	for (unsigned int i = 0; i < req->num_params && i < num_params; i++) {
		switch (req->param[i].attr & TEE_PARAM_ATTR_TYPE_MASK) {
		case TEE_PARAM_ATTR_TYPE_VALUE_OUTPUT:
		case TEE_PARAM_ATTR_TYPE_VALUE_INOUT:
			req->param[i].a = param[i].a;
			req->param[i].b = param[i].b;
			req->param[i].c = param[i].c;
			break;
		default:
			break;
		}
	}
	req->ret = ret;
	k_sem_give(&req->complete);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Probe / init                                                        */
/* ------------------------------------------------------------------ */

static int optee_ffa_probe(const struct device *dev)
{
	struct optee_ffa_data *data = dev->data;
	struct ffa_partition_info part_info[1];
	size_t count = 1;
	struct ffa_send_direct_data msg = {0};
	int rc;

	if (!ffa_is_available()) {
		LOG_WRN("FF-A subsystem not available; OP-TEE FFA probe deferred");
		return -ENODEV;
	}

	/* Discover OP-TEE partition */
	rc = ffa_partition_info_get(&optee_ffa_os_uuid, part_info, &count);
	if (rc || count == 0) {
		LOG_ERR("OP-TEE partition not found (rc=%d count=%zu)", rc, count);
		return -ENODEV;
	}

	data->optee_id = part_info[0].id;
	LOG_INF("OP-TEE FFA endpoint id=0x%04x", data->optee_id);

	/* GET_API_VERSION */
	memset(&msg, 0, sizeof(msg));
	msg.data0 = OPTEE_FFA_GET_API_VERSION;
	rc = optee_ffa_direct_req(dev, &msg);
	if (rc || msg.data0 != OPTEE_FFA_VERSION_MAJOR) {
		LOG_ERR("OP-TEE API version mismatch (w3=%lu rc=%d)",
			msg.data0, rc);
		return -ENOTSUP;
	}

	/* GET_OS_VERSION */
	memset(&msg, 0, sizeof(msg));
	msg.data0 = OPTEE_FFA_GET_OS_VERSION;
	(void)optee_ffa_direct_req(dev, &msg);
	LOG_INF("OP-TEE OS %lu.%lu (sha %08lx)", msg.data0, msg.data1, msg.data2);

	/* EXCHANGE_CAPABILITIES */
	memset(&msg, 0, sizeof(msg));
	msg.data0 = OPTEE_FFA_EXCHANGE_CAPABILITIES;
	rc = optee_ffa_direct_req(dev, &msg);
	if (rc || msg.data0 != 0) {
		LOG_ERR("EXCHANGE_CAPABILITIES failed (w3=%lu rc=%d)", msg.data0, rc);
		return -EINVAL;
	}

	data->sec_caps           = (uint32_t)msg.data2; /* w5 */
	data->max_notif_value    = (unsigned int)msg.data3; /* w6 */

	k_mutex_init(&data->call_mutex);
	sys_dlist_init(&data->shm_list);
	sys_dlist_init(&data->notif_list);
	k_mutex_init(&data->supp.mutex);
	k_sem_init(&data->supp.reqs_c, 0, 1);
	sys_dlist_init(&data->supp.reqs);

	LOG_INF("OP-TEE FFA transport ready (sec_caps=0x%x)", data->sec_caps);
	return 0;
}

/* ------------------------------------------------------------------ */

static DEVICE_API(tee, optee_ffa_driver_api) = {
	.get_version   = optee_ffa_get_version,
	.open_session  = optee_ffa_open_session,
	.close_session = optee_ffa_close_session,
	.cancel        = optee_ffa_cancel,
	.invoke_func   = optee_ffa_invoke_func,
	.shm_register  = optee_ffa_shm_register,
	.shm_unregister = optee_ffa_shm_unregister,
	.suppl_recv    = optee_ffa_suppl_recv,
	.suppl_send    = optee_ffa_suppl_send,
};

#define OPTEE_FFA_DEFINE(inst)					\
	static struct optee_ffa_data optee_ffa_data_##inst;	\
								\
	DEVICE_DT_INST_DEFINE(inst, optee_ffa_probe, NULL,	\
			      &optee_ffa_data_##inst, NULL,	\
			      POST_KERNEL,			\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			      &optee_ffa_driver_api);

DT_INST_FOREACH_STATUS_OKAY(OPTEE_FFA_DEFINE)

#ifdef CONFIG_ZTEST
/* Test helper: re-run probe with mock conduit already installed. */
int optee_ffa_test_reinit(const struct device *dev)
{
	struct optee_ffa_data *data = dev->data;

	/* Reset data to a known state */
	memset(data, 0, sizeof(*data));
	return optee_ffa_probe(dev);
}
#endif /* CONFIG_ZTEST */
