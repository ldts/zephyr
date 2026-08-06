/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <errno.h>
#include "ffa_internal.h"

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

ZTEST_SUITE(ffa_core, NULL, NULL, NULL, NULL, NULL);
