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

ZTEST_SUITE(ffa_core, NULL, NULL, NULL, NULL, NULL);
