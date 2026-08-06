/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/arch/arm64/arm-smccc.h>
#include <stddef.h>

/* The assembly in smccc-call.S assumes aN is at byte offset N*8. */
ZTEST(smccc_1_2, test_regs_layout)
{
	zassert_equal(sizeof(struct arm_smccc_1_2_regs), 18UL * sizeof(unsigned long),
		      "arm_smccc_1_2_regs must be 18 contiguous longs");
	zassert_equal(offsetof(struct arm_smccc_1_2_regs, a0), 0 * 8, "a0 offset");
	zassert_equal(offsetof(struct arm_smccc_1_2_regs, a2), 2 * 8, "a2 offset");
	zassert_equal(offsetof(struct arm_smccc_1_2_regs, a8), 8 * 8, "a8 offset");
	zassert_equal(offsetof(struct arm_smccc_1_2_regs, a16), 16 * 8, "a16 offset");
	zassert_equal(offsetof(struct arm_smccc_1_2_regs, a17), 17 * 8, "a17 offset");
}

/*
 * Reference both symbols so the test link-fails until they are implemented.
 * Guarded so it is never actually executed (no FF-A responder on this board).
 */
ZTEST(smccc_1_2, test_symbols_link)
{
	void (*smc_fn)(const struct arm_smccc_1_2_regs *, struct arm_smccc_1_2_regs *) =
		arm_smccc_1_2_smc;
	void (*hvc_fn)(const struct arm_smccc_1_2_regs *, struct arm_smccc_1_2_regs *) =
		arm_smccc_1_2_hvc;

	zassert_true(smc_fn != hvc_fn, "smc and hvc must be distinct routines");
}

ZTEST_SUITE(smccc_1_2, NULL, NULL, NULL, NULL, NULL);
