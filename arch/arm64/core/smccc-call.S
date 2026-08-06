/*
 * Copyright (c) 2019 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This file implements the common calling mechanism to be used with the Secure
 * Monitor Call (SMC) and Hypervisor Call (HVC).
 *
 * See https://developer.arm.com/docs/den0028/latest
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/arch/cpu.h>
#include <offsets_short.h>

.macro SMCCC instr
	\instr  #0
	ldr     x9, [sp]
	stp     x0, x1, [x9, __arm_smccc_res_t_a0_a1_OFFSET]
	stp     x2, x3, [x9, __arm_smccc_res_t_a2_a3_OFFSET]
	stp     x4, x5, [x9, __arm_smccc_res_t_a4_a5_OFFSET]
	stp     x6, x7, [x9, __arm_smccc_res_t_a6_a7_OFFSET]
	ret
.endm

/*
 * The SMC instruction is used to generate a synchronous exception that is
 * handled by Secure Monitor code running in EL3.
 */
GTEXT(arm_smccc_smc)
SECTION_FUNC(TEXT, arm_smccc_smc)
	SMCCC	smc

/*
 * The HVC instruction is used to generate a synchronous exception that is
 * handled by a hypervisor running in EL2.
 */
GTEXT(arm_smccc_hvc)
SECTION_FUNC(TEXT, arm_smccc_hvc)
	SMCCC	hvc

#ifdef CONFIG_ARM_SMCCC_1_2
/*
 * SMCCC v1.2 calling convention: pass and return x0-x17.
 *
 * x0 = const struct arm_smccc_1_2_regs *args
 * x1 = struct arm_smccc_1_2_regs *res
 *
 * struct arm_smccc_1_2_regs is 18 contiguous unsigned longs, so the byte
 * offset of member aN is N*8.
 */
.macro SMCCC_1_2 instr
	/* Preserve res (x1) across the call in callee-saved x19 */
	stp	x1, x19, [sp, #-16]!
	mov	x19, x1

	/* Load x0-x17 from *args; load x0/x1 last since x0 holds args */
	ldp	x16, x17, [x0, #(16 * 8)]
	ldp	x14, x15, [x0, #(14 * 8)]
	ldp	x12, x13, [x0, #(12 * 8)]
	ldp	x10, x11, [x0, #(10 * 8)]
	ldp	x8,  x9,  [x0, #(8 * 8)]
	ldp	x6,  x7,  [x0, #(6 * 8)]
	ldp	x4,  x5,  [x0, #(4 * 8)]
	ldp	x2,  x3,  [x0, #(2 * 8)]
	ldp	x0,  x1,  [x0, #(0 * 8)]

	\instr	#0

	/* Store x0-x17 into *res (held in x19) */
	stp	x0,  x1,  [x19, #(0 * 8)]
	stp	x2,  x3,  [x19, #(2 * 8)]
	stp	x4,  x5,  [x19, #(4 * 8)]
	stp	x6,  x7,  [x19, #(6 * 8)]
	stp	x8,  x9,  [x19, #(8 * 8)]
	stp	x10, x11, [x19, #(10 * 8)]
	stp	x12, x13, [x19, #(12 * 8)]
	stp	x14, x15, [x19, #(14 * 8)]
	stp	x16, x17, [x19, #(16 * 8)]

	ldp	x1, x19, [sp], #16
	ret
.endm

GTEXT(arm_smccc_1_2_smc)
SECTION_FUNC(TEXT, arm_smccc_1_2_smc)
	SMCCC_1_2	smc

GTEXT(arm_smccc_1_2_hvc)
SECTION_FUNC(TEXT, arm_smccc_1_2_hvc)
	SMCCC_1_2	hvc
#endif /* CONFIG_ARM_SMCCC_1_2 */
