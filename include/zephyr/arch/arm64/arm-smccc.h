/*
 * Copyright 2020 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM64_ARM_SMCCC_H_
#define ZEPHYR_INCLUDE_ARCH_ARM64_ARM_SMCCC_H_

/*
 * Result from SMC/HVC call
 * @a0-a7 result values from registers 0 to 7
 */
struct arm_smccc_res {
	unsigned long a0;
	unsigned long a1;
	unsigned long a2;
	unsigned long a3;
	unsigned long a4;
	unsigned long a5;
	unsigned long a6;
	unsigned long a7;
};

typedef struct arm_smccc_res arm_smccc_res_t;

enum arm_smccc_conduit {
	SMCCC_CONDUIT_NONE,
	SMCCC_CONDUIT_SMC,
	SMCCC_CONDUIT_HVC,
};

/*
 * @brief Make HVC calls
 *
 * @param a0 function identifier
 * @param a1-a7 parameters registers
 * @param res results
 */
void arm_smccc_hvc(unsigned long a0, unsigned long a1,
		   unsigned long a2, unsigned long a3,
		   unsigned long a4, unsigned long a5,
		   unsigned long a6, unsigned long a7,
		   struct arm_smccc_res *res);

/*
 * @brief Make SMC calls
 *
 * @param a0 function identifier
 * @param a1-a7 parameters registers
 * @param res results
 */
void arm_smccc_smc(unsigned long a0, unsigned long a1,
		   unsigned long a2, unsigned long a3,
		   unsigned long a4, unsigned long a5,
		   unsigned long a6, unsigned long a7,
		   struct arm_smccc_res *res);

#if defined(CONFIG_ARM_SMCCC_1_2)
/*
 * Result/argument registers for an SMCCC v1.2 call.
 * @a0-a17 correspond to registers x0-x17.
 */
struct arm_smccc_1_2_regs {
	unsigned long a0;
	unsigned long a1;
	unsigned long a2;
	unsigned long a3;
	unsigned long a4;
	unsigned long a5;
	unsigned long a6;
	unsigned long a7;
	unsigned long a8;
	unsigned long a9;
	unsigned long a10;
	unsigned long a11;
	unsigned long a12;
	unsigned long a13;
	unsigned long a14;
	unsigned long a15;
	unsigned long a16;
	unsigned long a17;
};

/*
 * @brief Make an SMCCC v1.2 SMC call passing/returning x0-x17
 *
 * @param args input register values (x0-x17)
 * @param res  output register values (x0-x17)
 */
void arm_smccc_1_2_smc(const struct arm_smccc_1_2_regs *args,
		       struct arm_smccc_1_2_regs *res);

/*
 * @brief Make an SMCCC v1.2 HVC call passing/returning x0-x17
 *
 * @param args input register values (x0-x17)
 * @param res  output register values (x0-x17)
 */
void arm_smccc_1_2_hvc(const struct arm_smccc_1_2_regs *args,
		       struct arm_smccc_1_2_regs *res);
#endif /* CONFIG_ARM_SMCCC_1_2 */

#endif /* ZEPHYR_INCLUDE_ARCH_ARM64_ARM_SMCCC_H_ */
