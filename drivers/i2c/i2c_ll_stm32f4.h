/*
 * Copyright (c) 2017, I-SENSE group of ICCS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32F4_I2C_H_
#define _STM32F4_I2C_H_

typedef void (*irq_config_func_t)(struct device *port);

/* device config */
struct i2c_stm32f4_config {
	struct stm32_pclken pclken;
	I2C_TypeDef *i2c;
#ifdef CONFIG_I2C_STM32F4_INTERRUPT
	irq_config_func_t irq_config_func;
#endif
};

/* driver data */
struct i2c_stm32f4_data {
	/* clock device */
	struct device *clock;
	/* Device config */
	union dev_config dev_config;
	/* ISR Sync */
	struct k_sem sync;
	/* Current message data */
	struct {
		struct i2c_msg *msg;
		u32_t len;
		u8_t *buf;
		u32_t is_err;
		u32_t is_nack;
		u32_t is_write;
		u32_t is_restart;
		u32_t flags;
	} current;
	/* Slave address */
	u16_t slave_address;
};

#endif /* _STM32F4_I2C_H_ */
