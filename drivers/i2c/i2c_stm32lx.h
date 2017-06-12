/*
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32LX_I2C_H_
#define _STM32LX_I2C_H_

typedef void (*irq_config_func_t)(struct device *port);

/* device config */
struct i2c_stm32lx_config {
	struct stm32_pclken pclken;
	I2C_TypeDef *i2c;
#ifdef CONFIG_I2C_STM32LX_INTERRUPT
	irq_config_func_t irq_config_func;
#endif

};

/* driver data */
struct i2c_stm32lx_data {
	struct device *clock;
	union dev_config dev_config;
#ifdef CONFIG_I2C_STM32LX_INTERRUPT
	struct k_sem device_sync_sem;
#endif
	struct {
		struct i2c_msg *msg;
		unsigned int len;
		u8_t *buf;
		unsigned int is_err;
		unsigned int is_nack;
		unsigned int is_write;
	} current;
};

#endif	/* _STM32LX_UART_H_ */
