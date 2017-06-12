/*
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <kernel.h>

#include <board.h>
#undef I2C_SPEED_STANDARD
#undef I2C_SPEED_FAST
#include <clock_control.h>

#include <misc/util.h>

#include <i2c.h>

#include <clock_control/stm32_clock_control.h>
#include "i2c_stm32.h"

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_I2C_LEVEL
#include <logging/sys_log.h>

/* convenience defines */
#define DEV_CFG(dev)							\
	((const struct i2c_stm32_config * const)(dev)->config->config_info)
#define DEV_DATA(dev)							\
	((struct i2c_stm32_data * const)(dev)->driver_data)
#define I2C_STRUCT(dev)							\
	((void *)(DEV_CFG(dev))->base)

static int i2c_stm32_runtime_configure(struct device *dev, u32_t config)
{
	struct i2c_stm32_data *data = DEV_DATA(dev);
	I2C_HandleTypeDef *I2CHandle = &data->hi2c;

	data->dev_config.raw = config;

	if (data->dev_config.bits.is_slave_read) {
		SYS_LOG_ERR("is slave set");
		return -EINVAL;
	}

	switch (data->dev_config.bits.speed) {
	case I2C_SPEED_STANDARD:
		I2CHandle->Init.ClockSpeed = 100000;
		break;
	case I2C_SPEED_FAST:
		I2CHandle->Init.ClockSpeed = 400000;
		I2CHandle->Init.DutyCycle = I2C_DUTYCYCLE_2;
		break;
	default:
		SYS_LOG_ERR("invalid speed");
		return -EINVAL;
	}

	if (data->dev_config.bits.use_10_bit_addr)
		I2CHandle->Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	else
		I2CHandle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

	I2CHandle->Init.OwnAddress1 = 0;
	I2CHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2CHandle->Init.OwnAddress2 = 0;
	I2CHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2CHandle->Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;

	HAL_I2C_DeInit(I2CHandle);

	if (HAL_I2C_Init(I2CHandle) != HAL_OK) {
		SYS_LOG_ERR("I2C initialization error");
		return -EIO;
	}

	while (HAL_I2C_GetState(I2CHandle) != HAL_I2C_STATE_READY) {
		;
	}

	return 0;
}

#ifdef CONFIG_I2C_STM32_INTERRUPT
static void i2c_stm32_ev_isr(void *arg)
{
	struct device * const dev = (struct device *)arg;
	struct i2c_stm32_data *data = DEV_DATA(dev);
	I2C_HandleTypeDef *I2CHandle = &data->hi2c;

	HAL_I2C_EV_IRQHandler(I2CHandle);
}

static void i2c_stm32_er_isr(void *arg)
{
	struct device * const dev = (struct device *)arg;
	struct i2c_stm32_data *data = DEV_DATA(dev);
	I2C_HandleTypeDef *I2CHandle = &data->hi2c;

	HAL_I2C_ER_IRQHandler(I2CHandle);
}
#endif

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2CHandle)
{
	struct i2c_stm32_data *data =
		CONTAINER_OF(I2CHandle, struct i2c_stm32_data, hi2c);

	data->is_err = false;

	k_sem_give(&data->device_sync_sem);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2CHandle)
{
	struct i2c_stm32_data *data =
		CONTAINER_OF(I2CHandle, struct i2c_stm32_data, hi2c);

	data->is_err = false;

	k_sem_give(&data->device_sync_sem);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2CHandle)
{
	struct i2c_stm32_data *data =
		CONTAINER_OF(I2CHandle, struct i2c_stm32_data, hi2c);

	data->is_err = true;

	k_sem_give(&data->device_sync_sem);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *I2CHandle)
{
	struct i2c_stm32_data *data =
		CONTAINER_OF(I2CHandle, struct i2c_stm32_data, hi2c);

	data->is_err = true;

	k_sem_give(&data->device_sync_sem);
}

__weak HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
	/* Nothing to do here */

	return HAL_OK;
}

#define TIMEOUT 1000

static int i2c_stm32_transfer(struct device *dev,
				struct i2c_msg *msgs, u8_t num_msgs,
				u16_t slave_address)
{
	struct i2c_stm32_data *data = DEV_DATA(dev);
	I2C_HandleTypeDef *I2CHandle = &data->hi2c;
	struct i2c_msg *cur_msg = msgs;
	u8_t msg_left = num_msgs;
	int ret = 0;

	while (HAL_I2C_GetState(I2CHandle) != HAL_I2C_STATE_READY) {
		;
	}

	while (HAL_I2C_IsDeviceReady(I2CHandle, slave_address, 10, 300) == HAL_TIMEOUT) {
		;
	}

	/* Process all messages one-by-one */
	while (msg_left > 0) {
		if ((cur_msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
#ifdef CONFIG_I2C_STM32_INTERRUPT
			if (HAL_I2C_Master_Transmit_IT(I2CHandle, slave_address,
						       cur_msg->buf,
						       cur_msg->len) != HAL_OK)
				ret = -EIO;
#else
			if (HAL_I2C_Master_Transmit(I2CHandle, slave_address,
						    cur_msg->buf, cur_msg->len,
						    TIMEOUT) != HAL_OK)
				ret = -EIO;
#endif
		} else {
#ifdef CONFIG_I2C_STM32_INTERRUPT
			if (HAL_I2C_Master_Receive_IT(I2CHandle, slave_address,
						      cur_msg->buf,
						      cur_msg->len) != HAL_OK)
				ret = -EIO;
#else
			if (HAL_I2C_Master_Receive(I2CHandle, slave_address,
						   cur_msg->buf, cur_msg->len,
						   TIMEOUT) != HAL_OK)
				ret = -EIO;
#endif
		}

		if (ret < 0) {
			ret = -EIO;
			break;
		}

#ifdef CONFIG_I2C_STM32_INTERRUPT

		if (data->is_err) {
			ret = -EIO;
			break;
		}
		__HAL_I2C_ENABLE(I2CHandle);

		k_sem_take(&data->device_sync_sem, K_FOREVER);

		__HAL_I2C_DISABLE(I2CHandle);
#endif

		cur_msg++;
		msg_left--;
	};

	return ret;
}

static const struct i2c_driver_api api_funcs = {
	.configure = i2c_stm32_runtime_configure,
	.transfer = i2c_stm32_transfer,
};

static inline void __i2c_stm32_get_clock(struct device *dev)
{
	struct i2c_stm32_data *data = DEV_DATA(dev);
	struct device *clk =
		device_get_binding(STM32_CLOCK_CONTROL_NAME);

	__ASSERT_NO_MSG(clk);

	data->clock = clk;
}

static int i2c_stm32_init(struct device *dev)
{
	struct i2c_stm32_data *data = DEV_DATA(dev);
	const struct i2c_stm32_config *cfg = DEV_CFG(dev);
	I2C_HandleTypeDef *I2CHandle = &data->hi2c;
	int ret;

	k_sem_init(&data->device_sync_sem, 0, UINT_MAX);

	__i2c_stm32_get_clock(dev);

	/* enable clock */
	clock_control_on(data->clock,
		(clock_control_subsys_t *)&cfg->pclken);

	I2CHandle->Instance = I2C_STRUCT(dev);

	/* Try to Setup HW */
	ret = i2c_stm32_runtime_configure(dev, data->dev_config.raw);
	if (ret) {
		return ret;
	}

#ifdef CONFIG_I2C_STM32_INTERRUPT
	cfg->irq_config_func(dev);
#endif

	return 0;
}

#ifdef CONFIG_I2C_1

#ifdef CONFIG_I2C_STM32_INTERRUPT
static void i2c_stm32_irq_config_func_1(struct device *port);
#endif

static const struct i2c_stm32_config i2c_stm32_cfg_1 = {
	.base = (u8_t *)I2C1_BASE,
	.pclken = {
		.bus = STM32_CLOCK_BUS_APB1,
		.enr = LL_APB1_GRP1_PERIPH_I2C1
	},
#ifdef CONFIG_I2C_STM32_INTERRUPT
	.irq_config_func = i2c_stm32_irq_config_func_1,
#endif
};

static struct i2c_stm32_data i2c_stm32_dev_data_1 = {
	.dev_config.raw = CONFIG_I2C_1_DEFAULT_CFG,
};

DEVICE_AND_API_INIT(i2c_stm32_1, CONFIG_I2C_1_NAME, &i2c_stm32_init,
		    &i2c_stm32_dev_data_1, &i2c_stm32_cfg_1,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &api_funcs);

#ifdef CONFIG_I2C_STM32_INTERRUPT
static void i2c_stm32_irq_config_func_1(struct device *dev)
{
	IRQ_CONNECT(I2C1_EV_IRQn, CONFIG_I2C_1_IRQ_PRI,
		i2c_stm32_ev_isr, DEVICE_GET(i2c_stm32_1), 0);
	irq_enable(I2C1_EV_IRQn);

	IRQ_CONNECT(I2C1_ER_IRQn, CONFIG_I2C_1_IRQ_PRI,
		i2c_stm32_er_isr, DEVICE_GET(i2c_stm32_1), 0);
	irq_enable(I2C1_ER_IRQn);
}
#endif

#endif /* CONFIG_I2C_1 */

#ifdef CONFIG_I2C_2

#ifdef CONFIG_I2C_STM32_INTERRUPT
static void i2c_stm32_irq_config_func_2(struct device *port);
#endif

static const struct i2c_stm32_config i2c_stm32_cfg_2 = {
	.base = (u8_t *)I2C2_BASE,
	.pclken = {
		.bus = STM32_CLOCK_BUS_APB1,
		.enr = LL_APB1_GRP1_PERIPH_I2C2
	},
#ifdef CONFIG_I2C_STM32_INTERRUPT
	.irq_config_func = i2c_stm32_irq_config_func_2,
#endif
};

static struct i2c_stm32_data i2c_stm32_dev_data_2 = {
	.dev_config.raw = CONFIG_I2C_2_DEFAULT_CFG,
};

DEVICE_AND_API_INIT(i2c_stm32_2, CONFIG_I2C_2_NAME, &i2c_stm32_init,
		    &i2c_stm32_dev_data_2, &i2c_stm32_cfg_2,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &api_funcs);

#ifdef CONFIG_I2C_STM32_INTERRUPT
static void i2c_stm32_irq_config_func_2(struct device *dev)
{
	IRQ_CONNECT(I2C2_EV_IRQn, CONFIG_I2C_2_IRQ_PRI,
		i2c_stm32_ev_isr, DEVICE_GET(i2c_stm32_2), 0);
	irq_enable(I2C2_EV_IRQn);

	IRQ_CONNECT(I2C2_ER_IRQn, CONFIG_I2C_2_IRQ_PRI,
		i2c_stm32_er_isr, DEVICE_GET(i2c_stm32_2), 0);
	irq_enable(I2C2_ER_IRQn);
}
#endif

#endif /* CONFIG_I2C_2 */
