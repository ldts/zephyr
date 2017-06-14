/*
 * Copyright (c) 2017, I-SENSE group of ICCS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <clock_control/stm32_clock_control.h>
#include <clock_control.h>
#include <misc/util.h>
#include <kernel.h>
#include <board.h>
#include <errno.h>
#include <i2c.h>

#include "i2c_ll_stm32f4.h"

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_I2C_LEVEL
#include <logging/sys_log.h>

#define I2C_REQUEST_WRITE	0x00
#define I2C_REQUEST_READ	0x01
#define HEADER			0xF0

#define DEV_CFG(dev)							\
	((const struct i2c_stm32f4_config * const)(dev)->config->config_info)

#define DEV_DATA(dev)							\
	((struct i2c_stm32f4_data * const)(dev)->driver_data)

#define NEXT(p, len)							\
	do { p++; len--; } while(0)

#ifdef CONFIG_I2C_STM32F4_INTERRUPT
static inline void handle_sb(struct i2c_stm32f4_data *data, I2C_TypeDef *i2c)
{
	u16_t saddr = data->slave_address;

	if (data->dev_config.bits.use_10_bit_addr) {
		u8_t slave = (((saddr & 0x0300) >> 7) & 0xFF);
		u8_t header = slave | HEADER;

		if (data->current.is_restart == 0) {
			data->current.is_restart = 1;
		} else {
			header |= I2C_REQUEST_READ;
			data->current.is_restart = 0;
		}
		LL_I2C_TransmitData8(i2c, header);

		return;
	}

	slave = (saddr << 1) & 0xFF;
	if (data->current.is_write) {
		LL_I2C_TransmitData8(i2c, slave |I2C_REQUEST_WRITE);
	} else {
		LL_I2C_TransmitData8(i2c, slave |I2C_REQUEST_READ);
	}
}

static inline void handle_addr(struct i2c_stm32f4_data *data, I2C_TypeDef *i2c)
{
	if (data->dev_config.bits.use_10_bit_addr) {
		if (!data->current.is_write && data->current.is_restart) {
			data->current.is_restart = 0;
			LL_I2C_ClearFlag_ADDR(i2c);
			LL_I2C_GenerateStartCondition(i2c);

			return;
		}
	}

	if (!data->current.is_write && data->current.len == 1) {
		LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);
	}
	LL_I2C_ClearFlag_ADDR(i2c);
}

static inline void handle_txe(struct i2c_stm32f4_data *data, I2C_TypeDef *i2c)
{
	if (data->current.len) {
		LL_I2C_TransmitData8(i2c, *data->current.buf);
		NEXT(data->current.buf, data->current.len);
	} else if (LL_I2C_IsActiveFlag_BTF(i2c) && !data->current.len) {
		if ((data->current.flags & I2C_MSG_RESTART) == 0) {
			LL_I2C_GenerateStopCondition(i2c);
		}
		k_sem_give(&data->sync);
	}
}

static inline void handle_rxne(struct i2c_stm32f4_data *data, I2C_TypeDef *i2c)
{
	if (data->current.len) {
		*data->current.buf = LL_I2C_ReceiveData8(i2c);
		NEXT(data->current.buf, data->current.len);
		if (data->current.len == 1) {
			LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);
			if ((data->current.flags & I2C_MSG_RESTART) == 0) {
				LL_I2C_GenerateStopCondition(i2c);
			}
			k_sem_give(&data->sync);
		}

		return;
	}

	data->current.is_err = 1;
	k_sem_give(&data->sync);
}

static void i2c_stm32f4_ev_isr(void *arg)
{
	const struct i2c_stm32f4_config *cfg = DEV_CFG(struct device *)arg);
	struct i2c_stm32f4_data *data = DEV_DATA((struct device *)arg);
	I2C_TypeDef *i2c = cfg->i2c;

	if (LL_I2C_IsActiveFlag_SB(i2c)) {
		handle_sb(i2c, data);
	} else if (LL_I2C_IsActiveFlag_ADD10(i2c)) {
		LL_I2C_TransmitData8(i2c, slave_address);
	} else if (LL_I2C_IsActiveFlag_ADDR(i2c)) {
		handle_addr(i2c, data);
	} else if (LL_I2C_IsActiveFlag_TXE(i2c)) {
		handle_txe(i2c, data);
	} else if (LL_I2C_IsActiveFlag_RXNE(i2c)) {
		handle_rxne(i2c, data);
	}
}

static void i2c_stm32f4_er_isr(void *arg)
{
	const struct i2c_stm32f4_config *cfg = DEV_CFG(struct device *)arg);
	struct i2c_stm32f4_data *data = DEV_DATA((struct device *)arg);
	I2C_TypeDef *i2c = cfg->i2c;

	if (LL_I2C_IsActiveFlag_AF(i2c)) {
		LL_I2C_ClearFlag_AF(i2c);
		data->current.is_nack = 1;
		k_sem_give(&data->sync);

		return;
	}

	data->current.is_err = 1;
	k_sem_give(&data->sync);
}

static inline s32_t msg_write(struct device *dev, struct i2c_msg *msg,
			      u32_t flags)
{
	const struct i2c_stm32f4_config *cfg = DEV_CFG(dev);
	struct i2c_stm32f4_data *data = DEV_DATA(dev);
	I2C_TypeDef *i2c = cfg->i2c;
	s32_t ret = 0;

	LL_I2C_EnableIT_EVT(i2c);
	LL_I2C_EnableIT_ERR(i2c);

	data->current.len = msg->len;
	data->current.buf = msg->buf;
	data->current.flags = flags;

	data->current.is_restart = 0;
	data->current.is_write = 1;
	data->current.is_nack = 0;
	data->current.is_err = 0;

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
	LL_I2C_GenerateStartCondition(i2c);

	LL_I2C_EnableIT_TX(i2c);
	LL_I2C_EnableIT_ERR(i2c);
	k_sem_take(&data->sync, K_FOREVER);

	if (data->current.is_nack || data->current.is_err) {
		LL_I2C_DisableIT_TX(i2c);
		LL_I2C_DisableIT_ERR(i2c);

		if (data->current.is_nack)
			SYS_LOG_DBG("%s: NACK", __func__);

		if (data->current.is_err)
			SYS_LOG_DBG("%s: ERR %d", __func__,
				    data->current.is_err);

		data->current.is_nack = 0;
		data->current.is_err = 0;

		ret = -EIO;
		goto error;
	}

	LL_I2C_DisableIT_TX(i2c);
	LL_I2C_DisableIT_ERR(i2c);

error:
	LL_I2C_DisableIT_EVT(i2c);
	LL_I2C_DisableIT_ERR(i2c);

	return ret;
}

static inline s32_t msg_read(struct device *dev, struct i2c_msg *msg,
			     u32_t flags)
{
	const struct i2c_stm32f4_config *cfg = DEV_CFG(dev);
	struct i2c_stm32f4_data *data = DEV_DATA(dev);
	I2C_TypeDef *i2c = cfg->i2c;
	s32_t ret = 0;


	LL_I2C_EnableIT_EVT(i2c);
	LL_I2C_EnableIT_ERR(i2c);

	data->current.len = msg->len;
	data->current.buf = msg->buf;
	data->current.flags = flags;

	data->current.is_restart = 0;
	data->current.is_write = 0;
	data->current.is_err = 0;

	LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);
	LL_I2C_GenerateStartCondition(i2c);


	LL_I2C_EnableIT_RX(i2c);
	k_sem_take(&data->sync, K_FOREVER);

	if (data->current.is_err) {
		LL_I2C_DisableIT_RX(i2c);
		SYS_LOG_DBG("%s: ERR %d", __func__, data->current.is_err);
		data->current.is_err = 0;

		ret = -EIO;
		goto error;
	}

	LL_I2C_DisableIT_RX(i2c);

error:
	LL_I2C_DisableIT_EVT(i2c);
	LL_I2C_DisableIT_ERR(i2c);

	return ret;
}
#endif

static inline s32_t msg_write(struct device *dev, struct i2c_msg *msg,
			      u32_t flags)
{
	const struct i2c_stm32f4_config *cfg = DEV_CFG(dev);
	struct i2c_stm32f4_data *data = DEV_DATA(dev);
	I2C_TypeDef *i2c = cfg->i2c;
	u32_t len = msg->len;
	u8_t *buf = msg->buf;

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
	LL_I2C_GenerateStartCondition(i2c);

	while (!LL_I2C_IsActiveFlag_SB(i2c)) {
		;
	}

	if (data->dev_config.bits.use_10_bit_addr) {
		u8_t slave = (((data->slave_address & 0x0300) >> 7) & 0xFF);
		u8_t header = slave | HEADER;

		LL_I2C_TransmitData8(i2c, header);
		while (!LL_I2C_IsActiveFlag_ADD10(i2c)) {
			;
		}
		slave = data->slave_address & 0xFF;
		LL_I2C_TransmitData8(i2c, slave);
	} else {
		u8_t slave = ((data->slave_address) << 1) & 0xFF;

		LL_I2C_TransmitData8(i2c, slave | I2C_REQUEST_WRITE);
	}
	while (!LL_I2C_IsActiveFlag_ADDR(i2c)) {
		;
	}

	LL_I2C_ClearFlag_ADDR(i2c);
	while (len) {
		while(1) {
			if (LL_I2C_IsActiveFlag_TXE(i2c)) {
				break;
			}

			if (LL_I2C_IsActiveFlag_AF(i2c)) {
				LL_I2C_ClearFlag_AF(i2c);
				SYS_LOG_DBG("%s: NACK", __func__);

				return -EIO;
			}
		};

		LL_I2C_TransmitData8(i2c, *buf);
		NEXT(buf, len);
	}

	while (!LL_I2C_IsActiveFlag_BTF(i2c)) {
		;
	}

	if ((flags & I2C_MSG_RESTART) == 0) {
		LL_I2C_GenerateStopCondition(i2c);
	}

	return 0;
}

static inline s32_t msg_read(struct device *dev, struct i2c_msg *msg,
			     u32_t flags)
{
	const struct i2c_stm32f4_config *cfg = DEV_CFG(dev);
	struct i2c_stm32f4_data *data = DEV_DATA(dev);
	I2C_TypeDef *i2c = cfg->i2c;
	u32_t len = msg->len;
	u8_t *buf = msg->buf;

	LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);
	LL_I2C_GenerateStartCondition(i2c);
	while (!LL_I2C_IsActiveFlag_SB(i2c)) {
		;
	}

	if (data->dev_config.bits.use_10_bit_addr) {
		u8_t slave = (((data->slave_address &	0x0300) >> 7) & 0xFF);
		u8_t header = slave | HEADER;

		LL_I2C_TransmitData8(i2c, header);
		while (!LL_I2C_IsActiveFlag_ADD10(i2c)) {
			;
		}

		slave = data->slave_address & 0xFF;
		LL_I2C_TransmitData8(i2c, slave);
		while (!LL_I2C_IsActiveFlag_ADDR(i2c)) {
			;
		}

		LL_I2C_ClearFlag_ADDR(i2c);
		LL_I2C_GenerateStartCondition(i2c);
		while (!LL_I2C_IsActiveFlag_SB(i2c)) {
			;
		}

		header |= I2C_REQUEST_READ;
		LL_I2C_TransmitData8(i2c, header);
	} else {
		u8_t slave = ((data->slave_address) << 1) & 0xFF;

		LL_I2C_TransmitData8(i2c, slave | I2C_REQUEST_READ);
	}

	while (!LL_I2C_IsActiveFlag_ADDR(i2c)) {
		;
	}

	if (len == 1) {
		LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);
	}

	LL_I2C_ClearFlag_ADDR(i2c);
	while (len) {
		while (!LL_I2C_IsActiveFlag_RXNE(i2c)){
			;
		}
		*buf = LL_I2C_ReceiveData8(i2c);
		NEXT(buf, len);
		if (len == 1) {
			LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
			if ((flags & I2C_MSG_RESTART) == 0) {
				LL_I2C_GenerateStopCondition(i2c);
			}
		}
	}

	return 0;
}

#define OPERATION(msg) (((struct i2c_msg *) msg)->flags & I2C_MSG_RW_MASK)

static s32_t i2c_stm32f4_transfer(struct device *dev, struct i2c_msg *message,
				u8_t messages, u16_t slave)
{
	const struct i2c_stm32f4_config *cfg = DEV_CFG(dev);
	struct i2c_stm32f4_data *data = DEV_DATA(dev);
	struct i2c_msg *current, *next;
	I2C_TypeDef *i2c = cfg->i2c;
	s32_t ret = 0;

	data->slave_address = slave_address;

	LL_I2C_Enable(i2c);

	current = message;
	while (messages > 0) {
		u32_t flags = 0;

		if (current->len > 255) {
			return -EINVAL;
		}

		next = current++;

		if ((OPERATION(current) != OPERATION(next)) && messages > 1)
			flags = I2C_MSG_RESTART;

		if ((current->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			ret = msg_write(dev, current, flags);
		} else {
			ret = msg_read(dev, current, flags);
		}

		if (ret < 0) {
			break;
		}

		NEXT(current, messages);
	};

	LL_I2C_Disable(i2c);

	return ret;
}

static s32_t i2c_stm32f4_configure(struct device *dev, u32_t config)
{
	const struct i2c_stm32f4_config *cfg = DEV_CFG(dev);
	struct i2c_stm32f4_data *data = DEV_DATA(dev);
	I2C_TypeDef *i2c = cfg->i2c;
	u32_t address_size;
	u32_t clock;

	if (data->dev_config.bits.is_slave_read) {
		return -EINVAL;
	}

	data->dev_config.raw = config;

	clock_control_get_rate(device_get_binding(STM32_CLOCK_CONTROL_NAME),
			(clock_control_subsys_t *)&cfg->pclken, &clock);

	LL_I2C_Disable(i2c);
	LL_I2C_SetMode(i2c, LL_I2C_MODE_I2C);

	switch (data->dev_config.bits.speed) {
	case I2C_SPEED_STANDARD:
		LL_I2C_ConfigSpeed(i2c, clock, 100000, LL_I2C_DUTYCYCLE_2);
		break;
	case I2C_SPEED_FAST:
		LL_I2C_ConfigSpeed(i2c, clock, 400000, LL_I2C_DUTYCYCLE_2);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct i2c_driver_api api_funcs = {
	.configure = i2c_stm32f4_configure,
	.transfer = i2c_stm32f4_transfer,
};

static s32_t i2c_stm32f4_init(struct device *dev)
{
	struct device *clk = device_get_binding(STM32_CLOCK_CONTROL_NAME);
	const struct i2c_stm32f4_config *cfg = DEV_CFG(dev);
	struct i2c_stm32f4_data *data = DEV_DATA(dev);

	__ASSERT_NO_MSG(clk);
	clock_control_on(clock, (clock_control_subsys_t *)&cfg->pclken);

#ifdef CONFIG_I2C_STM32F4_INTERRUPT
	k_sem_init(&data->sync, 0, UINT_MAX);
	cfg->irq_config_func(dev);
#endif

	return 0;
}

#ifdef CONFIG_I2C_1

#ifdef CONFIG_I2C_STM32LX_INTERRUPT
static void i2c_stm32lx_irq_config_func_1(struct device *port);
#endif

static const struct i2c_stm32f4_config i2c_stm32f4_cfg_1 = {
	.i2c = (I2C_TypeDef *) I2C1_BASE,
	.pclken = {
		.enr = LL_APB1_GRP1_PERIPH_I2C1
		.bus = STM32_CLOCK_BUS_APB1,
	},
#ifdef CONFIG_I2C_STM32F4_INTERRUPT
	.irq_config_func = i2c_stm32f4_irq_config_func_1,
#endif
};

static struct i2c_stm32f4_data i2c_stm32f4_dev_data_1 = {
	.dev_config.raw = CONFIG_I2C_1_DEFAULT_CFG,
};

DEVICE_AND_API_INIT(i2c_stm32f4_1, CONFIG_I2C_1_NAME, &i2c_stm32f4_init,
		    &i2c_stm32f4_dev_data_1, &i2c_stm32f4_cfg_1,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &api_funcs);

#ifdef CONFIG_I2C_STM32F4_INTERRUPT
static void i2c_stm32f4_irq_config_func_1(struct device *dev)
{
	IRQ_CONNECT(I2C1_EV_IRQn, CONFIG_I2C_1_IRQ_PRI,
		i2c_stm32f4_ev_isr, DEVICE_GET(i2c_stm32f4_1), 0);
	irq_enable(I2C1_EV_IRQn);

	IRQ_CONNECT(I2C1_ER_IRQn, CONFIG_I2C_1_IRQ_PRI,
		i2c_stm32f4_er_isr, DEVICE_GET(i2c_stm32f4_1), 0);
	irq_enable(I2C1_ER_IRQn);
}
#endif

#endif /* CONFIG_I2C_1 */

#ifdef CONFIG_I2C_2

#ifdef CONFIG_I2C_STM32LX_INTERRUPT
static void i2c_stm32lx_irq_config_func_2(struct device *port);
#endif

static const struct i2c_stm32f4_config i2c_stm32f4_cfg_2 = {
	.i2c = (I2C_TypeDef *) I2C2_BASE,
	.pclken = {
		.enr = LL_APB1_GRP1_PERIPH_I2C2
		.bus = STM32_CLOCK_BUS_APB1,
	},
#ifdef CONFIG_I2C_STM32F4_INTERRUPT
	.irq_config_func = i2c_stm32f4_irq_config_func_2,
#endif
};

static struct i2c_stm32f4_data i2c_stm32f4_dev_data_2 = {
	.dev_config.raw = CONFIG_I2C_2_DEFAULT_CFG,
};

DEVICE_AND_API_INIT(i2c_stm32f4_2, CONFIG_I2C_2_NAME, &i2c_stm32f4_init,
		    &i2c_stm32f4_dev_data_2, &i2c_stm32f4_cfg_2,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &api_funcs);

#ifdef CONFIG_I2C_STM32F4_INTERRUPT
static void i2c_stm32f4_irq_config_func_2(struct device *dev)
{
	IRQ_CONNECT(I2C2_EV_IRQn, CONFIG_I2C_2_IRQ_PRI,
		i2c_stm32f4_ev_isr, DEVICE_GET(i2c_stm32f4_2), 0);
	irq_enable(I2C2_EV_IRQn);

	IRQ_CONNECT(I2C2_ER_IRQn, CONFIG_I2C_2_IRQ_PRI,
		i2c_stm32f4_er_isr, DEVICE_GET(i2c_stm32f4_2), 0);
	irq_enable(I2C2_ER_IRQn);
}
#endif

#endif /* CONFIG_I2C_2 */

#ifdef CONFIG_I2C_3

#ifdef CONFIG_I2C_STM32LX_INTERRUPT
static void i2c_stm32lx_irq_config_func_3(struct device *port);
#endif

static const struct i2c_stm32f4_config i2c_stm32f4_cfg_3 = {
	.i2c = (I2C_TypeDef *) I2C3_BASE,
	.pclken = {
		.enr = LL_APB1_GRP1_PERIPH_I2C3
		.bus = STM32_CLOCK_BUS_APB1,
	},
#ifdef CONFIG_I2C_STM32F4_INTERRUPT
	.irq_config_func = i2c_stm32f4_irq_config_func_3,
#endif
};

static struct i2c_stm32f4_data i2c_stm32f4_dev_data_3 = {
	.dev_config.raw = CONFIG_I2C_3_DEFAULT_CFG,
};

DEVICE_AND_API_INIT(i2c_stm32f4_3, CONFIG_I2C_3_NAME, &i2c_stm32f4_init,
		    &i2c_stm32f4_dev_data_3, &i2c_stm32f4_cfg_3,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &api_funcs);

#ifdef CONFIG_I2C_STM32F4_INTERRUPT
static void i2c_stm32f4_irq_config_func_3(struct device *dev)
{
	IRQ_CONNECT(I2C3_EV_IRQn, CONFIG_I2C_3_IRQ_PRI,
		i2c_stm32f4_ev_isr, DEVICE_GET(i2c_stm32f4_3), 0);
	irq_enable(I2C3_EV_IRQn);

	IRQ_CONNECT(I2C3_ER_IRQn, CONFIG_I2C_3_IRQ_PRI,
		i2c_stm32f4_er_isr, DEVICE_GET(i2c_stm32f4_3), 0);
	irq_enable(I2C33_ER_IRQn);
}
#endif

#endif /* CONFIG_I2C_3 */
