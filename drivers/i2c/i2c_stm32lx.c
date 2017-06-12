/*
 * Copyright (c) 2016 BayLibre, SAS
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
#include "i2c_stm32lx.h"

#ifdef CONFIG_SOC_SERIES_STM32L4X
#include "stm32l4xx_ll_i2c.h"
#endif

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_I2C_LEVEL
#include <logging/sys_log.h>

#define DEV_CFG(dev)							\
	((const struct i2c_stm32lx_config * const)(dev)->config->config_info)

#define DEV_DATA(dev)   						\
	((struct i2c_stm32lx_data * const)(dev)->driver_data)

#define NEXT(p, len)							\
	do { p++; len--; } while(0)

static inline void msg_init(struct device *dev, struct i2c_msg *msg,
			unsigned int flags, u16_t slave, uint32_t transfer)
{
	const struct i2c_stm32lx_config *cfg = DEV_CFG(dev);
	struct i2c_stm32lx_data *data = DEV_DATA(dev);
	I2C_TypeDef *i2c = cfg->i2c;
	unsigned int len = msg->len;

	if (data->dev_config.bits.use_10_bit_addr) {
		LL_I2C_SetMasterAddressingMode(i2c,
						LL_I2C_ADDRESSING_MODE_10BIT);
		LL_I2C_SetSlaveAddr(i2c, (uint32_t) slave);
	} else {
		LL_I2C_SetMasterAddressingMode(i2c,LL_I2C_ADDRESSING_MODE_7BIT);
		LL_I2C_SetSlaveAddr(i2c, (uint32_t) slave << 1);
	}

	LL_I2C_SetTransferRequest(i2c, transfer);
	LL_I2C_SetTransferSize(i2c, len);

	if ((flags & I2C_MSG_RESTART) == I2C_MSG_RESTART) {
		LL_I2C_DisableAutoEndMode(i2c);
	} else {
		LL_I2C_EnableAutoEndMode(i2c);
	}

	LL_I2C_DisableReloadMode(i2c);
	LL_I2C_GenerateStartCondition(i2c);
}

static inline void msg_done(struct device *dev, unsigned int flags)
{
	const struct i2c_stm32lx_config *cfg = DEV_CFG(dev);
	I2C_TypeDef *i2c = cfg->i2c;

	if ((flags & I2C_MSG_RESTART) == 0) {
		while (!LL_I2C_IsActiveFlag_STOP(i2c)) {
			;
		}
		LL_I2C_GenerateStopCondition(i2c);
	}
}

#ifdef CONFIG_I2C_STM32LX_INTERRUPT
static void i2c_stm32lx_ev_isr(void *arg)
{
	const struct i2c_stm32lx_config *cfg = DEV_CFG((struct device *)arg);
	struct i2c_stm32lx_data *data = DEV_DATA((struct device *)arg);
	I2C_TypeDef *i2c = cfg->i2c;

	if (data->current.is_write) {
		if (data->current.len && LL_I2C_IsEnabledIT_TX(i2c)) {
			LL_I2C_TransmitData8(i2c, *data->current.buf);
		} else {
			LL_I2C_DisableIT_TX(i2c);
			goto error;
		}
	} else {
		if (data->current.len && LL_I2C_IsEnabledIT_RX(i2c)) {
			*data->current.buf = LL_I2C_ReceiveData8(i2c);
		} else {
			LL_I2C_DisableIT_RX(i2c);
			goto error;
		}
	}

	NEXT(data->current.buf, data->current.len);
	if (!data->current.len)
		k_sem_give(&data->device_sync_sem);

	return;
error:
	data->current.is_err = 1;
	k_sem_give(&data->device_sync_sem);
}

static void i2c_stm32lx_er_isr(void *arg)
{
	const struct i2c_stm32lx_config *cfg = DEV_CFG((struct device *)arg);
	struct i2c_stm32lx_data *data = DEV_DATA((struct device *)arg);
	I2C_TypeDef *i2c = cfg->i2c;

	if (LL_I2C_IsActiveFlag_NACK(i2c)) {
		LL_I2C_ClearFlag_NACK(i2c);
		data->current.is_nack = 1;
	} else {
		data->current.is_err = 1;
	}

	k_sem_give(&data->device_sync_sem);
}

static inline int msg_write(struct device *dev, struct i2c_msg *msg,
			   unsigned int flags, uint16_t slave)
{
	const struct i2c_stm32lx_config *cfg = DEV_CFG(dev);
	struct i2c_stm32lx_data *data = DEV_DATA(dev);
	I2C_TypeDef *i2c = cfg->i2c;

	data->current.len = msg->len;
	data->current.buf = msg->buf;
	data->current.is_write = 1;
	data->current.is_nack = 0;
	data->current.is_err = 0;

	msg_init(dev, msg, flags, slave, LL_I2C_REQUEST_WRITE);
	LL_I2C_EnableIT_TX(i2c);
	LL_I2C_EnableIT_NACK(i2c);

	k_sem_take(&data->device_sync_sem, K_FOREVER);
	if (data->current.is_nack || data->current.is_err) {
		goto error;
	}

	msg_done(dev, flags);
	LL_I2C_DisableIT_TX(i2c);
	LL_I2C_DisableIT_NACK(i2c);

	return 0;
error:
	LL_I2C_DisableIT_TX(i2c);
	LL_I2C_DisableIT_NACK(i2c);

	if (data->current.is_nack) {
		SYS_LOG_DBG("%s: NACK", __func__);
		data->current.is_nack = 0;
	}

	if (data->current.is_err) {
		SYS_LOG_DBG("%s: ERR %d", __func__,
				    data->current.is_err);
		data->current.is_err = 0;
	}

	return -EIO;
}

static inline int msg_read(struct device *dev, struct i2c_msg *msg,
			   unsigned int flags, uint16_t slave)
{
	const struct i2c_stm32lx_config *cfg = DEV_CFG(dev);
	struct i2c_stm32lx_data *data = DEV_DATA(dev);
	I2C_TypeDef *i2c = cfg->i2c;

	data->current.len = msg->len;
	data->current.buf = msg->buf;
	data->current.is_write = 0;
	data->current.is_err = 0;

	msg_init(dev, msg, flags, slave, LL_I2C_REQUEST_READ);
	LL_I2C_EnableIT_RX(i2c);

	k_sem_take(&data->device_sync_sem, K_FOREVER);
	if (data->current.is_err) {
		goto error;
	}

	msg_done(dev, flags);
	LL_I2C_DisableIT_RX(i2c);

	return 0;
error:
	LL_I2C_DisableIT_RX(i2c);
	SYS_LOG_DBG("%s: ERR %d", __func__, data->current.is_err);
	data->current.is_err = 0;

	return -EIO;
}

#else /* !CONFIG_I2C_STM32LX_INTERRUPT */
static inline int msg_write(struct device *dev, struct i2c_msg *msg,
			   unsigned int flags, uint16_t slave)
{
	const struct i2c_stm32lx_config *cfg = DEV_CFG(dev);
	I2C_TypeDef *i2c = cfg->i2c;
	unsigned int len = msg->len;
	u8_t *buf = msg->buf;

	msg_init(dev, msg, flags, slave, LL_I2C_REQUEST_WRITE);

	while (len) {
		while (1) {
			if (LL_I2C_IsActiveFlag_TXIS(i2c)) {
				break;
			}

			if (LL_I2C_IsActiveFlag_NACK(i2c)) {
				goto error;
			}
		}

		LL_I2C_TransmitData8(i2c, *buf);
		NEXT(buf, len);
	}

	msg_done(dev, flags);

	return 0;
error:
	LL_I2C_ClearFlag_NACK(i2c);
	SYS_LOG_DBG("%s: NACK", __func__);

	return -EIO;
}

static inline int msg_read(struct device *dev, struct i2c_msg *msg,
			   unsigned int flags, uint16_t slave)
{
	const struct i2c_stm32lx_config *cfg = DEV_CFG(dev);
	I2C_TypeDef *i2c = cfg->i2c;
	unsigned int len = msg->len;
	u8_t *buf = msg->buf;

	msg_init(dev, msg, flags, slave, LL_I2C_REQUEST_READ);

	while (len) {
		while (!LL_I2C_IsActiveFlag_RXNE(i2c)) {
			;
		}

		*buf = LL_I2C_ReceiveData8(i2c);
		NEXT(buf, len);
	}

	msg_done(dev, flags);

	return 0;
}
#endif

static int i2c_stm32lx_runtime_configure(struct device *dev, u32_t config)
{
	const struct i2c_stm32lx_config *cfg = DEV_CFG(dev);
	struct i2c_stm32lx_data *data = DEV_DATA(dev);
	I2C_TypeDef *i2c = cfg->i2c;
	u32_t i2c_hold_time_min, i2c_setup_time_min;
	u32_t i2c_h_min_time, i2c_l_min_time;
	u32_t timing, clock;
	u32_t presc = 1;

	if (data->dev_config.bits.is_slave_read)
		return -EINVAL;

	data->dev_config.raw = config;

	clock_control_get_rate(device_get_binding(STM32_CLOCK_CONTROL_NAME),
			(clock_control_subsys_t *)&cfg->pclken, &clock);

	LL_I2C_Disable(i2c);
	LL_I2C_SetMode(i2c, LL_I2C_MODE_I2C);

	switch (data->dev_config.bits.speed) {
	case I2C_SPEED_STANDARD:
		i2c_h_min_time = 4000;
		i2c_l_min_time = 4700;
		i2c_hold_time_min = 500;
		i2c_setup_time_min = 1250;
		break;
	case I2C_SPEED_FAST:
		i2c_h_min_time = 600;
		i2c_l_min_time = 1300;
		i2c_hold_time_min = 375;
		i2c_setup_time_min = 500;
		break;
	default:
		return -EINVAL;
	}

	/* Calculate period until prescaler matches */
	do {
		u32_t t_presc = clock / presc;
		u32_t ns_presc = NSEC_PER_SEC / t_presc;
		u32_t sclh = i2c_h_min_time / ns_presc;
		u32_t scll = i2c_l_min_time / ns_presc;
		u32_t sdadel = i2c_hold_time_min / ns_presc;
		u32_t scldel = i2c_setup_time_min / ns_presc;

		if ((sclh - 1) > 255 ||  (scll - 1) > 255) {
			++presc;
			continue;
		}

		if (sdadel > 15 || (scldel - 1) > 15) {
			++presc;
			continue;
		}

		timing = __LL_I2C_CONVERT_TIMINGS(presc - 1,
					scldel - 1, sdadel, sclh - 1, scll - 1);
		break;
	} while (presc < 16);

	if (presc >= 16) {
		SYS_LOG_DBG("I2C:failed to find prescaler value");
		return -EINVAL;
	}

	LL_I2C_SetTiming(i2c, timing);

	return 0;
}

#define OPERATION(msg) (((struct i2c_msg *) msg)->flags & I2C_MSG_RW_MASK)

static int i2c_stm32lx_transfer(struct device *dev, struct i2c_msg *message,
				u8_t messages, u16_t slave)
{
	const struct i2c_stm32lx_config *cfg = DEV_CFG(dev);
	struct i2c_msg *current, *next;
	I2C_TypeDef *i2c = cfg->i2c;
	int ret = 0;

	LL_I2C_Enable(i2c);

	current = message;
	while (messages > 0) {
		unsigned int flags = 0;

		if (current->len > 255)
			return -EINVAL;

		next = current++;

		if ((OPERATION(current) != OPERATION(next)) && messages > 1)
			flags = I2C_MSG_RESTART;

		if ((current->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			ret = msg_write(dev, current, flags, slave);
		} else {
			ret = msg_read(dev, current, flags, slave);
		}

		if (ret < 0) {
			break;
		}

		NEXT(current, messages);
	};

	LL_I2C_Disable(i2c);

	return ret;
}

static const struct i2c_driver_api api_funcs = {
	.configure = i2c_stm32lx_runtime_configure,
	.transfer = i2c_stm32lx_transfer,
};

static int i2c_stm32lx_init(struct device *dev)
{
	struct device *clock = device_get_binding(STM32_CLOCK_CONTROL_NAME);
	const struct i2c_stm32lx_config *cfg = DEV_CFG(dev);
	struct i2c_stm32lx_data *data = DEV_DATA(dev);

	__ASSERT_NO_MSG(clock);
	clock_control_on(clock, (clock_control_subsys_t *) &cfg->pclken);

	i2c_stm32lx_runtime_configure(dev, data->dev_config.raw);

#ifdef CONFIG_I2C_STM32LX_INTERRUPT
	k_sem_init(&data->device_sync_sem, 0, UINT_MAX);
	cfg->irq_config_func(dev);
#endif
	return 0;
}

#ifdef CONFIG_I2C_1

#ifdef CONFIG_I2C_STM32LX_INTERRUPT
static void i2c_stm32lx_irq_config_func_1(struct device *port);
#endif

static const struct i2c_stm32lx_config i2c_stm32lx_cfg_1 = {
	.i2c = (I2C_TypeDef *) I2C1_BASE,
	.pclken = {
		.bus = STM32_CLOCK_BUS_APB1,
		.enr = LL_APB1_GRP1_PERIPH_I2C1
	},
#ifdef CONFIG_I2C_STM32LX_INTERRUPT
	.irq_config_func = i2c_stm32lx_irq_config_func_1,
#endif
};

static struct i2c_stm32lx_data i2c_stm32lx_dev_data_1 = {
	.dev_config.raw = CONFIG_I2C_1_DEFAULT_CFG,
};

DEVICE_AND_API_INIT(i2c_stm32lx_1, CONFIG_I2C_1_NAME, &i2c_stm32lx_init,
		    &i2c_stm32lx_dev_data_1, &i2c_stm32lx_cfg_1,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &api_funcs);

#ifdef CONFIG_I2C_STM32LX_INTERRUPT
static void i2c_stm32lx_irq_config_func_1(struct device *dev)
{
	IRQ_CONNECT(I2C1_EV_IRQn, CONFIG_I2C_1_IRQ_PRI,
		i2c_stm32lx_ev_isr, DEVICE_GET(i2c_stm32lx_1), 0);
	irq_enable(I2C1_EV_IRQn);

	IRQ_CONNECT(I2C1_ER_IRQn, CONFIG_I2C_1_IRQ_PRI,
		i2c_stm32lx_er_isr, DEVICE_GET(i2c_stm32lx_1), 0);
	irq_enable(I2C1_ER_IRQn);
}
#endif

#endif /* CONFIG_I2C_1 */

#ifdef CONFIG_I2C_2

#ifdef CONFIG_I2C_STM32LX_INTERRUPT
static void i2c_stm32lx_irq_config_func_2(struct device *port);
#endif

static const struct i2c_stm32lx_config i2c_stm32lx_cfg_2 = {
	.i2c = (I2C_TypeDef *) I2C2_BASE,
	.pclken = {
		.bus = STM32_CLOCK_BUS_APB1,
		.enr = LL_APB1_GRP1_PERIPH_I2C2
	},
#ifdef CONFIG_I2C_STM32LX_INTERRUPT
	.irq_config_func = i2c_stm32lx_irq_config_func_2,
#endif
};

static struct i2c_stm32lx_data i2c_stm32lx_dev_data_2 = {
	.dev_config.raw = CONFIG_I2C_2_DEFAULT_CFG,
};

DEVICE_AND_API_INIT(i2c_stm32lx_2, CONFIG_I2C_2_NAME, &i2c_stm32lx_init,
		    &i2c_stm32lx_dev_data_2, &i2c_stm32lx_cfg_2,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &api_funcs);

#ifdef CONFIG_I2C_STM32LX_INTERRUPT
static void i2c_stm32lx_irq_config_func_2(struct device *dev)
{
	IRQ_CONNECT(I2C2_EV_IRQn, CONFIG_I2C_2_IRQ_PRI,
		i2c_stm32lx_ev_isr, DEVICE_GET(i2c_stm32lx_2), 0);
	irq_enable(I2C2_EV_IRQn);

	IRQ_CONNECT(I2C2_ER_IRQn, CONFIG_I2C_2_IRQ_PRI,
		i2c_stm32lx_er_isr, DEVICE_GET(i2c_stm32lx_2), 0);
	irq_enable(I2C2_ER_IRQn);
}
#endif

#endif /* CONFIG_I2C_2 */
