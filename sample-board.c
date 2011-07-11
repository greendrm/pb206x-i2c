/* arch/arm/mach-s5pc100/i2c-pb206x.c
 * 
 * Copyright (C) 2011 Pointchips, inc.
 * greendrm@gmail.com 
 * 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 * 
 * This program is distributed in the hope that is will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABLILITY of FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Genernal Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <mach/irqs.h>

//#include <linux/i2c-pb206x-platform.h>
#include "i2c-pb206x-platform.h" /* temporary */

/* s5pc100 specific header */
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

#define EBI_PHY_BASE	0x00 /* FIXME */

/* FIXME */
#define I2C_0_GPIO	S5PC1XX_GPH1(2)
#define I2C_1_GPIO	S5PC1XX_GPG1(0)
#define I2C_2_GPIO	S5PC1XX_GPH3(7)
#define I2C_3_GPIO	S5PC1XX_GPA1(2)
#define I2C_4_GPIO	S5PC1XX_GPA1(0)
#define I2C_5_GPIO	S5PC1XX_GPB0(0)

#define I2C_0_IRQ	0
#define I2C_1_IRQ	0
#define I2C_2_IRQ	0
#define I2C_3_IRQ	0
#define I2C_4_IRQ	0
#define I2C_5_IRQ	0

#define I2C_0_OFFSET	0x00
#define I2C_1_OFFSET	0x0c
#define I2C_2_OFFSET	0x18
#define I2C_3_OFFSET	0x24
#define I2C_4_OFFSET	0x30
#define I2C_5_OFFSET	0x40

static const char name[] = "i2c-pb206x"
/* reference clock */
#define PB206X_MAIN_CLOCK (19200) /* 19.2 MHz */

#define I2C_RESOURCE_BUILDER(base, irq)		\
{						\
	.start = (base),			\
	.end   = (base) + PAGE_SIZE,		\
	.flags = IORESOURCE_MEM,		\
},						\
{						\
	.start = (irq),				\
	.flags = IORESOURCE_IRQ,		\
},

static struct resource i2c_resources[][2] = {
	{ I2C_RESOURCE_BUILDER(EBI_PHY_BASE + I2C_0_OFFSET, I2C_0_IRQ) },
	{ I2C_RESOURCE_BUILDER(EBI_PHY_BASE + I2C_1_OFFSET, I2C_1_IRQ) },
	{ I2C_RESOURCE_BUILDER(EBI_PHY_BASE + I2C_2_OFFSET, I2C_2_IRQ) },
	{ I2C_RESOURCE_BUILDER(EBI_PHY_BASE + I2C_3_OFFSET, I2C_3_IRQ) },
	{ I2C_RESOURCE_BUILDER(EBI_PHY_BASE + I2C_4_OFFSET, I2C_4_IRQ) },
	{ I2C_RESOURCE_BUILDER(EBI_PHY_BASE + I2C_5_OFFSET, I2C_5_IRQ) },
};

#define I2C_DEV_DATA_BUILDER(id, func, speed, pdn, reset, clock)	\
{								\
	.platform_init = func,					\
	.speed = speed,						\
	.gpio_pdn = pdn,					\
	.gpio_reset = reset,					\
	.external_main_clock = clock,				\
}

#define I2C_DEV_BUILDER(bus_id, res, data)	\
{						\
	.id      = (bus_id),			\
	.name    = name,			\
	.num_resources = ARRAY_SIZE(res),	\
	.resource = (res),			\
	.dev     = {				\
		.platform_data = (data),	\
	},					\
}

static int platform_init(void)
{
	/* Init the EBI */

	/* Config the Pin and GPIOs */

	return 0;
}

static struct i2c_pb206x_platform_data platform_data[] = {
	I2C_DEV_DATA_BUILDER(0, platform_init, 400, -1, -1, -1),
	I2C_DEV_DATA_BUILDER(1, platform_init, 400, -1, -1, -1),
	I2C_DEV_DATA_BUILDER(2, platform_init, 400, -1, -1, -1),
	I2C_DEV_DATA_BUILDER(3, platform_init, 400, -1, -1, -1),
	I2C_DEV_DATA_BUILDER(4, platform_init, 400, -1, -1, -1),
	I2C_DEV_DATA_BUILDER(5, platform_init, 400, -1, -1, -1),
};

static struct platform_device i2c_devices[] = {
	I2C_DEV_BUILDER(0, i2c_resources[0], &platform_data[0]),
	I2C_DEV_BUILDER(1, i2c_resources[1], &platform_data[1]),
	I2C_DEV_BUILDER(2, i2c_resources[2], &platform_data[2]),
	I2C_DEV_BUILDER(3, i2c_resources[3], &platform_data[3]),
	I2C_DEV_BUILDER(4, i2c_resources[4], &platform_data[4]),
	I2C_DEV_BUILDER(5, i2c_resources[5], &platform_data[5]),
};

int __init pb206x_i2c_add_bus(int bus_id)
{
	struct platform_device *pdev;
	struct i2c_pb206x_platform_data *pdata;

	pdev = &i2c_devices[bus_id];
	pdata = pdev->dev.platform_data;
	/* do something if needed */

	return platform_device_register(pdev);
}
