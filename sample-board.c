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
#include <asm/irq.h>
#include <asm/io.h>

#include <linux/i2c-pb206x-platform.h>

/* s5pc100 specific header */
#include <mach/map.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-bank-a1.h>
#include <plat/gpio-bank-b.h>
#include <plat/gpio-bank-g1.h>
#include <plat/gpio-bank-h1.h>
#include <plat/gpio-bank-h3.h>

#define SMC_PHY_BASE	0x88000000 /* SMC Bank1 */

/* FIXME */
#define I2C_0_GPIO	S5PC1XX_GPH1(2) // eint
#define I2C_1_GPIO	S5PC1XX_GPG1(0)
#define I2C_2_GPIO	S5PC1XX_GPH3(7) // eint
#define I2C_3_GPIO	S5PC1XX_GPA1(2)
#define I2C_4_GPIO	S5PC1XX_GPA1(0)
#define I2C_5_GPIO	S5PC1XX_GPB(0)

#define I2C_0_IRQ	IRQ_EINT(10)
#define I2C_1_IRQ	S3C_IRQ_GPIO(I2C_1_GPIO)
#define I2C_2_IRQ	IRQ_EINT(31)
#define I2C_3_IRQ	S3C_IRQ_GPIO(I2C_3_GPIO)
#define I2C_4_IRQ	S3C_IRQ_GPIO(I2C_4_GPIO)
#define I2C_5_IRQ	S3C_IRQ_GPIO(I2C_5_GPIO)

#define I2C_0_OFFSET	0x00
#define I2C_1_OFFSET	0x0c
#define I2C_2_OFFSET	0x18
#define I2C_3_OFFSET	0x24
#define I2C_4_OFFSET	0x30
#define I2C_5_OFFSET	0x40

static const char name[] = "i2c-pb206x";
/* reference clock */
#define PB206X_MAIN_CLOCK (19200) /* 19.2 MHz */

#define I2C_RESOURCE_BUILDER(base, irq)		\
	{					\
		.start = (base),		\
		.end   = (base) + PAGE_SIZE,	\
		.flags = IORESOURCE_MEM,	\
	},					\
	{					\
		.start = (irq),			\
		.flags = IORESOURCE_IRQ,	\
	},

static struct resource i2c_resources[][2] = {
	{ I2C_RESOURCE_BUILDER(SMC_PHY_BASE + I2C_0_OFFSET, I2C_0_IRQ) },
	{ I2C_RESOURCE_BUILDER(SMC_PHY_BASE + I2C_1_OFFSET, I2C_1_IRQ) },
	{ I2C_RESOURCE_BUILDER(SMC_PHY_BASE + I2C_2_OFFSET, I2C_2_IRQ) },
	{ I2C_RESOURCE_BUILDER(SMC_PHY_BASE + I2C_3_OFFSET, I2C_3_IRQ) },
	{ I2C_RESOURCE_BUILDER(SMC_PHY_BASE + I2C_4_OFFSET, I2C_4_IRQ) },
	{ I2C_RESOURCE_BUILDER(SMC_PHY_BASE + I2C_5_OFFSET, I2C_5_IRQ) },
};

#define I2C_DEV_DATA_BUILDER(func, _speed, pdn, reset, clock)	\
	{							\
		.platform_init = func,				\
		.speed = _speed,				\
		.gpio_pdn = pdn,				\
		.gpio_reset = reset,				\
		.external_main_clock = clock,			\
	}

#define I2C_DEV_BUILDER(bus_id, res, data)		\
	{						\
		.id      = (bus_id),			\
		.name    = name,			\
		.num_resources = ARRAY_SIZE(res),	\
		.resource = (res),			\
		.dev     = {				\
			.platform_data = (data),	\
		},					\
	}

/* 
 * SMC Bank 1 for PB206X I2C controller
 */
static int smc_configure(void) {
	const unsigned smc_phy_base = S5PC1XX_PA_SROMC;
	void __iomem *smc_base = NULL;
	unsigned long v;

	if (!request_mem_region(smc_phy_base, PAGE_SIZE, "SMC")) {
		printk("%s: iomem request error\n", __func__);
		return -EBUSY;
	}

	smc_base = ioremap(smc_phy_base, PAGE_SIZE);
	if (smc_base == NULL) {
		printk("%s: iomem mapping error\n", __func__);
		release_mem_region(smc_phy_base, PAGE_SIZE);
		return -ENOMEM;
	}

	v = readl(smc_base);
	v &= ~(0xf << 4);
	v |= (0 << 4) | // data width 8bit
	     (0 << 5) | // address mode (ignored when data width 8bit)
	     (0 << 6) | // disabled WAIT
	     (1 << 7);  // Using UB/LB
	writel(v, smc_base);

	v = (0x6 << 4)  | // acp: 6 clock
	    (0x4 << 8)  | // cah: 4 clock
	    (0x1 << 12) | // coh: 1 clock
	    (0xe << 16) | // acc: 14 clock
	    (0x4 << 24) | // cos: 4 clock
	    (0x0 << 28);  // acs: 0 clock
	writel(v, smc_base + 0x08); // bank1

	return 0;
}

static int gpio_configure(void) {
	int ret;

	ret = gpio_request(S5PC1XX_GPH1(1), "GPH1");
	if (ret) {
		printk("%s: gpio(GPH1(2) request error: %d\n", __func__, ret);
	}
	else {
		s3c_gpio_cfgpin(S5PC1XX_GPH1(1), S5PC1XX_GPH1_2_WAKEUP_INT_10);
		s3c_gpio_setpull(S5PC1XX_GPH1(1), S3C_GPIO_PULL_NONE);
	}

	ret = gpio_request(S5PC1XX_GPG1(0), "GPG1");
	if (ret) {
		printk("%s: gpio(GPG1(0) request error: %d\n", __func__, ret);
	}
	else {
		s3c_gpio_cfgpin(S5PC1XX_GPG1(0), S5PC1XX_GPG1_0_GPIO_INT12_0);
		s3c_gpio_setpull(S5PC1XX_GPG1(0), S3C_GPIO_PULL_NONE);
	}
		
	ret = gpio_request(S5PC1XX_GPH1(1), "GPH3");
	if (ret) {
		printk("%s: gpio(GPH3(7) request error: %d\n", __func__, ret);
	}
	else {
		s3c_gpio_cfgpin(S5PC1XX_GPH3(7), S5PC1XX_GPH3_7_WAKEUP_INT_31);
		s3c_gpio_setpull(S5PC1XX_GPH3(7), S3C_GPIO_PULL_NONE);
	}

	ret = gpio_request(S5PC1XX_GPA1(2), "GPA1");
	if (ret) {
		printk("%s: gpio(GPA1(2) request error: %d\n", __func__, ret);
	}
	else {
		s3c_gpio_cfgpin(S5PC1XX_GPA1(2), S5PC1XX_GPA1_2_GPIO_INT1_2);
		s3c_gpio_setpull(S5PC1XX_GPA1(2), S3C_GPIO_PULL_NONE);
	}

	ret = gpio_request(S5PC1XX_GPA1(0), "GPA1");
	if (ret) {
		printk("%s: gpio(GPA1(0) request error: %d\n", __func__, ret);
	}
	else {
		s3c_gpio_cfgpin(S5PC1XX_GPA1(0), S5PC1XX_GPA1_0_GPIO_INT1_0);
		s3c_gpio_setpull(S5PC1XX_GPA1(0), S3C_GPIO_PULL_NONE);
	}

	ret = gpio_request(S5PC1XX_GPB(0), "GPB0");
	if (ret) {
		printk("%s: gpio(GPB0(0) request error: %d\n", __func__, ret);
	}
	else {
		s3c_gpio_cfgpin(S5PC1XX_GPB(0), S5PC1XX_GPB0_GPIO_INT2_0);
		s3c_gpio_setpull(S5PC1XX_GPB(0), S3C_GPIO_PULL_NONE);
	}

	return 0;
}

static atomic_t init_called = ATOMIC_INIT(0);

static int platform_init(void)
{
	int ret;

	if (atomic_inc_and_test(&init_called)) {
		/* Init the EBI */
		ret = smc_configure();
		if (ret)
			return ret;

		/* Config the Pin and GPIOs */
		ret = gpio_configure();
		if (ret)
			return ret;
	}

	return 0;
}

static struct i2c_pb206x_platform_data platform_data[] = {
	I2C_DEV_DATA_BUILDER(platform_init, 400, -1, -1, -1),
	I2C_DEV_DATA_BUILDER(platform_init, 400, -1, -1, -1),
	I2C_DEV_DATA_BUILDER(platform_init, 400, -1, -1, -1),
	I2C_DEV_DATA_BUILDER(platform_init, 400, -1, -1, -1),
	I2C_DEV_DATA_BUILDER(platform_init, 400, -1, -1, -1),
	I2C_DEV_DATA_BUILDER(platform_init, 400, -1, -1, -1),
};

/* 
 * i2c bus 0, bus 1 are already used in S5PC100
 */
static struct platform_device i2c_devices[] = {
	I2C_DEV_BUILDER(2, i2c_resources[0], &platform_data[0]),
	I2C_DEV_BUILDER(3, i2c_resources[1], &platform_data[1]),
	I2C_DEV_BUILDER(4, i2c_resources[2], &platform_data[2]),
	I2C_DEV_BUILDER(5, i2c_resources[3], &platform_data[3]),
	I2C_DEV_BUILDER(6, i2c_resources[4], &platform_data[4]),
	I2C_DEV_BUILDER(7, i2c_resources[5], &platform_data[5]),
};

/*
 * master_id : 0 .. 5
 */
int __init pb206x_i2c_add_bus(int master_id)
{
	struct platform_device *pdev;
	struct i2c_pb206x_platform_data *pdata;

	pdev = &i2c_devices[master_id];
	pdata = pdev->dev.platform_data;
	/* do something if needed */

	return platform_device_register(pdev);
}
