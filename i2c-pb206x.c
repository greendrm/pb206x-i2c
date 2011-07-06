/* drivers/i2c/busses/i2c-pb206x.c
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/io.h>

//#include <linux/i2c-pb206x-platform.h>
#include "i2c-pb206x-platform.h" /* temporary */


/* I2C controller registers */

struct pb206x_i2c_dev {
	struct device		*dev;
	void __iomem		*base;
	int			irq;
	struct completion	cmd_complete;
	struct resource		*ioarea;
	u32			speed;
	u16			cmd_err;
	u8			*buf;
	size_t			buf_len;
	struct i2c_adapter	adapter;
	u8			fifo_size;
};

static inline void pb206x_i2c_write_reg(struct pb206x_i2c_dev *i2c_dev,
		int reg, u8 val)
{
	__raw_writeb(val, i2c_dev->base + reg);
}

static inline void pb206x_i2c_read_reg(struct pb206x_i2c_dev *i2c_dev, int reg)
{
	__raw_readb(i2c_dev->base + reg);
}

static void pb206x_i2c_init(struct pb206x_i2c_dev *dev)
{
	return 0;
}

/* waiting for bus busy */
static int pb206x_i2c_wait_for_bb(struct pb206x_i2c_dev *dev)
{
	return 0;
}

static int pb206x_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
		int num)
{
	return 0;
}

static u32 pb206x_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static inline void pb206x_i2c_complete_cmd(struct pb206x_i2c_dev *dev, u16 err)
{
	dev->cmd_err |= err;
	complete(&dev->cmd_complete);
}

static irqreturn_t pb206x_i2c_isr(int this_irq, void *dev_id)
{
	int count = 0;
	return count ? IRQ_HANDLED : IRQ_NONE;
}

static const struct i2c_algorithm pb206x_i2c_algo = {
	.master_xfer	= pb206x_i2c_xfer,
	.functionality	= pb206x_i2c_func,
};

static int __devinit pb206x_i2c_probe(struct platform_device *pdev)
{
	return 0;
}


static int __devexit pb206x_i2c_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver pb206x_i2c_driver = {
	.driver = {
		.name  = "i2c-pb206x",
		.owner = THIS_MODULE,
	},
	.probe = pb206x_i2c_probe,
	.remove = __devexit_p(pb206x_i2c_remove),
};
	

/* I2C may be needed to bring up other drivers */
static int __init pb206x_i2c_init_driver(void)
{
	return platform_driver_register(&pb206x_i2c_driver);
}

static void __exit pb206x_i2c_exit_driver(void)
{
	platform_driver_unregister(&pb206x_i2c_driver);
}

module_init(pb206x_i2c_init_driver);
module_exit(pb206x_i2c_exit_driver);

MODULE_AUTHOR("Pointchips, Inc.");
MODULE_DESCRIPTION("Pointchips pb206x i2c adapter");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c-pb206x");
