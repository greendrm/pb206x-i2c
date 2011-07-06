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


/* I2C controller banks */
static const u32 i2c_bank_addrs[] = {
	0x00, /* master 0 */
	0x0c, /* master 1 */
	0x18, /* master 2 */
	0x24, /* master 3 */
	0x30, /* master 4 */
	0x40, /* master 5 */
};

/* I2C controller registers */
#define PB206X_I2C_FIFO_DATA_REG	0x00
#define PB206X_I2C_SAD_REG		0x01
#define PB206X_I2C_SSAH_REG		0x02
#define PB206X_I2C_SSAL_REG		0x03
#define PB206X_I2C_MC_REG		0x04
#define PB206X_I2C_MFS_REG		0x05
#define PB206X_I2C_MS_REG		0x06
#define PB206X_I2C_BC_REG		0x07
#define PB206X_I2C_CDH_REG		0x08
#define PB206X_I2C_CDL_REG		0x09
#define PB206X_I2C_PUPDEN_REG		0x0a
#define PB206X_I2C_PCON_REG		0x0b

/* I2C Master Control Register (PB206X_I2C_MC): */
#define PB206X_I2C_MC_RESET		(1 << 6)
#define PB206X_I2C_MC_ABORT		(1 << 5)
#define PB206X_I2C_MC_INT_CLR		(1 << 4)
#define PB206X_I2C_MC_RXFIFO_CLR	(1 << 3)
#define PB206X_I2C_MC_TXFIFO_CLR	(1 << 2)
#define PB206X_I2C_MC_RW_MODE		(1 << 1)
#define PB206X_I2C_MC_START		(1 << 0)

/* I2C Master Mode/FIFO Status Register (PB206X_I2C_MFS): */
#define PB206X_I2C_MFS_INT_POL		(1 << 7)
#define PB206X_I2C_MFS_TX_IE		(1 << 5)
#define PB206X_I2C_MFS_RX_IE		(1 << 4)
#define PB206X_I2C_MFS_RXFULL		(1 << 3)
#define PB206X_I2C_MFS_RXEMPTY		(1 << 2)
#define PB206X_I2C_MFS_TXFULL		(1 << 1)
#define PB206X_I2C_MFS_NEXT_TX_RDY	(1 << 0)

/* I2C Master Status Register (PB206X_I2C_MS): */
#define PB206X_I2C_MS_BUSY		(1 << 7)
#define PB206X_I2C_MS_TX_DONE		(1 << 6)
#define PB206X_I2C_MS_RX_DONE		(1 << 5)
#define PB206X_I2C_MS_ABORT_DONE	(1 << 4)
#define PB206X_I2C_MS_CMD_ERR		(1 << 3)
#define PB206X_I2C_MS_TX_ERR		(1 << 2)
#define PB206X_I2C_MS_RX_ERR		(1 << 1)
#define PB206X_I2C_MS_TIMEOUT		(1 << 0)

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

static int pb206x_i2c_init(struct pb206x_i2c_dev *dev)
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
