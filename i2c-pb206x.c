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

/* I2C controller info */
struct i2c_device_info {
	int id;
	u32 base;
	int tx_fifo_size;
	int rx_fifo_size;
};

static struct i2c_device_info pb206x_i2c_device_info[] = {
	{0, 0x00, 64, 256}, /* master 0 */
	{1, 0x0c, 64, 256}, /* master 1 */
	{2, 0x18, 64,  64}, /* master 2 */
	{3, 0x24, 64,  64}, /* master 3 */
	{4, 0x30, 64,  64}, /* master 4 */
	{5, 0x40, 64,  64}, /* master 5 */
};

/* timeout waiting for the controller to respond */
#define PB206X_I2C_TIMEOUT (msecs_to_jiffies(1000))

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

static void __iomem *pb206x_i2c_base = NULL;
static atomic_t pb206x_i2c_ref_count = ATOMIC_INIT(0);

struct pb206x_i2c_dev {
	struct device		*dev;
	void __iomem		*base;
	int			irq;
	int			id;
	struct completion	cmd_complete;
	u32			speed;
	u16			cmd_err;
	u8			*buf;
	size_t			buf_len;
	struct i2c_adapter	adapter;
	int			rx_fifo_size;
	int			tx_fifo_size;
};

static inline void pb206x_i2c_write_reg(struct pb206x_i2c_dev *i2c_dev,
		int reg, u8 val)
{
	__raw_writeb(val, i2c_dev->base + reg);
}

static inline u8 pb206x_i2c_read_reg(struct pb206x_i2c_dev *i2c_dev, int reg)
{
	return __raw_readb(i2c_dev->base + reg);
}

static int pb206x_i2c_init(struct pb206x_i2c_dev *dev)
{
	return 0;
}

static int find_device_id(struct pb206x_i2c_dev *dev)
{
	int i;
	struct i2c_device_info *info = pb206x_i2c_device_info;

	for (i = 0; i < ARRAY_SIZE(pb206x_i2c_device_info); i++) {
		if (info[i].base == (u32)dev->base)
			return info[i].id;
	}

	return -ENODEV;
}

/* waiting for bus busy */
static int pb206x_i2c_wait_for_bb(struct pb206x_i2c_dev *dev)
{
	unsigned long timeout;

	timeout = jiffies + PB206X_I2C_TIMEOUT;
	while(pb206x_i2c_read_reg(dev, 
				PB206X_I2C_MS_REG) & PB206X_I2C_MS_BUSY) {
		if (time_after(jiffies, timeout)) {
			dev_warn(dev->dev, "timeout waiting for bus ready\n");
			return -ETIMEDOUT;
		}
		msleep(1);
	}

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
	struct pb206x_i2c_dev *dev;
	struct i2c_pb206x_platform_data *pdata = NULL;
	struct i2c_adapter    *adap;
	struct resource       *mem, *irq, *ioarea = NULL;
	int ret;
	u32 speed = 0;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource\n");
		return -ENODEV;
	}

	if (atomic_read(&pb206x_i2c_ref_count) == 0) {
		ioarea = request_mem_region(mem->start & ~PAGE_MASK, PAGE_SIZE,
				pdev->name);
		if (!ioarea) {
			dev_err(&pdev->dev, "I2C region already claimed\n");
			return -EBUSY;
		}
	}

	dev = kzalloc(sizeof(struct pb206x_i2c_dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	if (pdev->dev.platform_data != NULL)
		pdata = pdev->dev.platform_data;

	if (pdata && pdata->platform_init) {
		ret = pdata->platform_init();
		dev_err(&pdev->dev, "failure platform_init\n");
		goto err_platform_init;
	}

	if (pdata && pdata->speed)
		speed = pdata->speed;
	else
		speed = 100; /* default speed */
	
	dev->speed = speed;
	dev->dev = &pdev->dev;
	dev->irq = irq->start;
	if (atomic_read(&pb206x_i2c_ref_count) == 0) {
		pb206x_i2c_base = ioremap(mem->start & ~PAGE_MASK, PAGE_SIZE);
		if (!pb206x_i2c_base) {
			ret = -ENOMEM;
			dev_err(&pdev->dev, "failure ioremap\n");
			goto err_ioremap;
		}
	}
	dev->base = pb206x_i2c_base + (mem->start & PAGE_MASK);

	dev->id = find_device_id(dev);
	if (dev->id < 0) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "no device\n");
		goto err_find_device_id;
	}
	dev->tx_fifo_size = pb206x_i2c_device_info[dev->id].tx_fifo_size;
	dev->rx_fifo_size = pb206x_i2c_device_info[dev->id].rx_fifo_size;
	
	platform_set_drvdata(pdev, dev);
			
	pb206x_i2c_init(dev);

	ret = request_irq(dev->irq, pb206x_i2c_isr, 0, pdev->name, dev);
	if (ret) {
		dev_err(dev->dev, "failure requesting irq %d\n", dev->irq);
		goto err_request_irq;
	}

	dev_info(dev->dev, "bus %d tx_fifo %dbytes rx_fifo %dbytes at %d kHz (master %d)\n",
			pdev->id, dev->tx_fifo_size, dev->rx_fifo_size, dev->speed, dev->id);

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strlcpy(adap->name, "PB206X I2C adapter", sizeof(adap->name));
	adap->algo = &pb206x_i2c_algo;
	adap->dev.parent = &pdev->dev;

	adap->nr = pdev->id;
	ret = i2c_add_numbered_adapter(adap);
	if (ret) {
		dev_err(dev->dev, "failure adding adaptuer\n");
		goto err_i2c_add_numbered_adapter;
	}

	atomic_inc(&pb206x_i2c_ref_count);

	return 0;


err_i2c_add_numbered_adapter:
	free_irq(dev->irq, dev);
err_request_irq:
	platform_set_drvdata(pdev, NULL);
err_find_device_id:
	iounmap(pb206x_i2c_base);
err_ioremap:
err_platform_init:
	kfree(dev);
err_kzalloc:
	if (!ioarea)
		release_mem_region(mem->start & ~PAGE_MASK, PAGE_SIZE);

	return ret;
}


static int __devexit pb206x_i2c_remove(struct platform_device *pdev)
{
	struct pb206x_i2c_dev *dev = platform_get_drvdata(pdev);
	struct resource       *mem;

	platform_set_drvdata(pdev, NULL);

	free_irq(dev->irq, dev);
	i2c_del_adapter(&dev->adapter);

	/* do something if needed */

	kfree(dev);

	if (atomic_dec_and_test(&pb206x_i2c_ref_count)) {
		iounmap(pb206x_i2c_base);
		mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		release_mem_region(mem->start & ~PAGE_MASK, PAGE_SIZE);
	}

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
