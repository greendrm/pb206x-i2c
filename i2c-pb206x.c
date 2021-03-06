/* drivers/i2c/busses/i2c-pb206x.c
 * 
 * Copyright (C) 2011 Pointchips, inc.
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
 * 2011.07.07 ver.0.1
 * supported smbus api:
 *  i2c_smbus_read_byte_data()
 *  i2c_smbus_write_byte_data()
 *  i2c_smbus_read_word_data()
 *  i2c_smbus_write_word_data()
 *  i2c_smbus_read_i2c_block_data()
 *  i2c_smbus_write_i2c_block_data()
 */

//#define DEBUG 1

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <linux/i2c-pb206x-platform.h>

/* I2C controller info */
struct i2c_device_info {
	int id;
	u32 offset;
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

/* reference clock */
#define PB206X_MAIN_CLOCK (19200) /* 19.2 MHz */

/* timeout waiting for the controller to respond */
#define PB206X_I2C_TIMEOUT 		(msecs_to_jiffies(5000)) // 5 sec

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
	int			id; /* master id */
	struct completion	cmd_complete;
	u32			speed;
	u8			cmd_err;
	u8			*buf;
	size_t			buf_len;
	struct i2c_adapter	adapter;
	int			rx_fifo_size;
	int			tx_fifo_size;
};

static inline void pb206x_i2c_write_reg(struct pb206x_i2c_dev *i2c_dev,
		int reg, u8 val)
{
	dev_dbg(i2c_dev->dev, "%s: %08x+%02x << %02x\n",
			__func__, i2c_dev->base, reg, val);
	__raw_writeb(val, i2c_dev->base + reg);
}

static inline u8 pb206x_i2c_read_reg(struct pb206x_i2c_dev *i2c_dev, int reg)
{
	u8 val;
	val =  __raw_readb(i2c_dev->base + reg);
	dev_dbg(i2c_dev->dev, "%s: %08x+%02x >> %02x\n",
			__func__, i2c_dev->base, reg, val);
	return val;
}

static int pb206x_i2c_init(struct pb206x_i2c_dev *dev)
{
	unsigned long timeout;
	unsigned long div;

	if (dev->id == 5) {
		__raw_writeb(1, ((u32)dev->base & ~0xFF)+0x3F);
	}
	
	/* reset */
	pb206x_i2c_write_reg(dev, PB206X_I2C_MC_REG, PB206X_I2C_MC_RESET);
	timeout = jiffies + PB206X_I2C_TIMEOUT;
	while(pb206x_i2c_read_reg(dev, 
				PB206X_I2C_MC_REG) & PB206X_I2C_MC_RESET) {
		if (time_after(jiffies, timeout)) {
			dev_warn(dev->dev, "timeout waiting for reset done\n");
			return -ETIMEDOUT;
		}
		msleep(1);
	}

	/* enable the interrupt */
	pb206x_i2c_write_reg(dev, PB206X_I2C_MFS_REG,
			PB206X_I2C_MFS_TX_IE | PB206X_I2C_MFS_RX_IE);

	/* clean the status */
	pb206x_i2c_write_reg(dev, PB206X_I2C_MC_REG,
			PB206X_I2C_MC_INT_CLR |
			PB206X_I2C_MC_RXFIFO_CLR |
			PB206X_I2C_MC_TXFIFO_CLR);

	/* set the speed */
	if (dev->speed <= 100)
		div = (PB206X_MAIN_CLOCK / (5 * dev->speed)) - 1;
	else
		div = (PB206X_MAIN_CLOCK / (6 * dev->speed));

	/* max divide value is 0x7ff */
	if (div > 0x7ff)
		div = 0x7ff;

	/* Recommend:
	 * If you use a external 19.2 MHz clock, you may assing 0x08.
	 * However, in case using internal RC oscillator, you should
	 * consider the fact that it has about 10% deviation on 
	 * semiconductor process. So approximately 0x09 is better
	 * than 0x08
	 */
	if (div <= 8)
		div = 9;

	pb206x_i2c_write_reg(dev, PB206X_I2C_CDL_REG, (u8)div);
	pb206x_i2c_write_reg(dev, PB206X_I2C_CDH_REG, (u8)(div >> 8));

	return 0;
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

static int __pb206x_smbus_xfer(struct i2c_adapter *adap, u16 addr,
		unsigned short flags, char read_write,
		u8 high_command, u8 low_command, 
		int size, union i2c_smbus_data *data, int word)
{
	struct pb206x_i2c_dev *dev = i2c_get_adapdata(adap);
	int i;
	int ret;
	u8 val;

	/* FIXME */
	//pb206x_i2c_init(dev);

	ret = pb206x_i2c_wait_for_bb(dev);
	if (ret < 0)
		goto out;

	/* select mode */
	switch (size) {
	case I2C_SMBUS_BYTE_DATA:
		dev->buf = &data->byte;
		dev->buf_len = 1;
		break;

	case I2C_SMBUS_WORD_DATA:
		dev->buf = (u8 *)&data->word;
		dev->buf_len = 2;
		break;

	case I2C_SMBUS_I2C_BLOCK_DATA:
		dev->buf = (u8 *)&data->block[1];
		dev->buf_len = data->block[0];
		break;
	default:
		return -1;
	}

	/* write fifo */
	if (read_write == I2C_SMBUS_WRITE) {
		for (i = 0; i < dev->buf_len; i++) {
			pb206x_i2c_write_reg(dev, PB206X_I2C_FIFO_DATA_REG,
					dev->buf[i]);
		}
	}

	/* slave address
	 * Linux SMBUS supports only byte command 
	 */
	if (word)
		val = 0x80;
	else
		val = 0;
	val |= addr & 0x7f;
	pb206x_i2c_write_reg(dev, PB206X_I2C_SAD_REG, val);

	/* command */
	pb206x_i2c_write_reg(dev, PB206X_I2C_SSAH_REG, high_command);
	pb206x_i2c_write_reg(dev, PB206X_I2C_SSAL_REG, low_command);

	/* set the byte count (real transfer byte = this value + 1) */
	pb206x_i2c_write_reg(dev, PB206X_I2C_BC_REG, dev->buf_len - 1);
	if (pb206x_i2c_read_reg(dev, PB206X_I2C_BC_REG) != dev->buf_len - 1) {
		ret = -EIO;
		goto out;
	}

	/* set the scl/sda pull up/down (if needed) */

	/* start i2c transaction */
	if (read_write == I2C_SMBUS_READ)
		val = PB206X_I2C_MC_RW_MODE; /* read */
	else
		val = 0;
	val |= PB206X_I2C_MC_START;
	pb206x_i2c_write_reg(dev, PB206X_I2C_MC_REG, val);

	init_completion(&dev->cmd_complete);
	dev->cmd_err = 0;

	ret = wait_for_completion_timeout(&dev->cmd_complete,
			PB206X_I2C_TIMEOUT);
	if (ret < 0)
		return ret;
	if (ret == 0) {
		u8 stat = pb206x_i2c_read_reg(dev, PB206X_I2C_MS_REG);
		if (!(stat & (PB206X_I2C_MS_RX_DONE | PB206X_I2C_MS_TX_DONE))) {
			dev_err(dev->dev, "controller timed out (status %x)\n",
					stat);
			pb206x_i2c_init(dev);
			return -ETIMEDOUT;
		}
		dev_warn(dev->dev, "controller timed out and recoverd (status %x)\n", stat);
		/* clean the irq pending status */
		pb206x_i2c_write_reg(dev, PB206X_I2C_MC_REG,
				PB206X_I2C_MC_INT_CLR);
	}

	/* We have an error */
	if (unlikely(dev->cmd_err)) {
		pb206x_i2c_init(dev);
		return -EIO;
	}

	if (read_write == I2C_SMBUS_READ) {
		for (i = 0; i < dev->buf_len; i++) {
			*dev->buf++ = pb206x_i2c_read_reg(dev, 
					PB206X_I2C_FIFO_DATA_REG);
		}
	}
	
	return 0;

out:
	/* clean the status */
	pb206x_i2c_write_reg(dev, PB206X_I2C_MC_REG,
			PB206X_I2C_MC_INT_CLR |
			PB206X_I2C_MC_RXFIFO_CLR |
			PB206X_I2C_MC_TXFIFO_CLR);

	return ret;
}

static int pb206x_smbus_xfer(struct i2c_adapter *adap, u16 addr,
		unsigned short flags, char read_write,
		u8 command, int size, union i2c_smbus_data *data)
{
	return __pb206x_smbus_xfer(adap, addr, flags, read_write,
			0, command, size, data, 0);
}

static u32 pb206x_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK;
}

static inline void pb206x_i2c_complete_cmd(struct pb206x_i2c_dev *dev, u8 err)
{
	dev->cmd_err |= err;
	complete(&dev->cmd_complete);
}

static irqreturn_t pb206x_i2c_isr(int this_irq, void *dev_id)
{
	struct pb206x_i2c_dev *dev = dev_id;
	u8 stat;
	const u8 bitmask_errors = PB206X_I2C_MS_TIMEOUT |
				PB206X_I2C_MS_RX_ERR |
				PB206X_I2C_MS_TX_ERR |
				PB206X_I2C_MS_CMD_ERR;

	stat = pb206x_i2c_read_reg(dev, PB206X_I2C_MS_REG);
	/* enable the interrupt */
	pb206x_i2c_write_reg(dev, PB206X_I2C_MFS_REG,
			PB206X_I2C_MFS_TX_IE | PB206X_I2C_MFS_RX_IE);
	/* interrupt clear */
	pb206x_i2c_write_reg(dev, PB206X_I2C_MC_REG,
			PB206X_I2C_MC_INT_CLR);

	if (stat & bitmask_errors) {
		dev_err(dev->dev, "ms: %x, mfs: %x\n",
				stat, 
				pb206x_i2c_read_reg(dev, PB206X_I2C_MFS_REG));
		if (stat & PB206X_I2C_MS_TIMEOUT) 
			dev_err(dev->dev, "Time out\n");

		if (stat & PB206X_I2C_MS_RX_ERR) 
			dev_err(dev->dev, "Rx error\n");

		if (stat & PB206X_I2C_MS_TX_ERR) 
			dev_err(dev->dev, "Tx error\n");

		if (stat & PB206X_I2C_MS_CMD_ERR) 
			dev_err(dev->dev, "Cmd error\n");

		pb206x_i2c_complete_cmd(dev, stat & bitmask_errors);
	}

	if (stat & PB206X_I2C_MS_RX_DONE) {
		pb206x_i2c_complete_cmd(dev, 0);
	}

	if (stat & PB206X_I2C_MS_TX_DONE) {
		pb206x_i2c_complete_cmd(dev, 0);
	}

	if (stat & PB206X_I2C_MS_ABORT_DONE) {
	}

	return IRQ_HANDLED;
}

static const struct i2c_algorithm pb206x_i2c_algo = {
	//.master_xfer	= pb206x_i2c_xfer,
	.smbus_xfer     = pb206x_smbus_xfer,
	.functionality	= pb206x_i2c_func,
};

static int __devinit pb206x_i2c_probe(struct platform_device *pdev)
{
	struct pb206x_i2c_dev *dev;
	struct i2c_pb206x_platform_data *pdata = NULL;
	struct i2c_adapter    *adap;
	struct resource       *irq, *ioarea = NULL;
	int ret;
	u32 speed = 0;

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "%s: no irq resource\n", __func__);
		return -ENODEV;
	}

	dev = kzalloc(sizeof(struct pb206x_i2c_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "%s: memory allocation failed\n", __func__);
		return -ENOMEM;
	}

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "%s: no platform data\n", __func__);
		return -ENODEV;
	}

	pdata = pdev->dev.platform_data;
	if (pdata->master_id < 0 || pdata->master_id > 5) {
		dev_err(&pdev->dev, "%s: wrong master id(%d)\n", __func__,
				pdata->master_id);
		return -EINVAL;
	}

	if (pdata->platform_init) {
		dev_dbg(&pdev->dev, "platform_init() will be called\n");
		ret = pdata->platform_init();
		if (ret < 0) {
			dev_err(&pdev->dev, "%s: platform_init failed\n",
					__func__);
			goto err_platform_init;
		}
	}

	if (pdata->speed)
		speed = pdata->speed;
	else
		speed = 100; /* default speed */

	if (speed > 400) {
		dev_warn(&pdev->dev, "max speed 400 kHz\n");
		speed = 400;
	}
	
	dev->speed = speed;
	dev->dev = &pdev->dev;
	dev->irq = irq->start;

	if (atomic_read(&pb206x_i2c_ref_count) == 0) {
		ioarea = request_mem_region(pdata->iomem & PAGE_MASK, 
				PAGE_SIZE, pdev->name);
		if (!ioarea) {
			dev_err(dev->dev, "%s: I2C region already claimed\n",
					__func__);
			ret = -EBUSY;
			goto err_request_mem;
		}


		pb206x_i2c_base = ioremap(pdata->iomem & PAGE_MASK, PAGE_SIZE);
		if (!pb206x_i2c_base) {
			dev_err(dev->dev, "%s: failure ioremap\n", __func__);
			ret = -ENOMEM;
			goto err_ioremap;
		}
		dev_dbg(dev->dev, "%s: iomem %x, virt %x\n", __func__,
				pdata->iomem, pb206x_i2c_base);
	}
	dev->id = pdata->master_id;
	dev->base = pb206x_i2c_base + pb206x_i2c_device_info[dev->id].offset;
	dev->tx_fifo_size = pb206x_i2c_device_info[dev->id].tx_fifo_size;
	dev->rx_fifo_size = pb206x_i2c_device_info[dev->id].rx_fifo_size;
	
	platform_set_drvdata(pdev, dev);
			
	pb206x_i2c_init(dev);

	//set_irq_type(dev->irq, IRQ_TYPE_EDGE_FALLING);
	set_irq_type(dev->irq, IRQ_TYPE_LEVEL_LOW);
	ret = request_irq(dev->irq, pb206x_i2c_isr, IRQF_DISABLED, 
			pdev->name, dev);
	if (ret) {
		dev_err(dev->dev, "%s: failure requesting irq %d (error %d)\n", 
				__func__, dev->irq, ret);
		goto err_request_irq;
	}

	dev_info(dev->dev, "%d: %08x: bus-%d tx-%d rx-%d @%dkHz\n",
			dev->id, dev->base, pdev->id,
			dev->tx_fifo_size, dev->rx_fifo_size,
			dev->speed);

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
		dev_err(dev->dev, "%s: failure adding adapter\n", __func__);
		goto err_i2c_add_numbered_adapter;
	}

	atomic_inc(&pb206x_i2c_ref_count);

	return 0;


err_i2c_add_numbered_adapter:
	free_irq(dev->irq, dev);
err_request_irq:
	platform_set_drvdata(pdev, NULL);
	iounmap(pb206x_i2c_base);
err_ioremap:
	if (ioarea)
		release_mem_region(pdata->iomem & PAGE_MASK, PAGE_SIZE);
err_request_mem:
err_platform_init:
	kfree(dev);

	return ret;
}

static int __devexit pb206x_i2c_remove(struct platform_device *pdev)
{
	struct pb206x_i2c_dev *dev = platform_get_drvdata(pdev);
	struct i2c_pb206x_platform_data *pdata = pdev->dev.platform_data;

	platform_set_drvdata(pdev, NULL);

	free_irq(dev->irq, dev);
	i2c_del_adapter(&dev->adapter);

	/* do something if needed */

	kfree(dev);

	if (atomic_dec_and_test(&pb206x_i2c_ref_count)) {
		iounmap(pb206x_i2c_base);
		if (pdata)
			release_mem_region(pdata->iomem & PAGE_MASK,
					PAGE_SIZE);
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

#ifdef MODULE
module_init(pb206x_i2c_init_driver);
#else
/* I2C may be needed to bring up other drivers */
subsys_initcall(pb206x_i2c_init_driver);
#endif
module_exit(pb206x_i2c_exit_driver);

MODULE_AUTHOR("Pointchips, Inc.");
MODULE_DESCRIPTION("Pointchips pb206x i2c adapter");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c-pb206x");

#ifdef CONFIG_PB206X_I2C_SMBUS_2
/* pb206x specific smbus helper functions */
s32 i2c_smbus_read_byte_data_2(struct i2c_client *client, u16 command)
{
        union i2c_smbus_data data;
        int status;

        status = __pb206x_smbus_xfer(client->adapter, client->addr, client->flags,
                                I2C_SMBUS_READ, command >> 8, command & 0xff,
                                I2C_SMBUS_BYTE_DATA, &data, 1);
        return (status < 0) ? status : data.byte;
}
EXPORT_SYMBOL(i2c_smbus_read_byte_data_2);

s32 i2c_smbus_write_byte_data_2(struct i2c_client *client, u16 command, u8 value)
{
        union i2c_smbus_data data;
        data.byte = value;
        return __pb206x_smbus_xfer(client->adapter,client->addr,client->flags,
                              I2C_SMBUS_WRITE,command >> 8, command & 0xff,
                              I2C_SMBUS_BYTE_DATA,&data, 1);
}
EXPORT_SYMBOL(i2c_smbus_write_byte_data_2);

s32 i2c_smbus_read_word_data_2(struct i2c_client *client, u16 command)
{
        union i2c_smbus_data data;
        int status;

        status = __pb206x_smbus_xfer(client->adapter, client->addr, client->flags,
                                I2C_SMBUS_READ, command >> 8, command & 0xff,
                                I2C_SMBUS_WORD_DATA, &data, 1);
        return (status < 0) ? status : data.word;
}
EXPORT_SYMBOL(i2c_smbus_read_word_data_2);

s32 i2c_smbus_write_word_data_2(struct i2c_client *client, u16 command, u16 value)
{
        union i2c_smbus_data data;
        data.word = value;
        return __pb206x_smbus_xfer(client->adapter,client->addr,client->flags,
                              I2C_SMBUS_WRITE,command >> 8, command & 0xff,
                              I2C_SMBUS_WORD_DATA,&data, 1);
}
EXPORT_SYMBOL(i2c_smbus_write_word_data_2);

s32 i2c_smbus_read_i2c_block_data_2(struct i2c_client *client, u16 command,
		                                  u8 length, u8 *values)
{
        union i2c_smbus_data data;
        int status;

        if (length > I2C_SMBUS_BLOCK_MAX)
                length = I2C_SMBUS_BLOCK_MAX;
        data.block[0] = length;
        status = __pb206x_smbus_xfer(client->adapter, client->addr, client->flags,
                                I2C_SMBUS_READ, command >> 8, command & 0xff,
                                I2C_SMBUS_I2C_BLOCK_DATA, &data, 1);
        if (status < 0)
                return status;

        memcpy(values, &data.block[1], data.block[0]);
        return data.block[0];
}
EXPORT_SYMBOL(i2c_smbus_read_i2c_block_data_2);


s32 i2c_smbus_write_i2c_block_data_2(struct i2c_client *client, u16 command,
		                                   u8 length, const u8 *values)
{
        union i2c_smbus_data data;

        if (length > I2C_SMBUS_BLOCK_MAX)
                length = I2C_SMBUS_BLOCK_MAX;
        data.block[0] = length;
        memcpy(data.block + 1, values, length);
        return __pb206x_smbus_xfer(client->adapter, client->addr, client->flags,
                              I2C_SMBUS_WRITE, command >> 8, command & 0xff,
                              I2C_SMBUS_I2C_BLOCK_DATA, &data, 1);
}
EXPORT_SYMBOL(i2c_smbus_write_i2c_block_data_2);
#endif /* CONFIG_PB206X_I2C_SMBUS_2 */
