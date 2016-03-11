/*
 * MPC83xx SPI controller driver.
 *
 * Maintainer: Kumar Gala
 *
 * Copyright (C) 2006 Polycom, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/qe.h>
#include <asm/qe_ic.h>
#include <sysdev/fsl_soc.h>
#include <linux/delay.h>

/* SPI Controller registers */
struct mpc83xx_spi_reg {
	u8 res1[0x20];
	__be32 mode;
	__be32 event;
	__be32 mask;
	__be32 command;
	__be32 transmit;
	__be32 receive;
};

/* SPI Controller mode register definitions */
#define	SPMODE_LOOP		(1 << 30)
#define	SPMODE_CI_INACTIVEHIGH	(1 << 29)
#define	SPMODE_CP_BEGIN_EDGECLK	(1 << 28)
#define	SPMODE_DIV16		(1 << 27)
#define	SPMODE_REV		(1 << 26)
#define	SPMODE_MS		(1 << 25)
#define	SPMODE_ENABLE		(1 << 24)
#define	SPMODE_LEN(x)		((x) << 20)
#define	SPMODE_PM(x)		((x) << 16)
#define	SPMODE_OP		(1 << 14)
#define	SPMODE_CG(x)		((x) << 7)

/*
 * Default for SPI Mode:
 * 	SPI MODE 0 (inactive low, phase middle, MSB, 8-bit length, slow clk
 */
#define	SPMODE_INIT_VAL (SPMODE_CI_INACTIVEHIGH | SPMODE_DIV16 | SPMODE_REV | \
			 SPMODE_MS | SPMODE_LEN(7) | SPMODE_PM(0xf))

/* SPIE register values */
#define	SPIE_NE		0x00000200	/* Not empty */
#define	SPIE_NF		0x00000100	/* Not full */
#define	SPIE_LT		0x00004000	/* Last character */

/* SPIM register values */
#define	SPIM_NE		0x00000200	/* Not empty */
#define	SPIM_NF		0x00000100	/* Not full */
#define	SPIM_LT		0x00004000	/* Last character */

#define SPCOM_LST	0x00400000

/* SPI Controller driver's private data. */
struct mpc83xx_spi {
	struct mpc83xx_spi_reg __iomem *base;

	/* rx & tx bufs from the spi_transfer */
	const void *tx;
	void *rx;

	/* functions to deal with different sized buffers */
	void (*get_rx) (u32 rx_data, struct mpc83xx_spi *);
	u32(*get_tx) (struct mpc83xx_spi *);

	unsigned int count;
	int irq;

	unsigned nsecs;		/* (clock cycle time)/2 */

	u32 spibrg;		/* SPIBRG input clock */
	u32 rx_shift;		/* RX data reg shift when in qe mode */
	u32 tx_shift;		/* TX data reg shift when in qe mode */

	bool qe_mode;

	void (*activate_cs) (u8 cs, u8 polarity);
	void (*deactivate_cs) (u8 cs, u8 polarity);

	u8 busy;

	struct workqueue_struct *workqueue;
	struct work_struct work;

	struct list_head queue;
	spinlock_t lock;

	struct completion done;
};

struct spi_mpc83xx_cs {
	/* functions to deal with different sized buffers */
	void (*get_rx) (u32 rx_data, struct mpc83xx_spi *);
	u32 (*get_tx) (struct mpc83xx_spi *);
	u32 rx_shift;		/* RX data reg shift when in qe mode */
	u32 tx_shift;		/* TX data reg shift when in qe mode */
	u32 hw_mode;		/* Holds HW mode register settings */
};

//added by yangyf at 2015-04-22

#define		GPIO_ADDR_OFFSET        0xc00
//#define 	GPIO_DAT				0x08
#define		SPI_CS_FPGA				1
#define		SPI_CS_Flash			3
#define		GPIO_REG_SIZE			4
#define 	MPC8XXX_GPIO_PINS		32

#define GPIO_DIR		       0x00
#define GPIO_ODR		0x04
#define GPIO_DAT		0x08

#define SLAVE_SWITCH      1
#define SLAVE_EPCS		9

static void __iomem *immap;
extern phys_addr_t get_immrbase(void);

static inline u32 mpc8xxx_gpio2mask(unsigned int gpio)
{
	return 1u << (MPC8XXX_GPIO_PINS - 1 - gpio);
}

static void switch_cs_enable(u8 cs, u8 polarity)
{
	if(immap)
	{
		clrbits32(immap+0xc00+ GPIO_DAT, mpc8xxx_gpio2mask(SLAVE_SWITCH));
	}
	else
		printk("\n the immap has not inita \n");

}

static void switch_cs_disable(u8 cs, u8 polarity)
{
	if(immap)
	{
		setbits32(immap+0xc00+ GPIO_DAT, mpc8xxx_gpio2mask(SLAVE_SWITCH));
	}
	else
		printk("\n the immap has not inita \n");

}
static void chip_sel_pin_init(void)
{
	//void __iomem *immap;
	int		d;
	immap = ioremap(get_immrbase(), 0xd00);
	if(immap)
	{

	       d = mpc8xxx_gpio2mask(23);//datasheet p167
	       setbits32(immap + 0x118, d);

	    	 d = mpc8xxx_gpio2mask(16);
	    	 setbits32(immap + 0x118, d);
		 d = mpc8xxx_gpio2mask(17);
	        setbits32(immap + 0x118, d);
		//printk("\n the date is %8x. \n",in_be32(immap +0x118));

		   d = mpc8xxx_gpio2mask(14);
	          setbits32(immap + 0x118, d);
		   d = mpc8xxx_gpio2mask(15);
	          setbits32(immap + 0x118, d);
		printk("\n the date is %8x. \n",in_be32(immap +0x118));

		d=mpc8xxx_gpio2mask(SLAVE_SWITCH);//switch enable
		setbits32(immap + 0xc00+GPIO_DIR, d);
		//clrbits32(immap +0xc00+ GPIO_DAT, d);
		setbits32(immap +0xc00+ GPIO_DAT, d);

//		d = mpc8xxx_gpio2mask(SLAVE_EPCS);//epcs disable
//		setbits32(immap + 0xc00+GPIO_DIR, d);
//		setbits32(immap + 0xc00+ GPIO_DAT, d);

		printk("\n the date is %x. \n",in_be32(immap +0xc00+ GPIO_DAT) );
	}
	else
	{
		printk("\n the immap has not inita \n");
	}
	//switch_cs_disable(0, 0);
	//setbits32(immap +0xc00+ GPIO_DAT, d);
}
//end add
static inline void mpc83xx_spi_write_reg(__be32 __iomem * reg, u32 val)
{
	out_be32(reg, val);
}

static inline u32 mpc83xx_spi_read_reg(__be32 __iomem * reg)
{
	return in_be32(reg);
}

#define MPC83XX_SPI_RX_BUF(type) 					  \
void mpc83xx_spi_rx_buf_##type(u32 data, struct mpc83xx_spi *mpc83xx_spi) \
{									  \
	type * rx = mpc83xx_spi->rx;					  \
	*rx++ = (type)(data >> mpc83xx_spi->rx_shift);			  \
	mpc83xx_spi->rx = rx;						  \
}

#define MPC83XX_SPI_TX_BUF(type)				\
u32 mpc83xx_spi_tx_buf_##type(struct mpc83xx_spi *mpc83xx_spi)	\
{								\
	u32 data;						\
	const type * tx = mpc83xx_spi->tx;			\
	if (!tx)						\
		return 0;					\
	data = *tx++ << mpc83xx_spi->tx_shift;			\
	mpc83xx_spi->tx = tx;					\
	return data;						\
}

MPC83XX_SPI_RX_BUF(u8)
MPC83XX_SPI_RX_BUF(u16)
MPC83XX_SPI_RX_BUF(u32)
MPC83XX_SPI_TX_BUF(u8)
MPC83XX_SPI_TX_BUF(u16)
MPC83XX_SPI_TX_BUF(u32)

static void mpc83xx_spi_chipselect(struct spi_device *spi, int value)
{
	struct mpc83xx_spi *mpc83xx_spi;
	u8 pol = spi->mode & SPI_CS_HIGH ? 1 : 0;
	struct spi_mpc83xx_cs	*cs = spi->controller_state;

	mpc83xx_spi = spi_master_get_devdata(spi->master);

	if (value == BITBANG_CS_INACTIVE) {
		if (mpc83xx_spi->deactivate_cs)
			mpc83xx_spi->deactivate_cs(spi->chip_select, pol);
	}

	if (value == BITBANG_CS_ACTIVE) {
		u32 regval = mpc83xx_spi_read_reg(&mpc83xx_spi->base->mode);

		mpc83xx_spi->rx_shift = cs->rx_shift;
		mpc83xx_spi->tx_shift = cs->tx_shift;
		mpc83xx_spi->get_rx = cs->get_rx;
		mpc83xx_spi->get_tx = cs->get_tx;

		if (cs->hw_mode != regval) {
			unsigned long flags;
			void *tmp_ptr = &mpc83xx_spi->base->mode;

			regval = cs->hw_mode;
			/* Turn off IRQs locally to minimize time that
			 * SPI is disabled
			 */
			local_irq_save(flags);
			/* Turn off SPI unit prior changing mode */
			mpc83xx_spi_write_reg(tmp_ptr, regval & ~SPMODE_ENABLE);
			mpc83xx_spi_write_reg(tmp_ptr, regval);
			local_irq_restore(flags);
		}
		if (mpc83xx_spi->activate_cs)
			mpc83xx_spi->activate_cs(spi->chip_select, pol);
	}
}

static
int mpc83xx_spi_setup_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct mpc83xx_spi *mpc83xx_spi;
	u32 regval;
	u8 bits_per_word, pm;
	u32 hz;
	struct spi_mpc83xx_cs	*cs = spi->controller_state;

	mpc83xx_spi = spi_master_get_devdata(spi->master);

	if (t) {
		bits_per_word = t->bits_per_word;
		hz = t->speed_hz;
	} else {
		bits_per_word = 0;
		hz = 0;
	}

	/* spi_transfer level calls that work per-word */
	if (!bits_per_word)
		bits_per_word = spi->bits_per_word;

	/* Make sure its a bit width we support [4..16, 32] */
	if ((bits_per_word < 4)
	    || ((bits_per_word > 16) && (bits_per_word != 32)))
		return -EINVAL;

	if (!hz)
		hz = spi->max_speed_hz;

	cs->rx_shift = 0;
	cs->tx_shift = 0;
	if (bits_per_word <= 8) {
		cs->get_rx = mpc83xx_spi_rx_buf_u8;
		cs->get_tx = mpc83xx_spi_tx_buf_u8;
		if (mpc83xx_spi->qe_mode) {
			cs->rx_shift = 16;
			cs->tx_shift = 24;
		}
	} else if (bits_per_word <= 16) {
		cs->get_rx = mpc83xx_spi_rx_buf_u16;
		cs->get_tx = mpc83xx_spi_tx_buf_u16;
		if (mpc83xx_spi->qe_mode) {
			cs->rx_shift = 16;
			cs->tx_shift = 16;
		}
	} else if (bits_per_word <= 32) {
		cs->get_rx = mpc83xx_spi_rx_buf_u32;
		cs->get_tx = mpc83xx_spi_tx_buf_u32;
	} else
		return -EINVAL;

	if (mpc83xx_spi->qe_mode && spi->mode & SPI_LSB_FIRST) {
		cs->tx_shift = 0;
		if (bits_per_word <= 8)
			cs->rx_shift = 8;
		else
			cs->rx_shift = 0;
	}

	mpc83xx_spi->rx_shift = cs->rx_shift;
	mpc83xx_spi->tx_shift = cs->tx_shift;
	mpc83xx_spi->get_rx = cs->get_rx;
	mpc83xx_spi->get_tx = cs->get_tx;

	if (bits_per_word == 32)
		bits_per_word = 0;
	else
		bits_per_word = bits_per_word - 1;

	/* mask out bits we are going to set */
	cs->hw_mode &= ~(SPMODE_LEN(0xF) | SPMODE_DIV16
				  | SPMODE_PM(0xF));

	cs->hw_mode |= SPMODE_LEN(bits_per_word);
printk("spi spibrg=%d\n",mpc83xx_spi->spibrg);
	if ((mpc83xx_spi->spibrg / hz) > 64) {
		cs->hw_mode |= SPMODE_DIV16;
		pm = mpc83xx_spi->spibrg / (hz * 64);
		if (pm > 16) {
			dev_err(&spi->dev, "Requested speed is too "
				"low: %d Hz. Will use %d Hz instead.\n",
				hz, mpc83xx_spi->spibrg / 1024);
			pm = 16;
		}
	} else
		pm = mpc83xx_spi->spibrg / (hz * 4);
	if (pm)
		pm--;

	cs->hw_mode |= SPMODE_PM(pm);
	regval =  mpc83xx_spi_read_reg(&mpc83xx_spi->base->mode);
	if (cs->hw_mode != regval) {
		unsigned long flags;
		void *tmp_ptr = &mpc83xx_spi->base->mode;

		regval = cs->hw_mode;
		/* Turn off IRQs locally to minimize time
		 * that SPI is disabled
		 */
		local_irq_save(flags);
		/* Turn off SPI unit prior changing mode */
		mpc83xx_spi_write_reg(tmp_ptr, regval & ~SPMODE_ENABLE);
		mpc83xx_spi_write_reg(tmp_ptr, regval);
		local_irq_restore(flags);
	}
	return 0;
}

static int mpc83xx_spi_bufs(struct spi_device *spi, struct spi_transfer *t)
{
	struct mpc83xx_spi *mpc83xx_spi;
	u32 word, len, bits_per_word;

	mpc83xx_spi = spi_master_get_devdata(spi->master);

	mpc83xx_spi->tx = t->tx_buf;
	mpc83xx_spi->rx = t->rx_buf;
	bits_per_word = spi->bits_per_word;
	if (t->bits_per_word)
		bits_per_word = t->bits_per_word;
	len = t->len;
	if (bits_per_word > 8) {
		/* invalid length? */
		if (len & 1)
			return -EINVAL;
		len /= 2;
	}
	if (bits_per_word > 16) {
		/* invalid length? */
		if (len & 1)
			return -EINVAL;
		len /= 2;
	}
	mpc83xx_spi->count = len;

	INIT_COMPLETION(mpc83xx_spi->done);

	/* enable rx and tx ints */
	mpc83xx_spi_write_reg(&mpc83xx_spi->base->mask, SPIM_NE);

	/* transmit word */
	word = mpc83xx_spi->get_tx(mpc83xx_spi);
	mpc83xx_spi_write_reg(&mpc83xx_spi->base->transmit, word);
	printk("len is %x, word is %x\n",len,word);

	wait_for_completion(&mpc83xx_spi->done);

	/* disable all ints */
	mpc83xx_spi_write_reg(&mpc83xx_spi->base->mask, 0);

	return mpc83xx_spi->count;
}

static void mpc83xx_spi_work(struct work_struct *work)
{
	struct mpc83xx_spi *mpc83xx_spi =
		container_of(work, struct mpc83xx_spi, work);

	spin_lock_irq(&mpc83xx_spi->lock);
	mpc83xx_spi->busy = 1;
	while (!list_empty(&mpc83xx_spi->queue)) {
		struct spi_message *m;
		struct spi_device *spi;
		struct spi_transfer *t = NULL;
		unsigned cs_change;
		int status, nsecs = 15;

		m = container_of(mpc83xx_spi->queue.next,
				struct spi_message, queue);
		list_del_init(&m->queue);
		spin_unlock_irq(&mpc83xx_spi->lock);

		spi = m->spi;
		cs_change = 1;
		status = 0;
		list_for_each_entry(t, &m->transfers, transfer_list) {
			if (t->bits_per_word || t->speed_hz) {
				/* Don't allow changes if CS is active */
				status = -EINVAL;

				if (cs_change)
					status = mpc83xx_spi_setup_transfer(spi, t);
				if (status < 0)
					break;
			}

			if (cs_change)
				mpc83xx_spi_chipselect(spi, BITBANG_CS_ACTIVE);
			cs_change = t->cs_change;
			if (t->len)
				status = mpc83xx_spi_bufs(spi, t);

			if (status) {
				status = -EMSGSIZE;
				break;
			}
			m->actual_length += t->len;

			if (t->delay_usecs)
				udelay(t->delay_usecs);

			if (cs_change) {
				ndelay(nsecs);
				mpc83xx_spi_chipselect(spi, BITBANG_CS_INACTIVE);
				ndelay(nsecs);
			}
		}

		m->status = status;
		m->complete(m->context);

		if (status || !cs_change) {
			ndelay(nsecs);
			mpc83xx_spi_chipselect(spi, BITBANG_CS_INACTIVE);
		}

		mpc83xx_spi_setup_transfer(spi, NULL);

		spin_lock_irq(&mpc83xx_spi->lock);
	}
	mpc83xx_spi->busy = 0;
	spin_unlock_irq(&mpc83xx_spi->lock);
}

/* the spi->mode bits understood by this driver: */
#define MODEBITS	(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH \
			| SPI_LSB_FIRST | SPI_LOOP)

static int mpc83xx_spi_setup(struct spi_device *spi)
{
	struct mpc83xx_spi *mpc83xx_spi;
	int retval;
	u32 hw_mode;
	struct spi_mpc83xx_cs	*cs = spi->controller_state;

	if (spi->mode & ~MODEBITS) {
		dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
			spi->mode & ~MODEBITS);
		return -EINVAL;
	}

	if (!spi->max_speed_hz)
		return -EINVAL;

	if (!cs) {
		cs = kzalloc(sizeof *cs, GFP_KERNEL);
		if (!cs)
			return -ENOMEM;
		spi->controller_state = cs;
	}
	mpc83xx_spi = spi_master_get_devdata(spi->master);

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	hw_mode = cs->hw_mode; /* Save orginal settings */
	cs->hw_mode = mpc83xx_spi_read_reg(&mpc83xx_spi->base->mode);
	/* mask out bits we are going to set */
	cs->hw_mode &= ~(SPMODE_CP_BEGIN_EDGECLK | SPMODE_CI_INACTIVEHIGH
			 | SPMODE_REV | SPMODE_LOOP);

	if (spi->mode & SPI_CPHA)
		cs->hw_mode |= SPMODE_CP_BEGIN_EDGECLK;
	if (spi->mode & SPI_CPOL)
		cs->hw_mode |= SPMODE_CI_INACTIVEHIGH;
	if (!(spi->mode & SPI_LSB_FIRST))
		cs->hw_mode |= SPMODE_REV;
	if (spi->mode & SPI_LOOP)
		cs->hw_mode |= SPMODE_LOOP;

	retval = mpc83xx_spi_setup_transfer(spi, NULL);
	if (retval < 0) {
		cs->hw_mode = hw_mode; /* Restore settings */
		return retval;
	}

	dev_dbg(&spi->dev, "%s, mode %d, %u bits/w, %u Hz\n",
		__func__, spi->mode & (SPI_CPOL | SPI_CPHA),
		spi->bits_per_word, spi->max_speed_hz);
#if 0 /* Don't think this is needed */
	/* NOTE we _need_ to call chipselect() early, ideally with adapter
	 * setup, unless the hardware defaults cooperate to avoid confusion
	 * between normal (active low) and inverted chipselects.
	 */

	/* deselect chip (low or high) */
	spin_lock(&mpc83xx_spi->lock);
	if (!mpc83xx_spi->busy)
		mpc83xx_spi_chipselect(spi, BITBANG_CS_INACTIVE);
	spin_unlock(&mpc83xx_spi->lock);
#endif
	return 0;
}

irqreturn_t mpc83xx_spi_irq(s32 irq, void *context_data)
{
	struct mpc83xx_spi *mpc83xx_spi = context_data;
	u32 event;
	irqreturn_t ret = IRQ_NONE;

	/* Get interrupt events(tx/rx) */
	event = mpc83xx_spi_read_reg(&mpc83xx_spi->base->event);

	/* We need handle RX first */
//	printk("irq event:0x%x\n",event);
	if (event & SPIE_NE) {
		u32 rx_data = mpc83xx_spi_read_reg(&mpc83xx_spi->base->receive);
//		printk("irq rx data:0x%x\n",rx_data);
		if (mpc83xx_spi->rx)
			mpc83xx_spi->get_rx(rx_data, mpc83xx_spi);

		if(0 == mpc83xx_spi->count)
		{
			complete(&mpc83xx_spi->done);
			printk("complete spi->done\n");
		}
		
		ret = IRQ_HANDLED;
	}

	if ((event & SPIE_NF) && (mpc83xx_spi->count > 0)) {
		u32 word = mpc83xx_spi->get_tx(mpc83xx_spi);
		printk("irq word:0x%x\n",word);
		mpc83xx_spi_write_reg(&mpc83xx_spi->base->transmit, word);
		mpc83xx_spi->count -= 1;

		if(0 == mpc83xx_spi->count)
			mpc83xx_spi_write_reg(&mpc83xx_spi->base->mask, SPIM_NE);
		
		ret = IRQ_HANDLED;
	}

	/* Clear the events */
	mpc83xx_spi_write_reg(&mpc83xx_spi->base->event, event);

	return ret;
}
static int mpc83xx_spi_transfer(struct spi_device *spi,
				struct spi_message *m)
{
	struct mpc83xx_spi *mpc83xx_spi = spi_master_get_devdata(spi->master);
	unsigned long flags;

	m->actual_length = 0;
	m->status = -EINPROGRESS;

	spin_lock_irqsave(&mpc83xx_spi->lock, flags);
	list_add_tail(&m->queue, &mpc83xx_spi->queue);
	queue_work(mpc83xx_spi->workqueue, &mpc83xx_spi->work);
	spin_unlock_irqrestore(&mpc83xx_spi->lock, flags);

	return 0;
}


static void mpc83xx_spi_cleanup(struct spi_device *spi)
{
	kfree(spi->controller_state);
}

static int __init mpc83xx_spi_probe(struct platform_device *dev)
{
	struct spi_master *master;
	struct mpc83xx_spi *mpc83xx_spi;
	struct fsl_spi_platform_data *pdata;
	struct resource *r;
	u32 regval;
	int ret = 0;
	
	/* Get resources(memory, IRQ) associated with the device */
	master = spi_alloc_master(&dev->dev, sizeof(struct mpc83xx_spi));

	if (master == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	platform_set_drvdata(dev, master);
	pdata = dev->dev.platform_data;

	if (pdata == NULL) {
		ret = -ENODEV;
		goto free_master;
	}

	r = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		ret = -ENODEV;
		goto free_master;
	}
	master->setup = mpc83xx_spi_setup;
	master->transfer = mpc83xx_spi_transfer;
	master->cleanup = mpc83xx_spi_cleanup;

	mpc83xx_spi = spi_master_get_devdata(master);
	mpc83xx_spi->activate_cs = switch_cs_enable; //pdata->activate_cs; //changed by yangyf
	mpc83xx_spi->deactivate_cs = switch_cs_disable; //pdata->deactivate_cs; //changed by yangyf
	mpc83xx_spi->qe_mode = pdata->qe_mode;
	mpc83xx_spi->get_rx = mpc83xx_spi_rx_buf_u8;
	mpc83xx_spi->get_tx = mpc83xx_spi_tx_buf_u8;
	mpc83xx_spi->spibrg = pdata->sysclk;

	mpc83xx_spi->rx_shift = 0;
	mpc83xx_spi->tx_shift = 0;
	if (mpc83xx_spi->qe_mode) {
		mpc83xx_spi->rx_shift = 16;
		mpc83xx_spi->tx_shift = 24;
	}

	init_completion(&mpc83xx_spi->done);

	mpc83xx_spi->base = ioremap(r->start, r->end - r->start + 1);
	if (mpc83xx_spi->base == NULL) {
		ret = -ENOMEM;
		goto put_master;
	}

	mpc83xx_spi->irq = platform_get_irq(dev, 0);

	if (mpc83xx_spi->irq < 0) {
		ret = -ENXIO;
		goto unmap_io;
	}

	/* Register for SPI Interrupt */
	ret = request_irq(mpc83xx_spi->irq, mpc83xx_spi_irq,
			  0, "mpc83xx_spi", mpc83xx_spi);

	if (ret != 0)
		goto unmap_io;

	master->bus_num = pdata->bus_num;
	master->num_chipselect = pdata->max_chipselect;

	/* SPI controller initializations */
	mpc83xx_spi_write_reg(&mpc83xx_spi->base->mode, 0);
	mpc83xx_spi_write_reg(&mpc83xx_spi->base->mask, 0);
	mpc83xx_spi_write_reg(&mpc83xx_spi->base->command, 0);
	mpc83xx_spi_write_reg(&mpc83xx_spi->base->event, 0xffffffff);

	/* Enable SPI interface */
	regval = pdata->initial_spmode | SPMODE_INIT_VAL | SPMODE_ENABLE;
	if (pdata->qe_mode)
		regval |= SPMODE_OP;

	mpc83xx_spi_write_reg(&mpc83xx_spi->base->mode, regval);
	spin_lock_init(&mpc83xx_spi->lock);
	init_completion(&mpc83xx_spi->done);

	chip_sel_pin_init();

	INIT_WORK(&mpc83xx_spi->work, mpc83xx_spi_work);
	INIT_LIST_HEAD(&mpc83xx_spi->queue);

	mpc83xx_spi->workqueue = create_singlethread_workqueue(
		master->dev.parent->bus_id);
	if (mpc83xx_spi->workqueue == NULL) {
		ret = -EBUSY;
		goto free_irq;
	}

	ret = spi_register_master(master);
	if (ret < 0)
		goto unreg_master;

	printk(KERN_INFO
	       "%s: MPC83xx SPI Controller driver at 0x%p (irq = %d)\n",
	       dev->dev.bus_id, mpc83xx_spi->base, mpc83xx_spi->irq);

	return ret;

unreg_master:
	destroy_workqueue(mpc83xx_spi->workqueue);
free_irq:
	free_irq(mpc83xx_spi->irq, mpc83xx_spi);
unmap_io:
	iounmap(mpc83xx_spi->base);
put_master:
	spi_master_put(master);
free_master:
	kfree(master);
err:
	return ret;
}

static int __exit mpc83xx_spi_remove(struct platform_device *dev)
{
	struct mpc83xx_spi *mpc83xx_spi;
	struct spi_master *master;

	master = platform_get_drvdata(dev);
	mpc83xx_spi = spi_master_get_devdata(master);

	flush_workqueue(mpc83xx_spi->workqueue);
	destroy_workqueue(mpc83xx_spi->workqueue);
	spi_unregister_master(master);

	free_irq(mpc83xx_spi->irq, mpc83xx_spi);
	iounmap(mpc83xx_spi->base);

	return 0;
}

MODULE_ALIAS("platform:mpc83xx_spi");
static struct platform_driver mpc83xx_spi_driver = {
	.remove = __exit_p(mpc83xx_spi_remove),
	.driver = {
		.name = "mpc83xx_spi",
		.owner = THIS_MODULE,
	},
};

static int __init mpc83xx_spi_init(void)
{
//	immap = ioremap(get_immrbase() + GPIO_ADDR_OFFSET + GPIO_DAT, GPIO_REG_SIZE);  //added by yangyf at 2015-4-22
	return platform_driver_probe(&mpc83xx_spi_driver, mpc83xx_spi_probe);
}

static void __exit mpc83xx_spi_exit(void)
{
	iounmap(immap);  //added by yangyf at 2015-4-22
	platform_driver_unregister(&mpc83xx_spi_driver);
}

module_init(mpc83xx_spi_init);
module_exit(mpc83xx_spi_exit);

MODULE_AUTHOR("Kumar Gala");
MODULE_DESCRIPTION("Simple MPC83xx SPI Driver");
MODULE_LICENSE("GPL");
