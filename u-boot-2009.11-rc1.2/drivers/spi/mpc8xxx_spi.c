/*
 * Copyright (c) 2006 Ben Warren, Qstreams Networks Inc.
 * With help from the common/soft_spi and cpu/mpc8260 drivers
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>

#include <malloc.h>
#include <spi.h>
#include <asm/mpc8xxx_spi.h>
#include <gpio.h>

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

#define	SPMODE_INIT_VAL (SPMODE_REV | \
			 SPMODE_MS | SPMODE_LEN(7) | SPMODE_PM(0x5))

#define SPI_EV_NE	(0x80000000 >> 22)	/* Receiver Not Empty */
#define SPI_EV_NF	(0x80000000 >> 23)	/* Transmitter Not Full */

#define ESPI_EV_RNE		(1 << 9)
#define ESPI_EV_TNF		(1 << 8)

#define SPI_TIMEOUT	1000

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	if(cs>3)
		return 0;
	return 1;
}

void spi_cs_activate(struct spi_slave *slave)
{
	gpio_direction_output(slave->cs, 0);
}
void spi_cs_deactivate(struct spi_slave *slave)
{
	udelay(10);
	gpio_direction_output(slave->cs, 1);
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	struct spi_slave *slave;

	if (!spi_cs_is_valid(bus, cs))
		return NULL;

	slave = malloc(sizeof(struct spi_slave));
	if (!slave)
		return NULL;

	slave->bus = bus;
	slave->cs = cs;
	/*
	 * TODO: Some of the code in spi_init() should probably move
	 * here, or into spi_claim_bus() below.
	 */
//	spi_init();
	return slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	free(slave);
}
#if 0 /*delete for fx12*/
void spi_init(void)
{
	volatile spi8xxx_t *spi = &((immap_t *) (CONFIG_SYS_IMMR))->spi;

	/*
	 * SPI pins on the MPC83xx are not muxed, so all we do is initialize
	 * some registers
	 */
	spi->mode = 0;
	spi->mode = SPMODE_INIT_VAL | SPMODE_ENABLE;
//	spi->mode = (spi->mode & 0xfff0ffff) | (1 << 16); /* Use SYSCLK / 8   (16.67MHz typ.) */
	spi->event = 0xffffffff;	/* Clear all SPI events */
	spi->mask = 0x00000000;	/* Mask  all SPI interrupts */
	spi->com = 0;		/* LST bit doesn't do anything, so disregard */
	printf("mode = 0x%x\n",spi->mode);
}
#endif
int spi_claim_bus(struct spi_slave *slave)
{
	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{

}
#if 1
int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags)
{
	volatile spi8xxx_t *spi = &((immap_t *) (CONFIG_SYS_IMMR))->spi;
	unsigned int tmpdout, tmpdin, event;
	int numBlks = bitlen / 32 + (bitlen % 32 ? 1 : 0);
	int tm=0, isRead = 0;
	unsigned char charSize = 32;

	debug("spi_xfer: slave %u:%u dout %08X din %08X bitlen %u, CONFIG_SYS_IMMR=%x\n",
	      slave->bus, slave->cs, *(uint *) dout, *(uint *) din, bitlen,CONFIG_SYS_IMMR);

	if (flags & SPI_XFER_BEGIN)
		spi_cs_activate(slave);

	spi->event = 0xffffffff;	/* Clear all SPI events */
	/* handle data in 32-bit chunks */
	while (numBlks--) {
		tmpdout = 0;
		charSize = (bitlen >= 32 ? 32 : bitlen);

		/* Shift data so it's msb-justified */
		tmpdout = *(u32 *) dout >> (32 - charSize);

		/* The LEN field of the SPMODE register is set as follows:
		 *
		 * Bit length             setting
		 * len <= 4               3
		 * 4 < len <= 16          len - 1
		 * len > 16               0
		 */
		if (bitlen <= 16) {
			if (bitlen <= 4)
				spi->mode = (spi->mode & 0xff0fffff) |
				            (3 << 20);
			else
			{
				unsigned int aa=spi->mode;
				spi->mode = 0;
				spi->mode = (aa & 0xff0fffff) |
				            ((bitlen - 1) << 20|SPMODE_ENABLE);
			}
		} else {
			spi->mode = (spi->mode & 0xff0fffff);
			/* Set up the next iteration if sending > 32 bits */
			bitlen -= 32;
			dout += 4;
		}
//printf("spmode=0x%x\n",spi->mode);
//	printf("tmpdout=0x%x\n",tmpdout);
		spi->tx = tmpdout;	/* Write the data out */
		debug("*** spi_xfer: ... %08x written\n", tmpdout);

		/*
		 * Wait for SPI transmit to get out
		 * or time out (1 second = 1000 ms)
		 * The NE event must be read and cleared first
		 */
		for (tm = 0, isRead = 0; tm < SPI_TIMEOUT; ++tm) {
			event = spi->event;
			if (event & SPI_EV_NE) {
				tmpdin = spi->rx;
				spi->event |= SPI_EV_NE;
				isRead = 1;

				*(u32 *) din = (tmpdin << (32 - charSize));
				if (charSize == 32) {
					/* Advance output buffer by 32 bits */
					din += 4;
				}
			}
			/*
			 * Only bail when we've had both NE and NF events.
			 * This will cause timeouts on RO devices, so maybe
			 * in the future put an arbitrary delay after writing
			 * the device.  Arbitrary delays suck, though...
			 */
			if (isRead && (event & SPI_EV_NF))
				break;
		}
		if (tm >= SPI_TIMEOUT)
			puts("*** spi_xfer: Time out during SPI transfer");

		debug("*** spi_xfer: transfer ended. Value=%08x\n", tmpdin);
	}

	if (flags & SPI_XFER_END)
		spi_cs_deactivate(slave);

	return 0;
}
#else
unsigned char	cmd_buf[16];
size_t cmd_len_t =0;
size_t data_len_t =0;
int max_tran_len = 0xfff0;
int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *data_out,
		void *data_in, unsigned long flags)
{
	//ccsr_espi_t *espi = (void *)(CONFIG_SYS_MPC85xx_ESPI_ADDR);
	volatile spi8xxx_t *espi = &((immap_t *) (CONFIG_SYS_IMMR))->spi;
	unsigned char  tmpdout, tmpdin, event;
	unsigned char *dout = NULL;
	void *din = NULL;
	int len = 0;
	int num_blks, num_chunks, tran_len;
	int num_bytes;
	unsigned char *ch;
	unsigned char *buffer = NULL;
	size_t buf_len;
	size_t cmd_len = cmd_len_t;
	size_t data_len = bitlen / 8;
	size_t rx_offset = 0;

	switch (flags) {
	case SPI_XFER_BEGIN:
		cmd_len = cmd_len_t = data_len;
		memcpy(cmd_buf, data_out, cmd_len);
		return 0;
	case 0:
	case SPI_XFER_END:
		if (bitlen == 0) {
			spi_cs_deactivate(slave);
			return 0;
		}
		buf_len = 2 * cmd_len + min(data_len, max_tran_len);
		len = cmd_len + data_len;
		rx_offset = cmd_len;
		buffer = (unsigned char *)malloc(buf_len);
		if (!buffer) {
			debug("SF: Failed to malloc memory.\n");
			return 1;
		}
		memcpy(buffer, cmd_buf, cmd_len);
		if (data_in == NULL)
			memcpy(buffer + cmd_len, data_out, data_len);
		break;
	case SPI_XFER_BEGIN | SPI_XFER_END:
		len = data_len;
		buffer = (unsigned char *)malloc(len * 2);
		if (!buffer) {
			debug("SF: Failed to malloc memory.\n");
			return 1;
		}
		memcpy(buffer, data_out, len);
		rx_offset = len;
		cmd_len = 0;
		break;
	}

	debug("spi_xfer: slave %u:%u dout %08X(%p) din %08X(%p) len %u\n",
	      slave->bus, slave->cs, *(uint *) dout,
	      dout, *(uint *) din, din, len);

	num_chunks = data_len / max_tran_len + (data_len % max_tran_len ? 1 : 0);
	while (num_chunks--) {
		if (data_in)
			din = buffer + rx_offset;
		dout = buffer;
		tran_len = min(data_len , max_tran_len);
		num_blks = (tran_len + cmd_len) / 4 +
			((tran_len + cmd_len) % 4 ? 1 : 0);
		num_bytes = (tran_len + cmd_len) % 4;
		data_len_t = tran_len + cmd_len;
		spi_cs_activate(slave);

		/* Clear all eSPI events */
		espi->event = 0xffffffff;

		num_blks = 3;//modified
		/* handle data in 32-bit chunks */
		while (num_blks--) {

			event = espi->event;
printf("event=0x%x\n",event);
			if (event & ESPI_EV_TNF) {
				tmpdout = *(dout+2-num_blks);

	printf("tmpdout=0x%x\n",tmpdout);
				espi->tx= tmpdout;
				espi->event= ESPI_EV_TNF;
				debug("***spi_xfer:...%08x written\n", tmpdout);
			}

			/* Wait for eSPI transmit to get out */
			udelay(80);

			event = espi->event;
			if (event & ESPI_EV_RNE) {
				tmpdin = espi->rx;
				if (num_blks == 0 && num_bytes != 0) {
					ch = (unsigned char *)&tmpdin;
					while (num_bytes--)
						*(unsigned char *)din++ = *ch++;
				} else {
					*(u32 *) din = tmpdin;
					din += 4;
				}

				espi->event= espi->event
						| ESPI_EV_RNE;
				debug("***spi_xfer:...%08x readed\n", tmpdin);
			}
		}
		if (data_in) {
			memcpy(data_in, buffer + 2 * cmd_len, tran_len);
			if (*buffer == 0x0b) {
				data_in += tran_len;
				data_len -= tran_len;
				*(int *)buffer += tran_len;
			}
		}
		spi_cs_deactivate(slave);
	}

	free(buffer);
	return 0;
}

#endif
