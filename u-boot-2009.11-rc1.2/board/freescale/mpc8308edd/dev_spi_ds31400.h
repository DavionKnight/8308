#ifndef _DEV_SPI_DS31400_H_
#define _DEV_SPI_DS31400_H_

struct spi_slave *spi_slave_init(void);
int spi_slave_free(struct spi_slave *slave);
int ds31400_read(struct spi_slave *spi, unsigned short addr, void *data, size_t data_len);
//int ds31400_write(struct spi_slave *spi, unsigned short addr, const void *data, size_t data_len);




#endif
