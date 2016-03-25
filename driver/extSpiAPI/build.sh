#!/bin/bash

export PATH=/home/kevin/Documents/ppc-tools/usr/bin:/home/kevin/Documents/ppc-tools/bin:$PATH

export CROSS_COMPILE=ppc_82xx-
make ARCH=powerpc  

ppc_82xx-gcc spi_api.c -o spi_api
ppc_82xx-gcc spidev_test.c -o spitest

cp spi_api /tftpboot/
cp spiApi.ko  /tftpboot/


