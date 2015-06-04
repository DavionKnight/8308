#!/bin/sh

export ARCH=powerpc
export PATH=/opt/eldk42/usr/bin:/opt/eldk42/bin:$PATH
export CROSS_COMPILE=ppc_82xx-

make clean
rm /tftpboot/panel_io_driver.ko
rm /tftpboot/test_panel
make

ppc_82xx-gcc test_panel.c -o test_panel
ppc_82xx-gcc spidev_test.c -o spidev_test

cp test_panel /tftpboot/
cp panel_io.sh /tftpboot/
cp spidev_test /tftpboot/
