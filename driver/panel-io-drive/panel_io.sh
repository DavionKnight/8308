#!/bin/sh

insmod panel_io_driver.ko
mknod /dev/panel_io c 231 0
./test_panel

insmod spi_drv.ko
mknod /dev/spi_drv c 207 0
