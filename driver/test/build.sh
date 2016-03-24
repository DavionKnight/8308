#!/bin/bash

export PATH=/home/kevin/Documents/ppc-tools/usr/bin:$PATH

rm ./build -rf


make ARCH=powerpc CROSS_COMPILE=ppc_82xx- 

cp spitest.ko  /tftpboot/


