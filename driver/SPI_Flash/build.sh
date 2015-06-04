#!/bin/sh

export ARCH=powerpc
export PATH=/opt/eldk42/usr/bin:/opt/eldk42/bin:$PATH
export CROSS_COMPILE=ppc_82xx-


ppc_82xx-gcc epcs_test.c -o epcs_test

cp epcs_test /tftpboot/
