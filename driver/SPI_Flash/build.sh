#!/bin/sh

export ARCH=powerpc
export PATH=/home/freddie/Compiler_PowerPC/usr/bin:/home/freddie/Compiler_PowerPC/bin:$PATH
export CROSS_COMPILE=ppc_82xx-


ppc_82xx-gcc epcs_test.c -o epcs_test

cp epcs_test /home/freddie/Documents/TFTP/CFG_PowerPC/
