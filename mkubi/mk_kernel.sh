#!/bin/sh

mkfs.ubifs -r ./bin -m 2048 -e 129024 -c 300 -o kernel.img

cp kernel.img /tftpboot/
