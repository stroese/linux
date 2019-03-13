#!/bin/bash

DEFCONFIG=gardena-smart-gateway-at91sam_defconfig
BOARD=at91sam9g25-gardena-smart-gateway
DESTDIR=/tftpboot/gardena-smart-gateway-at91sam
DTB=$BOARD.dtb

if [ "$1" == "savedefconfig"  ]
then
    echo make savedefconfig
    make savedefconfig
    cp defconfig arch/$ARCH/configs/$DEFCONFIG
    exit 0
fi

if [ "$1" == "config"  ]
then
    echo make mrproper
    make mrproper

    echo make $DEFCONFIG ...
    make $DEFCONFIG
    echo ... done

    sleep 1
fi

# build the image (vmlinux.bin) and dtb
LOADADDR=0x20008000 make -j$BUILD_CPUS zImage
LOADADDR=0x20008000 make -j$BUILD_CPUS $DTB

# generate FIT image and copy it to the dest directoy
mkimage -f $BOARD.its $BOARD.itb
cp $BOARD.itb $DESTDIR
