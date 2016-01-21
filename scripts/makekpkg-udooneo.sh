#!/bin/bash
#
# Script for Debian Package 
# Author: Ettore Chimenti <ek5.chimenti@gmail.com>
#

set -e

#board specific
dtb=( imx6sx-udoo-neo-{full,extended,basicks,basic}{,-lvds{15,7},-hdmi}{,-m4}.dtb )
defconfig="udoo_neo_defconfig"
LOCALVERSION="-udooneo" 

#generic
threads=$(grep -c 'processor' /proc/cpuinfo)
KCFLAGS="-O2 -march=armv7-a -mtune=cortex-a9 -mfpu=vfpv3-d16 -pipe -fomit-frame-pointer"
ARCH="arm"
CROSS_COMPILE="/usr/bin/arm-linux-gnueabihf-"
KBUILD_DEBARCH="armhf" 
KBUILD_IMAGE="zImage"
BUILD_HEADERS="yes"

#author
DEBFULLNAME="UDOO Team" 
EMAIL="social@udoo.org" 

#compile
make -j${threads} clean
make -j${threads} $defconfig
make -j${threads} ${dtb[*]}
make -j${threads} zImage modules
make -j${threads} deb-pkg
