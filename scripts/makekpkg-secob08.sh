#!/bin/bash
#
# Script for Debian Package 
# Author: Ettore Chimenti <ek5.chimenti@gmail.com>
#

set -e

#board specific
dtb=( imx6sx-seco-b08{,-lvds7,-lcdif}.dtb )
defconfig="seco_b08_config"
export LOCALVERSION="-secob08" 

#generic
threads=$(grep -c 'processor' /proc/cpuinfo)
export KCFLAGS="-O2 -march=armv7-a -mtune=cortex-a9 -mfpu=vfpv3-d16 -pipe -fomit-frame-pointer"
export ARCH="arm"
export CROSS_COMPILE="/usr/bin/arm-linux-gnueabihf-"
export KBUILD_DEBARCH="armhf" 
export KBUILD_IMAGE="zImage"
export BUILD_HEADERS="yes"

#author
export DEBFULLNAME="UDOO Team" 
export EMAIL="social@udoo.org" 

#compile
make -j${threads} clean
make -j${threads} $defconfig
make -j${threads} ${dtb[*]}
make -j${threads} zImage modules
make -j${threads} deb-pkg
