#!/bin/bash
#
# This script is meant to update all defconfigs when any Kconfig changes.
# Just go to kernel root directory and execute:
# $ scripts/updatedefconfigs.sh
#
# For each i386_XXX_defconfig inside arch/x86/configs it will ask to run either
# make oldconfig or make menuconfig.
# In the end all defconfigs will be updated and ready for a patch.

ALLDEFCONFIGS="`ls arch/x86/configs/i386_*_defconfig`"

for conf in $ALLDEFCONFIGS; do
	echo "Updating $conf (O=oldconfig, m=menuconfig): [O/m] "
	read -s -n1 ANSWER < /dev/tty
	cp $conf .config

	if [ "$ANSWER" = "m" ]; then
		make ARCH=i386 menuconfig
	else
		make ARCH=i386 oldconfig
	fi
	cp .config $conf
done;

make mrproper

