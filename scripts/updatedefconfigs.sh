#!/bin/bash
#
# This script is meant to update all defconfigs when any Kconfig changes.
# Just go to kernel root directory and execute:
# $ scripts/updatedefconfigs.sh
#
# For each i386_XXX_defconfig inside arch/x86/configs it will ask to run either
# make oldconfig, make menuconfig or make xconfig.
# In the end all defconfigs will be updated and ready for a patch.

ALLDEFCONFIGS="`ls arch/x86/configs/i386_*_defconfig`"

for conf in $ALLDEFCONFIGS; do
	echo "Updating $conf (O=oldconfig, m=menuconfig, x=xconfig): [O/m/x default=O]?"
	read -s -n1 ANSWER < /dev/tty
	cp $conf .config

	case "$ANSWER" in
		[mM] )
			make ARCH=i386 menuconfig
		;;
		[xX] )
			make ARCH=i386 xconfig
		;;
		 *)
			make ARCH=i386 oldconfig
		;;
	esac
	cp .config $conf
done;

make mrproper

