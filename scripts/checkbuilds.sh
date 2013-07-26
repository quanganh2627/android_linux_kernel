#!/bin/bash

ALLDEFCONFIGS="`ls arch/x86/configs/i386_*_defconfig`"
NJOBS=`cat /proc/cpuinfo | grep processor | wc -l`

usage()
{
	echo "Help not implemented yet..."
	echo
}

failed()
{
	make mrproper
	echo
	echo "Build failed!"
	exit 1
}

ALL=1

while getopts "i" OPTION; do
	case $OPTION in
	i)
		ALL=0
		;;
	?)
		usage
		exit 1
		;;
	esac
done

for conf in $ALLDEFCONFIGS; do
	echo -n "Check $conf: "
	if [ "$ALL" != "1" ]; then
		echo "[Y/a/n] "
		read -s -n1 ANSWER < /dev/tty
	else
		echo
	fi
	cp $conf .config

	if [ "$ANSWER" = "a" ]; then
		ALL=1
	elif [ "$ANSWER" = "n" ]; then
		continue
	fi
	echo
	echo "Building..."
	echo
	make ARCH=i386 clean
	make ARCH=i386 -j$NJOBS bzImage modules || failed
done;

make mrproper

echo
echo "All builds successfully done!"

