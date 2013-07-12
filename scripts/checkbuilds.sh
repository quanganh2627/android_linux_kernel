#!/bin/bash

ALLDEFCONFIGS="`ls arch/x86/configs/i386_*_defconfig`"
NJOBS=`cat /proc/cpuinfo | grep processor | wc -l`

failed()
{
	make mrproper
	echo
	echo "Build failed!"
	exit 1
}

for conf in $ALLDEFCONFIGS; do
	echo "Check $conf: [Y/a/n] "
	if [ "$ALL" != "1" ]; then
		read -s -n1 ANSWER < /dev/tty
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

