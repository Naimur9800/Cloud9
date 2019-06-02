#!/bin/bash
# v1.0
# R3X Kernel build script

DEFCONFIG=v3702_defconfig
DARCH_PLATFORM=mt6580
DARCH=arm
OUTDIR=out
TCPATH=/home/ubuntu/workspace/armebi-4.8-6580

JOBS=`grep processor /proc/cpuinfo | wc -l`

# Colors
cyan='\033[0;36m'
yellow='\033[0;33m'
red='\033[0;31m'
nocol='\033[0m'

function build() {
	BUILD_START=$(date +"%s");
	echo -e "$cyan"
	echo "***********************************************";
	echo "              Compiling R3X kernel          	     ";
	echo -e "***********************************************$nocol";
	echo -e "$red";
    mkdir -p $OUTDIR
    export ARCH=$DARCH ARCH_MTK_PLATFORM=$DARCH_PLATFORM
    make -C $PWD O=$PWD/$OUTDIR ARCH=$DARCH $DEFCONFIG
    make -j${JOBS} -C $PWD O=$PWD/$OUTDIR ARCH=$DARCH
    
	BUILD_END=$(date +"%s");
	DIFF=$(($BUILD_END - $BUILD_START));
	echo -e "$yellow";
	echo -e "Build completed in $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds.$nocol";
}

function clean() {
	echo -e "$red";
	echo -e "Cleaning build environment...$nocol";
	make -j${JOBS} mrproper;
	rm_if_exist ${OUTPUT_PATH};
	echo -e "$yellow";
	echo -e "Done!$nocol";
}

function main() {
    echo -e "$red";
    export CROSS_COMPILE=$TCPATH/bin/arm-eabi-
    echo -e "***************************************************************";
	echo "                R3X Kernel for Wiko MT6580 Devices";
	echo -e "***************************************************************";
	echo "Choices:";
	echo "1. Cleanup source";
	echo "2. Build kernel";
	echo "3. Build kernel then make flashable ZIP";
	echo "4. Make flashable ZIP package";
	echo "Leave empty to exit this script (it'll show invalid choice)";

	read -n 1 -p "Select your choice: " -s choice;
	case ${choice} in
		1) clean;;
		2) build;;
		3) build
		   make_zip;;
		4) make_zip;;
		*) echo
		   echo "Invalid choice entered. Exiting..."
		   sleep 2;
		   exit 1;;
	esac
}

main $@
