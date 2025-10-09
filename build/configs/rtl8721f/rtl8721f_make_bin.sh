#!/usr/bin/env bash
###########################################################################
#
# Copyright 2025 Samsung Electronics All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific
# language governing permissions and limitations under the License.
#
###########################################################################
THIS_PATH=`test -d ${0%/*} && cd ${0%/*}; pwd`
TOP_PATH=${THIS_PATH}/../../..
CONFIG=${TOP_PATH}/os/.config
BUILDDIR=${TOP_PATH}/build
BINDIR=${BUILDDIR}/output/bin
GNUUTL=${BUILDDIR}/tools/amebagreen2/gnu_utility
TOOL_PATH=${THIS_PATH}/../../tools/amebagreen2
BOOT_PATH=${TOOL_PATH}/bootloader
FLOADER_PATH=${TOOL_PATH}/flashloader

echo "Realtek Postbuild Actions"
export TOPDIR=$BUILDDIR/

cp $BINDIR/tinyara.axf $BINDIR/target_img2.axf

arm-none-eabi-nm $BINDIR/target_img2.axf | sort > $BINDIR/target_img2.map

arm-none-eabi-objdump -d $BINDIR/target_img2.axf > $BINDIR/target_img2.asm

#arm-none-eabi-objdump -h -S --disassemble-all $BINDIR/target_img2.axf > $BINDIR/target_img2.txt

cp $BINDIR/target_img2.axf $BINDIR/target_pure_img2.axf

arm-none-eabi-strip $BINDIR/target_pure_img2.axf

arm-none-eabi-objcopy -j .psram_image2.text.data -j .ARM.extab -j .ARM.exidx -Obinary $BINDIR/target_pure_img2.axf $BINDIR/psram_2.bin
arm-none-eabi-objcopy -j .sram_image2.text.data -Obinary $BINDIR/target_pure_img2.axf $BINDIR/sram_2.bin
arm-none-eabi-objcopy -j .xip_image2.text -Obinary $BINDIR/target_pure_img2.axf $BINDIR/xip_image2.bin



#For Bluetooth Trace
if [ "${CONFIG_BT_EN}" == "y" ];then
	arm-none-eabi-objcopy -j .bluetooth_trace.text \
	-Obinary $BINDIR/target_pure_img2.axf $BINDIR/APP.trace
fi

echo "========== Image Info HEX =========="
arm-none-eabi-size -A --radix=16 $BINDIR/target_img2.axf
arm-none-eabi-size -t --radix=16 $BINDIR/target_img2.axf

echo "========== Image Info DEC =========="
arm-none-eabi-size -A --radix=10 $BINDIR/target_img2.axf
arm-none-eabi-size -t --radix=10 $BINDIR/target_img2.axf

echo "========== Image manipulating start =========="

# Pad binaries (32 bytes alignment)
$GNUUTL/pad_align.sh $BINDIR/psram_2.bin 32
$GNUUTL/pad_align.sh $BINDIR/sram_2.bin 32
$GNUUTL/pad_align.sh $BINDIR/xip_image2.bin 32

# Prepend headers
$GNUUTL/prepend_header.sh $BINDIR/sram_2.bin __sram_image2_start__ $BINDIR/target_img2.map
$GNUUTL/prepend_header.sh $BINDIR/psram_2.bin __psram_image2_start__ $BINDIR/target_img2.map
$GNUUTL/prepend_header.sh $BINDIR/xip_image2.bin __flash_text_start__ $BINDIR/target_img2.map


cat $BINDIR/xip_image2_prepend.bin $BINDIR/sram_2_prepend.bin $BINDIR/psram_2_prepend.bin > $BINDIR/km4tz_image2_all.bin

#python $GNUUTL/axf2bin.py imagetool $BINDIR/km4tz_image2_all.bin



function copy_bootloader()
{
	if [ ! -f ${CONFIG} ];then
		echo "No .config file"
		exit 1
	fi

	source ${CONFIG}

	echo "========== Copy_bootloader =========="
	if [ ! -f $BOOT_PATH/amebagreen2_boot.bin ];then
		echo "No amebagreen2_boot.bin"
		exit 1
	fi
	cp $BOOT_PATH/amebagreen2_boot.bin $BINDIR/amebagreen2_boot.bin

}

function concatenate_binary_without_signing()
{
	if [ ! -f ${CONFIG} ];then
		echo "No .config file"
		exit 1
	fi

	source ${CONFIG}

 	echo "========== Concatenate_binary =========="

	if [ ! -f $BINDIR/km4ns_image2_all.bin ] || [ ! -f $BINDIR/km4tz_image2_all.bin ];then
		echo "No km4ns_image2_all.bin or km4tz_image2_all.bin"
		exit 1
	fi
 	#cp $BINDIR/km4ns_image2_all.bin $GNUUTL/km4ns_image2_all_temp.bin
 	# $GNUUTL/rmcert.sh $GNUUTL/km4ns_image2_all_temp.bin
 	# $GNUUTL/pad.sh $GNUUTL/km4ns_image2_all_temp.bin
 	# cat $GNUUTL/cert.bin $GNUUTL/km4ns_image2_all_temp.bin $BINDIR/km4tz_image2_all.bin > $BINDIR/amebagreen2_app.bin
 	cat $GNUUTL/app_cert.bin $GNUUTL/manifest_app.bin $GNUUTL/km4ns_image2_all.bin $BINDIR/km4tz_image2_all.bin > $BINDIR/amebagreen2_app.bin
 	#rm -rf $GNUUTL/km4ns_image2_all_temp.bin

}

function concatenate_binary_with_signing()
{
	if [ ! -f ${CONFIG} ];then
		echo "No .config file"
		exit 1
	fi

	source ${CONFIG}

	#signing
	echo "========== Binary SIGNING =========="
	bash $BUILDDIR/configs/rtl8721f/rtl8721f_signing.sh kernel
}

#*****************************************************************************#
#              COPY flashloader into bin output folder                        #
#*****************************************************************************#
function copy_flashloader()
{
	if [ ! -f ${CONFIG} ];then
		echo "No .config file"
		exit 1
	fi

	source ${CONFIG}

	echo "========== Copy flashloader into bin output folder=========="
	cp $FLOADER_PATH/flash_loader_ram_1.bin $BINDIR/flash_loader_ram_1.bin
	cp $FLOADER_PATH/target_FPGA.axf $BINDIR/target_FPGA.axf
}

#*****************************************************************************#
#              COPY km0_km4_image_all into bin output folder                       #
#*****************************************************************************#
function copy_km0_km4_image()
{
	if [ ! -f ${CONFIG} ];then
		echo "No .config file"
		exit 1
	fi

	source ${CONFIG}

	echo "========== Copy km4ns_image2_all into bin output folder=========="
	if [ ! -f $GNUUTL/km4ns_image2_all.bin ];then
		echo "No km4ns_image2_all.bin"
		exit 1
	fi
	cp $GNUUTL/km4ns_image2_all.bin $BINDIR/km4ns_image2_all.bin
}

#*****************************************************************************#
#              Temporary: Remove the Large Size Binary                        #
#*****************************************************************************#
function remove_large_binary_temp()
{
	if [ ! -f ${CONFIG} ];then
		echo "No .config file"
		exit 1
	fi

	source ${CONFIG}

	echo "========== Remove large size binary, temporary fix=========="
	if [ ! -f $BINDIR/tinyara.axf.bin ];then
		echo "No tinyara.axf.bin"
		exit 1
	fi
	rm $BINDIR/tinyara.axf.bin
}


copy_bootloader;
copy_flashloader;
copy_km0_km4_image;
if [ "${CONFIG_BINARY_SIGNING}" == "y" ];then
	concatenate_binary_with_signing;

	# Binary Signing is not support in public, so copy the non-signing binary
	concatenate_binary_without_signing;
else
	concatenate_binary_without_signing;
fi
remove_large_binary_temp;
 
