#!/bin/bash

#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
#;; Revisions	:
#;; Date		Version		Author		description
#;; 2020/11/05		1.0		kibum.lee	inital version
#;; 2021/04/06		1.1		kibum.lee	change script for wide model
#;; 2025/05/14		1.2		kibum.lee	refine kernel build script
#=====================================================================
#			environment variable
#---------------------------------------------------------------------

CURDIR=`pwd`
CPUNUM=`grep -c processor /proc/cpuinfo`

DEFCONFIG_DIR=${CURDIR}/kernel/arch/arm/configs
UTIL_PATH=prebuilts
KERNEL_CONFIG=daudiokk_defconfig
SOURCE_KERNEL_DIR=${CURDIR}/kernel
OBJECT_KERNEL_DIR=${CURDIR}/out
OBJ_KERNEL_CONFIG_FILE=${OBJECT_KERNEL_DIR}/.config
KERNEL_VMLINUX_FILE=${OBJECT_KERNEL_DIR}/vmlinux

ARCH_T=arm
TOOLCHAIN_PATH=${CURDIR}/toolchain/arm-eabi-4.7/bin
CROSS_COMPILE_T=${TOOLCHAIN_PATH}/arm-eabi-
OUT_DTS_ROOT=${OBJECT_KERNEL_DIR}/arch/arm/boot/dts
SRC_DTS_ROOT=./kernel/arch/arm/boot/dts
SRC_DTS_DIR=${SRC_DTS_ROOT}/tcc


rm -f ${SRC_DTS_DIR}/tcc8971-android-lcn.dtsi
cat ${SRC_DTS_DIR}/daudiokkwide_common.dtsi ${SRC_DTS_DIR}/daudiokk_fm8802.dtsi ${SRC_DTS_DIR}/daudiokk_kor.dtsi ${SRC_DTS_DIR}/daudiokk_3rd_board.dtsi ${SRC_DTS_DIR}/daudiokk_hdmi.dtsi ${SRC_DTS_DIR}/daudiokk_2G.dtsi > ${SRC_DTS_DIR}/tcc8971-android-lcn.dtsi

rm -f ${DEFCONFIG_DIR}/${KERNEL_CONFIG}
cat ${DEFCONFIG_DIR}/daudiokk_common_defconfig ${DEFCONFIG_DIR}/daudiokk_2G_defconfig ${DEFCONFIG_DIR}/daudiokk_hdmi_defconfig ${DEFCONFIG_DIR}/daudiokk_intrtc_defconfig ${DEFCONFIG_DIR}/daudiokk_lgit_4G_9x28_dual_modem_defconfig > ${DEFCONFIG_DIR}/${KERNEL_CONFIG}

#=============================================================================================

usage() {
	echo ""
        echo "Usage: ./mobis_kernel_build.sh {clean}"
        echo "       exam) ./mobis_kernel_build.sh"
	echo ""
}
#=============================================================================================

if [ ! -d ${TOOLCHAIN_PATH} ]; then 
	echo ""
	echo "You have to install toolchain!!!"
	echo ""
	exit 1
fi;

if [ $# -gt 1 ]  ; then
        usage
        exit 1
fi

if [ "$1" == "clean" ]; then
	echo ""
	echo "delete out directory!!!"
	rm -rf ${OBJECT_KERNEL_DIR}
fi

if [ ! -d ${OBJECT_KERNEL_DIR} ] ; then
	mkdir -p ${OBJECT_KERNEL_DIR}
fi

if [ ! -f ${OBJ_KERNEL_CONFIG_FILE} ]; then
	echo ""
	pushd ${SOURCE_KERNEL_DIR}
	make ARCH=${ARCH_T} CROSS_COMPILE=${CROSS_COMPILE_T} -C ${SOURCE_KERNEL_DIR} O=${OBJECT_KERNEL_DIR} ${KERNEL_CONFIG}
fi

if [ -f ${OBJ_KERNEL_CONFIG_FILE} ]; then
	echo ""
	echo "kernel configure successed!!!"
fi

make ARCH=${ARCH_T} CROSS_COMPILE=${CROSS_COMPILE_T} -C ${SOURCE_KERNEL_DIR} zImage O=${OBJECT_KERNEL_DIR} -j${CPUNUM}
if [ -f ${KERNEL_VMLINUX_FILE} ]; then
	echo ""
	echo "kernel build successed!!!"
fi
popd



