#! /bin/sh

STM32G0_URL=https://raw.githubusercontent.com/STMicroelectronics/STM32CubeG0/master
CMSISG0_URL=https://raw.githubusercontent.com/STMicroelectronics/cmsis-device-g0/master

mkdir CMSIS

HEADERS="\
  Include/cmsis_compiler.h \
  Include/cmsis_gcc.h \
  Include/cmsis_version.h \
  Include/core_cm0plus.h \
  Include/mpu_armv7.h \
"

for file in $HEADERS
do
  name=`basename $file`
  curl -L $STM32G0_URL/Drivers/CMSIS/$file -o CMSIS/$name
  sed -i 's/\ *$//' CMSIS/$name
done

HEADERS="\
  Include/stm32g071xx.h \
  Include/stm32g0xx.h \
  Include/system_stm32g0xx.h \
"

for file in $HEADERS
do
  name=`basename $file`
  curl -L $CMSISG0_URL/$file -o CMSIS/$name
  sed -i 's/\ *$//' CMSIS/$name
done
