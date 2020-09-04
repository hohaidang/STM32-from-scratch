# Overview

This project is using cmake for build code for STM32 boards, the tested board is nucleo-f446re. If users use the different board,
you need to change the CMakeLists.txt file

Haven't Test on board for working or not :)

Reference: https://dev.to/younup/cmake-on-stm32-the-beginning-3766

# Environment

Windows 10 (tested)

mingw32 (gcc and make), CubeMX, arm-none-eabi-gcc compiler.

# Build Command

$ mkdir cmake-build-debug // this folder will be used to store binary file after build

$ cd cmake-build-debug

$ cmake -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Debug ..

$ cmake --build .

Normal build make file

$ mingw32-make.exe

![cmake_build_img](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/cmake_build_img.PNG)
