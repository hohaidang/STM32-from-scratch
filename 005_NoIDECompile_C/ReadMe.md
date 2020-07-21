// -c just compile not to link
arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb main.c -o main.o

// generate only assembly file
arm-none-eabi-gcc -S -mcpu=cortex-m4 -mthumb main.c -o main.s