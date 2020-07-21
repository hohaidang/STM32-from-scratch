#Build command
#Reference: https://www.youtube.com/watch?v=4RjEmcT6JpM
// -c just compile not to link
arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb main.c -o main.o

// generate only assembly file
arm-none-eabi-gcc -S -mcpu=cortex-m4 -mthumb main.c -o main.s

arm-none-eabi-objdump -h main.o 
arm-none-eabi-objdump -d main.o > main_log

// apply linker
arm-none-eabi-gcc -nostdlib -T stm32_ls.ld *.o -o final.elf


// write flash 
arm-none-eabi-gdb.exe
target remote localhost:3333
monitor reset init
monitor flash write_image erase final.elf
monitor reset halt
monitor resume