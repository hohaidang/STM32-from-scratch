CC= arm-none-eabi-gcc
CPP= arm-none-eabi-g++
MACH= cortex-m4
CFLAGS= -c -mcpu=$(MACH) -mthumb -std=c++11 -Wall -O0 -fno-exceptions
# linker flag, -Map -> create map file
LDFLAGS= -nostdlib -T stm32_ls.ld -Wl,-Map=final.map

# $^: dependency (main.cpp)
# $@: target (main.o)
all: main.o stm32f446re_gpio_driver.o stm32_startup.o final.elf

main.o:main.cpp
	$(CPP) $(CFLAGS) -o $@ $^

stm32f446re_gpio_driver.o:stm32f446re_gpio_driver.cpp
	$(CPP) $(CFLAGS) -o $@ $^
	
stm32_startup.o:stm32_startup.cpp
	$(CPP) $(CFLAGS) -o $@ $^

final.elf: main.o stm32f446re_gpio_driver.o stm32_startup.o
	$(CPP) $(LDFLAGS) -o $@ $^
	
clean:
	del -rf *.o *.elf
	
load:
	openocd -f board/stm32f4discovery.cfg