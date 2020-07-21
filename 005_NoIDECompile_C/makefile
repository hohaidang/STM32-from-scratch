CC= arm-none-eabi-gcc
MACH= cortex-m4
CFLAGS= -c -mcpu=$(MACH) -mthumb -std=gnu11 -Wall -O0
# linker flag, -Map -> create map file
LDFLAGS= -nostdlib -T stm32_ls.ld -Wl,-Map=final.map

# $^: dependency (main.c)
# $@: target (main.o)
all: main.o stm32f407xx_gpio_driver.o stm32_startup.o final.elf

main.o:main.c
	$(CC) $(CFLAGS) -o $@ $^

stm32f407xx_gpio_driver.o:stm32f407xx_gpio_driver.c
	$(CC) $(CFLAGS) -o $@ $^
	
stm32_startup.o:stm32_startup.c
	$(CC) $(CFLAGS) -o $@ $^

final.elf: main.o stm32f407xx_gpio_driver.o stm32_startup.o
	$(CC) $(LDFLAGS) -o $@ $^
	
clean:
	del -rf *.o *.elf
	
load:
	openocd -f board/stm32f4discovery.cfg