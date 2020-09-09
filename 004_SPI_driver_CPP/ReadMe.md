# SPI driver for reading sensor data

# Environment

Nuclero STM32F446RE with ARM Cortex M4

BME280 Bosch sensor for reading temperature, humidity, pressure data.

Saleae Black 4 Logic Analyzer for debugging

MacOS - BigSur or Windows 10

STM32Cube 1.4

# User Mannual

Download project, import to STM32 Cube with "Import ac6" option.

Remove Debug folder -> Re-built -> Debug

Main function is in *spi_readSensorData.cpp*

Configuration "cout" command for print information via Debugger (search on Internet) 

# PIN

This project is using SPI1 for communication

PA5 - SPI1_SCK

PA6 - SPI1_MISO

PA7 - SPI1_MOSI

PB6 - GPIO Output for slave select

VDD 3.3V

GND


![Sensor_Board_Setup1](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/Board_Sensor_1.jpeg)
![Sensor_Board_Setup2](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/Board_Sensor_2.jpeg)
![Read_SensorData](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/ReadSensor.png)
![Read_SensorData](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/ReadSensor_2.png)

# Google Unit Test Run:

$ bazel run unit_test:ut_gpio

# Generate test coverage report

Only available in Ubuntu (Tested 18.04)

Copy stm32f446re_gpio_driver.cpp, stm32f446re_gpio_driver.h, core_cm4.h, stm32f4xx.h into unit_test folder

$ bazel coverage unit_test:ut_gpio --combined_report=lcov

Generate html file

$ genhtml bazel-out/_coverage/_coverage_report.dat

Open index-sort-l.html

![unit_test](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/UT_004.png)
