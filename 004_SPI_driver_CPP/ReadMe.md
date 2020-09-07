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

# Google Unit Test Run:

$ bazel run unit_test:ut_gpio

![Sensor_Board_Setup1](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/Board_Sensor_1.jpeg)
![Sensor_Board_Setup2](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/Board_Sensor_2.jpeg)
![Read_SensorData](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/ReadSensor.png)
![Read_SensorData](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/ReadSensor_2.png)
![unit_test](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/UT_004.png)
