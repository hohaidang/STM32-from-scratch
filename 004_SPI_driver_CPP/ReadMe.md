# SPI driver for reading sensor data

# Environment

Nuclero STM32F446RE with ARM Cortex M4

BME280 Bosch sensor for reading temperature, humidity, pressure data.

Saleae Black 4 Logic Analyzer for debugging

MacOS - BigSur

STM32Cube 1.4

# PIN

This project is using SPI1 for communication
PA5 - SPI1_SCK
PA6 - SPI1_MISO
PA7 - SPI1_MOSI
PB6 - GPIO Output for slave select
![Sensor_Board_Setup1](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/Board_Sensor_1.jpeg)
![Sensor_Board_Setup2](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/Board_Sensor_2.jpeg)
![Read_SensorData](https://github.com/hohaidang/STM32-from-scratch/blob/master/Documents/Images/ReadSensor.png)
