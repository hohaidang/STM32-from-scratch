/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include <iostream>
#include <string>
using namespace std;

// Const data stored in ROM (Flash Memory)
const string pMessage = "Hi there, good morning !";

// Const data stored in ROM (Flash Memory)
const char *dang = "123";

// Const data stored in ROM (Flash Memory)
const int value = 200;

// this is not a constant data. This will be stored in SRAM
char myData[50];


int main(void)
{
	int data = value;
	for(uint32_t i = 0; i < pMessage.size(); ++i) {
		myData[i] = pMessage[i];
	}
	for(;;);
}