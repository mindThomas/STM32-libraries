/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include "MPU9250_Bus.h"
#include "I2C.h"  // I2C library
#include "SPI.h" // SPI Library
		
/* writes a byte to MPU9250 register given a register address and data */
bool MPU9250_I2C::writeRegister(uint8_t subAddress, uint8_t data){
	uint8_t buff;

	_bus->Write(subAddress, data);
	osDelay(10); // need to slow down how fast I write to MPU9250

	/* read back the register */
	buff = _bus->Read(subAddress);

	/* check the read back register against the written register */
	if(buff == data) {
		return true;
	}
	else{
		return false;
	}
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */		
void MPU9250_I2C::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	_bus->Read(subAddress, dest, count);
}

		
/* writes a byte to MPU9250 register given a register address and data */
bool MPU9250_SPI::writeRegister(uint8_t subAddress, uint8_t data){
	uint8_t buff;

	_bus->Write(subAddress, data);
	osDelay(10); // need to slow down how fast I write to MPU9250

	/* read back the register */
	buff = _bus->Read(subAddress | 0x80); // set top bit to perform read

	/* check the read back register against the written register */
	if(buff == data) {
		return true;
	}
	else{
		return false;
	}
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */		
void MPU9250_SPI::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	_bus->Read(subAddress | 0x80, dest, count);
}

void MPU9250_SPI::setBusLowSpeed()
{
	_bus->ReconfigureFrequency(SPI_LOW_FREQUENCY);
}

void MPU9250_SPI::setBusHighSpeed()
{
	_bus->ReconfigureFrequency(SPI_HIGH_FREQUENCY);
}
