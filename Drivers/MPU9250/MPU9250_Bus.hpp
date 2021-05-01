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
 
#ifndef MPU9250_BUS_H
#define MPU9250_BUS_H

#include <IMU/IMU.hpp>
#include <I2C/I2C.hpp>  // I2C library
#include <SPI/SPI.hpp> // SPI Library

class MPU9250_Bus
{
	public:
		static const int I2C_FREQUENCY = 400000;			// 400 kHz
		static const int SPI_LOW_FREQUENCY = 1000000;		// 1 MHz
		static const int SPI_HIGH_FREQUENCY = 1000000;		// 10 MHz

	public:
		virtual ~MPU9250_Bus() {};
		virtual bool writeRegister(uint8_t subAddress, uint8_t data) { return false; };
		virtual void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) {};
		virtual void setBusLowSpeed() {};
		virtual void setBusHighSpeed() {};
};

class MPU9250_I2C : public MPU9250_Bus
{
	public:
		MPU9250_I2C(I2C * bus) : _bus(bus) {};
		~MPU9250_I2C() {};
		
		/* writes a byte to MPU9250 register given a register address and data */
		bool writeRegister(uint8_t subAddress, uint8_t data);

		/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */		
		void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
		
	private:
		I2C * _bus;
};

class MPU9250_SPI : public MPU9250_Bus
{
	public:
		MPU9250_SPI(SPI * bus) : _bus(bus) {};
		~MPU9250_SPI() {};
		
		/* writes a byte to MPU9250 register given a register address and data */
		bool writeRegister(uint8_t subAddress, uint8_t data);

		/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */		
		void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
		
		void setBusLowSpeed();

		void setBusHighSpeed();

	private:
		SPI * _bus;
};

#endif
