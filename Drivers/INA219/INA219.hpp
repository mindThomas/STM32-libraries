/*!
 * @file Adafruit_INA219.h
 *
 * This is a library for the Adafruit INA219 breakout board
 * ----> https://www.adafruit.com/products/904
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

/* Modified version for use in STM32H7 project by
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */ 
 
#pragma once

#include <IO/IO.hpp>
#include <I2C/I2C.hpp>

class INA219 {
	public:
		/**************************************************************************/
		/*!
			@brief  default I2C address
		*/
		/**************************************************************************/
			static const uint8_t I2C_ADDRESS1       =               (0x40);    // 1000000 (A0=GND, A1=GND)
			static const uint8_t I2C_ADDRESS2       =               (0x41);    // 1000001 (A0=VCC, A1=GND)
			static const uint8_t I2C_ADDRESS3       =               (0x44);    // 1000100 (A0=GND, A1=VCC)
			static const uint8_t I2C_ADDRESS4       =               (0x45);    // 1000101 (A0=VCC, A1=VCC)

	private:
		/**************************************************************************/
		/*!
			@brief  read
		*/
		/**************************************************************************/
			const uint8_t READ              =              (0x01);

		/*=========================================================================
			CONFIG REGISTER (R/W)
			-----------------------------------------------------------------------*/

		/**************************************************************************/
		/*!
			@brief  config register address
		*/
		/**************************************************************************/
			const uint8_t REG_CONFIG         =             (0x00);
			/*---------------------------------------------------------------------*/

		/**************************************************************************/
		/*!
			@brief  reset bit
		*/
		/**************************************************************************/
			const uint8_t CONFIG_RESET       =             (0x8000);  // Reset Bit

		/**************************************************************************/
		/*!
			@brief  mask for bus voltage range
		*/
		/**************************************************************************/
			const uint8_t CONFIG_BVOLTAGERANGE_MASK   =    (0x2000);  // Bus Voltage Range Mask

		/**************************************************************************/
		/*!
			@brief  bus voltage range values
		*/
		/**************************************************************************/
		typedef enum CONFIG_BVOLTAGERANGE {
			CONFIG_BVOLTAGERANGE_16V =        (0x0000),  // 0-16V Range
			CONFIG_BVOLTAGERANGE_32V =        (0x2000),  // 0-32V Range
		} CONFIG_BVOLTAGERANGE;

		/**************************************************************************/
		/*!
			@brief  mask for gain bits
		*/
		/**************************************************************************/
			const uint8_t CONFIG_GAIN_MASK       =         (0x1800);  // Gain Mask

		/**************************************************************************/
		/*!
			@brief  values for gain bits
		*/
		/**************************************************************************/
		typedef enum CONFIG_GAIN {
			CONFIG_GAIN_1_40MV        =       (0x0000),  // Gain 1, 40mV Range
			CONFIG_GAIN_2_80MV        =       (0x0800),  // Gain 2, 80mV Range
			CONFIG_GAIN_4_160MV       =       (0x1000),  // Gain 4, 160mV Range
			CONFIG_GAIN_8_320MV       =       (0x1800),  // Gain 8, 320mV Range
		  } CONFIG_GAIN;

		  /**************************************************************************/
		/*!
			@brief  mask for bus ADC resolution bits
		*/
		/**************************************************************************/
			const uint8_t CONFIG_BADCRES_MASK      =       (0x0780);  // Bus ADC Resolution Mask

		/**************************************************************************/
		/*!
			@brief  values for bus ADC resolution
		*/
		/**************************************************************************/
		typedef enum CONFIG_BADCRES {
			CONFIG_BADCRES_9BIT       =       (0x0000),  // 9-bit bus res = 0..511
			CONFIG_BADCRES_10BIT      =       (0x0080),  // 10-bit bus res = 0..1023
			CONFIG_BADCRES_11BIT      =       (0x0100),  // 11-bit bus res = 0..2047
			CONFIG_BADCRES_12BIT       =      (0x0180),  // 12-bit bus res = 0..4097
		} CONFIG_BADCRES;

		/**************************************************************************/
		/*!
			@brief  mask for shunt ADC resolution bits
		*/
		/**************************************************************************/
			const uint8_t CONFIG_SADCRES_MASK      =       (0x0078);  // Shunt ADC Resolution and Averaging Mask

		/**************************************************************************/
		/*!
			@brief  values for shunt ADC resolution
		*/
		/**************************************************************************/
		typedef enum CONFIG_SADCRES {
			CONFIG_SADCRES_9BIT_1S_84US     = (0x0000),  // 1 x 9-bit shunt sample
			CONFIG_SADCRES_10BIT_1S_148US   = (0x0008),  // 1 x 10-bit shunt sample
			CONFIG_SADCRES_11BIT_1S_276US   = (0x0010),  // 1 x 11-bit shunt sample
			CONFIG_SADCRES_12BIT_1S_532US   = (0x0018),  // 1 x 12-bit shunt sample
			CONFIG_SADCRES_12BIT_2S_1060US  = (0x0048),	 // 2 x 12-bit shunt samples averaged together
			CONFIG_SADCRES_12BIT_4S_2130US  = (0x0050),  // 4 x 12-bit shunt samples averaged together
			CONFIG_SADCRES_12BIT_8S_4260US  = (0x0058),  // 8 x 12-bit shunt samples averaged together
			CONFIG_SADCRES_12BIT_16S_8510US = (0x0060),  // 16 x 12-bit shunt samples averaged together
			CONFIG_SADCRES_12BIT_32S_17MS   = (0x0068),  // 32 x 12-bit shunt samples averaged together
			CONFIG_SADCRES_12BIT_64S_34MS   = (0x0070),  // 64 x 12-bit shunt samples averaged together
			CONFIG_SADCRES_12BIT_128S_69MS =  (0x0078),  // 128 x 12-bit shunt samples averaged together
		} CONFIG_SADCRES;

		/**************************************************************************/
		/*!
			@brief  mask for operating mode bits
		*/
		/**************************************************************************/
			const uint8_t CONFIG_MODE_MASK       =         (0x0007);  // Operating Mode Mask

		/**************************************************************************/
		/*!
			@brief  values for operating mode
		*/
		/**************************************************************************/
		 typedef enum CONFIG_MODE {
			CONFIG_MODE_POWERDOWN          =  (0x0000),
			CONFIG_MODE_SVOLT_TRIGGERED    =  (0x0001),
			CONFIG_MODE_BVOLT_TRIGGERED    =  (0x0002),
			CONFIG_MODE_SANDBVOLT_TRIGGERED = (0x0003),
			CONFIG_MODE_ADCOFF             =  (0x0004),
			CONFIG_MODE_SVOLT_CONTINUOUS  =  (0x0005),
			CONFIG_MODE_BVOLT_CONTINUOUS   =  (0x0006),
			CONFIG_MODE_SANDBVOLT_CONTINUOUS = (0x0007),
		} CONFIG_MODE;
		/*=========================================================================*/

		/**************************************************************************/
		/*!
			@brief  shunt voltage register
		*/
		/**************************************************************************/
			const uint8_t REG_SHUNTVOLTAGE    =            (0x01);
		/*=========================================================================*/

		/**************************************************************************/
		/*!
			@brief  bus voltage register
		*/
		/**************************************************************************/
			const uint8_t REG_BUSVOLTAGE       =           (0x02);
		/*=========================================================================*/

		/**************************************************************************/
		/*!
			@brief  power register
		*/
		/**************************************************************************/
			const uint8_t REG_POWER             =          (0x03);
		/*=========================================================================*/

		/**************************************************************************/
		/*!
			@brief  current register
		*/
		/**************************************************************************/
			const uint8_t REG_CURRENT           =         (0x04);
		/*=========================================================================*/

		/**************************************************************************/
		/*!
			@brief  calibration register
		*/
		/**************************************************************************/
			const uint8_t REG_CALIBRATION        =         (0x05);
		/*=========================================================================*/

		/**************************************************************************/
		/*!
			@brief  Class that stores state and functions for interacting with INA219 current/power monitor IC
		*/
		/**************************************************************************/

	public:
		INA219(I2C& i2c);
		void setCalibration_32V_2A(void);
		void setCalibration_16V_6A(void);
		void setCalibration_32V_1A(void);
		void setCalibration_16V_400mA(void);
		float getBusVoltage_V(void);
		float getShuntVoltage_mV(void);
		float getCurrent_mA(void);
		float getPower_mW(void);

		bool getCurrentAmps(float * amps);

	private:
		I2C& _i2c;
		
		uint32_t _calValue;
		// The following multipliers are used to convert raw current and power
		// values to mA and mW, taking into account the current config settings
		uint32_t _currentDivider_mA;
		float    _powerMultiplier_mW;

		void init();
		bool wireWriteRegister(uint8_t reg, uint16_t value);
		bool wireReadRegister(uint8_t reg, uint16_t *value);
		int16_t getBusVoltage_raw(void);
		int16_t getShuntVoltage_raw(void);
		int16_t getCurrent_raw(void);
		int16_t getPower_raw(void);

		bool getCurrent_raw(int16_t * rawValue);
};

#endif
