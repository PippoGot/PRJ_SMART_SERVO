/*
 * AS5600.hpp
 *
 * Module to interface with AS5600 magnetic encoder chip.
 *
 */

#pragma once

#include "i2c_device.hpp"



// --- Device default address -----------------------------------------------------------

// See data-sheet page 13 for more details on address

const uint8_t AS5600_DEFAULT_ADDRESS = 0x36;



// --------------------------------------------------------- AS5600 class declaration ---

class AS5600 : public I2C_Device {

public:
	// --- Miscellaneous sensor options -------------------------------------------------

	enum OUTPUT_ANGLE_UNIT : uint8_t {		// Output angle unit
		DEGREES = 0x00,
		RADIANS = 0x01,
	};

	// TODO make it possible to acquire it from board with a digital pin ???
	enum ROTATION_DIRECTION : uint8_t {		// Rotation direction
		CLOCK_WISE 			= 0,
		COUNTERCLOCK_WISE 	= 1,
	};


	// --- Device constructor -----------------------------------------------------------

	AS5600(
			I2C_HandleTypeDef *device_handle,
			uint8_t device_address = AS5600_DEFAULT_ADDRESS,
			ROTATION_DIRECTION direction = AS5600::CLOCK_WISE,
			uint32_t response_delay = HAL_MAX_DELAY
			);


	// --- Sensor core methods ----------------------------------------------------------

	// --- Raw values

	uint16_t getRawAngle(void);

	uint16_t getAngle(void);


	// --- Real values
	// TODO float getRealRawAngle ???

	float getRealAngle(OUTPUT_ANGLE_UNIT unit = AS5600::RADIANS);


	// TODO changing direction methods ???


	// --- Sensor utility methods -------------------------------------------------------

	// --- Reduced angle setting

	bool setZPosition(uint16_t value);
	uint16_t getZPosition(void);

	bool setMPosition(uint16_t value);
	uint16_t getMPosition(void);

	bool setMaxAngle(uint16_t value);
	uint16_t getMaxAngle(void);


	// --- Magnet detection

	bool isMagnetDetected(void);

	bool isMagnetStrong(void);

	bool isMagnetWeak(void);


	// --- Sensor configuration options -------------------------------------------------

	// See data-sheet page 19, figure 22 for more details on configuration

	enum POWER_MODE : uint8_t {				// AS5600 power modes
		NOM 	= 0x00 << 0,				// 6.5 mA
		LPM1 	= 0x01 << 0,				// 3.5 mA, 5 ms polling time
		LPM2 	= 0x02 << 0,				// 1.8 mA, 20 ms polling time
		LPM3 	= 0x03 << 0,				// 1.5 mA, 100 ms polling time
	};

	enum HYSTERESIS : uint8_t {				// AS5600 hysteresis value
		HYST_0 = 0x00 << 2,					// Off
		HYST_1 = 0x01 << 2,					// 1 LSB
		HYST_2 = 0x02 << 2,					// 2 LSBs
		HYST_3 = 0x03 << 2,					// 3 LSBs
	};

	enum OUTPUT_MODE : uint8_t {			// AS5600 output modes
		ANALOG_FULL_RANGE 		= 0x00 << 4,// Analog output, 0% = GND 100% = VDD
		ANALOG_REDUCED_RANGE	= 0x01 << 4,// Analog output, 10% = GND 90% = VDD
		DIGITAL_PWM 			= 0x02 << 4,// Digital PWM output
	};

	enum PWM_FREQUENCY : uint8_t {			// AS5600 PWM frequency
		PWM_115Hz = 0x00 << 6,				// PWM frequency of 115Hz
		PWM_230Hz = 0x01 << 6,				// PWM frequency of 230Hz
		PWM_460Hz = 0x02 << 6,				// PWM frequency of 460Hz
		PWM_920Hz = 0x03 << 6,				// PWM frequency of 920Hz
	};

	enum SLOW_FILTER : uint8_t {			// AS5600 slow filter value
		SLOW_FILTER_16x = 0x00 << 0,		// 16x
		SLOW_FILTER_8x 	= 0x01 << 0,		// 8x
		SLOW_FILTER_4x 	= 0x02 << 0,		// 4x
		SLOW_FILTER_2x 	= 0x03 << 0,		// 2x
	};

	enum FAST_FILTER_TH : uint8_t {			// AS5600 fast filter threshold value
		SLOW_FILTER_ONLY 	= 0x00 << 2,	// Slow filter only
		FAST_FILTER_6_LSBs 	= 0x01 << 2,	// 6 LSBs
		FAST_FILTER_7_LSBs 	= 0x02 << 2,	// 7 LSBs
		FAST_FILTER_9_LSBs	= 0x03 << 2,	// 9 LSBs
		FAST_FILTER_18_LSBs	= 0x04 << 2,	// 18 LSBs
		FAST_FILTER_21_LSBs = 0x05 << 2,	// 21 LSBs
		FAST_FILTER_24_LSBs = 0x06 << 2,	// 24 LSBs
		FAST_FILTER_10_LSBs = 0x07 << 2,	// 10 LSBs
	};

	enum WATCHDOG : uint8_t {				// AS5600 watch-dog state
		WATCHDOG_OFF 	= 0x00 << 5,		// Watch-dog off
		WATCHDOG_ON 	= 0x01 << 5,		// Watch-dog on
	};


	// --- Sensor configuration methods -------------------------------------------------

	// --- General configuration
	// TODO better management

	bool setConfiguration(uint16_t value);
	uint16_t getConfiguration(void);


	// --- Specific configuration

	bool setPowerMode(AS5600::POWER_MODE option);
	AS5600::POWER_MODE getPowerMode(void);

	bool setHysteresis(AS5600::HYSTERESIS option);
	AS5600::HYSTERESIS getHysteresis(void);

	bool setOutputMode(AS5600::OUTPUT_MODE option);
	AS5600::OUTPUT_MODE getOutputMode(void);

	bool setPWMFrequency(AS5600::PWM_FREQUENCY option);
	AS5600::PWM_FREQUENCY getPWMFrequency(void);

	bool setSlowFilter(AS5600::SLOW_FILTER option);
	AS5600::SLOW_FILTER getSlowFilter(void);

	bool setFastFilter(AS5600::FAST_FILTER_TH option);
	AS5600::FAST_FILTER_TH getFastFilterThresh(void);

	bool setWatchDog(AS5600::WATCHDOG option);
	AS5600::WATCHDOG getWatchDog(void);


	// --- Miscellaneous methods --------------------------------------------------------

	uint8_t getZMCO(void);

	uint8_t getStatus(void);

	uint8_t getAGC(void);

	uint16_t getMagnitude(void);


protected:
	// --- Device variables -------------------------------------------------------------

	ROTATION_DIRECTION _direction;

	// --- Sensor register map ----------------------------------------------------------

	// See data-sheet page 18, figure 21 for more details on registers map

	enum REGISTER : uint8_t {
		// Configuration registers
		ZMCO 	= 0x00,
		ZPOS_H 	= 0x01,
		ZPOS_L 	= 0x02,
		MPOS_H 	= 0x03,
		MPOS_L 	= 0x04,
		MANG_H 	= 0x05,
		MANG_L 	= 0x06,
		CONF_H 	= 0x07,
		CONF_L 	= 0x08,

		// Output registers
		RAW_ANGLE_H = 0x0C,
		RAW_ANGLE_L = 0x0D,
		ANGLE_H 	= 0x0E,
		ANGLE_L 	= 0x0F,

		// Status registers
		STATUS 		= 0x0B,
		AGC 		= 0x1A,
		MAGNITUDE_H = 0x1B,
		MAGNITUDE_L	= 0x1C,

		// Burn is not reported
	};

	// --- Utility masks ----------------------------------------------------------------

	// --- Configuration bits masks

	// See data-sheet page 19, table 22 for more details on configuration bits

	const uint8_t POWER_MODE_MASK    	= 0x03;
	const uint8_t HYSTERESIS_MASK    	= 0x0C;
	const uint8_t OUTPUT_MODE_MASK   	= 0x30;
	const uint8_t PWM_FREQUENCY_MASK 	= 0xC0;
	const uint8_t SLOW_FILTER_MASK   	= 0x03;
	const uint8_t FAST_FILTER_TH_MASK	= 0x1C;
	const uint8_t WATCHDOG_MASK     	= 0x20;


	// --- Magnet status bits masks

	// See data-sheet page 20, table 23 for more details on status bits

	const uint8_t MAGNET_STRONG   	= 0x08;
	const uint8_t MAGNET_WEAK    	= 0x10;
	const uint8_t MAGNET_DETECTED 	= 0x20;


	// --- Utility conversion constants -------------------------------------------------

	const float PI = 3.14159265359;

	const float ADC_TO_DEGREES = 360.0 / 4096;
	const float DEGREES_TO_ADC = 4096 / 360.0;

	const float ADC_TO_RADIANS = (PI * 2.0) / 4096;
	const float RADIANS_TO_ADC = 4096 / (PI * 2.0);

};


// END OF FILE
