/*
 * AS5600.hpp
 *
 * Module to interface with AS5600 magnetic encoder chip.
 */

#pragma once

#include "i2c_device.hpp"



// --- Device address -------------------------------------------------------------------

const uint8_t AS5600_DEFAULT_ADDRESS = 0x36;



// --- Configuration options ------------------------------------------------------------

// Refer to data-sheet page 19 for more details
enum AS5600_POWER_MODE : uint8_t {				// AS5600 power modes
	AS5600_NOM 	= 0x00 << 0,
	AS5600_LPM1 = 0x01 << 0,
	AS5600_LPM2 = 0x02 << 0,
	AS5600_LPM3 = 0x03 << 0,
};

enum AS5600_HYSTERESIS : uint8_t {				// AS5600 hysteresis value
	AS5600_HYST_0 = 0x00 << 2,					// Off
	AS5600_HYST_1 = 0x01 << 2,					// 1 LSB
	AS5600_HYST_2 = 0x02 << 2,					// 2 LSBs
	AS5600_HYST_3 = 0x03 << 2,					// 3 LSBs
};

enum AS5600_OUTPUT_MODE : uint8_t {				// AS5600 output modes
	AS5600_ANALOG_FULL_RANGE 	= 0x00 << 4,	// Analog output, 0% = GND 100% = VDD
	AS5600_ANALOG_REDUCED_RANGE = 0x01 << 4,	// Analog output, 10% = GND 90% = VDD
	AS5600_DIGITAL_PWM 			= 0x02 << 4,	// Digital PWM output
};

enum AS5600_PWM_FREQUENCY : uint8_t {			// AS5600 PWM frequency
	AS5600_PWM_115Hz = 0x00 << 6,				// PWM frequency of 115Hz
	AS5600_PWM_230Hz = 0x01 << 6,				// PWM frequency of 230Hz
	AS5600_PWM_460Hz = 0x02 << 6,				// PWM frequency of 460Hz
	AS5600_PWM_920Hz = 0x03 << 6,				// PWM frequency of 920Hz
};

enum AS5600_SLOW_FILTER : uint8_t {				// AS5600 slow filter value
	AS5600_SLOW_FILTER_16x 	= 0x00 << 0,		// 16x
	AS5600_SLOW_FILTER_8x 	= 0x01 << 0,		// 8x
	AS5600_SLOW_FILTER_4x 	= 0x02 << 0,		// 4x
	AS5600_SLOW_FILTER_2x 	= 0x03 << 0,		// 2x
};

enum AS5600_FAST_FILTER_TH : uint8_t {			// AS5600 fast filter threshold value
	AS5600_SLOW_FILTER_ONLY 	= 0x00 << 2,	// Slow filter only
	AS5600_FAST_FILTER_6LSBs 	= 0x01 << 2,	// 6 LSBs
	AS5600_FAST_FILTER_7LSBs 	= 0x02 << 2,	// 7 LSBs
	AS5600_FAST_FILTER_9LSBs	= 0x03 << 2,	// 9 LSBs
	AS5600_FAST_FILTER_18LSBs	= 0x04 << 2,	// 18 LSBs
	AS5600_FAST_FILTER_21LSBs 	= 0x05 << 2,	// 21 LSBs
	AS5600_FAST_FILTER_24LSBs 	= 0x06 << 2,	// 24 LSBs
	AS5600_FAST_FILTER_10LSBs 	= 0x07 << 2,	// 10 LSBs
};

enum AS5600_WATCHDOG : uint8_t {				// AS5600 watch-dog state
	AS5600_WATCHDOG_OFF = 0x00 << 5,			// Watch-dog off
	AS5600_WATCHDOG_ON 	= 0x01 << 5,			// Watch-dog on
};



// --- Miscellaneous options ------------------------------------------------------------

enum OUTPUT_ANGLE_UNIT : uint8_t {		// Output angle unit
	AS5600_DEGREES 	= 0x00,
	AS5600_RADIANS 	= 0x01,
	AS5600_ADC 		= 0x02,
};

enum ROTATION_DIRECTION : uint8_t {		// Rotation direction
	AS5600_CLOCK_WISE 			= 0,
	AS5600_COUNTERCLOCK_WISE 	= 1,
};



// --- AS5600 class ---------------------------------------------------------------------

class AS5600 : public I2C_Device {

public:
	// Constructor
	AS5600(
			I2C_HandleTypeDef *device_handle,
			uint8_t device_address = AS5600_DEFAULT_ADDRESS,
			ROTATION_DIRECTION direction = AS5600_CLOCK_WISE,
			uint32_t response_delay = HAL_MAX_DELAY
			);

	// Register writing methods
	bool setZPosition(uint16_t value);
	bool setMPosition(uint16_t value);
	bool setMaxAngle(uint16_t value);

	bool setConfiguration(uint16_t value);

	bool setPowerMode(AS5600_POWER_MODE option);
	bool setHysteresis(AS5600_HYSTERESIS option);
	bool setOutputMode(AS5600_OUTPUT_MODE option);
	bool setPWMFrequency(AS5600_PWM_FREQUENCY option);
	bool setSlowFilter(AS5600_SLOW_FILTER option);
	bool setFastFilter(AS5600_FAST_FILTER_TH option);
	bool setWatchDog(AS5600_WATCHDOG option);

	// Register reading methods
	uint8_t getZMCO(void);

	uint16_t getZPosition(void);
	uint16_t getMPosition(void);
	uint16_t getMaxAngle(void);

	uint16_t getConfiguration(void);

	AS5600_POWER_MODE getPowerMode(void);
	AS5600_HYSTERESIS getHysteresis(void);
	AS5600_OUTPUT_MODE getOutputMode(void);
	AS5600_PWM_FREQUENCY getPWMFrequency(void);
	AS5600_SLOW_FILTER getSlowFilter(void);
	AS5600_FAST_FILTER_TH getFastFilterThresh(void);
	AS5600_WATCHDOG getWatchDog(void);

	uint16_t getRawAngle(void);
	uint16_t getAngle(void);
	float getRealAngle(OUTPUT_ANGLE_UNIT unit = AS5600_RADIANS);

	uint8_t getStatus(void);
	uint8_t getAGC(void);
	uint16_t getMagnitude(void);

	// Boolean methods
	bool isMagnetDetected(void);
	bool isMagnetStrong(void);
	bool isMagnetWeak(void);

protected:
	// Variables
	ROTATION_DIRECTION _direction;
};

