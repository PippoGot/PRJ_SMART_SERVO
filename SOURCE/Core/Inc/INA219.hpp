/*
 * INA219.hpp
 *
 * Module to interface with INA219 current sensor chip.
 *
 */

#pragma once

#include "i2c_device.hpp"



// --- Device default address -----------------------------------------------------------

// See data-sheet page 14, table 1 for more details on address options

const uint8_t INA219_DEFAULT_ADDRESS = 0x40;



// --------------------------------------------------------- INA219 class declaration ---

class INA219 : public I2C_Device {

public:
	// --- Device constructors ----------------------------------------------------------

	INA219(
			I2C_HandleTypeDef *device_handle,
			float max_expected_current,
			float shunt_resistor,
			uint8_t device_address = INA219_DEFAULT_ADDRESS,
			uint32_t response_delay = HAL_MAX_DELAY
			);


	// --- Sensor core methods ----------------------------------------------------------

	// --- Raw sensor readings

	int16_t getRawBusVoltage(void);
	int16_t getRawShuntVoltage(void);
	int16_t getRawCurrent(void);
	int16_t getRawPower(void);


	// --- Sensor readings

	float getBusVoltage_V(void);

	float getShuntVoltage_V(void);

	float getCurrent_A(void);

	float getPower_W(void);


	// --- Calibration

	void calibrateSensor(float max_expected_current, float shunt_resistor);
	uint16_t getCalibration(void);


	// --- Sensor Parameters

	float getMaxCurrent(void){ return _max_expected_current; };
	float getShuntResistor(void){ return _shunt_resistor; };


	// --- Sensor utility methods -------------------------------------------------------

	// ---  Sensor readings (smaller measurement units)

	float getBusVoltage_mV(void){ return getBusVoltage_V() * 1e3; };
	float getBusVoltage_uV(void){ return getBusVoltage_V() * 1e6; };

	float getShuntVoltage_mV(void){ return getShuntVoltage_V() * 1e3; };
	float getShuntVoltage_uV(void){ return getShuntVoltage_V() * 1e6; };

	float getCurrent_mA(void){ return getCurrent_A() * 1e3; };
	float getCurrent_uA(void){ return getCurrent_A() * 1e6; };

	float getPower_mW(void){ return getPower_W() * 1e3; };
	float getPower_uW(void){ return getPower_W() * 1e6; };


	// --- LSB getters

	float getCurrentLSB(void){ return _current_lsb; };
	float getPowerLSB(void){ return _power_lsb; };

	float getCurrentLSB_mA(void){ return _current_lsb * 1e3; };
	float getCurrentLSB_uA(void){ return _current_lsb * 1e6; };

	float getPowerLSB_mW(void){ return _power_lsb * 1e3; };
	float getPowerLSB_uW(void){ return _power_lsb * 1e6; };


	// --- Sensor configuration options -------------------------------------------------

	// See data-sheet page 19, table 3 for more details on configuration

	enum BUS_VOLTAGE_RANGE : uint16_t {		// Bus voltage ranges (defaults to 32 V)
		RANGE_16V_FSR	= 0x00 << 13,		// 16 volts
		RANGE_32V_FSR 	= 0x01 << 13,		// 32 volts
	};

	// See data-sheet page 19, table 4 for more details on configuration

	enum PGA : uint16_t {					// PGA gain and range (defaults to 320 mV)
		PGA_40mV 	= 0x00 << 11,			// /1, +- 40 mV
		PGA_80mV 	= 0x01 << 11,			// /2, +- 80 mV
		PGA_160mV 	= 0x02 << 11,			// /4, +- 160 mV
		PGA_320mV	= 0x03 << 11,			// /8, +- 320 mV
	};

	// See data-sheet page 19, table 5 for more details on configuration

	enum BUS_ADC_RESOLUTION : uint16_t {	// ADC resolution or number of samples (defaults to 12 bits)
		BUS_ADC_9_BITS 		= 0x00 << 7,	// 9 bits, 84 us conversion time
		BUS_ADC_10_BITS 	= 0x01 << 7,	// 10 bits, 148 us conversion time
		BUS_ADC_11_BITS 	= 0x02 << 7,	// 11 bits, 276 us conversion time
		BUS_ADC_12_BITS 	= 0x03 << 7,	// 12 bits, 532 us conversion time
		BUS_ADC_2_SMPL		= 0x09 << 7,	// 2 samples, 1.06 ms conversion time
		BUS_ADC_4_SMPL		= 0x0A << 7,	// 4 samples, 2.13 ms conversion time
		BUS_ADC_8_SMPL		= 0x0B << 7,	// 8 samples, 4.26 ms conversion time
		BUS_ADC_16_SMPL 	= 0x0C << 7,	// 16 samples, 8.51 ms conversion time
		BUS_ADC_32_SMPL 	= 0x0D << 7,	// 32 samples, 17.02 ms conversion time
		BUS_ADC_64_SMPL 	= 0x0E << 7,	// 64 samples, 34.05 ms conversion time
		BUS_ADC_128_SMPL 	= 0x0F << 7,	// 128 samples, 68.1 ms conversion time
	};

	// See data-sheet page 19, table 5 for more details on configuration

	enum SHUNT_ADC_RESOLUTION : uint16_t {	// ADC resolution or number of samples (defaults to 12 bits)
		SHUNT_ADC_9_BITS 	= 0x00 << 3,	// 9 bits, 84 us conversion time
		SHUNT_ADC_10_BITS 	= 0x01 << 3,	// 10 bits, 148 us conversion time
		SHUNT_ADC_11_BITS 	= 0x02 << 3,	// 11 bits, 236 us conversion time
		SHUNT_ADC_12_BITS 	= 0x03 << 3,	// 12 bits, 532 us conversion time
		SHUNT_ADC_2_SMPL	= 0x09 << 3,	// 2 samples, 1.06 ms conversion time
		SHUNT_ADC_4_SMPL	= 0x0A << 3,	// 4 samples, 2.13 ms conversion time
		SHUNT_ADC_8_SMPL	= 0x0B << 3,	// 8 samples, 4.26 ms conversion time
		SHUNT_ADC_16_SMPL 	= 0x0C << 3,	// 16 samples, 8.51 ms conversion time
		SHUNT_ADC_32_SMPL 	= 0x0D << 3,	// 32 samples, 13.02 ms conversion time
		SHUNT_ADC_64_SMPL 	= 0x0E << 3,	// 64 samples, 34.05 ms conversion time
		SHUNT_ADC_128_SMPL 	= 0x0F << 3,	// 128 samples, 68.1 ms conversion time
	};

	// See data-sheet page 19, table 6 for more details on configuration

	enum OPERATING_MODE : uint16_t {		// Continuous, triggered or power-down (defaults to both continuous)
		POWER_DOWN 			= 0x00 << 0,
		SHUNT_VOLTAGE_TRGD 	= 0x01 << 0,
		BUS_VOLTAGE_TRGD 	= 0x02 << 0,
		BOTH_VOLTAGE_TRGD 	= 0x03 << 0,
		ADC_OFF 			= 0x04 << 0,
		SHUNT_VOLTAGE_CONT 	= 0x05 << 0,
		BUS_VOLTAGE_CONT 	= 0x06 << 0,
		BOTH_VOLTAGE_CONT 	= 0x07 << 0,
	};


	// --- Sensor configuration methods -------------------------------------------------

	// --- General configuration
	// TODO better management

	bool setConfiguration(uint16_t value);
	uint16_t getConfiguration(void);


	// --- Specific configuration

	bool setBusVoltageRange(INA219::BUS_VOLTAGE_RANGE option);
	INA219::BUS_VOLTAGE_RANGE getBusVoltageRange(void);

	bool setPGA(INA219::PGA option);
	INA219::PGA getPGA(void);

	bool setBusADCResolution(INA219::BUS_ADC_RESOLUTION option);
	INA219::BUS_ADC_RESOLUTION getBusADCResolution(void);

	bool setShuntADCResolution(INA219::SHUNT_ADC_RESOLUTION option);
	INA219::SHUNT_ADC_RESOLUTION getShuntADCResolution(void);

	bool setOperatingMode(INA219::OPERATING_MODE option);
	INA219::OPERATING_MODE getOperatingMode(void);


	// --- Miscellaneous methods --------------------------------------------------------

	bool reset(void);

	bool shutDown(void){ return INA219::setOperatingMode(INA219::POWER_DOWN); };


protected:
	// --- Device variables -------------------------------------------------------------

	float _max_expected_current;
	float _shunt_resistor;

	float _current_lsb;
	float _power_lsb;


	// --- Sensor register map ----------------------------------------------------------

	// See data-sheet page 18, table 2 for more details on registers map

	enum REGISTER : uint8_t {
		CONF 			= 0x00,
		SHUNT_VOLTAGE 	= 0x01,
		BUS_VOLTAGE 	= 0x02,
		POWER 			= 0x03,
		CURRENT 		= 0x04,
		CALIBRATION 	= 0x05,
	};


	// --- Utility masks ----------------------------------------------------------------

	// --- Configuration bits masks

	// See data-sheet page 19 for more details on configuration bits

	const uint16_t RESET_MASK					= 0x8000;
	const uint16_t BUS_VOLTAGE_RANGE_MASK    	= 0x2000;
	const uint16_t PGA_MASK    					= 0x1800;
	const uint16_t BUS_ADC_RESOLUTION_MASK   	= 0x0780;
	const uint16_t SHUNT_ADC_RESOLUTION_MASK 	= 0x0078;
	const uint16_t OPERATING_MODE_MASK   		= 0x0007;


	// --- Bus voltage register bits masks

	// See data-sheet page 23 for more details on bus voltage flag bits

	const uint16_t BUS_REGISTER_FLAGS_MASK 		= 0x0003;
	const uint16_t BUS_REGISTER_CNVR_MASK		= 0x0002;
	const uint16_t BUS_REGISTER_OVF_MASK		= 0x0001;
};


// END OF FILE
