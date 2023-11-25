/*
 * INA219.cpp
 *
 * Implementation of INA219.hpp header file.
 *
 */

#include "INA219.hpp"



// ------------------------------------------------------ INA219 class implementation ---

// --- Device constructor ---------------------------------------------------------------

INA219::INA219(
I2C_HandleTypeDef *device_handle,
float max_expected_current,
float shunt_resistor,
uint8_t device_address,
uint32_t response_delay
) :
		I2C_Device(device_handle, device_address, response_delay)
	{
		INA219::calibrateSensor(max_expected_current, shunt_resistor);
	}


// --- Sensor core methods --------------------------------------------------------------

// --- Raw sensor readings

int16_t INA219::getRawBusVoltage(void){
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::BUS_VOLTAGE, &register_content);

	return (int16_t)(register_content >> 3) * 4;
}

int16_t INA219::getRawShuntVoltage(void){
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::SHUNT_VOLTAGE, &register_content);

	return (int16_t)register_content;
}

int16_t INA219::getRawCurrent(void){
	INA219::calibrateSensor(INA219::_max_expected_current, INA219::_shunt_resistor);

	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CURRENT, &register_content);

	return (int16_t)register_content;
}

int16_t INA219::getRawPower(void){
	INA219::calibrateSensor(INA219::_max_expected_current, INA219::_shunt_resistor);

	uint16_t register_content;
	INA219::LLR_16Bits(INA219::POWER, &register_content);

	return (int16_t)register_content;
}


// --- Sensor readings

float INA219::getBusVoltage(void){
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::BUS_VOLTAGE, &register_content);

	uint16_t register_flags = INA219::mask_16Bits(register_content, INA219::BUS_REGISTER_FLAGS_MASK);

	if(register_flags & INA219::BUS_REGISTER_OVF_MASK) return -100;

	return (register_content >> 3) * 4e-3; 	// Fixed 4 mV LSB
}

float INA219::getShuntVoltage(void){
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::SHUNT_VOLTAGE, &register_content);

	return (int16_t)register_content * 1e-5;	// Fixed 10 uV LSB
}

float INA219::getCurrent(void){
	INA219::calibrateSensor(INA219::_max_expected_current, INA219::_shunt_resistor);

	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CURRENT, &register_content);

	return (int16_t)register_content * INA219::_current_lsb;
}

float INA219::getPower(void){
	INA219::calibrateSensor(INA219::_max_expected_current, INA219::_shunt_resistor);

	uint16_t register_content;
	INA219::LLR_16Bits(INA219::POWER, &register_content);

	return (int16_t)register_content * INA219::_power_lsb;
}


// --- Calibration and parameters

void INA219::calibrateSensor(float max_expected_current, float shunt_resistor){
	INA219::_max_expected_current = max_expected_current;
	INA219::_shunt_resistor = shunt_resistor;

	INA219::_current_lsb = _max_expected_current / 32768;
	INA219::_power_lsb = 20 * INA219::_current_lsb;

	uint16_t calibration_value = (uint16_t)(0.04096 / (INA219::_current_lsb * INA219::_shunt_resistor));
	INA219::LLW_16Bits(INA219::CALIBRATION, calibration_value);
}

uint16_t INA219::getCalibration(void){
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CALIBRATION, &register_content);

	return register_content;
}


// --- Sensor utility methods -----------------------------------------------------------


// --- Sensor configuration methods -----------------------------------------------------

// --- General configuration

bool INA219::setConfiguration(uint16_t value){
	HAL_StatusTypeDef error = INA219::LLW_16Bits(INA219::CONF, value);
	if(error == HAL_OK) return true;

	return false;
}

uint16_t INA219::getConfiguration(void){
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	return register_content;
}


// --- Specific configuration

bool INA219::setBusVoltageRange(INA219::BUS_VOLTAGE_RANGE option){
	uint16_t previous_configuration;
	HAL_StatusTypeDef error = INA219::LLR_16Bits(INA219::CONF, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_16Bits(previous_configuration, INA219::BUS_VOLTAGE_RANGE_MASK, true);
	previous_configuration |= option;

	error = INA219::LLW_16Bits(INA219::CONF, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

INA219::BUS_VOLTAGE_RANGE INA219::getBusVoltageRange(void){
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	uint16_t result = INA219::mask_16Bits(register_content, INA219::BUS_VOLTAGE_RANGE_MASK);

	return INA219::BUS_VOLTAGE_RANGE(result);
}


bool INA219::setPGA(INA219::PGA option){
	uint16_t previous_configuration;
	HAL_StatusTypeDef error = INA219::LLR_16Bits(INA219::CONF, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_16Bits(previous_configuration, INA219::PGA_MASK, true);
	previous_configuration |= option;

	error = INA219::LLW_16Bits(INA219::CONF, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

INA219::PGA INA219::getPGA(void){
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	uint16_t result = INA219::mask_16Bits(register_content, INA219::PGA_MASK);

	return INA219::PGA(result);
}


bool INA219::setBusADCResolution(INA219::BUS_ADC_RESOLUTION option){
	uint16_t previous_configuration;
	HAL_StatusTypeDef error = INA219::LLR_16Bits(INA219::CONF, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_16Bits(previous_configuration, INA219::BUS_ADC_RESOLUTION_MASK, true);
	previous_configuration |= option;

	error = INA219::LLW_16Bits(INA219::CONF, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

INA219::BUS_ADC_RESOLUTION INA219::getBusADCResolution(void){
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	uint16_t result = INA219::mask_16Bits(register_content, INA219::BUS_ADC_RESOLUTION_MASK);

	return INA219::BUS_ADC_RESOLUTION(result);
}


bool INA219::setShuntADCResolution(INA219::SHUNT_ADC_RESOLUTION option){
	uint16_t previous_configuration;
	HAL_StatusTypeDef error = INA219::LLR_16Bits(INA219::CONF, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_16Bits(previous_configuration, INA219::SHUNT_ADC_RESOLUTION_MASK, true);
	previous_configuration |= option;

	error = INA219::LLW_16Bits(INA219::CONF, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

INA219::SHUNT_ADC_RESOLUTION INA219::getShuntADCResolution(void){
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	uint16_t result = INA219::mask_16Bits(register_content, INA219::SHUNT_ADC_RESOLUTION_MASK);

	return INA219::SHUNT_ADC_RESOLUTION(result);
}


bool INA219::setOperatingMode(INA219::OPERATING_MODE option){
	uint16_t previous_configuration;
	HAL_StatusTypeDef error = INA219::LLR_16Bits(INA219::CONF, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_16Bits(previous_configuration, INA219::OPERATING_MODE_MASK, true);
	previous_configuration |= option;

	error = INA219::LLW_16Bits(INA219::CONF, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

INA219::OPERATING_MODE INA219::getOperatingMode(void){
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	uint16_t result = INA219::mask_16Bits(register_content, INA219::OPERATING_MODE_MASK);

	return INA219::OPERATING_MODE(result);
}


// --- Miscellaneous methods ------------------------------------------------------------

bool INA219::reset(void){
	HAL_StatusTypeDef error = INA219::LLW_16Bits(INA219::CONF, INA219::RESET_MASK);
	if(error == HAL_OK) return true;

	return false;
}

