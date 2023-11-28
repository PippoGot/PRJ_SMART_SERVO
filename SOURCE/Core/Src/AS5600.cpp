/*
 * AS5600.cpp
 *
 * Implementation of AS5600.hpp header file.
 *
 */

#include "AS5600.hpp"



// ------------------------------------------------------ AS5600 class implementation ---

// --- Device constructor ---------------------------------------------------------------

AS5600::AS5600(
I2C_HandleTypeDef *device_handle,
uint8_t device_address,
AS5600::ROTATION_DIRECTION direction,
uint32_t response_delay
) :
		I2C_Device(device_handle, device_address, response_delay),
		_direction(direction)
	{}


// --- Sensor core methods --------------------------------------------------------------

// Raw values

uint16_t AS5600::getRawAngle(void){
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::RAW_ANGLE_H, &register_content);

	if(AS5600::_direction == AS5600::COUNTERCLOCK_WISE) return (0x0FFF - register_content) & 0x0FFF;

	return register_content;
}

uint16_t AS5600::getAngle(void){
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::ANGLE_H, &register_content);

	if(AS5600::_direction == AS5600::COUNTERCLOCK_WISE) return (0x0FFF - register_content) & 0x0FFF;

	return register_content;
}


// Real values

float AS5600::getRealAngle(AS5600::OUTPUT_ANGLE_UNIT unit){
	uint16_t angle = AS5600::getAngle();

	if(unit == AS5600::RADIANS){
		return angle * AS5600::ADC_TO_RADIANS;
	}

	return angle * AS5600::ADC_TO_DEGREES;
}


// --- Sensor utility methods -----------------------------------------------------------

// --- Reduced angle setting

bool AS5600::setZPosition(uint16_t value){
	if(value > 0x0FFF) return false;

	HAL_StatusTypeDef error = AS5600::LLW_16Bits(AS5600::ZPOS_H, value);
	if(error == HAL_OK) return true;

	return false;
}

uint16_t AS5600::getZPosition(void){
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::ZPOS_H, &register_content);

	return register_content;
}


bool AS5600::setMPosition(uint16_t value){
	if(value > 0x0FFF) return false;

	HAL_StatusTypeDef error = AS5600::LLW_16Bits(AS5600::MPOS_H, value);
	if(error == HAL_OK) return true;

	return false;
}

uint16_t AS5600::getMPosition(void){
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::MPOS_H, &register_content);

	return register_content;
}


bool AS5600::setMaxAngle(uint16_t value){
	if(value > 0x0FFF) return false;

	HAL_StatusTypeDef error = AS5600::LLW_16Bits(AS5600::MANG_H, value);
	if(error == HAL_OK) return true;

	return false;
}

uint16_t AS5600::getMaxAngle(void){
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::MANG_H, &register_content);

	return register_content;
}


// --- Magnet detection

bool AS5600::isMagnetDetected(void){
	uint8_t status = AS5600::getStatus();

	return status == AS5600::MAGNET_DETECTED;
}

bool AS5600::isMagnetStrong(void){
	uint8_t status = AS5600::getStatus();

	return status == AS5600::MAGNET_STRONG;
}

bool AS5600::isMagnetWeak(void){
	uint8_t status = AS5600::getStatus();

	return status == AS5600::MAGNET_WEAK;
}


// --- Sensor configuration methods -----------------------------------------------------

// --- General configuration

bool AS5600::setConfiguration(uint16_t value){
	if(value > 0x3FFF) return false;

	HAL_StatusTypeDef error = AS5600::LLW_16Bits(AS5600::CONF_H, value);
	if(error == HAL_OK) return true;

	return false;
}

uint16_t AS5600::getConfiguration(void){
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::CONF_H, &register_content);

	return register_content;
}


// --- Specific configuration

bool AS5600::setPowerMode(AS5600::POWER_MODE option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_8Bits(previous_configuration, AS5600::POWER_MODE_MASK, true);
	previous_configuration |= option;

	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

AS5600::POWER_MODE AS5600::getPowerMode(void){
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_L, &register_content);

	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::POWER_MODE_MASK);

	return AS5600::POWER_MODE(result);
}


bool AS5600::setHysteresis(AS5600::HYSTERESIS option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_8Bits(previous_configuration, AS5600::HYSTERESIS_MASK, true);
	previous_configuration |= option;

	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

AS5600::HYSTERESIS AS5600::getHysteresis(void){
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_L, &register_content);

	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::HYSTERESIS_MASK);

	return AS5600::HYSTERESIS(result);
}


bool AS5600::setOutputMode(AS5600::OUTPUT_MODE option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_8Bits(previous_configuration, AS5600::OUTPUT_MODE_MASK, true);
	previous_configuration |= option;

	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

AS5600::OUTPUT_MODE AS5600::getOutputMode(void){
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_L, &register_content);

	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::OUTPUT_MODE_MASK);

	return AS5600::OUTPUT_MODE(result);
}


bool AS5600::setPWMFrequency(AS5600::PWM_FREQUENCY option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_8Bits(previous_configuration, AS5600::PWM_FREQUENCY_MASK, true);
	previous_configuration |= option;

	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

AS5600::PWM_FREQUENCY AS5600::getPWMFrequency(void){
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_L, &register_content);

	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::PWM_FREQUENCY_MASK);

	return AS5600::PWM_FREQUENCY(result);
}


bool AS5600::setSlowFilter(AS5600::SLOW_FILTER option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_8Bits(previous_configuration, AS5600::SLOW_FILTER_MASK, true);
	previous_configuration |= option;

	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

AS5600::SLOW_FILTER AS5600::getSlowFilter(void){
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_H, &register_content);

	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::SLOW_FILTER_MASK);

	return AS5600::SLOW_FILTER(result);
}


bool AS5600::setFastFilter(AS5600::FAST_FILTER_TH option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_8Bits(previous_configuration, AS5600::FAST_FILTER_TH_MASK, true);
	previous_configuration |= option;

	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

AS5600::FAST_FILTER_TH AS5600::getFastFilterThresh(void){
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_H, &register_content);

	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::FAST_FILTER_TH_MASK);

	return AS5600::FAST_FILTER_TH(result);
}


bool AS5600::setWatchDog(AS5600::WATCHDOG option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = mask_8Bits(previous_configuration, AS5600::WATCHDOG_MASK, true);
	previous_configuration |= option;

	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

AS5600::WATCHDOG AS5600::getWatchDog(void){
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_H, &register_content);

	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::WATCHDOG_MASK);

	return AS5600::WATCHDOG(result);
}


// --- Miscellaneous methods ------------------------------------------------------------

uint8_t AS5600::getZMCO(void){
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::ZMCO, &register_content);

	return register_content;
}

uint8_t AS5600::getStatus(void){
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::STATUS, &register_content);

	return register_content;
}

uint8_t AS5600::getAGC(void){
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::AGC, &register_content);

	return register_content;
}

uint16_t AS5600::getMagnitude(void){
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::MAGNITUDE_H, &register_content);

	return register_content;
}

