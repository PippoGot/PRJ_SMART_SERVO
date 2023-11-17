/*
 * AS5600.cpp
 *
 * Implementation of AS5600.hpp header file.
 *
 */

#include "AS5600.hpp"



const float PI = 3.14159265359;


// --- AS5600 module register map -------------------------------------------------------

// Refer to data-sheet page 18 for more details
enum AS5600_REGISTER : uint8_t {
	// Configuration Register2
	AS5600_ZMCO 		= 0x00,
	AS5600_ZPOS_H 		= 0x01,
	AS5600_ZPOS_L 		= 0x02,
	AS5600_MPOS_H 		= 0x03,
	AS5600_MPOS_L 		= 0x04,
	AS5600_MANG_H 		= 0x05,
	AS5600_MANG_L 		= 0x06,
	AS5600_CONF_H 		= 0x07,
	AS5600_CONF_L 		= 0x08,

	// Output Register2
	AS5600_RAW_ANGLE_H 	= 0x0C,
	AS5600_RAW_ANGLE_L 	= 0x0D,
	AS5600_ANGLE_H 		= 0x0E,
	AS5600_ANGLE_L 		= 0x0F,

	// Status Register2
	AS5600_STATUS 		= 0x0B,
	AS5600_AGC 			= 0x1A,
	AS5600_MAGNITUDE_H 	= 0x1B,
	AS5600_MAGNITUDE_L	= 0x1C,

	// Burn is not reported
};



// --- Configuration bits masks ---------------------------------------------------------

const uint8_t AS5600_POWER_MODE_MASK    	= 0x03;
const uint8_t AS5600_HYSTERESIS_MASK    	= 0x0C;
const uint8_t AS5600_OUTPUT_MODE_MASK   	= 0x30;
const uint8_t AS5600_PWM_FREQUENCY_MASK 	= 0xC0;
const uint8_t AS5600_SLOW_FILTER_MASK   	= 0x03;
const uint8_t AS5600_FAST_FILTER_TH_MASK	= 0x1C;
const uint8_t AS5600_WATCHDOG_MASK     		= 0x20;



// --- Magnet status bits masks ---------------------------------------------------------

const uint8_t AS5600_MAGNET_STRONG   	= 0x08;
const uint8_t AS5600_MAGNET_WEAK    	= 0x10;
const uint8_t AS5600_MAGNET_DETECTED 	= 0x20;



// --- Conversion constants -------------------------------------------------------------

const float AS5600_ADC_TO_DEGREES = 360.0 / 4096;
const float AS5600_DEGREES_TO_ADC = 4096 / 360.0;

const float AS5600_ADC_TO_RADIANS = (PI * 2.0) / 4096;
const float AS5600_RADIANS_TO_ADC = 4096 / (PI * 2.0);



// ------------------------------------------------------ AS5600 class implementation ---

// --- Constructor ----------------------------------------------------------------------

AS5600::AS5600(I2C_HandleTypeDef *device_handle, uint8_t device_address, ROTATION_DIRECTION direction, uint32_t response_delay) :
		I2C_Device(device_handle, device_address, response_delay),
		_direction(direction)
	{}



// --- Register writing methods ---------------------------------------------------------

bool AS5600::setZPosition(uint16_t value){
	if(value > 0x0FFF) return false;

	HAL_StatusTypeDef error = AS5600::writeRegister2(AS5600_ZPOS_H, value);
	if(error == HAL_OK) return true;

	return false;
}

bool AS5600::setMPosition(uint16_t value){
	if(value > 0x0FFF) return false;

	HAL_StatusTypeDef error = AS5600::writeRegister2(AS5600_MPOS_H, value);
	if(error == HAL_OK) return true;

	return false;
}

bool AS5600::setMaxAngle(uint16_t value){
	if(value > 0x0FFF) return false;

	HAL_StatusTypeDef error = AS5600::writeRegister2(AS5600_MANG_H, value);
	if(error == HAL_OK) return true;

	return false;
}



bool AS5600::setConfiguration(uint16_t value){
	if(value > 0x3FFF) return false;

	HAL_StatusTypeDef error = AS5600::writeRegister2(AS5600_CONF_H, value);
	if(error == HAL_OK) return true;

	return false;
}



bool AS5600::setPowerMode(AS5600_POWER_MODE option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::readRegister(AS5600_CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = maskByte(previous_configuration, AS5600_POWER_MODE_MASK, true);
	previous_configuration |= option;

	error = AS5600::writeRegister(AS5600_CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

bool AS5600::setHysteresis(AS5600_HYSTERESIS option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::readRegister(AS5600_CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = maskByte(previous_configuration, AS5600_HYSTERESIS_MASK, true);
	previous_configuration |= option;

	error = AS5600::writeRegister(AS5600_CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

bool AS5600::setOutputMode(AS5600_OUTPUT_MODE option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::readRegister(AS5600_CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = maskByte(previous_configuration, AS5600_OUTPUT_MODE_MASK, true);
	previous_configuration |= option;

	error = AS5600::writeRegister(AS5600_CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

bool AS5600::setPWMFrequency(AS5600_PWM_FREQUENCY option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::readRegister(AS5600_CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = maskByte(previous_configuration, AS5600_PWM_FREQUENCY_MASK, true);
	previous_configuration |= option;

	error = AS5600::writeRegister(AS5600_CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

bool AS5600::setSlowFilter(AS5600_SLOW_FILTER option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::readRegister(AS5600_CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = maskByte(previous_configuration, AS5600_SLOW_FILTER_MASK, true);
	previous_configuration |= option;

	error = AS5600::writeRegister(AS5600_CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

bool AS5600::setFastFilter(AS5600_FAST_FILTER_TH option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::readRegister(AS5600_CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = maskByte(previous_configuration, AS5600_FAST_FILTER_TH_MASK, true);
	previous_configuration |= option;

	error = AS5600::writeRegister(AS5600_CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}

bool AS5600::setWatchDog(AS5600_WATCHDOG option){
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::readRegister(AS5600_CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	previous_configuration = maskByte(previous_configuration, AS5600_WATCHDOG_MASK, true);
	previous_configuration |= option;

	error = AS5600::writeRegister(AS5600_CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	return false;
}



// --- Register reading methods ---------------------------------------------------------

uint8_t AS5600::getZMCO(void){
	uint8_t register_content;
	AS5600::readRegister(AS5600_ZMCO, &register_content);

	return register_content;
}



uint16_t AS5600::getZPosition(void){
	uint16_t register_content;
	AS5600::readRegister2(AS5600_ZPOS_H, &register_content);

	return register_content;
}

uint16_t AS5600::getMPosition(void){
	uint16_t register_content;
	AS5600::readRegister2(AS5600_MPOS_H, &register_content);

	return register_content;
}

uint16_t AS5600::getMaxAngle(void){
	uint16_t register_content;
	AS5600::readRegister2(AS5600_MANG_H, &register_content);

	return register_content;
}



uint16_t AS5600::getConfiguration(void){
	uint16_t register_content;
	AS5600::readRegister2(AS5600_CONF_H, &register_content);

	return register_content;
}



AS5600_POWER_MODE AS5600::getPowerMode(void){
	uint8_t register_content;
	AS5600::readRegister(AS5600_CONF_L, &register_content);

	uint8_t result = I2C_Device::maskByte(register_content, AS5600_POWER_MODE_MASK);

	return AS5600_POWER_MODE(result);
}

AS5600_HYSTERESIS AS5600::getHysteresis(void){
	uint8_t register_content;
	AS5600::readRegister(AS5600_CONF_L, &register_content);

	uint8_t result = I2C_Device::maskByte(register_content, AS5600_HYSTERESIS_MASK);

	return AS5600_HYSTERESIS(result);
}

AS5600_OUTPUT_MODE AS5600::getOutputMode(void){
	uint8_t register_content;
	AS5600::readRegister(AS5600_CONF_L, &register_content);

	uint8_t result = I2C_Device::maskByte(register_content, AS5600_OUTPUT_MODE_MASK);

	return AS5600_OUTPUT_MODE(result);
}

AS5600_PWM_FREQUENCY AS5600::getPWMFrequency(void){
	uint8_t register_content;
	AS5600::readRegister(AS5600_CONF_L, &register_content);

	uint8_t result = I2C_Device::maskByte(register_content, AS5600_PWM_FREQUENCY_MASK);

	return AS5600_PWM_FREQUENCY(result);
}

AS5600_SLOW_FILTER AS5600::getSlowFilter(void){
	uint8_t register_content;
	AS5600::readRegister(AS5600_CONF_H, &register_content);

	uint8_t result = I2C_Device::maskByte(register_content, AS5600_SLOW_FILTER_MASK);

	return AS5600_SLOW_FILTER(result);
}

AS5600_FAST_FILTER_TH AS5600::getFastFilterThresh(void){
	uint8_t register_content;
	AS5600::readRegister(AS5600_CONF_H, &register_content);

	uint8_t result = I2C_Device::maskByte(register_content, AS5600_FAST_FILTER_TH_MASK);

	return AS5600_FAST_FILTER_TH(result);
}

AS5600_WATCHDOG AS5600::getWatchDog(void){
	uint8_t register_content;
	AS5600::readRegister(AS5600_CONF_H, &register_content);

	uint8_t result = I2C_Device::maskByte(register_content, AS5600_WATCHDOG_MASK);

	return AS5600_WATCHDOG(result);
}



uint16_t AS5600::getRawAngle(void){
	uint16_t register_content;
	AS5600::readRegister2(AS5600_RAW_ANGLE_H, &register_content);

	if(AS5600::_direction == AS5600_COUNTERCLOCK_WISE) return (0x0FFF - register_content) & 0x0FFF;

	return register_content;
}

uint16_t AS5600::getAngle(void){
	uint16_t register_content;
	AS5600::readRegister2(AS5600_ANGLE_H, &register_content);

	if(AS5600::_direction == AS5600_COUNTERCLOCK_WISE) return (0x0FFF - register_content) & 0x0FFF;

	return register_content;
}

float AS5600::getRealAngle(OUTPUT_ANGLE_UNIT unit){
	uint16_t angle = AS5600::getAngle();

	if(unit == AS5600_RADIANS){
		return angle * AS5600_ADC_TO_RADIANS;
	}

	return angle * AS5600_ADC_TO_DEGREES;
}



uint8_t AS5600::getStatus(void){
	uint8_t register_content;
	AS5600::readRegister(AS5600_STATUS, &register_content);

	return register_content;
}

uint8_t AS5600::getAGC(void){
	uint8_t register_content;
	AS5600::readRegister(AS5600_AGC, &register_content);

	return register_content;
}

uint16_t AS5600::getMagnitude(void){
	uint16_t register_content;
	AS5600::readRegister2(AS5600_MAGNITUDE_H, &register_content);

	return register_content;
}



// --- Boolean methods ------------------------------------------------------------------

bool AS5600::isMagnetDetected(void){
	uint8_t status = AS5600::getStatus();

	return status == AS5600_MAGNET_DETECTED;
}

bool AS5600::isMagnetStrong(void){
	uint8_t status = AS5600::getStatus();

	return status == AS5600_MAGNET_STRONG;
}

bool AS5600::isMagnetWeak(void){
	uint8_t status = AS5600::getStatus();

	return status == AS5600_MAGNET_WEAK;
}

