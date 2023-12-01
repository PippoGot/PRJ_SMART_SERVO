/*
 * AS5600.cpp
 *
 * Implementation of AS5600.hpp header file.
 *
 */

#include "AS5600.hpp"



// ------------------------------------------------------ AS5600 class implementation ---

// --- Device constructor ---------------------------------------------------------------

/*
 * @biref Constructor for an AS5600 magnetic encoder.
 *
 * @param device_handle		I2C bus handle object;
 * @param device_address	Address of the I2C device;
 * @param direction			Rotation direction for upward counting;
 * @param response_delay	Time to wait for the device response;
 *
 */
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

// --- Raw values

/*
 * @brief Gets the content of the raw angle register (unfiltered position in ADC format).
 *
 */
uint16_t AS5600::getRawAngle(void){
	// Read the register
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::RAW_ANGLE_H, &register_content);

	// If direction is counter clock wise reverse the value
	if(AS5600::_direction == AS5600::COUNTERCLOCK_WISE) return (0x0FFF - register_content) & 0x0FFF;

	// Return result
	return register_content;
}

/*
 * @brief Gets the content of the angle register (filtered position in ADC format).
 *
 */
uint16_t AS5600::getAngle(void){
	// Read the register
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::ANGLE_H, &register_content);

	// If direction is counter clock wise reverse the value
	if(AS5600::_direction == AS5600::COUNTERCLOCK_WISE) return (0x0FFF - register_content) & 0x0FFF;

	// Return result
	return register_content;
}


// --- Real values

/*
 * @brief Returns the sensed position in degrees or radians.
 *
 * @param unit	Unit of measure of the output value;
 *
 */
float AS5600::getRealAngle(AS5600::OUTPUT_ANGLE_UNIT unit){
	// Get the ADC value
	uint16_t angle = AS5600::getAngle();

	// If radians is selected, return result in radians
	if(unit == AS5600::RADIANS){
		return angle * AS5600::ADC_TO_RADIANS;
	}

	// Return result in degrees
	return angle * AS5600::ADC_TO_DEGREES;
}


// --- Sensor utility methods -----------------------------------------------------------

// --- Reduced angle setting

/*
 * @brief Sets the Z position register value.
 *
 * @param value	Angle value to set;
 *
 */
bool AS5600::setZPosition(uint16_t value){
	// If value is over 12 bits return failure
	if(value > 0x0FFF) return false;

	// Write the register to given value, then if no errors return success
	HAL_StatusTypeDef error = AS5600::LLW_16Bits(AS5600::ZPOS_H, value);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the Z position register value.
 *
 */
uint16_t AS5600::getZPosition(void){
	// Read the register
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::ZPOS_H, &register_content);

	// Return result
	return register_content;
}


/*
 * @brief Sets the M position register value.
 *
 * @param value	Angle value to set;
 *
 */
bool AS5600::setMPosition(uint16_t value){
	// If value is over 12 bits return failure
	if(value > 0x0FFF) return false;

	// Write the register to given value, then if no errors return success
	HAL_StatusTypeDef error = AS5600::LLW_16Bits(AS5600::MPOS_H, value);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the M position register value.
 *
 */
uint16_t AS5600::getMPosition(void){
	// Read the register
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::MPOS_H, &register_content);

	// Return result
	return register_content;
}


/*
 * @brief Sets the max angle register value.
 *
 * @param value Angle value to set;
 *
 */
bool AS5600::setMaxAngle(uint16_t value){
	// If value is over 12 bits return failure
	if(value > 0x0FFF) return false;

	// Write the register to given value, then if no errors return success
	HAL_StatusTypeDef error = AS5600::LLW_16Bits(AS5600::MANG_H, value);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the max angle register value.
 *
 */
uint16_t AS5600::getMaxAngle(void){
	// Read the register
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::MANG_H, &register_content);

	// Return result
	return register_content;
}


// --- Magnet detection

// See data-sheet page 20, table 23 for more details on magnet detection

/*
 * @brief Checks if the magnet is detected.
 *
 */
bool AS5600::isMagnetDetected(void){
	// Read the status register
	uint8_t status = AS5600::getStatus();

	// Check the status
	return status == AS5600::MAGNET_DETECTED;
}

/*
 * @brief Checks if the magnet is too close/strong.
 *
 */
bool AS5600::isMagnetStrong(void){
	// Read the status register
	uint8_t status = AS5600::getStatus();

	// Check the status
	return status == AS5600::MAGNET_STRONG;
}

/*
 * @brief Checks if the magnet is too far/weak.
 *
 */
bool AS5600::isMagnetWeak(void){
	// Read the status register
	uint8_t status = AS5600::getStatus();

	// Check the status
	return status == AS5600::MAGNET_WEAK;
}


// --- Sensor configuration methods -----------------------------------------------------

// --- General configuration

/*
 * @brief Sets the configuration register to the given value.
 * (See data-sheet page 18 and 19 for more details on configuration)
 * TODO remove or improve !!!
 *
 * @param value	The configuration to set;
 *
 */
bool AS5600::setConfiguration(uint16_t value){
	// If value is out of range return failure
	if(value > 0x3FFF) return false;

	// Write the configuration register, if no error return success
	HAL_StatusTypeDef error = AS5600::LLW_16Bits(AS5600::CONF_H, value);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the value in the configuration register.
 * (See data-sheet page 18 and 19 for more details on configuration)
 * TODO cache the configuration for faster reading ???
 */
uint16_t AS5600::getConfiguration(void){
	// Read the register
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::CONF_H, &register_content);

	// Return result
	return register_content;
}


// --- Specific configuration

// See data-sheet page 19 and 20 form more details on configuration

/*
 * @brief Sets the device power mode.
 *
 */
bool AS5600::setPowerMode(AS5600::POWER_MODE option){
	// Read previous configuration to change only interested bits
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_8Bits(previous_configuration, AS5600::POWER_MODE_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the device power mode.
 *
 */
AS5600::POWER_MODE AS5600::getPowerMode(void){
	// Read the register
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_L, &register_content);

	// Mask bits of interest
	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::POWER_MODE_MASK);

	// Find and return the corresponding enumeration
	return AS5600::POWER_MODE(result);
}


/*
 * @brief Sets the device hysteresis.
 *
 */
bool AS5600::setHysteresis(AS5600::HYSTERESIS option){
	// Read previous configuration to change only interested bits
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_8Bits(previous_configuration, AS5600::HYSTERESIS_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the device hysteresis.
 *
 */
AS5600::HYSTERESIS AS5600::getHysteresis(void){
	// Read the register
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_L, &register_content);

	// Mask bits of interest
	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::HYSTERESIS_MASK);

	// Find and return the corresponding enumeration
	return AS5600::HYSTERESIS(result);
}


/*
 * @brief Sets the device output mode.
 *
 */
bool AS5600::setOutputMode(AS5600::OUTPUT_MODE option){
	// Read previous configuration to change only interested bits
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_8Bits(previous_configuration, AS5600::OUTPUT_MODE_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the device output mode.
 *
 */
AS5600::OUTPUT_MODE AS5600::getOutputMode(void){
	// Read the register
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_L, &register_content);

	// Mask bits of interest
	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::OUTPUT_MODE_MASK);

	// Find and return the corresponding enumeration
	return AS5600::OUTPUT_MODE(result);
}


/*
 * @brief Sets the output PWM frequency.
 *
 */
bool AS5600::setPWMFrequency(AS5600::PWM_FREQUENCY option){
	// Read previous configuration to change only interested bits
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_8Bits(previous_configuration, AS5600::PWM_FREQUENCY_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the output PWM frequency.
 *
 */
AS5600::PWM_FREQUENCY AS5600::getPWMFrequency(void){
	// Read the register
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_L, &register_content);

	// Mask bits of interest
	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::PWM_FREQUENCY_MASK);

	// Find and return the corresponding enumeration
	return AS5600::PWM_FREQUENCY(result);
}


/*
 * @brief Sets the slow filter configuration.
 *
 */
bool AS5600::setSlowFilter(AS5600::SLOW_FILTER option){
	// Read previous configuration to change only interested bits
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_8Bits(previous_configuration, AS5600::SLOW_FILTER_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the slow filter configuration.
 *
 */
AS5600::SLOW_FILTER AS5600::getSlowFilter(void){
	// Read the register
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_H, &register_content);

	// Mask bits of interest
	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::SLOW_FILTER_MASK);

	// Find and return the corresponding enumeration
	return AS5600::SLOW_FILTER(result);
}


/*
 * @brief Sets the fast filter threshold.
 *
 */
bool AS5600::setFastFilter(AS5600::FAST_FILTER_TH option){
	// Read previous configuration to change only interested bits
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_8Bits(previous_configuration, AS5600::FAST_FILTER_TH_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the fast filter threshold.
 *
 */
AS5600::FAST_FILTER_TH AS5600::getFastFilterThresh(void){
	// Read the register
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_H, &register_content);

	// Mask bits of interest
	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::FAST_FILTER_TH_MASK);

	// Find and return the corresponding enumeration
	return AS5600::FAST_FILTER_TH(result);
}


/*
 * @brief Sets the watchdog state.
 *
 */
bool AS5600::setWatchDog(AS5600::WATCHDOG option){
	// Read previous configuration to change only interested bits
	uint8_t previous_configuration;
	HAL_StatusTypeDef error = AS5600::LLR_8Bits(AS5600::CONF_L, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_8Bits(previous_configuration, AS5600::WATCHDOG_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = AS5600::LLW_8Bits(AS5600::CONF_H, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the watchdog state.
 *
 */
AS5600::WATCHDOG AS5600::getWatchDog(void){
	// Read the register
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::CONF_H, &register_content);

	// Mask bits of interest
	uint8_t result = AS5600::mask_8Bits(register_content, AS5600::WATCHDOG_MASK);

	// Find and return the corresponding enumeration
	return AS5600::WATCHDOG(result);
}


// --- Miscellaneous methods ------------------------------------------------------------

/*
 * @brief Gets the value in the ZMCO register.
 *
 */
uint8_t AS5600::getZMCO(void){
	// Read the value
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::ZMCO, &register_content);

	// Return result
	return register_content;
}

/*
 * @brief Gets the value in the status register.
 *
 */
uint8_t AS5600::getStatus(void){
	// Read the value
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::STATUS, &register_content);

	// Return result
	return register_content;
}

/*
 * @brief Gets the value in the AGC register.
 *
 */
uint8_t AS5600::getAGC(void){
	// Read the value
	uint8_t register_content;
	AS5600::LLR_8Bits(AS5600::AGC, &register_content);

	// Return result
	return register_content;
}

/*
 * @brief Gets the value in the magnitude register.
 *
 */
uint16_t AS5600::getMagnitude(void){
	// Read the value
	uint16_t register_content;
	AS5600::LLR_16Bits(AS5600::MAGNITUDE_H, &register_content);

	// Return result
	return register_content;
}


// END OF FILE
