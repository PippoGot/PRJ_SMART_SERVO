/*
 * INA219.cpp
 *
 * Implementation of INA219.hpp header file.
 *
 */

#include "INA219.hpp"



// ------------------------------------------------------ INA219 class implementation ---

// --- Device constructors --------------------------------------------------------------

/*
 * @brief Constructor for an INA219 current sensor.
 *
 * @param device_handle 		I2C bus handle object;
 * @param max_expexted_current 	Value of highest current passing through shunt resistor;
 * @oaram shunt_resistor 		Value of shunt resistor used to measure current;
 * @param device_address		Address of the I2C device;
 * @param response_delay		Time to wait for the device response;
 *
 */
INA219::INA219(
I2C_HandleTypeDef *device_handle,
float max_expected_current,
float shunt_resistor,
uint8_t device_address,
uint32_t response_delay
) :
		I2C_Device(device_handle, device_address, response_delay)
	{
		// Calibrate sensor with given current and resistor values
		INA219::calibrateSensor(max_expected_current, shunt_resistor);
	}


// --- Sensor core methods --------------------------------------------------------------

// --- Raw sensor readings

/*
 * @brief Gets the content of the bus voltage register, without flag bits.
 *
 */
int16_t INA219::getRawBusVoltage(void){
	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::BUS_VOLTAGE, &register_content);

	// Remove flag bits, multiply by the LSB and then return result (see data-sheet page 23)
	return (int16_t)(register_content >> 3) * 4;
}

/*
 * @brief Gets the content of the shunt voltage register.
 *
 */
int16_t INA219::getRawShuntVoltage(void){
	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::SHUNT_VOLTAGE, &register_content);

	// Return result
	return (int16_t)register_content;
}

/*
 * @brief Gets the content of the current register.
 *
 */
int16_t INA219::getRawCurrent(void){
	// Re-calibration to avoid problems (based on Adafruit library)
	INA219::calibrateSensor(INA219::_max_expected_current, INA219::_shunt_resistor);

	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CURRENT, &register_content);

	// Return result
	return (int16_t)register_content;
}

/*
 * @brief Gets the content of the power register.
 *
 */
int16_t INA219::getRawPower(void){
	// Re-calibration to avoid problems (based on Adafruit library)
	INA219::calibrateSensor(INA219::_max_expected_current, INA219::_shunt_resistor);

	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::POWER, &register_content);

	// Return result
	return (int16_t)register_content;
}


// --- Sensor readings

/*
 * @brief Returns the sensed bus voltage after proper conversion.
 *
 */
float INA219::getBusVoltage_V(void){
	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::BUS_VOLTAGE, &register_content);

	// Checks flags: return -100 if overflow
	uint16_t register_flags = INA219::mask_16Bits(register_content, INA219::BUS_REGISTER_FLAGS_MASK);
	if(register_flags & INA219::BUS_REGISTER_OVF_MASK) return -100;

	// Remove flag bits, multiply by fixed 4 mV LSB and return result (see data-sheet page 23)
	return (register_content >> 3) * 4e-3;
}

/*
 * @brief Returns the sensed shunt voltage after proper conversion.
 *
 */
float INA219::getShuntVoltage_V(void){
	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::SHUNT_VOLTAGE, &register_content);

	// Multiply by fixed 10 uV LSB and return the result (see data-sheet page 21)
	return (int16_t)register_content * 1e-5;
}

/*
 * @brief Return the sensed current after proper conversion.
 *
 */
float INA219::getCurrent_A(void){
	// Re-calibration to avoid problems (based on Adafruit library)
	INA219::calibrateSensor(INA219::_max_expected_current, INA219::_shunt_resistor);

	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CURRENT, &register_content);

	// Multiply by the current LSB and return the result
	return (int16_t)register_content * INA219::_current_lsb;
}

/*
 * @brief Return the sensed power after proper conversion.
 *
 */
float INA219::getPower_W(void){
	// Re-calibration to avoid problems (based on Adafruit library)
	INA219::calibrateSensor(INA219::_max_expected_current, INA219::_shunt_resistor);

	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::POWER, &register_content);

	// Multiply by the power LSB and return the result
	return (int16_t)register_content * INA219::_power_lsb;
}


// --- Calibration and parameters

/*
 * @brief Sets the calibration register based on the given parameters.
 *
 * @param max_expected_current	Value of the highest current expected through shunt resistor;
 * @param shunt_resistor		Value of shunt resistor employed;
 *
 */
void INA219::calibrateSensor(float max_expected_current, float shunt_resistor){
	// Updates variables
	INA219::_max_expected_current = max_expected_current;
	INA219::_shunt_resistor = shunt_resistor;

	// Compute LSBs (see data-sheet page 12, equations 2 and 3)
	INA219::_current_lsb = INA219::_max_expected_current / 32768;
	INA219::_power_lsb = 20 * INA219::_current_lsb;

	// Compute calibration value (see data-sheet page 12, equation 1)
	uint16_t calibration_value = (uint16_t)(0.04096 / (INA219::_current_lsb * INA219::_shunt_resistor));

	// Write computed calibration value
	INA219::LLW_16Bits(INA219::CALIBRATION, calibration_value);
}

/*
 * @brief Gets the value in the calibration register.
 *
 */
uint16_t INA219::getCalibration(void){
	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CALIBRATION, &register_content);

	// Return result
	return register_content;
}


// --- Sensor utility methods -----------------------------------------------------------

// Empty for now


// --- Sensor configuration methods -----------------------------------------------------

// --- General configuration

/*
 * @brief Sets the configuration register to the given value.
 * (See data-sheet page 19 and 20 for more details on configuration)
 * TODO remove or improve !!!
 *
 * @param value	The configuration to set;
 *
 */
bool INA219::setConfiguration(uint16_t value){
	// Write configuration
	HAL_StatusTypeDef error = INA219::LLW_16Bits(INA219::CONF, value);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the value in the configuration register.
 * (See data-sheet page 19 and 20 for more details on configuration)
 * TODO cache the configuration for faster reading ???
 */
uint16_t INA219::getConfiguration(void){
	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	// Return result
	return register_content;
}


// --- Specific configuration

/*
 * @brief Sets the bus voltage range configuration.
 * (See data-sheet page 19, table 3 for more details on configuration)
 *
 */
bool INA219::setBusVoltageRange(INA219::BUS_VOLTAGE_RANGE option){
	// Read previous configuration to change only interested bits
	uint16_t previous_configuration;
	HAL_StatusTypeDef error = INA219::LLR_16Bits(INA219::CONF, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_16Bits(previous_configuration, INA219::BUS_VOLTAGE_RANGE_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = INA219::LLW_16Bits(INA219::CONF, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the bus voltage range configuration.
 * (See data-sheet page 19, table 3 for more details on configuration)
 *
 */
INA219::BUS_VOLTAGE_RANGE INA219::getBusVoltageRange(void){
	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	// Mask bits of interest
	uint16_t result = INA219::mask_16Bits(register_content, INA219::BUS_VOLTAGE_RANGE_MASK);

	// Find and return the corresponding enumeration
	return INA219::BUS_VOLTAGE_RANGE(result);
}


/*
 * @brief Sets the bus programmable gain configuration.
 * (See data-sheet page 19, table 4 for more details on configuration)
 *
 */
bool INA219::setPGA(INA219::PGA option){
	// Read previous configuration to change only interested bits
	uint16_t previous_configuration;
	HAL_StatusTypeDef error = INA219::LLR_16Bits(INA219::CONF, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_16Bits(previous_configuration, INA219::PGA_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = INA219::LLW_16Bits(INA219::CONF, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the bus voltage range configuration.
 * (See data-sheet page 19, table 4 for more details on configuration)
 *
 */
INA219::PGA INA219::getPGA(void){
	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	// Mask bits of interest
	uint16_t result = INA219::mask_16Bits(register_content, INA219::PGA_MASK);

	// Find and return the corresponding enumeration
	return INA219::PGA(result);
}


/*
 * @brief Sets the bus ADC resolution.
 * (See data-sheet page 19, table 5 for more details on configuration)
 *
 */
bool INA219::setBusADCResolution(INA219::BUS_ADC_RESOLUTION option){
	// Read previous configuration to change only interested bits
	uint16_t previous_configuration;
	HAL_StatusTypeDef error = INA219::LLR_16Bits(INA219::CONF, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_16Bits(previous_configuration, INA219::BUS_ADC_RESOLUTION_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = INA219::LLW_16Bits(INA219::CONF, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the bus ADC resolution.
 * (See data-sheet page 19, table 5 for more details on configuration)
 *
 */
INA219::BUS_ADC_RESOLUTION INA219::getBusADCResolution(void){
	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	// Mask bits of interest
	uint16_t result = INA219::mask_16Bits(register_content, INA219::BUS_ADC_RESOLUTION_MASK);

	// Find and return the corresponding enumeration
	return INA219::BUS_ADC_RESOLUTION(result);
}


/*
 * @brief Sets the shunt ADC resolution.
 * (See data-sheet page 19, table 5 for more details on configuration)
 *
 */
bool INA219::setShuntADCResolution(INA219::SHUNT_ADC_RESOLUTION option){
	// Read previous configuration to change only interested bits
	uint16_t previous_configuration;
	HAL_StatusTypeDef error = INA219::LLR_16Bits(INA219::CONF, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_16Bits(previous_configuration, INA219::SHUNT_ADC_RESOLUTION_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = INA219::LLW_16Bits(INA219::CONF, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the shunt ADC resolution.
 * (See data-sheet page 19, table 5 for more details on configuration)
 *
 */
INA219::SHUNT_ADC_RESOLUTION INA219::getShuntADCResolution(void){
	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	// Mask bits of interest
	uint16_t result = INA219::mask_16Bits(register_content, INA219::SHUNT_ADC_RESOLUTION_MASK);

	// Find and return the corresponding enumeration
	return INA219::SHUNT_ADC_RESOLUTION(result);
}


/*
 * @brief Sets the device operating mode.
 * (See data-sheet page 19, table 6 for more details on configuration)
 *
 */
bool INA219::setOperatingMode(INA219::OPERATING_MODE option){
	// Read previous configuration to change only interested bits
	uint16_t previous_configuration;
	HAL_StatusTypeDef error = INA219::LLR_16Bits(INA219::CONF, &previous_configuration);
	if(error != HAL_OK) return false;

	// Mask and set configuration bits of interest
	previous_configuration = mask_16Bits(previous_configuration, INA219::OPERATING_MODE_MASK, true);
	previous_configuration |= option;

	// Set new configuration
	error = INA219::LLW_16Bits(INA219::CONF, previous_configuration);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}

/*
 * @brief Gets the device operating mode.
 * (See data-sheet page 19, table 6 for more details on configuration)
 *
 */
INA219::OPERATING_MODE INA219::getOperatingMode(void){
	// Read the register
	uint16_t register_content;
	INA219::LLR_16Bits(INA219::CONF, &register_content);

	// Mask bits of interest
	uint16_t result = INA219::mask_16Bits(register_content, INA219::OPERATING_MODE_MASK);

	// Find and return the corresponding enumeration
	return INA219::OPERATING_MODE(result);
}


// --- Miscellaneous methods ------------------------------------------------------------

/*
 * @brief Resets the device to default configuration.
 * (See data-sheet page 19, table 3 for more details on reset)
 *
 */
bool INA219::reset(void){
	// Set reset bit
	HAL_StatusTypeDef error = INA219::LLW_16Bits(INA219::CONF, INA219::RESET_MASK);
	if(error == HAL_OK) return true;

	// Return failure as default
	return false;
}


// END OF FILE
