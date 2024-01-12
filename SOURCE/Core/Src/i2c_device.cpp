/*
 * i2c_device.cpp
 *
 * Implementation of i2c_device.hpp header file.
 *
 */

#include "i2c_device.hpp"



// --- Constructor ----------------------------------------------------------------------

/*
 * @brief Constructs a I2c device object. Derive from this class when implementing interfaces
 * for I2C devices.
 *
 * @param device_handle		I2C bus handle object;
 * @param device_address	Address of the I2C device;
 * @param response_delay	Time to wait for the device response;
 *
 */
I2C_Device::I2C_Device(I2C_HandleTypeDef *device_handle, uint8_t device_address, uint32_t response_delay) :
		_device_handle(device_handle),
		_device_address(device_address << 1),
		_response_delay(response_delay)
	{}


// --- Low-level I2C methods ------------------------------------------------------------

// --- Read

/*
 * @brief Read single-byte data from I2C device.
 *
 */
HAL_StatusTypeDef I2C_Device::LLR_8Bits(uint8_t register_address, uint8_t *data_buffer){
	// I2C read
	return HAL_I2C_Mem_Read(
			I2C_Device::_device_handle,
			I2C_Device::_device_address,
			register_address,
			I2C_MEMADD_SIZE_8BIT,
			data_buffer,
			1,
			I2C_Device::_response_delay
			);
}

/*
 * @brief Read multiple-byte data from I2C device.
 *
 */
HAL_StatusTypeDef I2C_Device::LLR_16Bits(uint8_t register_address, uint16_t *data_buffer){
	// Create buffer for data
	uint8_t buffer[2];

	// I2C read
	HAL_StatusTypeDef error = HAL_I2C_Mem_Read(
			I2C_Device::_device_handle,
			I2C_Device::_device_address,
			register_address,
			I2C_MEMADD_SIZE_8BIT,
			buffer,
			2,
			I2C_Device::_response_delay
			);

	// Concatenate and save data
	*data_buffer = I2C_Device::concat_8to16Bits(buffer);

	// Return error if any
	return error;
}


// --- Write

/*
 * @brief Write single-byte data to I2C device.
 *
 */
HAL_StatusTypeDef I2C_Device::LLW_8Bits(uint8_t register_address, uint8_t data_buffer){
	// I2C write
	return HAL_I2C_Mem_Write(
			I2C_Device::_device_handle,
			I2C_Device::_device_address,
			register_address,
			I2C_MEMADD_SIZE_8BIT,
			&data_buffer,
			1,
			I2C_Device::_response_delay
			);
}

/*
 * @brief Write multiple-byte to I2C device.
 *
 */
HAL_StatusTypeDef I2C_Device::LLW_16Bits(uint8_t register_address, uint16_t data_buffer){
	// Create buffer for data
	uint8_t buffer[2];

	// Break data to smaller chunks for transfer
	I2C_Device::break_16to8Bits(data_buffer, buffer);

	// I2C write
	HAL_StatusTypeDef error = HAL_I2C_Mem_Write(
			I2C_Device::_device_handle,
			I2C_Device::_device_address,
			register_address,
			I2C_MEMADD_SIZE_8BIT,
			buffer,
			2,
			I2C_Device::_response_delay
			);

	// Return error if any
	return error;
}


// --- Utility methods ------------------------------------------------------------------

/*
 * @brief Checks if the device is connected and working.
 *
 */
bool I2C_Device::isConnected(void){
	// I2C check
	HAL_StatusTypeDef success =	HAL_I2C_IsDeviceReady(
			I2C_Device::_device_handle,
			I2C_Device::_device_address,
			5,
			I2C_Device::_response_delay
			);

	// If device responds return success
	if(success == HAL_OK){
		return true;
	}

	// Return failure as default
	return false;
}


// --- Utility methods for bit operations -----------------------------------------------

// --- Protected

/*
 * @brief Apply a mask to the 8 bit data. Optionally invert the mask.
 *
 * @param data			Data to mask;
 * @param mask			Mask for the data;
 * @param invert_mask	Optionally, can invert the mask;
 *
 */
uint8_t I2C_Device::mask_8Bits(uint8_t data, uint8_t mask, bool invert_mask){
	// If requested invert mask and return masked data
	if(invert_mask) return data & ~mask;

	// Return masked data as default
	return data & mask;
}

/*
 * @brief Apply a mask to the 16 bit data. Optionally invert the mask.
 *
 * @param data			Data to mask;
 * @param mask			Mask for the data;
 * @param invert_mask	Optionally, can invert the mask;
 *
 */
uint16_t I2C_Device::mask_16Bits(uint16_t data, uint16_t mask, bool invert_mask){
	// If requested invert mask and return masked data
	if(invert_mask) return data & ~mask;

	// Return masked data as default
	return data & mask;
}


// --- Private

/*
 * @brief Concatenate two 8 bit data to form a 16 bit data.
 *
 * @param bytes	The data to concatenate;
 *
 */
uint16_t I2C_Device::concat_8to16Bits(uint8_t *bytes){
	// Shift first value to get the high half, then sum the second value for low half
	return ((bytes[0] & 0xFF) << 8) | bytes[1];
}

/*
 * @brief Breaks 16 bit data in to two 8 bit data.
 *
 * @param bytes		The data to break;
 * @param result	The buffer for the result storage;
 *
 */
void I2C_Device::break_16to8Bits(uint16_t bytes, uint8_t *result){
	// Shifts data for high half, then mask and stores the low half
	result[0] = (uint8_t)(bytes >> 8);
	result[1] = (uint8_t)(bytes & 0xFF);
}


// END OF FILE
