/*
 * i2c_device.cpp
 *
 * Implementation of i2c_device.hpp header file.
 *
 */

#include "i2c_device.hpp"



// --- Constructor ----------------------------------------------------------------------

I2C_Device::I2C_Device(I2C_HandleTypeDef *device_handle, uint8_t device_address, uint32_t response_delay) :
		_device_handle(device_handle),
		_device_address(device_address << 1),
		_response_delay(response_delay)
	{}



// --- Low-level I2C methods ------------------------------------------------------------

HAL_StatusTypeDef I2C_Device::LLR_8Bits(uint8_t register_address, uint8_t *data_buffer){
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

HAL_StatusTypeDef I2C_Device::LLR_16Bits(uint8_t register_address, uint16_t *data_buffer){
	uint8_t buffer[2];

	HAL_StatusTypeDef error = HAL_I2C_Mem_Read(
			I2C_Device::_device_handle,
			I2C_Device::_device_address,
			register_address,
			I2C_MEMADD_SIZE_8BIT,
			buffer,
			2,
			I2C_Device::_response_delay
			);

	*data_buffer = I2C_Device::concat_8to16Bits(buffer);

	return error;
}



HAL_StatusTypeDef I2C_Device::LLW_8Bits(uint8_t register_address, uint8_t data_buffer){
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

HAL_StatusTypeDef I2C_Device::LLW_16Bits(uint8_t register_address, uint16_t data_buffer){
	uint8_t buffer[2];
	I2C_Device::break_16to8Bits(data_buffer, buffer);

	HAL_StatusTypeDef error = HAL_I2C_Mem_Write(
			I2C_Device::_device_handle,
			I2C_Device::_device_address,
			register_address,
			I2C_MEMADD_SIZE_8BIT,
			buffer,
			2,
			I2C_Device::_response_delay
			);

	return error;
}



// --- Boolean methods ------------------------------------------------------------------

bool I2C_Device::isConnected(void){
	HAL_StatusTypeDef success =	HAL_I2C_IsDeviceReady(
			I2C_Device::_device_handle,
			I2C_Device::_device_address,
			5,
			I2C_Device::_response_delay
			);

	if(success == HAL_OK){
		return true;
	}

	return false;
}



// --- Utility methods for bit operations -----------------------------------------------

uint8_t I2C_Device::mask_8Bits(uint8_t data, uint8_t mask, bool invert_mask){
	if(invert_mask) return data & ~mask;

	return data & mask;
}

uint16_t I2C_Device::mask_16Bits(uint16_t data, uint16_t mask, bool invert_mask){
	if(invert_mask) return data & ~mask;

	return data & mask;
}



uint16_t I2C_Device::concat_8to16Bits(uint8_t *bytes){
	return ((bytes[0] & 0xFF) << 8) | bytes[1];
}

void I2C_Device::break_16to8Bits(uint16_t bytes, uint8_t *result){
	result[0] = (uint8_t)(bytes >> 8);
	result[1] = (uint8_t)(bytes & 0xFF);
}


