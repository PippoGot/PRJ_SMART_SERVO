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

HAL_StatusTypeDef I2C_Device::readRegister(uint8_t register_address, uint8_t *data_buffer){
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

HAL_StatusTypeDef I2C_Device::readRegister2(uint8_t register_address, uint16_t *data_buffer){
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

	*data_buffer = concatBytes(buffer);

	return error;
}



HAL_StatusTypeDef I2C_Device::writeRegister(uint8_t register_address, uint8_t data_buffer){
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

HAL_StatusTypeDef I2C_Device::writeRegister2(uint8_t register_address, uint16_t data_buffer){
	uint8_t buffer[2];
	breakBytes(data_buffer, buffer);

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

uint8_t I2C_Device::maskByte(uint8_t data, uint8_t mask, bool invert_mask){
	if(invert_mask) return data & ~mask;

	return data & mask;
}



uint16_t I2C_Device::concatBytes(uint8_t *bytes){
	return ((bytes[0] & 0x0F) << 8) | bytes[1];
}

void I2C_Device::breakBytes(uint16_t bytes, uint8_t *result){
	result[0] = (uint8_t)(bytes >> 8);
	result[1] = (uint8_t)(bytes & 0xFF);
}


