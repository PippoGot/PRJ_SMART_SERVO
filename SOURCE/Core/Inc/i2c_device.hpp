/*
 * i2c_device.hpp
 *
 * Module containing a class for communicating with a generic I2C device.
 *
 * Contains reading and writing (low-level) functions.
 *
 */

#pragma once

#include "stm32f1xx_hal.h"



// --- I2C device class -----------------------------------------------------------------

class I2C_Device {

public:
	// Constructor
	I2C_Device(
			I2C_HandleTypeDef *device_handle,
			uint8_t device_address,
			uint32_t response_delay = HAL_MAX_DELAY
			);

	// Getter methods
	uint8_t getDeviceAddress(void){ return I2C_Device::_device_address >> 1; };
	I2C_HandleTypeDef getDeviceHandle(void){ return *I2C_Device::_device_handle; };
	uint32_t getResponseDelay(void){ return I2C_Device::_response_delay; };

	// Boolean methods
	bool isConnected(void);

protected:
	// Variables
	I2C_HandleTypeDef *_device_handle;	// Bus to which the device is connected to
	const uint8_t _device_address; 		// I2C address of the device
	const uint32_t _response_delay;		// Response delay used in read-write operations

	// Low-level I2C methods
	HAL_StatusTypeDef readRegister(uint8_t register_address, uint8_t *data_buffer);		// Read single-byte data from I2C device
	HAL_StatusTypeDef readRegister2(uint8_t register_address, uint16_t *data_buffer);	// Read multiple-byte data from I2C device

	HAL_StatusTypeDef writeRegister(uint8_t register_address, uint8_t data_buffer);		// Write single-byte data to I2C device
	HAL_StatusTypeDef writeRegister2(uint8_t register_address, uint16_t data_buffer);	// Write multiple-byte to I2C device

	// Utility methods for bit operations
	uint8_t maskByte(uint8_t data, uint8_t mask, bool invert_mask = false);	// Apply a mask to the byte

private:
	// Utility methods for bit operations
	uint16_t concatBytes(uint8_t *bytes);				// Concatenate two uint8_t to form a uint16_t
	void breakBytes(uint16_t bytes, uint8_t *result);	// Breaks a uint16_t to form two uint8_t
};






