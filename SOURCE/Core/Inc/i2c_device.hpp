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
	HAL_StatusTypeDef LLR_8Bits(uint8_t register_address, uint8_t *data_buffer);		// Read single-byte data from I2C device
	HAL_StatusTypeDef LLR_16Bits(uint8_t register_address, uint16_t *data_buffer);	// Read multiple-byte data from I2C device

	HAL_StatusTypeDef LLW_8Bits(uint8_t register_address, uint8_t data_buffer);		// Write single-byte data to I2C device
	HAL_StatusTypeDef LLW_16Bits(uint8_t register_address, uint16_t data_buffer);	// Write multiple-byte to I2C device

	// Utility methods for bit operations
	uint8_t mask_8Bits(uint8_t data, uint8_t mask, bool invert_mask = false);		// Apply a mask to the byte
	uint16_t mask_16Bits(uint16_t data, uint16_t mask, bool invert_mask = false);	// Apply a mask to the bytes

private:
	// Utility methods for bit operations
	uint16_t concat_8to16Bits(uint8_t *bytes);				// Concatenate two uint8_t to form a uint16_t
	void break_16to8Bits(uint16_t bytes, uint8_t *result);	// Breaks a uint16_t to form two uint8_t
};





