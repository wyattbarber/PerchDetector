/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include "platform.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <cstdio>
#include <chrono>
#include <thread>

#include "Logging.hpp"

void open_VL53L8CX(const char* fname, VL53L8CX_Platform* dev)
{
	dev->wordsize = 8;
	dev->freq = 100000;
	dev->spimode = 0;
	dev->delay = 0;

	dev->fd = open(fname, O_RDWR);
	if(dev->fd < 0)
	{
		Logger::instance() << "[ERROR][VL53L8CX] Failed to open SPI device " << fname << std::endl;
		return;
	}

	if(ioctl(dev->fd, SPI_IOC_WR_BITS_PER_WORD, &dev->spimode) < 0)
	{
		Logger::instance() << "[ERROR][VL53L8CX] Failed to set SPI mode to " << dev->spimode << std::endl;
		close(dev->fd);
		return;
	}

	if(ioctl(dev->fd, SPI_IOC_WR_MAX_SPEED_HZ, &dev->freq) < 0)
	{
		Logger::instance() << "[ERROR][VL53L8CX] Failed to set SPI frequency to " << dev->freq << std::endl;
		close(dev->fd);
		return;
	}

	Logger::instance() << "[INFO][VL53L8CX] Opened SPI device " << fname << std::endl;
}


void close_VL53L8CX(VL53L8CX_Platform* dev)
{
	close(dev->fd);
	Logger::instance() << "[INFO][VL53L8CX] Closed SPI device" << std::endl;
}


uint8_t VL53L8CX_RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	uint8_t status = 255;
	
	struct spi_ioc_transfer xfer[2] = {0, 0};

	const char buffer[] = {
		static_cast<uint8_t>((RegisterAdress & 0xFF00) >> 8), 
		static_cast<uint8_t>(RegisterAdress & 0x00FF)
	};

	xfer[0].tx_buf = (uint64_t)buffer;
	xfer[0].rx_buf = 0;
	xfer[0].len = (uint32_t)2;
	xfer[1].tx_buf = 0;
	xfer[1].rx_buf = (uint64_t)p_value;
	xfer[1].len = 1;

	if(ioctl(p_platform->fd, SPI_IOC_MESSAGE(2), &xfer) < 0)
	{
		Logger::instance() << "[ERROR][VL53L8CX] Failed to read from register " << RegisterAdress << std::endl;
	}
	else
	{
		status = 0;
	}

	return status;
}

uint8_t VL53L8CX_WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	uint8_t status = 255;

	struct spi_ioc_transfer xfer[1] = {0};

	const char buffer[] = {
		static_cast<uint8_t>((RegisterAdress & 0xFF00) >> 8), 
		static_cast<uint8_t>(RegisterAdress & 0x00FF), 
		value
	};
	xfer[0].tx_buf = (uint64_t)buffer;
	xfer[0].rx_buf = 0;
	xfer[0].len = (uint32_t)3;

	if(ioctl(p_platform->fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		Logger::instance() << "[ERROR][VL53L8CX] Failed to write to register " << RegisterAdress << std::endl;
	}
	else
	{
		status = 0;
	}

	return status;
}

uint8_t VL53L8CX_WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status = 255;

	struct spi_ioc_transfer xfer[1] = {0};

	char* buffer = (char*)malloc(size+2);
	buffer[0] = static_cast<uint8_t>((RegisterAdress & 0xFF00) >> 8);
	buffer[1] = static_cast<uint8_t>(RegisterAdress & 0x00FF);
	memcpy(buffer+2, (void*)p_values, size);

	xfer[0].tx_buf = (uint64_t)buffer;
	xfer[0].rx_buf = 0;
	xfer[0].len = size+2;

	if(ioctl(p_platform->fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		Logger::instance() << "[ERROR][VL53L8CX] Failed to write to register " << RegisterAdress << std::endl;
	}	
	else
	{
		status = 0;
	}

	return status;
}

uint8_t VL53L8CX_RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status = 255;
	
	struct spi_ioc_transfer xfer[2] = {0, 0};

	char* buffer = (char*)malloc(size+2);
	buffer[0] = static_cast<uint8_t>((RegisterAdress & 0xFF00) >> 8);
	buffer[1] = static_cast<uint8_t>(RegisterAdress & 0x00FF);
	memcpy(buffer+2, (void*)p_values, size);

	xfer[0].tx_buf = (uint64_t)buffer;
	xfer[0].rx_buf = 0;
	xfer[0].len = 2;
	xfer[1].tx_buf = 0;
	xfer[1].rx_buf = (uint64_t)p_values;
	xfer[1].len = size;

	if(ioctl(p_platform->fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		Logger::instance() << "[ERROR][VL53L8CX] Failed to read from register " << RegisterAdress << std::endl;
	}	
	else
	{
		status = 0;
	}
	
	return status;
}

uint8_t VL53L8CX_Reset_Sensor(
		VL53L8CX_Platform *p_platform)
{
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	/* Set pin CORE_1V8 to LOW */
	VL53L8CX_WaitMs(p_platform, 100);

	/* Set pin LPN to HIGH */
	/* Set pin AVDD to HIGH */
	/* Set pin VDDIO to HIGH */
	/* Set pin CORE_1V8 to HIGH */
	VL53L8CX_WaitMs(p_platform, 100);

	return status;
}

void VL53L8CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t VL53L8CX_WaitMs(
		VL53L8CX_Platform *p_platform,
		uint32_t TimeMs)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(TimeMs));
	return 0;
}
