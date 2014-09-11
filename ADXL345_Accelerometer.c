// Name        : ADXL345_Accelerometer.c
// Author      : Mahendra Gunawardena
// Version     : Rev 0.01
// Copyright   : Your copyright notice
// Description : Tiva ADXL345Accelerometer in C++, Ansi-style
//============================================================================
/*
 * ADXL345_Accelerometer.c
 * Implementation of a interface with the Analog Devices ADXL345 3 Axis Accelerometer
 * over the I2C bus
 *
 * Copyright Mahendra Gunawardena, Mitisa LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL I
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include "Tiva_i2c.h"
#include "ADXL345_Accelerometer.h"

void init_ADXL345_Accelerometer(){
	initI2C0();
}

uint8_t getAccelerometer_ID() {
	return (readI2C0(0x53,0x00));
}

void SetPowerMode(unsigned char powerMode) {
	uint8_t i2c_data = readI2C0(0x53,0x2d);
	if (powerMode==1){
		i2c_data = i2c_data | (powerMode<<3);
	} else if (powerMode==0){
		i2c_data &= ~(1<<3);
	}
	i2c_data = i2c_data | (powerMode<<3);
	writeI2C0(0x53,0x2d,i2c_data);
}

int getAccelerationData(){

	return 0;
}

uint16_t getAcceleration_rawX(){
	uint8_t accelData1, accelData2;
	accelData1=readI2C0(0x53,0x32);
	accelData2=readI2C0(0x53,0x33);
	rawX = (accelData2<<8)|accelData1;
	return (rawX);
}

uint16_t getAcceleration_rawY(){
	uint8_t accelData1, accelData2;
	accelData1=readI2C0(0x53,0x34);
	accelData2=readI2C0(0x53,0x35);
	rawY = (accelData2<<8)|accelData1;
	return (rawY);
}

uint16_t getAcceleration_rawZ(){
	uint8_t accelData1, accelData2;
	accelData1=readI2C0(0x53,0x36);
	accelData2=readI2C0(0x53,0x37);
	rawZ = (accelData2<<8)|accelData1;
	return (rawZ);
}

signed int getAcceleration_X(){
	signed int short raw = (signed int short) getAcceleration_rawX();
	signed int acceleration = (signed int)(((signed int)raw) * 3.9);
	return acceleration;
}

signed int getAcceleration_Y(){
	signed int short raw = (signed int short) getAcceleration_rawY();
	signed int acceleration = (signed int)(((signed int)raw) * 3.9);
	return acceleration;
}

signed int getAcceleration_Z(){
	signed int short raw = (signed int short) getAcceleration_rawZ();
	signed int acceleration = (signed int)(((signed int)raw) * 3.9);
	return acceleration;
}

signed int getPitch(){
	return (0);
}
signed int getRoll(){
	return (0);
}
