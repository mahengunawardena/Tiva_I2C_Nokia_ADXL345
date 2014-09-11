//============================================================================
// Name        : Tiva_Accelerometer.c
// Author      : Mahendra Gunawardena
// Version     : Rev 0.01
// Copyright   : Your copyright notice
// Description : Tiva ADXL345Accelerometer in C++, Ansi-style
//============================================================================
/*
 * Tiva_Accelerometer.c
 * Implementation of a class to interface with the Analog Devices ADXL345 3 Axis Accelerometer
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
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "Nokia5110.h"
#include "ADXL345_Accelerometer.h"
#include "inc/tm4c123gh6pm.h"

/*#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define PF4                     (*((volatile unsigned long *)0x40025040))
#define PF3                     (*((volatile unsigned long *)0x40025020))
#define PF2                     (*((volatile unsigned long *)0x40025010))
#define PF1                     (*((volatile unsigned long *)0x40025008))
#define PF0                     (*((volatile unsigned long *)0x40025004))
#define GPIO_PORTF_DR2R_R       (*((volatile unsigned long *)0x40025500))
#define GPIO_PORTF_DR4R_R       (*((volatile unsigned long *)0x40025504))
#define GPIO_PORTF_DR8R_R       (*((volatile unsigned long *)0x40025508))
#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))

void My_Delay(void){unsigned long volatile time;
  time = 145448;  // 0.1sec
  while(time){
		time--;
  }
}
void PortF_Init(void){
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) activate clock for Port F
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}
unsigned long Led;*/

int main (void)
{
	uint8_t i2c_data;
	unsigned char dev_id[2];
	signed int accelerationX, accelerationY, accelerationZ;

	Nokia5110_Init();
	// Set the clocking to run directly from the external crystal/oscillator.
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);

	init_ADXL345_Accelerometer();

//	PortF_Init();

	Nokia5110_Clear();
	Nokia5110_SetCursor(0, 0);
	Nokia5110_OutString("TIVA/ADXL345");

	Nokia5110_SetCursor(0, 2);
	Nokia5110_OutString("Device ID:");

	i2c_data = getAccelerometer_ID();
	dev_id[0] = ((i2c_data & 0xF0) >> 4) + 0x30 + 0x07;
	dev_id[1] = (i2c_data & 0x0F) + 0x30;
	Nokia5110_OutString(dev_id);

	SetPowerMode(0x01);

	while(1){
/*		Led = GPIO_PORTF_DATA_R;	// read previous
		Led = Led^0x02;				// toggle red LED, PF1
		GPIO_PORTF_DATA_R = Led;	// output
	    My_Delay();
*/

		accelerationX = getAcceleration_X();
		Nokia5110_SetCursor(0, 3);
		if (accelerationX>=0) {
			Nokia5110_OutString("X Axl: ");
			Nokia5110_OutUDec((unsigned short) accelerationX);
		} else {
			Nokia5110_OutString("X Axl:-");
			Nokia5110_OutUDec((unsigned short) (accelerationX*-1));
		}

		accelerationY = getAcceleration_Y();
		Nokia5110_SetCursor(0, 4);
		if (accelerationY>=0) {
			Nokia5110_OutString("Y Axl: ");
			Nokia5110_OutUDec((unsigned short) accelerationY);
		} else {
			Nokia5110_OutString("Y Axl:-");
			Nokia5110_OutUDec((unsigned short) (accelerationY*-1));
		}

		accelerationZ = getAcceleration_Z();
		Nokia5110_SetCursor(0, 5);
		if (accelerationZ>=0) {
			Nokia5110_OutString("Z Axl: ");
			Nokia5110_OutUDec((unsigned short) accelerationZ);
		} else {
			Nokia5110_OutString("Z Axl:-");
			Nokia5110_OutUDec((unsigned short) (accelerationZ*-1));
		}
	}

	return 0;
}
