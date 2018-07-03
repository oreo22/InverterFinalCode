//*****************************************************************************
//
// led_task.h - Prototypes for the LED task.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#ifndef __ADC_TASK_H__
#define __ADC_TASK_H__

#define SAMPLING_FREQ 6000
#define ARRAY_SIZE 500 //window of the array

struct AdcData{
	uint32_t PE0;
	uint32_t PE1;
	uint32_t PE3;
	uint32_t PE4;
};
typedef struct AdcData AdcData_t;

struct acValue{
	int inst;
	int sum;
	int rms;
	//mean
	//avg_sum
};
typedef struct acValue acValues;

struct ACPower{
	acValues V;
	acValues I;
	acValues P;
	int Q;
	int S;
	
};
typedef struct ACPower ACPower_t;

struct DCPower{
	uint32_t V;
	uint32_t I;
	int P;
	int Q;
};
typedef struct DCPower DCPower_t;

//*****************************************************************************
//
// Prototypes for the LED task.
//
//*****************************************************************************
uint32_t ADCTaskInit(void(*pTask)(AdcData_t pDataStruct));
void ADC_Print(void);
void ADC_PrintJSON(void); 
uint16_t ADC_PrintFreq(void);
void ADCTask(void);

#endif // __LED_TASK_H__
