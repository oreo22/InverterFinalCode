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
	int PE0;
	int PE1;
	int PE2;
	int PE3;
//	uint32_t PE0;
//	uint32_t PE1;
//	uint32_t PE2;
//	uint32_t PE3;
};
typedef struct AdcData AdcData_t;

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
