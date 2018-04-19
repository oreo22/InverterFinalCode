//*****************************************************************************
//
// led_task.c - A simple flashing LED task.
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
//#include "../inc/tm4c123gh6pm.h"
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "pwm_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "pwm.h" //for pwm lib
#include "hw_memmap.h" //for address bases
#include "sysctl.h" //for init ports
#include "gpio.h" //for gpio to be interfaced to adc
#include "interrupt.h" //for interrupt
#include "hw_ints.h" //for INT_TIMER2A
#include "pin_map.h" //for GPIO_PB6_M0PWM0
#include "timer.h" //for timer
#include "interrupt.h" //for interrupt
#include "adc_task.h"

#define GPIO_PORTB_DR8R_R       (*((volatile uint32_t *)0x40005508))
#define PWM0_0_CMPA_R           (*((volatile uint32_t *)0x40028058))
#define PWM0_0_GENA_R           (*((volatile uint32_t *)0x40028060))
#define PWM0_0_GENB_R           (*((volatile uint32_t *)0x40028064))
//*****************************************************************************
//
// The stack size for the task.
//
//*****************************************************************************
#define PWMTASKSTACKSIZE        128         // Stack size in words
#define TIMER1_PRIORITY 2
#define SWITCHING_FREQ  33000

//*****************************************************************************
//
// ADC Init Macros
//
//*****************************************************************************
#define ADC_SEQUENCE2           2
#define ADC_SEQUENCE2_PRIORITY  2

//*****************************************************************************
//
// The item size and queue size for the message queue.
//
//*****************************************************************************
#define PWM_ITEM_SIZE           sizeof(uint8_t)
#define PWM_QUEUE_SIZE          5

//*****************************************************************************
//
// The queue that holds messages sent to the LED task.
//
//*****************************************************************************
xQueueHandle g_pPwmQueue;

extern xSemaphoreHandle g_pUARTSemaphore;
extern AdcData_t *adcRawInput;
extern uint16_t adc_input_index;
int inputValue=0;
extern xSemaphoreHandle g_pUARTSemaphore;
//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************

static void PWMTask(void *pvParameters)
{
}

void configureTimer1A(void){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Enable Timer 1 Clock
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic
  TimerLoadSet(TIMER1_BASE, TIMER_A, 165); //SysCtlClockGet()/SWITCHING_FREQ is set to 80MHz according to main file, Reload Value = fclk/fswitch

//Configuring the interrupts	
	TimerIntRegister(TIMER1_BASE, TIMER_A, &Timer1AIntHandler);
	IntPrioritySet(INT_TIMER1A, TIMER1_PRIORITY);
	TimerEnable(TIMER1_BASE, TIMER_A);	// Start Timer 1A
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	IntDisable(INT_TIMER1A);
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

//*****************************************************************************
//
// Initializes the PWM task to output a PWM to PB6 and it's complement to PB7.
//
//*****************************************************************************
uint32_t PWMTaskInit(void)
{
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1); //set PWM clock to processor clock with multiplier of 1
		SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
	  GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_DB_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 8000); //80MHz/60kHz
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0); //D= D*8000, so.5=4000
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,0 );
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
   // PWMOutputInvert(PWM0_BASE, PWM_OUT_0_BIT, false);	//AC Chopper takes a PWM signal and it's complement.
    GPIO_PORTB_DR8R_R |=0xC0; //The chopper driver must have 8mA output
	//	PWM0_0_GENA_R = 0xE0;					//low on UpA, high on downA 
 // PWM0_0_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
//	PWM0_0_GENA_R = 0xB0; //high with CompA up: x30, low on pWMA down, x8 or xB0
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_ZERO);
		PWMGenIntRegister(PWM0_BASE, PWM_GEN_0, &PWM0IntHandler);
		IntPrioritySet(INT_PWM0_0, 1);
	 	PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0);
		PWMGenEnable(PWM0_BASE, PWM_GEN_0);


		GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_DB_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 4000);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    //PWMOutputInvert(PWM0_BASE, PWM_OUT_1_BIT, true);
   // PWMDeadBandEnable(PWM0_BASE, PWM_GEN_1, 0xF, 0xF);
    GPIO_PORTB_DR8R_R |=0xC0; // 8mA output
		 PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_1, PWM_INT_CNT_ZERO);
		PWMGenIntRegister(PWM0_BASE, PWM_GEN_1, &PWM0IntHandler);
		IntPrioritySet(INT_PWM0_0, 0);
	 	PWMIntEnable(PWM0_BASE, PWM_INT_GEN_1);
		PWMGenEnable(PWM0_BASE, PWM_GEN_1);
		//IntDisable(INT_PWM0_0);

	//	configureTimer1A();
	
    /* Used for more intense signals
    PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0); 
    IntMasterEnable();
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
    IntEnable(INT_PWM0_0);
     */

}
int sawtooth[33] = {1650, 1860, 2060,2270,2480,2680,2890,3090, 3300,3090,2890,2680, 2480, 2270,2060, 1860 ,1650 ,1440, 1240 ,1030 ,830 ,620, 410, 210, 0,210,410,620,830,1030,1240,1440};
	//float VCtrlIn[18]={1,0.7,0.8,0.9,0.1,0.7,1.1,1.5,1.8,12,13,14,15,17,18,21,2.3,2.4}; 
uint32_t saw_index=0;
// PD2 is the A branch, PD3 is the B branch 
//sFactor=1
	int cSawtoothValue=0;
void Timer1AIntHandler(void){
  TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);                // clear the timer interrupt
////	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
////	inputValue=(int)(1650.0*new_theta);
//	PWM0_0_LOAD_R=inputValue;
//	cSawtoothValue=PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1)/(SysCtlClockGet()/SWITCHING_FREQ); //normalize the count value to the 3300 range for open loop ctrl
//	cSawtoothValue=cSawtoothValue*3300;
//	int inputNeg= (-1* inputValue)+3300;
////	cSawtoothValue=sawtooth[saw_index-1];//*dcValue; 
//	//cSawtoothValue=cSawtoothValue-1650;///dcDesired;
//		
//		if(inputValue >= cSawtoothValue) {
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
//		}
//		if(inputValue < cSawtoothValue  ){ 
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00);
//		}
//		if (inputNeg > cSawtoothValue){
////			GPIO_PORTD_DATA_R =0x010;  //PD2 off, PD3 on
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
//		}
//		if(inputNeg < cSawtoothValue){ 
//			//GPIO_PORTD_DATA_R =0x020;  //PD2 on, PD3 off 
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);
//		}
//	saw_index= (saw_index %33 )+1;
//	//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);
}
uint32_t oldPulseW=0;
uint32_t pulseW=0;
uint32_t pulseWArray[100];
void PWM0IntHandler(void)
{
		//Just set duty cycles of the incoming waveform 
	//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
    PWMGenIntClear(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_ZERO);
		int inputNeg= (-1* inputValue)+3300;
 		pulseW=(inputValue*7997)/3300; //control ma by alternating the magnitude of the inputValue 	
		uint32_t negpulseW=(inputNeg*7997)/3300; //can't go up to 7998 for some reason?! creates a notch
		pulseWArray[saw_index]=pulseW;
		
	saw_index= (saw_index %100)+1;
	if( pulseW<=oldPulseW){
		bool fun=false;
	}
	oldPulseW=pulseW;
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,pulseW); //PB6
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,negpulseW); //PB7
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);

}

