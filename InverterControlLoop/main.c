//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
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

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "pwm_task.h"
#include "adc_task.h"
#include "gpio_task.h"
#include "Control.h"
#include "interrupt.h"
#include "hw_ints.h" //for INT_TIMER2A
#include "PLL.h"


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>FreeRTOS Example (freertos_demo)</h1>
//!
//! This application demonstrates the use of FreeRTOS on Launchpad.
//!
//! The application blinks the user-selected LED at a user-selected frequency.
//! To select the LED press the left button and to select the frequency press
//! the right button.  The UART outputs the application status at 115,200 baud,
//! 8-n-1 mode.
//!
//! This application utilizes FreeRTOS to perform the tasks in a concurrent
//! fashion.  The following tasks are created:
//!
//! - An LED task, which blinks the user-selected on-board LED at a
//!   user-selected rate (changed via the buttons).
//!
//! - A Switch task, which monitors the buttons pressed and passes the
//!   information to LED task.
//!
//! In addition to the tasks, this application also uses the following FreeRTOS
//! resources:
//!
//! - A Queue to enable information transfer between tasks.
//!
//! - A Semaphore to guard the resource, UART, from access by multiple tasks at
//!   the same time.
//!
//! - A non-blocking FreeRTOS Delay to put the tasks in blocked state when they
//!   have nothing to do.
//!
//! For additional details on FreeRTOS, refer to the FreeRTOS web page at:
//! http://www.freertos.org/
//
//*****************************************************************************




//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif



//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
	
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

void Producer(AdcData_t pDataStruct) {

}
extern int inputValue;
//uint32_t Grid_freq=60;
//long DELTA_T=1/SAMPLING_FREQ; //time diff
 //pointer to the SPLL_1ph_SOGI object
volatile LPF_COEFF lpf_coeff;
extern SPLL_1ph_SOGI VSync;
extern uint8_t ctrlFlag;
int main(void){
	//use static globsls
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_25MHZ);  // Set the clocking to run at 80 MHz from the PLL.
	GPIOTaskInit() ;
	ADCTaskInit(&Producer);
	PWMTaskInit();
	PLLTaskInit((float)60,(double)1/(double)SAMPLING_FREQ, lpf_coeff,&VSync); //uint16_t Grid_freq, long DELTA_T, volatile SPLL_1ph_SOGI *spll_obj, volatile LPF_COEFF lpf_coeff
	ConfigureUART();
	//configureTimer2A();
	UARTprintf("Start Program \n");
	IntEnable(INT_ADC0SS2);
	IntEnable(INT_PWM0_0);
//	IntEnable(INT_TIMER2A);
	//IntEnable(INT_TIMER0A);		
	//	IntPendSet(INT_TIMER0A); 
	//IntEnable(INT_TIMER1A);
	

	//IntEnable(INT_TIMER2A);
	while(1){
		ADCTask();
	if(ctrlFlag==1){VarControl();}
		
		
	//	GPIOTask();
		/*uint16_t rms_filter(uint16_t sample)
		{
    static uint16_t rms = INITIAL;
    static uint32_t sum_squares = 1UL * SAMPLES * INITIAL * INITIAL;

    sum_squares -= sum_squares / SAMPLES;
    sum_squares += (uint32_t) sample * sample;
    if (rms == 0) rms = 1;    // do not divide by zero 
			rms = (rms + sum_squares / SAMPLES / rms) / 2;
			return rms;
		}
				
				
					}*/
	}
		
}
