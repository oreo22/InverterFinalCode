#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "gpio_task.h"
#include "adc_task.h"
#include "sysctl.h" //for init ports

#include "adc.h" //for adc lib
#include "hw_memmap.h" //for address bases
#include "sysctl.h" //for init ports
#include "gpio.h" //for gpio to be interfaced to adc
#include "timer.h" //for timer
#include "interrupt.h" //for interrupt
#include "hw_ints.h" //for INT_TIMER2A

/*
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
*/

//*****************************************************************************
//
// The stack size for the task.
//
//*****************************************************************************
#define GPIOSTASKSTACKSIZE        128         // Stack size in words

#define TIMER1A_PRIORITY 1
#define dcDesired 7000
#define voltageDivider 3

extern AdcData_t *adcRawInput;
extern uint16_t adc_input_index;
//int inputValue=0;
uint16_t dcValue=0;

//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
uint16_t counter=0;
void GPIOTask(void) //void *pvParameters)
{
	//UARTprintf("GPIO Started");
	counter++;
//		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
	//	IntEnable(INT_TIMER1A);
	//	xSemaphoreGive(g_pUARTSemaphore);
		//TimerIntDisable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	 
	
		GPIO_PB2_SET_HIGH();
		if(counter == 10000){
			 GPIO_PB2_SET_LOW();
		//	UARTprintf("ma= %i \n",ma);
			counter=0;
		}
		
}

void GPIO_PB2_SET_HIGH(void) {
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

void GPIO_PB2_SET_LOW(void) {
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
}
	
extern double new_theta;
//*****************************************************************************
//
// Initializes the GPIO and the Timer for Leg A & B Switching
//
//*****************************************************************************
uint32_t GPIOTaskInit(void)
{	
	//GPIO Initialization
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); 
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,  GPIO_PIN_6 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0 );
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_2 | GPIO_PIN_3 );
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3 );
	//configureTimer1A();
    // Create the task.
	
	
   /* if(xTaskCreate(GPIOTask, (const portCHAR *)"GPIO", GPIOSTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_GPIO_TASK, NULL) != pdTRUE) 
    {
        return(1);
    }
    // Success.
    return(0);*/
	return(1);
	
}
//uint32_t clockTime=0;
//void configureTimer1A(void){
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Enable Timer 1 Clock
//	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic
//  TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/SWITCHING_FREQ); // is set to 80MHz according to main file, Reload Value = fclk/fswitch
//	//Configuring the interrupts	
//	TimerIntRegister(TIMER1_BASE, TIMER_A, &Timer1AIntHandler);
//	IntPrioritySet(INT_TIMER1A, 1);
//	TimerEnable(TIMER1_BASE, TIMER_A);	// Start Timer 1A
//	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
//	IntDisable(INT_TIMER1A);
//	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
//}

//  //double sFactor=1; //ma factor, since sawtooth is set to 3.3, it's unknown if the max of the incoming modulated sine wave is at 3.3. 
////If the peak isn't, change the scaling factor of the sawtooth to the ratio of the maxIncomingSignal/3.3, so it's a percentage of the full. Modulate the whole sawtooth to 
////(if incoming signal is greater than 3.3, ignore value), do this calculation elsewhere 

//int sawtooth[33] = {1650, 1860, 2060,2270,2480,2680,2890,3090, 3300,3090,2890,2680, 2480, 2270,2060, 1860 ,1650 ,1440, 1240 ,1030 ,830 ,620, 410, 210, 0,210,410,620,830,1030,1240,1440};
//	//float VCtrlIn[18]={1,0.7,0.8,0.9,0.1,0.7,1.1,1.5,1.8,12,13,14,15,17,18,21,2.3,2.4}; 
//uint32_t saw_index=0;
//// PD2 is the A branch, PD3 is the B branch 
////sFactor=1
//	int cSawtoothValue=0;
//void Timer1AIntHandler(void){
//  TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);                // clear the timer interrupt
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
//	cSawtoothValue=sawtooth[saw_index-1];//*dcValue; 
//	cSawtoothValue=cSawtoothValue-1650;///dcDesired;
///*	inputValue=(int)(1650.0*new_theta);
//	int inputNeg= (-1* inputValue);
//	
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
//		}*/
//	saw_index= (saw_index %33 )+1;
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);
//}
