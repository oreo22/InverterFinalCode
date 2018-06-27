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

//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
uint16_t counter=0;
void GPIOTask(void) //void *pvParameters)
{

}

void GPIO_PB2_SET_HIGH(void) {
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

void GPIO_PB2_SET_LOW(void) {
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
}
	
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
