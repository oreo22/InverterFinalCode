#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "adc_task.h"
#include "priorities.h"

#include "adc.h" //for adc lib
#include "hw_memmap.h" //for address bases
#include "sysctl.h" //for init ports
#include "gpio.h" //for gpio to be interfaced to adc
#include "timer.h" //for timer
#include "interrupt.h" //for interrupt
#include "hw_ints.h" //for INT_TIMER2A
#include "gpio_task.h" //for gpio to be interfaced to adc

#include "PLL.h"
#include "IQmathLib.h"
//#include "heap_1.h"

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
#define ADC_ITEM_SIZE           sizeof(uint8_t)
#define ADC_QUEUE_SIZE          5

//*****************************************************************************
//
// Private functions in this file  
//
//*****************************************************************************
void (*ProducerTask)(AdcData_t pDataStruct);  
void ADC0Seq2_Handler(void);
void clearAdcData (AdcData_t *data);
void setAdcData (AdcData_t *data);


//*****************************************************************************
//
// ADC Parameters
//
//*****************************************************************************
//AdcData_t *adcRawInput;
uint16_t DataSize=ARRAY_SIZE * sizeof(AdcData_t);
AdcData_t adcRawInput[ARRAY_SIZE ];

uint16_t adc_input_index = 0;
extern int inputValue;
extern uint16_t dcValue;
uint16_t rmsFlag;

extern double sFactor; //ma
//static AdcData_t max;
//static AdcData_t min;
//static AdcData_t rms;
uint32_t rms=0;
uint16_t rmsFlag=0;
static int sum_squares=0; //accumulator of the instant samples 
uint32_t scbLevelShift=1650; //(1.65*4095/3.3)
extern double sFactor;

unsigned long adcCount = 0; //debug
int sqrt(int input) {
  
  int guess = 1050; //midpoint of the 0 to 1.65 range in int form
  int guess_sq = 1102500;
  int delta = 525;
  for(int i = 0; i < 9; i++) {
    if(input > guess_sq) {
      guess += delta;
      guess_sq = (guess) * (guess);
    } else {
      guess -= delta;
      guess_sq = (guess) * (guess);
    }
    delta /= 2;
  }
  return guess; 
} 

static double one_step_newton_raphson_sqrt(double val, double hint)
{
	double probe;
	if (hint <= 0) return val /2;
	probe = val / hint;
	return (probe+hint) /2;
}

uint32_t status=0;
void ADCTask(void)//void *pvParameters
{
		if(rmsFlag==1){// Polish RMS value 
				status ^=GPIO_PIN_3;
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, status);
			sum_squares/= ARRAY_SIZE;
      sum_squares = sqrt(sum_squares);
      rms = ((sum_squares) * 3300) / 4095;
			rmsFlag=0;
		//	UARTprintf("RMS on PE0 %d \n",rms);
			UARTprintf("DC Voltage on PE1 %u \n",7000/(dcValue*3));
		}
		//UARTprintf("Current InputValue %d \n", inputValue);
		// https://stackoverflow.com/questions/28807537/any-faster-rms-value-calculation-in-c#28808472 
	
}

void ADC_Print(void) {
	
	//UARTprintf("PE3: %d   |   PE2: %d   |    PE1: %d    |   PE0: %d\n", adcRawInput[adc_input_index].PE0,adcRawInput[adc_input_index].PE1,adcRawInput[adc_input_index].PE2, adcRawInput[adc_input_index].PE3);	UARTprintf("Min PE3: %d   |   PE2: %d   |    PE1: %d    |   PE0: %d\n", min.PE3, min.PE2, min.PE1, min.PE0);
}

void ADC_PrintJSON(void) {

	//UARTprintf("@{\"pv\" : %d, \"inverter\" : %d, \"wind\" : %d, \"grid\" : %d, \"load\" : %d,}\n", rms.PE0, rms.PE1, rms.PE2, rms.PE3, 419);
}


//*****************************************************************************
//
// Initializes the LED task.
//
//*****************************************************************************
uint32_t ADCTaskInit(void(*pTask)(AdcData_t pDataStruct))
{
    //IntMasterEnable(); //needed? Should be non critical
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0); //must have enabled ADC first
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true); //enable TIMER0A trigger to ADC
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // timer module is disabled before being configured to periodic, left in disabled state
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/SAMPLING_FREQ); //SysCltClockGet returns count for 1 second so SysCtlClockGet() / 1000 sets the Timer0B load value to 1ms.
		TimerIntDisable(TIMER0_BASE, 0xFFFFFFFF ); //disable all interrupts for this timer
    TimerEnable(TIMER0_BASE, TIMER_A);
		
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_EIGHTH , 1); //last param is divider
    ADCSequenceDisable(ADC0_BASE, ADC_SEQUENCE2); 
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE2, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 0, ADC_CTL_CH3 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 1, ADC_CTL_CH2 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 2, ADC_CTL_CH1 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 3, ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE); //adc_base, Sequence Number, Step, set flag and end after first
    ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE2); //adc_base, sequence 
    
		//Interrupt Setup
		ADCIntEnable(ADC0_BASE, ADC_SEQUENCE2);
    ADCIntRegister(ADC0_BASE, ADC_SEQUENCE2, &ADC0Seq2_Handler);
    IntPrioritySet(INT_ADC0SS2, ADC_SEQUENCE2_PRIORITY);
		IntDisable(INT_ADC0SS2);
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE2);
}

void clearAdcData (AdcData_t *data) {
	data->PE0 = 0;
	data->PE1 = 0;
	data->PE2 = 0;
	data->PE3 = 0;
}
void setAdcData (AdcData_t *data) {
	data->PE0 = 4095;
	data->PE1 = 4095;
	data->PE2 = 4095;
	data->PE3 = 4095;
}
int correctedInput=0;
float testing=0;
extern SPLL_1ph_SOGI PLLSync;
void ADC0Seq2_Handler(void)
{
	//GPIO_PB2_SET_HIGH();
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE2); // Clear the timer interrupt flag.
	//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE0);
		ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE1); //dc value
	//	inputValue= (adcRawInput[adc_input_index].PE0* 3300)/4095;
		correctedInput=(adcRawInput[adc_input_index].PE0* 3300)/4095;
		dcValue=(adcRawInput[adc_input_index].PE1* 3300)/4095;
		dcValue=dcValue*3;
		PLLSync.u[0] =((float) correctedInput)/3300;
		correctedInput=inputValue-scbLevelShift; //take out that level shift 
		sum_squares+=correctedInput*correctedInput;
			adc_input_index = (adc_input_index + 1) % (ARRAY_SIZE);
		if(adc_input_index==(ARRAY_SIZE - 1)){
			rmsFlag=1;
		}

		//Calling the PLL		
		IntEnable(INT_TIMER2A);
		IntPendSet(INT_TIMER2A); 
	/*	
			xSemaphoreGiveFromISR(arrayFull, &xHigherPriorityTaskWoken);
			sqrt(rms/ARRAY_SIZE) //alt way to calculate when the array is full
		}*/
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);

}

