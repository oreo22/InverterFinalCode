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
#include "pwm_task.h"
#include "PLL.h"
#include "IQmathLib.h"
//#include "heap_1.h"

//*****************************************************************************
//
// ADC Init Macros
//
//*****************************************************************************
#define ADC_SEQUENCE2           2
#define ADC_SEQUENCE0           0
#define ADC_SEQUENCE2_PRIORITY  1

//*****************************************************************************
//
// The item size and queue size for the message queue.
//
//*****************************************************************************
#define ADC_ITEM_SIZE           sizeof(uint8_t)
#define ADC_QUEUE_SIZE          5

// Private functions, shouldn't be touched outside of adc_task.c
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
AdcData_t shifted_adc;

extern SPLL_1ph_SOGI VSync;
uint16_t adc_input_index = 0;
extern int inputValue;
extern uint16_t dcValue;
uint8_t rmsFlag=0;

extern double sFactor; //ma
//static AdcData_t max;
//static AdcData_t min;
//static AdcData_t rms;

//RMS values
int Vref=10000;
int Vctrl=0;
int dcMeas=0;
int Vdc=0;
uint32_t rmsError[2];
float Ki,Kp=0;
static int sum_squares=0; //accumulator of the instant samples 
uint32_t scbLevelShift=1650; //(1.65*4095/3.3)
double sFactor=1; //ma factor, since sawtooth is set to 3.3, it's unknown if the max of the incoming modulated sine wave is at 3.3. 
double ma=0;

		int avg_sum_load_vrms;
		int avg_sum_load_irms;
		int avg_sum_dist_vrms;
		int avg_sum_dist_irms; 
		int scb_mean_load_vrms; //2022 //Power Supply 4.131
		int scb_mean_load_irms; //2891 //
		int scb_mean_dist_vrms; //0000 //Power Supply 4.131
		int scb_mean_dist_irms; //0000 //
		int result_load_vrms = 0;
		int result_load_irms = 0;
		int result_dist_vrms = 0;
		int result_dist_irms = 0;
		int sum_load_vrms = 0;
		int sum_load_irms = 0;
		int sum_dist_vrms = 0;
		int sum_dist_irms = 0;

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
	//Get the rms values of Igrid and Vgrid
	//Update frequency
	//Run control loop for volt and var
	//if anti-islanding time, switch strategies and same for bat discharge
	
					
//					load_v_rms = undo_signal_conditioning_load_vrms(result_load_vrms);
//			 		load_i_rms = undo_signal_conditioning_load_irms(result_load_irms);
//					dist_v_rms = undo_signal_conditioning_dist_vrms(result_dist_vrms);
//					dist_i_rms = undo_signal_conditioning_dist_irms(result_dist_irms);
					
					//UARTprintf("Volt: %d\n", undo_signal_conditioning_load_vrms(23));
					//UARTprintf("Curr: %d\n", result_dist_irms);
					//load_v_rms = result_load_vrms;
					//load_i_rms = result_load_irms;	
					//dist_i_rms
					/*
					UARTprintf("Volt: %d\n", result_dist_vrms);
					UARTprintf("Curr: %d\n", result_dist_irms);
					UARTprintf("Volt: %d\n", result_dist_vrms);
					UARTprintf("Curr: %d\n", result_dist_irms);	
						*/				
	
//UARTprintf("AVG: %d\n",scb_mean_load_vrms);*/
 		if(rmsFlag==1){// Polish RMS value 
			
			scb_mean_load_vrms = avg_sum_load_vrms/ARRAY_SIZE;
			scb_mean_load_irms = avg_sum_load_irms/ARRAY_SIZE;
			scb_mean_dist_vrms = avg_sum_dist_vrms/ARRAY_SIZE;
			scb_mean_dist_irms = avg_sum_dist_irms/ARRAY_SIZE; 
			//UARTprintf("AVG: %d\n",scb_mean_load_vrms);
			 
			status ^=GPIO_PIN_3;
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, status);
	//------------Updating Values------------
          sum_load_vrms /= ARRAY_SIZE;
					sum_load_irms /= ARRAY_SIZE;
					sum_dist_vrms /= ARRAY_SIZE;
					sum_dist_irms /= ARRAY_SIZE;
					
          sum_load_vrms = sqrt(sum_load_vrms);
					sum_load_irms = sqrt(sum_load_irms);
					sum_dist_vrms = sqrt(sum_dist_vrms); 
					sum_dist_irms = sqrt(sum_dist_irms);
					
					result_load_vrms = ((sum_load_vrms) * 3300) / 4095;
					result_load_irms = ((sum_load_irms) * 3300) / 4095;
					result_dist_vrms = ((sum_dist_vrms) * 3300) / 4095;
					result_dist_irms = ((sum_dist_irms) * 3300) / 4095;
					//undo_sig_board
					result_load_vrms=result_load_vrms*8.96; //xfmr ratio is actually 4.48 and the SCB is double the input AC waveform
			rmsFlag=0;
			UARTprintf("Grid Vrms %d \n",  result_load_vrms);
					avg_sum_load_vrms = 0;
					avg_sum_load_irms = 0;
					avg_sum_dist_vrms = 0;
					avg_sum_dist_irms = 0;
			//-------Update the DC Values-----------
		Vdc=(dcMeas * 3300) / 4095;
		Vdc=Vdc*6.02; //multiply by 8.5247 for the voltage divider and then divide by 1.414 
		UARTprintf("DC Voltage %d \n", Vdc);
		//Volt Control 
		rmsError[0]=Vref - result_load_vrms;
		Vctrl=(rmsError[1]*Ki+rmsError[0]*Kp)+Vref;
		ma=((double)Vctrl)/ ((double) Vdc);

		//UARTprintf("Current corrected ma %d \n", sFactor);
		rmsError[1]=rmsError[0];
		
	
		}
		
		//freq done in the PLL loop
		
		//Volt Control
	
		
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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0); //must have enabled ADC first
    TimerDisable(TIMER2_BASE, TIMER_A);
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true); //enable TIMER2A trigger to ADC
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // timer module is disabled before being configured to periodic, left in disabled state
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/SAMPLING_FREQ); //SysCltClockGet returns count for 1 second so SysCtlClockGet() / 1000 sets the TIMER2B load value to 1ms.
		TimerIntDisable(TIMER2_BASE, 0xFFFFFFFF ); //disable all interrupts for this timer
    TimerEnable(TIMER2_BASE, TIMER_A);
		
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_EIGHTH , 1); //last param is divider
    ADCSequenceDisable(ADC0_BASE, ADC_SEQUENCE2); 
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE2, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 0, ADC_CTL_CH3 );  //sequencer 2 is only 4 fifo deep, and CH3 is PE0, Ch0 is PE3
  //  ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 2, ADC_CTL_CH2 );
    //ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 2, ADC_CTL_CH1 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 3, ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE); //adc_base, Sequence Number, Step, set flag and end after first
    ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE2); //adc_base, sequence 
//		ADCSoftwareOversampleConfigure(ADC0_BASE, ADC_SEQUENCE1, 4);
//	ADCHardwareOversampleConfigure(ADC0_BASE, 8);

		//Set up Sequence0 for dc voltage, but the interrupt will be using the Sequence2 interrupt 
		ADCSequenceDisable(ADC0_BASE, ADC_SEQUENCE0); 
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE0, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 0, ADC_CTL_CH2 );  //sequencer0 only uses PE1, has a 8 deep fifo
		ADCSoftwareOversampleConfigure(ADC0_BASE, ADC_SEQUENCE0, 4);
    ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE0); //adc_base, sequence 
		
    
		//Interrupt Setup
		ADCIntEnable(ADC0_BASE, ADC_SEQUENCE2);
    ADCIntRegister(ADC0_BASE, ADC_SEQUENCE2, &ADC0Seq2_Handler);
    IntPrioritySet(INT_ADC0SS2, ADC_SEQUENCE2_PRIORITY);
		IntDisable(INT_ADC0SS2);
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE2);
		
		//Two options: 1) use hardware averaging meaning you set up ADC1 base, set up alternate interrupt, but interrupts at a lesser frequency
		//2) Add Sequence0 for a deeper fifo and just use software averaging on sequence 0
		
		//Set up the ADC1 for DC voltages to enable hardware sampling? 
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
float correctedInput=0;
float testing=0;
extern SPLL_1ph_SOGI VSync;
	double fwave[100 ];
uint16_t w_index = 0;
void ADC0Seq2_Handler(void)
{
  ADCIntClear(ADC0_BASE, ADC_SEQUENCE2); // Clear the timer interrupt flag.
	//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE0);
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawInput[adc_input_index].PE1); //dc value
	dcMeas=adcRawInput[adc_input_index].PE1;
	//avg_sum_load_vrms += adcRawInput[adc_input_index].PE0;
	shifted_adc.PE0 = adcRawInput[adc_input_index].PE0 - 1600;//problem with subtracting the 1600 is that if voltage is 0, it gives it an rms
	sum_load_vrms += shifted_adc.PE0 * shifted_adc.PE0;
	 
	//fwave[w_index]=VSync.AC_input;
	correctedInput=(adcRawInput[adc_input_index].PE0*3300)/4095;
	correctedInput=correctedInput/3300;

//	UARTprintf("Grid Vrms %d \n", correctedInput);
	VSync.AC_input =(((float) correctedInput)-0.5)*2;
	//correctedInput=(correctedInput* 3300);
	//correctedInput=adcRawInput[adc_input_index].PE0-scbLevelShift; //take out that level shift

	
	//inputValue= (adcRawInput[adc_input_index].PE0* 3300)/4095;
//	
//	dcValue=(adcRawInput[adc_input_index].PE1* 3300)/4095;
//	dcValue=dcValue*3;
		
		adc_input_index = (adc_input_index + 1) % (ARRAY_SIZE);
		//w_index = (w_index + 1) % (100);
		if(adc_input_index==(ARRAY_SIZE - 1)){
			rmsFlag=1;
		}

		//Calling the PLL	
		PLLRun(&VSync);
	/*	
			xSemaphoreGiveFromISR(arrayFull, &xHigherPriorityTaskWoken);
			sqrt(rms/ARRAY_SIZE) //alt way to calculate when the array is full
		}*/
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);

}

