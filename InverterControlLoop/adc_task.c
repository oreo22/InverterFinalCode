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
#define ADC_SEQUENCE0_PRIORITY  2

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
void ADC0Seq0_Handler(void);
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
double sFactor=1; //ma factor, since sawtooth is set to 3.3, it's unknown if the max of the incoming modulated sine wave is at 3.3. 
double ma=0;
int dc_offset=1887;
ACPower_t Sgrid;
int Vmin=0;
int Vmax=0;

int sqrt(int input)  // OR isqrt16 ( uint16 n ) OR  isqrt8 ( uint8 n ) - respectively [ OR overloaded as isqrt (uint?? n) in C++ ]  
{  
    register uint32_t // OR register uint16 OR register uint8 - respectively  
    root, remainder, place;  
  
    root = 0;  
    remainder = input;  
    place = 0x40000000; // OR place = 0x4000; OR place = 0x40; - respectively  
  
    while (place > remainder)  
        place = place >> 2;  
    while (place)  
    {  
        if (remainder >= root + place)  
        {  
            remainder = remainder - root - place;  
            root = root + (place << 1);  
        }  
        root = root >> 1;  
        place = place >> 2;  
    }  
    return root;  
} 
int tempMax=0;
uint32_t status=0;
int avg_sum_V =0;
int avg_V=0;
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
					//Divide by N
    Sgrid.V.sum /= ARRAY_SIZE;
		Sgrid.I.sum /= ARRAY_SIZE;
		Sgrid.P.sum /= ARRAY_SIZE;	
			//Sqrt the thing
		Sgrid.V.sum = sqrt(Sgrid.V.sum);
		Sgrid.I.sum = sqrt(Sgrid.I.sum);
					
			//Undo the scaling from adc
		Sgrid.V.rms = ((Sgrid.V.sum) * 3300) / 4095;
		Sgrid.I.rms = ((Sgrid.I.sum) * (6600)) / 4095; //(2* (Vrefp - Vrefn)) / 4095
		Sgrid.P.rms = ((Sgrid.P.sum) );
		Sgrid.V.rms = Sgrid.V.rms;	// (for upper numbers)
		
		//Undo the scaling from the signal conditioning board
		Sgrid.V.rms=Sgrid.V.rms*14.36; //xfmr ratio is actually 4.48 and the SCB is double the input AC waveform
		Sgrid.I.rms=Sgrid.I.rms*7.45;
		Sgrid.P.rms=Sgrid.P.rms*107;
	//		Sgrid.P.rms=Sgrid.I.rms*Sgrid.V.rms;
		UARTprintf("Current Measurement: %d \n",Sgrid.I.rms); 		
	UARTprintf("Voltage Measurement: %d \n",Sgrid.V.rms);
	UARTprintf("Power Measurement: %d \n",Sgrid.P.rms); 		
//	tempMax=tempMax* (6600) / 4095;
//	UARTprintf("PE3 Max %d \n",  tempMax );	
			//dc_offset=(Vmin+Vmax)/2;
		//	 Vmin=100;
// Vmax=0;
		rmsFlag=0;
						status ^=GPIO_PIN_3;
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, status);
	}
//			scb_mean_load_vrms = avg_sum_load_vrms/ARRAY_SIZE;
//			scb_mean_load_irms = avg_sum_load_irms/ARRAY_SIZE;
//			scb_mean_dist_vrms = avg_sum_dist_vrms/ARRAY_SIZE;
//			scb_mean_dist_irms = avg_sum_dist_irms/ARRAY_SIZE; 
//			//UARTprintf("AVG: %d\n",scb_mean_load_vrms);
//			 

//	//------------Updating Values------------
//          sum_load_vrms /= ARRAY_SIZE;
//					sum_load_irms /= ARRAY_SIZE;
//					sum_dist_vrms /= ARRAY_SIZE;
//					sum_dist_irms /= ARRAY_SIZE;
//					
//          sum_load_vrms = sqrt(sum_load_vrms);
//					sum_load_irms = sqrt(sum_load_irms);
//					sum_dist_vrms = sqrt(sum_dist_vrms); 
//					sum_dist_irms = sqrt(sum_dist_irms);
//					
//					result_load_vrms = ((sum_load_vrms) * 3300) / 4095;
//					result_load_irms = ((sum_load_irms) * 3300) / 4095;
//					result_dist_vrms = ((sum_dist_vrms) * 3300) / 4095;
//					result_dist_irms = ((sum_dist_irms) * 3300) / 4095;
//					//undo_sig_board
//					result_load_vrms=result_load_vrms; //xfmr ratio is actually 4.48 and the SCB is double the input AC waveform: *2.1875
//			rmsFlag=0;
//			// UARTprintf("PE1 %d \n",  result_load_vrms); //use 1.14 
//		//	UARTprintf("PE4 %d \n",  shifted_adc.PE4);
//		  //shifted_adc.PE3=  (shifted_adc.PE3*1650*2)/4095;
//	//		sum_load_vrms=shifted_adc.PE3-2048;
//	//		sum_load_vrms=(sum_load_vrms*3300)/4095;
////	tempMax/= ARRAY_SIZE;
//		//	UARTprintf("PE3 %d \n",  shifted_adc.PE3 );
	//UARTprintf("PE3 Max %d \n",  tempMax );
	//				tempMax=0;
//					avg_sum_load_vrms = 0;
//					avg_sum_load_irms = 0;
//					avg_sum_dist_vrms = 0;
//					avg_sum_dist_irms = 0;
//			//-------Update the DC Values-----------
//		Vdc=(dcMeas * 3300) / 4095;
//		Vdc=Vdc*6.02; //multiply by 8.5247 for the voltage divider and then divide by 1.414 
//		UARTprintf("DC Voltage %d \n", Vdc);
//		//Volt Control 
//		rmsError[0]=Vref - result_load_vrms;
//		Vctrl=(rmsError[1]*Ki+rmsError[0]*Kp)+Vref;
//		ma=((double)Vctrl)/ ((double) Vdc);

//		//UARTprintf("Current corrected ma %d \n", sFactor);
//		rmsError[1]=rmsError[0];
//}
		
	
	
		
		//freq done in the PLL loop
		
		//Volt Control
	
		
		//UARTprintf("Current InputValue %d \n", inputValue);
		// https://stackoverflow.com/questions/28807537/any-faster-rms-value-calculation-in-c#28808472 
	
}

void ADC_Print(void) {
	
	//UARTprintf("PE3: %d   |   PE4: %d   |    PE1: %d    |   PE0: %d\n", adcRawInput[adc_input_index].PE0,adcRawInput[adc_input_index].PE1,adcRawInput[adc_input_index].PE4, adcRawInput[adc_input_index].PE3);	UARTprintf("Min PE3: %d   |   PE4: %d   |    PE1: %d    |   PE0: %d\n", min.PE3, min.PE4, min.PE1, min.PE0);
}

void ADC_PrintJSON(void) {

	//UARTprintf("@{\"pv\" : %d, \"inverter\" : %d, \"wind\" : %d, \"grid\" : %d, \"load\" : %d,}\n", rms.PE0, rms.PE1, rms.PE4, rms.PE3, 419);
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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);	
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0); //must have enabled ADC first
	   GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 ); //must have enabled ADC first

    TimerDisable(TIMER2_BASE, TIMER_A);
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true); //enable TIMER2A trigger to ADC
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // timer module is disabled before being configured to periodic, left in disabled state
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/SAMPLING_FREQ); //SysCltClockGet returns count for 1 second so SysCtlClockGet() / 1000 sets the TIMER2B load value to 1ms.
		TimerIntDisable(TIMER2_BASE, 0xFFFFFFFF ); //disable all interrupts for this timer
    TimerEnable(TIMER2_BASE, TIMER_A);
		
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_EIGHTH , 1); //last param is divider
	
	
	// Configure sequence 2 for Vac 
		ADCSequenceDisable(ADC0_BASE, ADC_SEQUENCE2); 
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE2, ADC_TRIGGER_TIMER, 0);
		ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 0, ADC_CTL_CH2); //PE1, Vbus, single-ended
		ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 1, ADC_CTL_D | ADC_CTL_CH0); //PE3 & PE2, Ibus, differential sampling on both channels
		ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 2, ADC_CTL_CH8 |  ADC_CTL_END | ADC_CTL_IE  ); // PE4
		ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE2); //adc_base, sequence
	
//	//DC Voltage Sequencer 
//		ADCSequenceDisable(ADC0_BASE, ADC_SEQUENCE0); 
//    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE0, ADC_TRIGGER_TIMER, 0);
//		ADCSoftwareOversampleConfigure(ADC0_BASE, ADC_SEQUENCE0, 4);
//		ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 0, ADC_CTL_CH3 |  ADC_CTL_END | ADC_CTL_IE ); //PE0, Vdc, single-ended
//		ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE0); //adc_base, sequence 

 
		//	ADCHardwareOversampleConfigure(ADC0_BASE, 8);

		//Interrupt Setup 
		ADCIntEnable(ADC0_BASE, ADC_SEQUENCE2);
    ADCIntRegister(ADC0_BASE, ADC_SEQUENCE2, &ADC0Seq2_Handler);
    IntPrioritySet(INT_ADC0SS2, ADC_SEQUENCE2_PRIORITY);
		IntDisable(INT_ADC0SS2);
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE2);
		
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
	data->PE4 = 0;
	data->PE3 = 0;
}
void setAdcData (AdcData_t *data) {
	data->PE0 = 4095;
	data->PE1 = 4095;
	data->PE4 = 4095;
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
//	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawInput[adc_input_index].PE0); //dc bus
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE1);  //PE1, Vbus, single-ended
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE3);  //PE4 & PE3, Ibus
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE4); //PE4, Vgrid, single-ended, 


	//Grabbing AC Values
	//Sgrid.V_avg_sum += adcRawInput[adc_input_index].PE0;
	Sgrid.V.inst = adcRawInput[adc_input_index].PE1-2048; //1650 in hex form
	Sgrid.V.sum += Sgrid.V.inst * Sgrid.V.inst;
	
//	avg_sum_V+= Sgrid.V.inst;
//		if(Sgrid.V.inst<0){
//		tempMax=Sgrid.V.inst;
//	}
	Sgrid.I.inst= adcRawInput[adc_input_index].PE3-2048; //2028;
	Sgrid.I.sum += Sgrid.I.inst * Sgrid.I.inst;
//	if(Sgrid.I.inst>tempMax){
//		tempMax=Sgrid.I.inst;
//	}
	Sgrid.P.inst =(((Sgrid.I.inst* 6600) / 4095) * ((Sgrid.V.inst* 3300) / 4095));
	Sgrid.P.sum =Sgrid.P.sum +Sgrid.P.inst;

	
//	//shifted_adc.PE1 = adcRawInput[adc_input_index].PE1;
//	shifted_adc.PE4 = adcRawInput[adc_input_index].PE4;
//	shifted_adc.PE3 = adcRawInput[adc_input_index].PE3;
//	//	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE3); //dc value
////	dcMeas=adcRawInput[adc_input_index].PE0;
////	avg_sum_load_vrms += adcRawInput[adc_input_index].PE1;
//	shifted_adc.PE1 = adcRawInput[adc_input_index].PE1;//-1600; //-1600; // - 1680;//problem with subtracting the 1600 is that if voltage is 0, it gives it an rms
//	sum_load_vrms += shifted_adc.PE1 * shifted_adc.PE1;
//		shifted_adc.PE3 = adcRawInput[adc_input_index].PE3-2028; //-1600; // - 1680;//problem with subtracting the 1600 is that if voltage is 0, it gives it an rms
//		shifted_adc.PE3=(shifted_adc.PE3*6600)/4095;
//	if(shifted_adc.PE3>tempMax){
//		tempMax=shifted_adc.PE3;
//	}
//	
//	sum_load_irms += shifted_adc.PE3 * shifted_adc.PE3;
//	tempMax=shifted_adc.PE3+tempMax;
//	shifted_adc.PE3=(shifted_adc.PE3*3300)/4095;	
	
//	
//		
////	shifted_adc.PE4 = adcRawInput[adc_input_index].PE4;//-1600; //-1600; // - 1680;//problem with subtracting the 1600 is that if voltage is 0, it gives it an rms
//	sum_load_irms += shifted_adc.PE4 * shifted_adc.PE4;
//	//7.49 
//	/*If the array is full, subtract oldest value */
//	 
//	//fwave[w_index]=VSync.AC_input;
	correctedInput=(adcRawInput[adc_input_index].PE1*3300)/4095;
	correctedInput=correctedInput/3300;
	VSync.AC_input =(((float) correctedInput)-0.5)*2;
	//correctedInput=(correctedInput* 3300);
	//correctedInput=adcRawInput[adc_input_index].PE0-scbLevelShift; //take out that level shift

	 
	//inputValue= (adcRawInput[adc_input_index].PE0* 3300)/4095;
		adc_input_index = (adc_input_index + 1) % (ARRAY_SIZE);
		if(adc_input_index==(ARRAY_SIZE - 1)){
			rmsFlag=1;
		}
		//Calling the PLL	
		PLLRun(&VSync);

//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);

}


void ADC0Seq0_Handler(void)
{
  ADCIntClear(ADC0_BASE, ADC_SEQUENCE0); // Clear the timer interrupt flag.
	//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawInput[adc_input_index].PE0); //dc bus
	dcMeas=adcRawInput[adc_input_index].PE0;
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);

}

