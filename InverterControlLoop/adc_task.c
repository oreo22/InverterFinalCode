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
void initializeACValues (acValues *acVal);
//*****************************************************************************
//
// ADC Parameters
//
//*****************************************************************************
//AdcData_t *adcRawInput;
uint16_t DataSize=ARRAY_SIZE * sizeof(AdcData_t);
AdcData_t adcRawInput[ARRAY_SIZE ];
ACPower_t Sgrid;

extern SPLL_1ph_SOGI VSync;
uint16_t adc_input_index = 0;
extern int inputValue;
extern uint16_t dcValue;

uint8_t acCalcsDoneFlag=0;
uint8_t rmsFlag=0;

extern double sFactor; //ma
//static AdcData_t max;
//static AdcData_t min;
//static AdcData_t rms;

//RMS values
int Vref=10000;
int Vctrl=0;
DCPower_t dcMeas;
DCPower_t ctrlVars;
int Vdc=0;
int rmsError[2];
float Kp=0.03;
float Ki=0.8;
uint32_t scbLevelShift=1650; //(1.65*4095/3.3)
double sFactor=1; //ma factor, since sawtooth is set to 3.3, it's unknown if the max of the incoming modulated sine wave is at 3.3. 
double ma=0;

double reactance=0.37691184;
int Vfactor;


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

void paramsInialize(void){
			//when you're not lazy, write an inializer for your structs 
		initializeACValues(&Sgrid.I);
		initializeACValues(&Sgrid.V);
		initializeACValues(&Sgrid.P);
		Sgrid.Q=0;
		Vfactor=Vref/reactance;
		rmsError[0]=0;
		rmsError[1]=0;
}
uint32_t status=0;
void ADCTask(void)//Control Params 
{						
 		if(rmsFlag==1){// Polish RMS value 
			
//			scb_mean_load_vrms = avg_Sgrid.V_sum/ARRAY_SIZE;
//			scb_mean_load_irms = avg_sum_load_irms/ARRAY_SIZE;
//			scb_mean_dist_vrms = avg_sum_dist_vrms/ARRAY_SIZE;
//			scb_mean_dist_irms = avg_sum_dist_irms/ARRAY_SIZE; 
			 
			status ^=GPIO_PIN_3;
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, status);
	//------------Updating Values------------
			
		
			//Divide by N
    Sgrid.V.sum /= ARRAY_SIZE;
		Sgrid.I.sum /= ARRAY_SIZE;
		Sgrid.P.sum /= ARRAY_SIZE;
					
			//Sqrt the thing
		Sgrid.V.sum = sqrt(Sgrid.V.sum);
		Sgrid.I.sum = sqrt(Sgrid.I.sum);
					
			//Undo the scaling from adc
		Sgrid.V.rms = ((Sgrid.V.sum) * 3300) / 4095;
		Sgrid.I.rms = ((Sgrid.I.sum) * 3300) / 4095;
		Sgrid.P.rms = ((Sgrid.P.sum) * 3300) / 4095;
		
		//Undo the scaling from the signal conditioning board
		Sgrid.V.rms=Sgrid.V.rms*8.96; //xfmr ratio is actually 4.48 and the SCB is double the input AC waveform
		Sgrid.I.rms=Sgrid.I.rms*1;
		Sgrid.P.rms=Sgrid.P.rms*8.96*1;
		
		rmsFlag=0;
		acCalcsDoneFlag=1;
		
	
		}
		if(acCalcsDoneFlag==1){
			Sgrid.S=Sgrid.V.rms * Sgrid.I.rms;
			Sgrid.S=Sgrid.S *Sgrid.S;
			Sgrid.P.rms=Sgrid.P.rms*Sgrid.P.rms;
			Sgrid.Q=sqrt(Sgrid.S - Sgrid.P.rms); //this is the measured Q
			
			
			//Volt Control by changing Q
			ctrlVars.Q= Vfactor*(Vref - Sgrid.V.rms);
			rmsError[0]=Sgrid.Q - ctrlVars.Q;
			ctrlVars.V= 1+ Kp *(rmsError[0]) + Ki*(rmsError[1]);
			rmsError[1] +=rmsError[0];
			ctrlVars.V=Vref*ctrlVars.V;
			
			//-------Update the DC Values-----------
			Vdc= (dcMeas.V * 3300) / 4095;
			Vdc = Vdc*6.02; //multiply by 8.5247 for the voltage divider and then divide by 1.414 
			
			ma=((double)ctrlVars.V)/ ((double) Vdc);
			acCalcsDoneFlag=0;
		}
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
	
	// 	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
	
	
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE0); //dc value
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawInput[adc_input_index].PE1); //Vinv
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE2); //Iinv
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawInput[adc_input_index].PE3); //Vgrid
	
	dcMeas.V=adcRawInput[adc_input_index].PE0;
	
	//Grabbing AC Values
	//Sgrid.V_avg_sum += adcRawInput[adc_input_index].PE0;
	Sgrid.V.inst = adcRawInput[adc_input_index].PE1 - 1600;//problem with subtracting the 1600 is that if voltage is 0, it still gives it an rms
								//what if you don't subtract, what if you just subtract at the end? 
	Sgrid.V.sum += Sgrid.V.inst * Sgrid.V.inst;
	Sgrid.I.inst= adcRawInput[adc_input_index].PE2 - 1600;
	Sgrid.I.sum += Sgrid.V.inst * Sgrid.V.inst;
	//Sgrid.P.inst =(Sgrid.I.inst * Sgrid.V.inst);
	Sgrid.P.sum += (Sgrid.I.inst * Sgrid.V.inst); //Sgrid.P.sum =Sgrid.P.sum +Sgrid.P.inst;
	
	
//inputValue= (adcRawInput[adc_input_index].PE0* 3300)/4095;
		
	adc_input_index = (adc_input_index + 1) % (ARRAY_SIZE);
		if(adc_input_index==(ARRAY_SIZE - 1)){
			rmsFlag=1;
		}

	//Synchronize to the Vgrid, but don't need the measurements 
	correctedInput=(adcRawInput[adc_input_index].PE3*3300)/4095;
	correctedInput=correctedInput/3300;
	VSync.AC_input =(((double) correctedInput)-0.5)*2;
	PLLRun(&VSync); //Calling the PLL	

//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);

}

void initializeACValues (acValues *acVal){
	acVal->inst=0;
	acVal->rms=0;
	acVal->sum=0;
}
