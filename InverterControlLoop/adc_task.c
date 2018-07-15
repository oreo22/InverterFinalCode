#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "adc_task.h"
#include "priorities.h"
#include "math.h"
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
#include "Control.h" //for adc lib
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

//Accessible to other files 
uint8_t rmsFlag=0;
uint8_t ctrlFlag=0;
double ma=0.5;
ACPower_t Sbus;
ACPower_t Sgrid;
int dcMeas=0;
int maxValue=0;
double scaling[11]={18.75,15.036, 15.293, 14.706, 13.573, 12.272, 10.95, 10.331, 10.466, 10.923, 11.879}; 
uint32_t status=0;
uint16_t DataSize=ARRAY_SIZE * sizeof(AdcData_t);
AdcData_t adcRawInput[ARRAY_SIZE ];
uint16_t adc_input_index = 0;
char str[80];
extern SPLL_1ph_SOGI VSync;
extern double Vref;
extern int inputValue;
double Vrms=0;
double Irms=0;
double Qmeas=0;
double Pmeas=0;
int count=0; 
int sqrtInt(int input)  // OR isqrt16 ( uint16 n ) OR  isqrt8 ( uint8 n ) - respectively [ OR overloaded as isqrt (uint?? n) in C++ ]  
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
void ADCTask(void)//void *pvParameters
{
	//Get the rms values of Igrid and Vgrid
	//Update frequency
	//Run control loop for volt and var
	//if anti-islanding time, switch strategies and same for bat discharge
	
 		if(rmsFlag==1){// Polish RMS value 
					//Divide by N
    Sbus.V.rms /= ARRAY_SIZE;
		Sbus.I.sum /= ARRAY_SIZE;
	//	Sbus.P.sum =Sbus.P.sum *	1.29882;
	//	Sbus.P.rms = Sbus.P.sum/ARRAY_SIZE;	
		Sbus.V.mean =Sbus.V.avg_sum /ARRAY_SIZE;
			Sbus.V.avg_sum=0;
	//	Sctrl.I.sum /= ARRAY_SIZE;
		//Sctrl.P.sum /= ARRAY_SIZE;	
			//Sqrt the thing
		Sbus.V.rms  = sqrtInt(Sbus.V.rms );
		Sbus.I.sum = sqrtInt(Sbus.I.sum);

//		Sctrl.I.sum = sqrt(Sctrl.I.sum);
					
			//Undo the scaling from adc
		Sbus.V.rms = ((Sbus.V.rms ) * 3300) / 4095;
		Sbus.I.rms = ((Sbus.I.sum) * (6600)) / 4095; //(2* (Vrefp - Vrefn)) / 4095
	//	Sbus.P.rms = ((Sbus.P.sum)); //(2* (Vrefp - Vrefn)) / 4095
			
	//	Sctrl.I.rms = ((Sctrl.I.sum) * (6600)) / 4095; //(2* (Vrefp - Vrefn)) / 4095


			
		//Undo the scaling from the signal conditioning board
		
		int scale_index= (Sbus.V.rms/100);
		
		Sbus.V.rms=Sbus.V.rms*scaling[scale_index]; //xfmr ratio is actually 4.48 and the SCB is double the input AC waveform
		Sbus.V.rms-=100;
//		UARTprintf("Vbus: %d \n",(int)Sbus.V.rms); 
		Sbus.I.rms=Sbus.I.rms*9.122;//8.36;	
		Sbus.P.rms=Sbus.P.sum;
		Sbus.P.rms=((double)(Sbus.P.rms))*scaling[scale_index]*9.122;
		Pmeas = Sbus.P.rms/1000000;
		
		Vrms=Sbus.V.rms/1000;
		Irms=Sbus.I.rms/1000;
		Sbus.S = Vrms*Irms; //can't keep it in ints cause it overflows
		Qmeas=sqrt((Sbus.S *Sbus.S)-(Sbus.P.rms*Sbus.P.rms));
	//	UARTprintf("Current Measurement: %d \n",(int)Sbus.I.rms); 
		UARTprintf("Q Meas: %d \n",(int)Qmeas); 


//	UARTprintf("Power Measurement: %d \n",Sbus.P.rms); 



	rmsFlag=0;
	Irms= 12;
		//if((Vrms<(Vref*0.95) || Vrms>(Vref*1.05))){ //deadband, +/- 0.5 pu of 10 Vrms
		//	ctrlFlag=1;
		//}
		
		count++;
		if(count>=130&&Vrms!=5){
			ctrlFlag=1;
		}
							status ^=GPIO_PIN_3;
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, status);
	}



	
	
		
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
	
	//DC Voltage Sequencer 
		ADCSequenceDisable(ADC0_BASE, ADC_SEQUENCE0); 
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE0, ADC_TRIGGER_TIMER, 0);
		ADCSoftwareOversampleConfigure(ADC0_BASE, ADC_SEQUENCE0, 4);
		ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 0, ADC_CTL_CH3 |  ADC_CTL_END | ADC_CTL_IE ); //PE0, Vdc, single-ended
		ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE0); //adc_base, sequence 

 
		//	ADCHardwareOversampleConfigure(ADC0_BASE, 8);

		//Interrupt Setup 
		ADCIntEnable(ADC0_BASE, ADC_SEQUENCE2);
    ADCIntRegister(ADC0_BASE, ADC_SEQUENCE2, &ADC0Seq2_Handler);
    IntPrioritySet(INT_ADC0SS2, ADC_SEQUENCE2_PRIORITY);
		IntDisable(INT_ADC0SS2);
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE2);
		
		//Interrupt Setup
		ADCIntEnable(ADC0_BASE, ADC_SEQUENCE0);
    ADCIntRegister(ADC0_BASE, ADC_SEQUENCE0, &ADC0Seq0_Handler);
    IntPrioritySet(INT_ADC0SS2, ADC_SEQUENCE0);
		IntDisable(INT_ADC0SS2);
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE0);
		
		//Two options: 1) use hardware averaging meaning you set up ADC1 base, set up alternate interrupt, but interrupts at a lesser frequency
		//2) Add Sequence0 for a deeper fifo and just use software averaging on sequence 0
		
		//Set up the ADC1 for DC voltages to enable hardware sampling? 
}

//float correctedInput=0;

void ADC0Seq2_Handler(void)
{
  ADCIntClear(ADC0_BASE, ADC_SEQUENCE2); // Clear the timer interrupt flag.
	//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
//	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawInput[adc_input_index].PE0); //dc bus
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE1);  //PE1, Vbus, single-ended
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE3);  //PE4 & PE3, Ibus
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE4); //PE4, Vgrid, single-ended, 


	//Grabbing AC Values
	//Sbus.V_avg_sum += adcRawInput[adc_input_index].PE0;
	
	Sbus.V.inst = adcRawInput[adc_input_index].PE1-Sbus.V.mean; //1650 in hex form
	Sbus.V.avg_sum+=adcRawInput[adc_input_index].PE1;				
	Sbus.V.sum += Sbus.V.inst * Sbus.V.inst;
	//	Sbus.V.sum -=Sbus.V.sum / ARRAY_SIZE;
//	Sbus.V.sum+=Sbus.V.inst/ARRAY_SIZE;

	
	Sbus.I.inst= adcRawInput[adc_input_index].PE3-2048; //2028;
	Sbus.I.sum += Sbus.I.inst * Sbus.I.inst;
//	Sbus.I.sum -=Sbus.I.sum / ARRAY_SIZE;
//	Sbus.I.sum+=(Sbus.I.inst * Sbus.I.inst)/ARRAY_SIZE;

	Sbus.P.inst =(((Sbus.I.inst* 6600) / 4095) * ((Sbus.V.inst* 3300) / 4095));
	//Sbus.P.inst =(Sbus.I.inst) * (Sbus.V.inst);
	Sbus.P.sum -=Sbus.P.sum / ARRAY_SIZE;
	Sbus.P.sum+=Sbus.P.inst/ARRAY_SIZE;
//	Sbus.P.sum =Sbus.P.sum +Sbus.P.inst;

	//Vgrid
	float correctedInput=(adcRawInput[adc_input_index].PE1*3300)/4095;
	correctedInput=correctedInput/3300;
	VSync.AC_input =(((float) correctedInput)-0.5)*2;
	//correctedInput=(correctedInput* 3300);
	//correctedInput=adcRawInput[adc_input_index].PE0-scbLevelShift; //take out that level shift

	 
	//inputValue= (adcRawInput[adc_input_index].PE0* 3300)/4095;
		adc_input_index = (adc_input_index + 1) % (ARRAY_SIZE);
		if(adc_input_index==(ARRAY_SIZE - 1)){
			rmsFlag=1;
			Sbus.V.rms=Sbus.V.sum;
			Sbus.V.sum=0;
		}
		
		/*double approxRollingAverage (double avg, double new_sample) {

    avg -= avg / N;
    avg += new_sample / N;

    return avg;
}*/
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

