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
uint8_t status=0;
uint8_t batCharge=0;
acValues Vpcc;
acValues Pgrid;

ACPower_t Sctrl;
ACPower_t Sinv;
int dcMeas=0;
uint32_t avg_dcMeas=0;
double ma_avg=0.4;
int avg_Vbus=0;

double scaling[12]={14.583,15.04540, 15.912, 15.14179609 , 13.126, 11.13196 ,10.1894, 10.2422, 10.5, 11.47, 12.6225, 13.3132};
//double scaling[11]={14.583,15.04540, 14.860, 14.589 , 13.9, 11.06,10.6798, 10.331, 10.466, 10.923, 11.879}; 

AdcData_t adcRawInput[ARRAY_SIZE ];
uint16_t adc_input_index = 0;
extern SPLL_1ph_SOGI VSync;
extern int inputValue;

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
		
		Sinv.V.mean =Sinv.V.avg_sum /ARRAY_SIZE;
		Sinv.V.avg_sum=0;
		 
		Sinv.I.mean =Sinv.I.avg_sum /ARRAY_SIZE;
		Sinv.I.avg_sum=0;

		Vpcc.rms=Vpcc.sum ;
		Vpcc.sum=0;	
		Vpcc.rms/=ARRAY_SIZE;
		Vpcc.mean =Vpcc.avg_sum /ARRAY_SIZE;
		Vpcc.avg_sum=0;
		Vpcc.rms = sqrtInt(Vpcc.rms);
		Vpcc.rms = ((Vpcc.rms ) * 3300) / 4095;	
	 
	 							//Calculating the DC
		//dcMeas=(avg_dcMeas);
		//dcMeas=(dcMeas* 3300) / 4095;
		//dcMeas=(dcMeas*7.9385 ); //multiply by 8.5247 for the voltage divider and then divide by 1.414 
	 dcMeas=24900;

			
		int scale_index=((Vpcc.rms)/100);
		if(scale_index>11){
			scale_index=11;
		}

		avg_Vbus-=avg_Vbus / 6;
		avg_Vbus+=Vpcc.rms/6;

		Vpcc.rms=Vpcc.rms*scaling[scale_index]; 
		UARTprintf("Vpcc rms: %d \n",(int)Vpcc.rms); 
		
		if(count>=10 && (Vpcc.rms*1.414)>=dcMeas){
			ma_avg=0; //turn off control signals and let the battery charge 
			//IntDisable(INT_PWM0_0);
			batCharge=1;
		}
		if(count>=250 && batCharge==0){ //&&(Sinv.V.rms<(4.9) || Sinv.V.rms>(5.1))
			ctrlFlag=1;
		}
		if(batCharge==1 && (Vpcc.rms*1.414)<=dcMeas){
			batCharge=0;
		}
		if(ctrlFlag==0 && batCharge==0){
			ma_avg=(Vpcc.rms)/dcMeas;
			
		}
			Sinv.V.rms=Sinv.V.sum ;
			Sinv.V.sum=0;
		 
			Sinv.I.rms=Sinv.I.sum ;
			Sinv.I.sum=0;

			//Divide by N
			Sinv.V.rms /= ARRAY_SIZE;
			Sinv.I.rms /= ARRAY_SIZE;
			
			//Sqrt the thing
			Sinv.V.rms  = sqrtInt(Sinv.V.rms );
			Sinv.I.rms = sqrtInt(Sinv.I.rms);
						
			//Undo the scaling from adc
			Sinv.V.rms = ((Sinv.V.rms ) * 3300) / 4095;
			Sinv.I.rms = ((Sinv.I.rms) * (6600)) / 4095; //(2* (Vrefp - Vrefn)) / 4095
			
			//Undo the scaling from SCB and XFMRs
			Sinv.V.rms=Sinv.V.rms*14.642; //xfmr ratio is actually 4.48 and the SCB is double the input AC waveform
			Sinv.I.rms=Sinv.I.rms*9.122;//8.36;	
			
			Sinv.P.rms=Sinv.P.sum;
			Sinv.P.rms=((double)(Sinv.P.rms))*14.642*9.122;
		//	UARTprintf("Vdc %d \n",(int) (dcMeas));
			Pgrid.rms=Pgrid.sum;
			Pgrid.rms=((double)(Pgrid.rms))*scaling[scale_index]*9.122;
			
	//		UARTprintf("Vinv: %d \n",(int)Sinv.V.rms); 

		//	UARTprintf("Pinv: %d \n",(int)Sinv.P.rms);
			//UARTprintf("Iinv: %d \n",(int)Sinv.I.rms);
			Sinv.P.rms/=1000000;
			Sinv.V.rms/=1000;
			Sinv.I.rms/=1000;
			Sinv.S = Sinv.V.rms*Sinv.I.rms; //can't keep it in ints cause it overflows
			Sinv.Q=sqrt((Sinv.S *Sinv.S)-(Sinv.P.rms*Sinv.P.rms));
			Vpcc.rms=((double)Vpcc.rms)/1000;
		rmsFlag=0;
		count++;
if(ctrlFlag==1){
		VarControl(&Sinv, &Sctrl,&Vpcc, &Pgrid);
}
		status ^=GPIO_PIN_3;
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, status);
	}
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
		ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 2, ADC_CTL_CH4 |  ADC_CTL_END | ADC_CTL_IE  ); // PD4
		ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE2); //adc_base, sequence
	
	//DC Voltage Sequencer 
		ADCSequenceDisable(ADC0_BASE, ADC_SEQUENCE0); 
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE0, ADC_TRIGGER_TIMER, 0);
		//	ADCSoftwareOversampleConfigure(ADC0_BASE, ADC_SEQUENCE0, 4);
		//	ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 0, ADC_CTL_CH2); //PE1, Vbus, single-ended
		//	ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 1, ADC_CTL_D |ADC_CTL_CH5 | ADC_CTL_END | ADC_CTL_IE ); //PB4 and PB5, Vdc, single-ended
		ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 0, ADC_CTL_CH11 | ADC_CTL_END | ADC_CTL_IE ); //PB4 and PB5, Vdc, single-ended
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
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PD3); //PD3, Vgrid, single-ended, 
	
	//Grabbing AC Values for the Inverter 
	Sinv.V.inst = adcRawInput[adc_input_index].PD3-Sinv.V.mean; //1650 in hex form
	Sinv.V.avg_sum+=adcRawInput[adc_input_index].PD3;	
	Sinv.V.sum += (Sinv.V.inst * Sinv.V.inst);///ARRAY_SIZE;;
	
	Sinv.I.inst= adcRawInput[adc_input_index].PE3-Sinv.I.mean; //2028;
	Sinv.I.avg_sum+=adcRawInput[adc_input_index].PE3;	
	Sinv.I.sum += Sinv.I.inst * Sinv.I.inst;

	Sinv.P.inst =(((Sinv.I.inst* 6600) / 4095) * ((Sinv.V.inst* 3300) / 4095));
	Sinv.P.sum -=Sinv.P.sum / ARRAY_SIZE;
	Sinv.P.sum+=Sinv.P.inst/ARRAY_SIZE;

	//Grabbing AC Values for the Grid 	
	Vpcc.inst = adcRawInput[adc_input_index].PE1-Vpcc.mean; //1650 in hex form
	Vpcc.avg_sum+=adcRawInput[adc_input_index].PE1;	
	Vpcc.sum += (Vpcc.inst * Vpcc.inst);///ARRAY_SIZE;;

	/*Pgrid.inst =(((1* 6600) / 4095) * ((Vpcc.inst* 3300) / 4095));
	Pgrid.sum -=Pgrid.sum / ARRAY_SIZE;
	Pgrid.sum+=Pgrid.inst/ARRAY_SIZE;*/
	
	float correctedInput=(adcRawInput[adc_input_index].PE1*3300)/4095;
	correctedInput=correctedInput/3300;
	VSync.AC_input =(((float) correctedInput)-0.5)*2;
			
	//inputValue= (adcRawInput[adc_input_index].PE0* 3300)/4095;
		adc_input_index = (adc_input_index + 1) % (ARRAY_SIZE);
		if(adc_input_index==(ARRAY_SIZE - 1)){
			rmsFlag=1;
		}
		//Calling the PLL	
	//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);

		PLLRun(&VSync);


}


void ADC0Seq0_Handler(void)
{
  ADCIntClear(ADC0_BASE, ADC_SEQUENCE0); // Clear the timer interrupt flag.
		
	ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &avg_dcMeas); //dc bus
//	avg_dcMeas -=avg_dcMeas / 10;
//	avg_dcMeas+=(dcMeas)/10;


}

