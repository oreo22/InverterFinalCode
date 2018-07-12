#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "adc_task.h"
#include "priorities.h"
#include <math.h>

#include "adc.h" //for adc lib
#include "hw_memmap.h" //for address bases
#include "sysctl.h" //for init ports
#include "timer.h" //for timer
#include "interrupt.h" //for interrupt
#include "Control.h" //for adc lib
#include "hw_ints.h" //for INT_TIMER2A


#define TIMER2A_PRIORITY 1
#define CTRL_FREQ 3000

extern ACPower_t Sgrid;
extern double degreeDesired;

extern uint16_t adc_input_index;
extern int inputValue;

extern uint8_t ctrlFlag;
extern double sFactor; //ma
//RMS values
extern int dcMeas;
double Vdc=0;
double QError[2]={0,0};
double Qerror_running;
double Qerror_inst;
double PError[2];
double Vref=5;
float QKp=1.2;
float QKi=0.3;
float PKp=0.8;
float PKi=0.01; 
extern double ma;

ACPower_t Sctrl;
int Vmin=0;
int Vmax=0;
double ctrlV=0;
extern double Vrms;
extern double Irms;
extern	double Pmeas;
extern double Qmeas;

uint32_t clockTime=0;
void configureTimer2A(void){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); // Enable Timer 2 Clock
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic
  TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/CTRL_FREQ); // is set to 80MHz according to main file, Reload Value = fclk/fswitch
	//Configuring the interrupts	
	TimerIntRegister(TIMER2_BASE, TIMER_A, &Timer2AIntHandler);
	IntPrioritySet(INT_TIMER2A, 4);
	TimerEnable(TIMER2_BASE, TIMER_A);	// Start Timer 1A
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	IntDisable(INT_TIMER2A);
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}

void VarControl(void){ //double Vrms, double Irms, double Pmeas, double Qmeas
	//	UARTprintf("Corrected ma %d \n", (int)(ma*1000));
//		rmsError[1]=rmsError[0];
	// clear the timer interrupt
	//	UARTprintf("Corrected ma %d \n", (int)(ma*1000));
//		rmsError[1]=rmsError[0];
	
		//double deltaVoltage=Vref-Vrms; //do a lookup table for Q, like 1V difference=
		Sctrl.Q=((Vref-Vrms)*Vref); //0.014

	
		//PI Control P Loop 
		Sctrl.S=Vrms*Irms; //41,842.42 VA in rl so 41484424328
		Sctrl.P.rms=(Sctrl.S*Sctrl.S)-(Sctrl.Q*Sctrl.Q);
		Sctrl.P.rms=sqrt(Sctrl.P.rms);
		//use a while loop to calculate until S is no longer negative, that's when the ma is correct, but if it's too much ma, you'll have to determine how to fix that++++++++++++++++++++
		//UARTprintf("Required Q: %d \n",Sctrl.Q);
//	UARTprintf("Sctrl.S: %d \n",(int)Sctrl.S);
		degreeDesired=(Sctrl.P.rms*0.014)/(Sgrid.V.rms*Vref);//should I multiply Vref back to 5000?
		PError[0]=(Pmeas-Sctrl.P.rms);
		//degreeDesired=degreeDesired*1000;
		degreeDesired=degreeDesired+PKi*PError[1]+PKp*PError[0];
		PError[1]=PError[0]+PError[1];
		//degreeDesired=powerFlag*degreeDesired;
		if(degreeDesired>30){
			degreeDesired=30;
		}

		
				//PI Control loop  for Q 
		
				//-------Update the DC Values-----------
		Vdc=(dcMeas * 3600) / 4095;
		Vdc=Vdc*8.35; //multiply by 8.5247 for the voltage divider and then divide by 1.414 
		Vdc=Vdc/1000;
		//Volt Control 
		//rmsError[0]=Vref - Vrms;
		//double ctrlV=(rmsError[1]*Ki+rmsError[0]*Kp)+Vref;
		ctrlV=(Vref*1.414)+8; //inverter loses about 2 Vrms, so that's around 6 Vdc + 2 Vdc for safety 
	//	UARTprintf("Vdc: %d \n",(int)(Vdc)); 
	Qerror_inst=(Vref-Vrms);  //just adjust V accordingly until ya get it to the reference. Power control is seperate
	ctrlV=ctrlV+QKi*Qerror_running+QKp*Qerror_inst;
		UARTprintf("ctrlV: %d \n",(int)(ctrlV));
		Qerror_running=Qerror_inst+Qerror_running;
		//Changing the ma accordingly 
		ma=((double)ctrlV)/ ((double) Vdc);
UARTprintf("ma: %d \n",(int)(ma*1000)); 
		if(ma>1){
			ma=1;
		}
		ctrlFlag=0;
	}

void Timer2AIntHandler(void){
  TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);   
}