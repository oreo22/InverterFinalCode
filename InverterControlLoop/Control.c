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
double Qerror_running;
double Qerror_inst;
double Perror_running;
double Perror_inst;
double Vref=5;
float QKp=1;
float QKi=0.001;
float PKp=0.8;
float PKi=0.01; 
double ma=0.5;
double ma_avg=0.6;
char uartInput[5];

double Pdesired=0;
int Vmax=0;
double ctrlV=0;


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


double sqrtDb(double n){
  double lo = 0, hi = n, mid;
  for(int i = 0 ; i < 1000 ; i++){
      mid = (lo+hi)/2;
      if(mid*mid == n) return mid;
      if(mid*mid > n){
          hi = mid;
      }else{
          lo = mid;
      }
  }
  return mid;
}
	
	void VarControl(ACPower_t *Sinv, ACPower_t *Sctrl, acValues *Vpcc){ //double Vrms, double Irms, double Pmeas, double Qmeas
		
		//Sctrl.Q=((Vref-Vrms)*Vref);///0.014;///0.0044825;
		
		//Calculating the reference for Q, P, and S
		Sctrl->Q=(Vref-Vpcc->rms)*4; //b/c 12 VAR at 5.0, 13.5 VA at 5.4 V, using the slope of the Volt/Var curve 		
		Sctrl->S=Vpcc->rms*4.1; //41,842.42 VA in rl so 41484424328
		
	//	Sctrl->P.rms=(Sctrl->S*Sctrl->S)-(Sctrl->Q*Sctrl->Q);
	//	Sctrl->P.rms=sqrtDb(Sctrl->P.rms);
		
		//PI Control Q Loop 
		
		Qerror_inst=Sctrl->Q - Sinv->Q;
//		UARTprintf("Qinv %d \n",(int) (Sinv->Q));
	//	Sctrl->Q=Sctrl->Q+QKp*Qerror_inst;//+QKi*Qerror_running;
		Sctrl->V.rms=Vpcc->rms*(1+Qerror_inst*QKp+QKi*Qerror_running); //Vinv=Vrms+ nQ;
		Sctrl->V.rms=(Vpcc->rms*2); //Calculate the base Vinv before +/- necessary Q
		
		Qerror_running=Qerror_inst+Qerror_running;
		
		
		//Changing the ma accordingly
		//Vdc=(dcMeas * 3300) / 4095;
		//Vdc=(Vdc*7.8125)/1000; //multiply by 8.5247 for the voltage divider and then divide by 1.414 
		Vdc=25;
		if(Vpcc->rms<(4.9) || Vpcc->rms>(5.1)){
				ma=((double)Sctrl->V.rms)/ ((double) Vdc);
				if(ma>1){
					ma=1;
				}
				if(ma<0){
					ma=0;
				}
				ma_avg -=ma_avg / 10;
				ma_avg+=(ma)/10;
				UARTprintf("ma %d \n",(int) (ma_avg*1000));
		}
		ctrlFlag=0;
	/*	if(ma==0){
			UARTprintf("Enter degrees: \n");
			UARTgets(uartInput, 5);
			degreeDesired=(uartInput[0]-48)+13.5;
		}*/
	}		

		
	
		//PI Control P Loop 

//		Pdesired=
//		//use a while loop to calculate until S is no longer negative, that's when the ma is correct, but if it's too much ma, you'll have to determine how to fix that
//		degreeDesired=(Pdesired*0.014)/(Sinv->V.rms*Vref);//should I multiply Vref back to 5000?
//		
//		Perror_inst=Pdesired-Sinv->P.rms;
//		//UARTprintf("Q Desired: %d \n",(int)Qmeas);
//		//UARTprintf("Q needed: %d \n",(int)Qerror_inst);
//	degreeDesired=13.5;//+degreeDesired+PKp*Perror_inst;//+PKi*Perror_running;
//		Perror_running=Perror_inst+Perror_running;
//		if(degreeDesired>30){
//			degreeDesired=30;
//		}
	
		
void Timer2AIntHandler(void){
  TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);   
}