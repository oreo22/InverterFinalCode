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
#include "gpio_task.h" //for gpio to be interfaced to adc
#include "gpio.h" //for gpio to be interfaced to adc


#define TIMER2A_PRIORITY 1
#define CTRL_FREQ 3000
#define dt_2 0.000083333
#define dt_2 0.000083333

extern ACPower_t Sgrid;
extern double degreeDesired;

extern uint16_t adc_input_index;
extern int inputValue;

extern uint8_t ctrlFlag;
extern double sFactor; //ma
//RMS values
extern int dcMeas;
double Vdc=0;
double Qerr_run=0;
double Qerror_inst=0;
double Perr_run=0;
double Perr_inst=0;
float QKp=4; //Kp=0.55 * m=6
float QKi=1;
float PKp=0.8;
float PKi=0.01; 
double Vref=5;
double record[100];
int r_idx=0;

double ma=0.5;
extern double ma_avg;
char uartInput[5];

double Pdesired=0;
int Vmax=0;
double ctrlV=0;

void ctrlInit(void){

}

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
double min(double A, double B){
	if(A<=B){
		return A;
	}
	return B;
}
double max(double A, double B){
	if(A>=B){
		return A;
	}
	return B;
}

void VarControl(ACPower_t *Sinv, ACPower_t *Sctrl, acValues *Vpcc, acValues *Pgrid){ //double Vrms, double Irms, double Pmeas, double Qmeas	
	double Srated=113.8614;
	double Qmax=0;

	//-------------------PI Control P Loop-------------
	
	
	
	if((Vpcc->rms<(Vref*0.99) || Vpcc->rms>(Vref*1.01))){
		{
		Qerror_inst=(Vref- Vpcc->rms); //deltaQ;
			//-------------------PI Control Q Loop-------------
		/*	if(Pgrid->rms<0){
				Pgrid->rms=Pgrid->rms*-1;
			}
			Qmax=(Srated*Srated) - (Pgrid->rms*Pgrid->rms);
			Qmax=sqrtDb(Qmax);

			if(Qmax < (Sinv->Q + Qerror_inst)){ //if the Verror can't be supported due to limited Q, just supply the max Q difference
				Qerror_inst=Qmax-Sinv->Q;
			}*/
			
			Qerr_run=Qerr_run + (QKi*Qerror_inst); //dt_2*
			Sctrl->V.rms= Qerror_inst*QKp+ Qerr_run + Vpcc->rms;
		
		
			dcMeas=dcMeas/1000;
			ma=((double)Sctrl->V.rms)/ ((double) dcMeas);
			ma=min(1,ma);
			ma=max(0,ma);
			
			ma_avg -=ma_avg / 10;
			ma_avg+=(ma)/10;

			UARTprintf("ma %d \n",(int) (ma_avg*1000));
				degreeDesired=20;
			ctrlFlag=0;
		}
	}

	ctrlFlag=0;

		

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
}
		
void Timer2AIntHandler(void){
  TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);   
}