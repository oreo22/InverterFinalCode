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
#define dt 0.000083333
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
int Vref=5000;
double record[100];
int r_idx=0;

double ma=0.5;
extern double ma_avg;
char uartInput[5];

int Pref=0;
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

void WattControl(acValues *Pgrid,ACPower_t *Sinv){
			//PI Control P Loop 
		int Pmax=dcMeas*10000;
		Pref=min(Pgrid->rms,Pmax);
		Perr_inst=Pref-Sinv->P.rms;
		Perr_inst=Perr_inst/1000000;
		Perr_run=Perr_run + (PKi*Perr_inst); //dt_2*
	//how do i account for the non-linearity of degreeDesired=13.5 as 0?
		degreeDesired=degreeDesired+ Perr_inst*PKp + Perr_run;
		if(degreeDesired>30){
			degreeDesired=30;
		}
}

void VarControl(ACPower_t *Sinv, ACPower_t *Sctrl, acValues *Vpcc){ //double Vrms, double Irms, double Pmeas, double Qmeas	
	int Srated=1250000000;
	int Qmax=0;

	if((Vpcc->rms<(Vref*0.99) || Vpcc->rms>(Vref*1.01))){
		{
		Qerror_inst=(Vref- Vpcc->rms); //deltaQ;
			//-------------------PI Control Q Loop-------------
			long long int  S_sqed=Srated *Srated;
			if(Sinv->P.rms<0){
				Sinv->P.rms=Sinv->P.rms*-1;
			}
			long long int P_sqed=Sinv->P.rms*Sinv->P.rms;
			if(S_sqed >=P_sqed){
				Qmax=(S_sqed) - (P_sqed);
				Qmax=sqrtInt(Qmax);
			}


			if(Qmax < (Sinv->Q + Qerror_inst)){ //if the Verror can't be supported due to limited Q, just supply the max Q difference
				Qerror_inst=Qmax-Sinv->Q;
			}
			
			Qerr_run=Qerr_run + (QKi*Qerror_inst*dt_2); //dt_2*
			Sctrl->V.rms= Qerror_inst*QKp + Qerr_run + Vpcc->rms;
	
			ma=((double)Sctrl->V.rms)/ ((double) dcMeas);
			ma=min(1,ma);
			ma=max(0,ma);
			
			ma_avg -=ma_avg / 10;
			ma_avg+=(ma)/10;

			UARTprintf("ma %d \n",(int) (ma_avg*1000));
			ctrlFlag=0;
		}
	}

	ctrlFlag=0;
}
		
void Timer2AIntHandler(void){
  TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);   
}