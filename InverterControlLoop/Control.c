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
float QKp=0.55;
float QKi=1;
float PKp=0.8;
float PKi=0.01; 
double Vref=5;
double record[100];
int r_idx=0;

double ma=0.5;
double ma_avg=0.4;
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

void VarControl(ACPower_t *Sinv, ACPower_t *Sctrl, acValues *Vpcc){ //double Vrms, double Irms, double Pmeas, double Qmeas
float m=5.3;	
	/*	//Sctrl.Q=((Vref-Vrms)*Vref);///0.014;///0.0044825;
		
		//Calculating the reference for Q, P, and S
		
		Qerror_inst=(Vref-Vpcc->rms);
		Vpcc->rms; //b/c 12 VAR at 5.0, 13.5 VA at 5.4 V, using the slope of the Volt/Var curve
				
		
		Ki2 = Ki*dt 
x_integral = x_integral + Ki2*e;
x = Kp*e + x_integral
		
		Sctrl->Q=Qerror_inst*QKp + QKi*Qerror_running;
		Qerror_running=Qerror_inst+Qerror_running; */
		
			//-------------------PI Control Q Loop-------------
	
	Qerror_inst=m*(Vref- Vpcc->rms); //deltaQ;
	Qerr_run=Qerr_run + (QKi*Qerror_inst); //dt_2*
	Sctrl->V.rms= Qerror_inst*QKp+ Vpcc->rms + Qerr_run + Vpcc->rms;
//	UARTprintf("Qinv %d \n",(int) (Sinv->Q*1000));
//	Sctrl->V.rms=Vpcc->rms*(1+Qerror_inst*QKp+
//	QKi**dt_2); //Vinv=Vrms+ nQ;
	//Changing the ma accordingly
	Vdc=25; //(dcMeas* 3300) / 4095;
	//Vdc=(Vdc*7.564 )/1000; //multiply by 8.5247 for the voltage divider and then divide by 1.414 

UARTprintf("Vdc %d \n",(int) Vdc);
	ma=((double)Sctrl->V.rms)/ ((double) Vdc);
	ma=min(1,ma);
	ma=max(0,ma);
	
	ma_avg -=ma_avg / 100;
	ma_avg+=(ma)/100;

	UARTprintf("ma %d \n",(int) (ma_avg*1000));
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