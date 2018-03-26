#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "sysctl.h" //for init ports
#include "interrupt.h" //for interrupt
#include "hw_ints.h" //for INT_TIMER2A
#include <IQmathLib.h>
#include "IQmathLib.h"
#include "PLL.h"

#include <math.h>
#include "adc_task.h"
#include "timer.h" //for timer
//#include "heap_1.h"

//*****************************************************************************
//
// Initialize the constants of the filter
//
//*****************************************************************************
#define B0_LPF SPLL_SOGI_Q(166.877556)
#define B1_LPF SPLL_SOGI_Q(-166.322444)
#define A1_LPF SPLL_SOGI_Q(-1.0)
#define k 0.5 //k=0.5 for the SOGIntegrator


//*****************************************************************************
//
// The queue that holds messages sent to the LED task.
//
//*****************************************************************************
//xQueueHandle g_pPLLQueue;


#define PLLSTACKSIZE        128         // Stack size in words
#define TIMER2A_PRIORITY		2

//*****************************************************************************
//
// This task is the calculations of the PLL using the data from ADC. 
//
//*****************************************************************************
void PLLTask(void)
{
		// Print the current loggling LED and frequency.
	
    UARTprintf("PLL Init\n");
			/*PLLTaskInit(GRID_FREQ,_IQ23((float)(1.0/SAMPLING_FREQ)),&spll2,spll_lpf_coef2);
		PLLCoeffUpdate(((float)(1.0/SAMPLING_FREQ)),(float)(2*PI*60),&spll2);*/
	//shouldn't change unless grid freq changes 
	

}
SPLL_1ph_SOGI PLLSync;

//*****************************************************************************
//
// Initializes the PLL task.
//
//*****************************************************************************
uint32_t PLLTaskInit(uint16_t Grid_freq, long DELTA_T,volatile LPF_COEFF lpf_coeff, SPLL_1ph_SOGI *spll_obj)
{
	//Memset(&spll_obj, SPLL_SOGI_Q(0.0), sizeof(spll_obj));
	spll_obj->u[0]=SPLL_SOGI_Q(0.0); //SPLL_SOGI_Q transforms it into fixed point of IQ23, if you wish to extend the decimal allocation, use Q24 or up
	spll_obj->u[1]=SPLL_SOGI_Q(0.0);
	spll_obj->u[2]=SPLL_SOGI_Q(0.0);
	spll_obj->osg_u[0]=SPLL_SOGI_Q(0.0);
	spll_obj->osg_u[1]=SPLL_SOGI_Q(0.0);
	spll_obj->osg_u[2]=SPLL_SOGI_Q(0.0);
	spll_obj->osg_qu[0]=SPLL_SOGI_Q(0.0);
	spll_obj->osg_qu[1]=SPLL_SOGI_Q(0.0);
	spll_obj->osg_qu[2]=SPLL_SOGI_Q(0.0);
	spll_obj->u_Q[0]=SPLL_SOGI_Q(0.0);
	spll_obj->u_Q[1]=SPLL_SOGI_Q(0.0);
	spll_obj->u_D[0]=SPLL_SOGI_Q(0.0);
	spll_obj->u_D[1]=SPLL_SOGI_Q(0.0);
	spll_obj->ylf[0]=SPLL_SOGI_Q(0.0);
	spll_obj->ylf[1]=SPLL_SOGI_Q(0.0);
	spll_obj->fo=SPLL_SOGI_Q(0.0);
	spll_obj->fn=SPLL_SOGI_Q(Grid_freq);
	spll_obj->theta[0]=SPLL_SOGI_Q(0.0);
	spll_obj->theta[1]=SPLL_SOGI_Q(0.0);
	spll_obj->sin=SPLL_SOGI_Q(0.0);
	spll_obj->cos=SPLL_SOGI_Q(0.0);
	//coefficients for the loop filter
	spll_obj->lpf_coeff.B1_lf=B1_LPF;//the loop filter params don't change???
	spll_obj->lpf_coeff.B0_lf=B0_LPF;
	spll_obj->lpf_coeff.A1_lf=A1_LPF;
	spll_obj->delta_T=DELTA_T;
	configureTimer2A();
//	spll_obj->osg_u[0]=spll_obj->osg_coeff.osg_b0*(spll_obj->u[0]- spll_obj->u[2])+spll_obj->osg_coeff.osg_a1*spll_obj->osg_u[1]+spll_obj->osg_coeff.osg_a2*spll_obj->osg_u[2];
	
}




void configureTimer2A(void){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); // Enable Timer 1 Clock
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic
  TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/SAMPLING_FREQ); //SysCtlClockGet()/SWITCHING_FREQ is set to 80MHz according to main file, Reload Value = fclk/fswitch

//Configuring the interrupts	
	TimerIntRegister(TIMER2_BASE, TIMER_A, &Timer2AIntHandler);
	IntPrioritySet(INT_TIMER2A, TIMER2A_PRIORITY);
	TimerEnable(TIMER2_BASE, TIMER_A);	// Start Timer 0B
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	IntEnable(INT_TIMER2A); //Have to have it enabled in order
}


void PLLRun(SPLL_1ph_SOGI *spll_obj) {
// Update the spll_obj->u[0] with the grid value before calling this routine
	
	
	//*****************************************************************************
	//Orthogonal Signal Generator  
	//Creates a voltage signal that has a phase 90 degrees off of the original and cancels out 2fsw
	//Returns: Vbeta 
	//*****************************************************************************
	spll_obj->osg_u[0]=spll_obj->osg_coeff.osg_b0*(spll_obj->u[0]- spll_obj->u[2])+spll_obj->osg_coeff.osg_a1*spll_obj->osg_u[1]+spll_obj->osg_coeff.osg_a2*spll_obj->osg_u[2];
	spll_obj->osg_u[2]=spll_obj->osg_u[1]; //why would you subtract the two times? 
	spll_obj->osg_u[1]=spll_obj->osg_u[0];
	spll_obj->osg_qu[0]=spll_obj->osg_coeff.osg_qb0*spll_obj->u[0]+spll_obj->osg_coeff.osg_qb1*spll_obj->u[1]+spll_obj->osg_coeff.osg_qb2*spll_obj->u[2]+spll_obj->osg_coeff.osg_a1*spll_obj->osg_qu[1]+spll_obj->osg_coeff.osg_a2*spll_obj->osg_qu[2];
	spll_obj->osg_qu[2]=spll_obj->osg_qu[1];
	spll_obj->osg_qu[1]=spll_obj->osg_qu[0];
	spll_obj->u[2]=spll_obj->u[1];
	spll_obj->u[1]=spll_obj->u[0];
	//*****************************************************************************
	//Park Transform 
	//Linearizes the incoming signal and the orthogonal signal generator from alpha beta to D-Q axis
	//Returns: VD and VQ signal
	//*****************************************************************************
		spll_obj->u_Q[0]=(spll_obj->cos*spll_obj->osg_u[0])+(spll_obj->sin*spll_obj->osg_qu[0]);
		spll_obj->u_D[0]=(spll_obj->cos*spll_obj->osg_qu[0])-(spll_obj->sin*spll_obj->osg_u[0]);
	//*****************************************************************************
	//Low Pass Filter
	//PI Controller that filters out harmonics of Vq 
	//Returns: Filtered output signal
	//*****************************************************************************
	spll_obj->ylf[0]=spll_obj->ylf[1]+(spll_obj->lpf_coeff.B0_lf*spll_obj->u_Q[0])+(spll_obj->lpf_coeff.B1_lf*spll_obj->u_Q[1]);
	//spll_obj->ylf[0]=(spll_obj->ylf[0]>SPLL_SOGI_Q(20.0))?SPLL_SOGI_Q(20.0):spll_obj->ylf[0];
	//spll_obj->ylf[0]=(spll_obj->ylf[0] < SPLL_SOGI_Q(-20.0))?SPLL_SOGI_Q(-20.0):spll_obj->ylf[0];
	spll_obj->ylf[1]=spll_obj->ylf[0];
	spll_obj->u_Q[1]=spll_obj->u_Q[0];
	//spll_obj->u_D[1]=spll_obj->u_D[0];
	//---------------------------------//
	// VCO 
	// Internal voltage oscillator that changes based on the error from PLL
	//---------------------------------//
	spll_obj->fo=spll_obj->fn+spll_obj->ylf[0];
	spll_obj->theta[0]=spll_obj->theta[1]+((spll_obj->fo*spll_obj->delta_T)*SPLL_SOGI_Q(2*3.1415926));
	if(spll_obj->theta[0]>SPLL_SOGI_Q(2*3.1415926)){
		spll_obj->theta[0]=spll_obj->theta[0]-SPLL_SOGI_Q(2*3.1415926);
	}
	spll_obj->theta[1]=spll_obj->theta[0];
	
	spll_obj->sin=sin(spll_obj->theta[0]); //SPLL_SOGI_SINE
	spll_obj->cos=cos(spll_obj->theta[0]);
}

//*********** Structure Coeff Update *****/
void PLLCoeffUpdate(float delta_T, float wn, volatile SPLL_1ph_SOGI *spll)
{
	float osgx,osgy,temp; //why? 
	spll->osg_coeff.osg_k=SPLL_SOGI_Q(k); 
	osgx=(float)(2.0*0.5*wn*delta_T);
	spll->osg_coeff.osg_x=SPLL_SOGI_Q(osgx);
	osgy=(float)(wn*delta_T*wn*delta_T);
	spll->osg_coeff.osg_y=SPLL_SOGI_Q(osgy);
	temp=(float)1.0/(osgx+osgy+4.0);
	spll->osg_coeff.osg_b0=SPLL_SOGI_Q((float)osgx*temp);
	spll->osg_coeff.osg_b2=(SPLL_SOGI_Q(-1.0)*spll->osg_coeff.osg_b0);
	spll->osg_coeff.osg_a1=SPLL_SOGI_Q((float)(2.0*(4.0-osgy))*temp);
	spll->osg_coeff.osg_a2=SPLL_SOGI_Q((float)(osgx-osgy-4)*temp);
	spll->osg_coeff.osg_qb0=SPLL_SOGI_Q((float)(0.5*osgy)*temp);
	spll->osg_coeff.osg_qb1=(spll->osg_coeff.osg_qb0*SPLL_SOGI_Q(2.0));
	spll->osg_coeff.osg_qb2=spll->osg_coeff.osg_qb0;
}

void Timer2AIntHandler(void){
		TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
		PLLRun(&PLLSync);
	
	
}