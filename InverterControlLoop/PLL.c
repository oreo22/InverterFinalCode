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
#include "PLL.h"
//#include "IQmathCPP.h"

#include <math.h>
#include "adc_task.h"
#include "timer.h" //for timer
#include "gpio.h"
//#include "heap_1.h"

//*****************************************************************************
//
// Initialize the constants of the filter
//
//*****************************************************************************
#define B0_LPF (100)
#define B1_LPF (-100.005)
#define A1_LPF (-1.0)
#define k 0.5 //k=0.5 for the SOGIntegrator


//*****************************************************************************
//
// The queue that holds messages sent to the LED task.
//
//*****************************************************************************
//xQueueHandle g_pPLLQueue;

extern int inputValue;
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
void PLLTaskInit(uint16_t Grid_freq, double DELTA_T,volatile LPF_COEFF lpf_coeff, SPLL_1ph_SOGI *spll_obj)
{
	//Memset(&spll_obj, (0.0), sizeof(spll_obj));
spll_obj->u[0]=(0.0);
spll_obj->u[1]=(0.0);
spll_obj->u[2]=(0.0);
spll_obj->osg_u[0]=(0.0);
spll_obj->osg_u[1]=(0.0);
spll_obj->osg_u[2]=(0.0);
spll_obj->osg_qu[0]=(0.0);
spll_obj->osg_qu[1]=(0.0);
spll_obj->osg_qu[2]=(0.0);
spll_obj->u_Q[0]=(0.0);
spll_obj->u_Q[1]=(0.0);
spll_obj->u_D[0]=(0.0);
spll_obj->u_D[1]=(0.0);
spll_obj->ylf[0]=(0.0);
spll_obj->ylf[1]=(0.0);
spll_obj->fo=(0.0);
spll_obj->fn=(Grid_freq);
spll_obj->theta[0]=(0.0);
spll_obj->theta[1]=(0.0);
spll_obj->sin=(0.0);
spll_obj->cos=(0.0);
	/*//coefficients for the loop filter
	spll_obj->lpf_coeff.B1_lf=B1_LPF;//the loop filter params don't change???
	spll_obj->lpf_coeff.B0_lf=B0_LPF;
	spll_obj->lpf_coeff.A1_lf=A1_LPF;
	spll_obj->delta_T=	DELTA_T;*/
	PLLCoeffUpdate((double)1/(double)SAMPLING_FREQ,(float)(2*3.14159*(float)60),&PLLSync);
	configureTimer2A();	
}




void configureTimer2A(void){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); // Enable Timer 1 Clock
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic
  TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()); //SysCtlClockGet()/SWITCHING_FREQ is set to 80MHz according to main file, Reload Value = fclk/fswitch

//Configuring the interrupts	
	TimerIntRegister(TIMER2_BASE, TIMER_A, &Timer2AIntHandler);
	IntPrioritySet(INT_TIMER2A, TIMER2A_PRIORITY);
	TimerEnable(TIMER2_BASE, TIMER_A);	// Start Timer 0B
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	//IntEnable(INT_TIMER2A); //Have to have it enabled in order
}

	double waveform=0;
static inline void PLLRun(SPLL_1ph_SOGI *spll_obj) {
// Update the spll_obj->u[0] with the grid value before calling this routine
	
	//status ^=GPIO_PIN_3;
	//*****************************************************************************
	//Orthogonal Signal Generator  
	//Creates a voltage signal that has a phase 90 degrees off of the original and cancels out 2fsw
	//Returns: Vbeta 
	//*****************************************************************

	spll_obj->osg_u[0]=spll_obj->osg_coeff.osg_b0*(spll_obj->u[0]- spll_obj->u[2])+spll_obj->osg_coeff.osg_a1*spll_obj->osg_u[1]+spll_obj->osg_coeff.osg_a2*spll_obj->osg_u[2];
	spll_obj->osg_u[2]=spll_obj->osg_u[1];  
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
	spll_obj->ylf[1]=spll_obj->ylf[0];
	spll_obj->u_Q[1]=spll_obj->u_Q[0];
	spll_obj->u_D[1]=spll_obj->u_D[0];
	//---------------------------------//
	// VCO 
	// Internal voltage oscillator that changes based on the error from PLL
	//---------------------------------//

spll_obj->fo=spll_obj->fn+spll_obj->ylf[0]; //update output frequency: w0+ylf[0]
	spll_obj->theta[0]=spll_obj->theta[1]+((spll_obj->fo*spll_obj->delta_T)*((float)2.0*(float)3.1415926));
	if(spll_obj->theta[0]>(2*3.1415926)){
		spll_obj->theta[0]=spll_obj->theta[0]-(2*3.1415926); //convert it back to degrees b/c it was in radians???
	}
	spll_obj->theta[1]=spll_obj->theta[0];

	spll_obj->sin=sinf (spll_obj->theta[0]); //SPLL_SOGI_SINE
	spll_obj->cos=cosf (spll_obj->theta[0]);
		//ADD IN THE ERROR FOR THE THETA FOR POWER CONTROL
	//MULTIPLY THE SIN(THETA) BY 1000 in order to compare with the Triangle Wave 
	inputValue=(int)ceil((3300*spll_obj->sin));
	
			
}

//*********** Structure Coeff Update *****/
 
void PLLCoeffUpdate(double delta_T, float wn, volatile SPLL_1ph_SOGI *spll)
{
	float osgx,osgy,temp;
		spll->lpf_coeff.B1_lf=B1_LPF;
	spll->lpf_coeff.B0_lf=B0_LPF;
	spll->lpf_coeff.A1_lf=A1_LPF;
	spll->delta_T=	(double)1/(double)SAMPLING_FREQ;
	spll->fn=((float)60);
	spll->osg_coeff.osg_k=0.5; 
	osgx=(float)(2.0*0.5*wn*delta_T); //overwritting fn
	spll->osg_coeff.osg_x=osgx;
	osgy=(float)(wn*delta_T*wn*delta_T);
	spll->osg_coeff.osg_y=(osgy);
	temp=(float)1.0/(osgx+osgy+4.0);
	spll->osg_coeff.osg_b0=((float)osgx*temp);
	spll->osg_coeff.osg_b2=((-1.0)*spll->osg_coeff.osg_b0);
	spll->osg_coeff.osg_a1=((float)(2.0*(4.0-osgy))*temp);
	spll->osg_coeff.osg_a2=((float)(osgx-osgy-4)*temp);
	spll->osg_coeff.osg_qb0=((float)(0.5*osgy)*temp);
	spll->osg_coeff.osg_qb1=(spll->osg_coeff.osg_qb0*(2.0));
	spll->osg_coeff.osg_qb2=spll->osg_coeff.osg_qb0;
}

void Timer2AIntHandler(void){
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	PLLCoeffUpdate((double)1/(double)SAMPLING_FREQ,(float)(2*3.14159*(float)60),&PLLSync);
		PLLRun(&PLLSync);
		//IntDisable(INT_TIMER2A);
	
}