#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "sysctl.h" //for init ports
#include "interrupt.h" //for interrupt
#include "hw_ints.h" //for INT_TIMER0A
#include "PLL.h"
//#include "IQmathCPP.h"

#include "adc_task.h"
#include "timer.h" //for timer
#include "gpio.h"
#include <math.h>
//#include "heap_1.h"
//*****************************************************************************
//
// Initialize the constants of the filter
//
//*****************************************************************************
#define B0_LPF (166.8776)
#define B1_LPF (-166.3224)
//#define B0_LPF (100
//#define B1_LPF (-100.5)

#define A1_LPF (-1.0)
#define B0_Notch (0.9896)
#define B1_Notch (-1.9638)
#define A1_Notch (-1.9638)
#define A2_Notch (0.9793)

#define k 0.5 //k=0.5 for the SOGIntegrator


//*****************************************************************************
//
// The queue that holds messages sent to the LED task.
//
//*****************************************************************************
//xQueueHandle g_pPLLQueue;

extern int inputValue;
#define PLLSTACKSIZE        128         // Stack size in words
#define TIMER0A_PRIORITY		2

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
spll_obj->Upd[0]=0.0;
spll_obj->Upd[1]=0.0;
spll_obj->Upd[2]=0.0;
spll_obj->ynotch[0]=0.0;
spll_obj->ynotch[1]=0.0;
spll_obj->ynotch[2]=0.0;
spll_obj->ylf[0]=0.0;
spll_obj->ylf[1]=0.0;
spll_obj->sin[0]=0.0;
spll_obj->sin[1]=0.0;
spll_obj->cos[0]=(1);
spll_obj->cos[1]=(1);
spll_obj->theta[0]=0.0;
spll_obj->theta[1]=0.0;
spll_obj->wn=(2*3.14159*Grid_freq);
//coefficients for the loop filter
spll_obj->lpf_coeff.B1_lf=B1_LPF;
spll_obj->lpf_coeff.B0_lf=B0_LPF;
spll_obj->lpf_coeff.A1_lf=A1_LPF;
spll_obj->notch_coeff.B0_notch=B0_Notch;
spll_obj->notch_coeff.B1_notch=B1_Notch;
spll_obj->notch_coeff.A1_notch=A1_Notch;
spll_obj->notch_coeff.A2_notch=A2_Notch;
spll_obj->lpf_coeff.B0_lf=B0_LPF;
spll_obj->lpf_coeff.A1_lf=A1_LPF;
spll_obj->delta_t=DELTA_T;
	/*//coefficients for the loop filter
	spll_obj->lpf_coeff.B1_lf=B1_LPF;//the loop filter params don't change???
	spll_obj->lpf_coeff.B0_lf=B0_LPF;
	spll_obj->lpf_coeff.A1_lf=A1_LPF;
	spll_obj->delta_T=	DELTA_T;*/
	//PLLCoeffUpdate((double)1/(double)SAMPLING_FREQ,(float)(2*3.14159*(float)60),&PLLSync);
	//SPLL_1ph_notch_coeff_update((double)1/(double)SAMPLING_FREQ,(float)(2*3.14159*(float)60),0.1,0.00001,&PLLSync);
	//configureTIMER0A();	
}

double new_Mag=0;
void configureTIMER0A(void){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // Enable Timer 1 Clock
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic
  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/SAMPLING_FREQ); //SysCtlClockGet()/SWITCHING_FREQ is set to 80MHz according to main file, Reload Value = fclk/fswitch

//Configuring the interrupts	
	TimerIntRegister(TIMER0_BASE, TIMER_A, &TIMER0AIntHandler);
	IntPrioritySet(INT_TIMER0A, 2);
	TimerEnable(TIMER0_BASE, TIMER_A);	// Start Timer 0B
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	//IntEnable(INT_TIMER0A); //Have to have it enabled in order
}

	double waveform=0;
unsigned char	signF=1;
double sinewave[1000 ];
uint16_t s_index = 0;
	double coswave[1000 ];
uint16_t c_index = 0;

void PLLRun(SPLL_1ph_SOGI *spll_obj) {
// Update the spll_obj->u[0] with the grid value before calling this routine
	
	//status ^=GPIO_PIN_3;
	//*****************************************************************************
	//Phase Detect
	//*****************************************************************
	spll_obj->Upd[0]=spll_obj->AC_input*spll_obj->cos[1];
/*	coswave[c_index]= spll_obj->AC_input;
	c_index = (c_index + 1) % (100);*/
	
	//Do we need to multiply the error by 2?
	//spll_obj->Upd[0]=spll_obj->Upd[0]*2;

	
	//*****************************************************************************
	//Notch Filter
	//Linearizes the incoming signal and the orthogonal signal generator from alpha beta to D-Q axis
	//Returns: VD and VQ signal
	//*****************************************************************************
	spll_obj->ynotch[0]=(spll_obj->notch_coeff.B0_notch*(spll_obj->Upd[0]+spll_obj->Upd[2]))+(spll_obj->notch_coeff.B1_notch*spll_obj->Upd[1])-(spll_obj->notch_coeff.A1_notch*spll_obj->ynotch[1])- (spll_obj->notch_coeff.A2_notch*spll_obj->ynotch[2]);
	//*****************************************************************************
	//Low Pass Filter
	//PI Controller that filters out harmonics of Vq 
	//Returns: Filtered output signal
	//*****************************************************************************
	spll_obj->ylf[0]=spll_obj->ylf[1]+(spll_obj->lpf_coeff.B0_lf*spll_obj->ynotch[0])+(spll_obj->lpf_coeff.B1_lf*spll_obj->ynotch[1]);

	//---------------------------------//
	// VCO 
	// Internal voltage oscillator that changes based on the error from PLL
	//---------------------------------//
	//update the output frequency in w (2*pi*f)
	spll_obj->wo=spll_obj->wn+spll_obj->ylf[0];
	float newWo=(2*3.14159*100)+spll_obj->ylf[0]; 
	//ylf(1)=min([ylf(1) 200]);
	

//	fwave[w_index]=spll_obj->wo;
//	w_index = (w_index + 1) % (100);
	
	
	//integration process to compute sine and cosine
	spll_obj->sin[0]=spll_obj->sin[1]+((spll_obj->delta_t*spll_obj->wo)*spll_obj->cos[1]);
	spll_obj->cos[0]=spll_obj->cos[1]-((spll_obj->delta_t*spll_obj->wo)*spll_obj->sin[1]);
	float newSin=spll_obj->sin[1]+(spll_obj->delta_t*(newWo)*spll_obj->cos[1]);

	//limit the oscillator integrators
	
	if(spll_obj->sin[0]>(float)(1)){ //Mysin(1)=min([Mysin(1) 1]);
		spll_obj->sin[0]=(float)(1);
	}
	else if(spll_obj->sin[0]<(float)(-1)){ //Mysin(1)=max([Mysin(1) -1]);
		spll_obj->sin[0]=(float)(-1);
	}
	if(newSin>(float)(1)){ //Mysin(1)=min([Mysin(1) 1]);
		newSin=(float)(1);
	}
	else if(newSin<(float)(-1)){ //Mysin(1)=max([Mysin(1) -1]);
		newSin=(float)(-1);
	}
	if(spll_obj->cos[0]>(float)(1)){ //Mycos(1)=min([Mycos(1) 1]);
		spll_obj->cos[0]=(float)(1);
	}
	else if(spll_obj->cos[0]<(float)(-1)){ //Mycos(1)=max([Mycos(1) -1]);
		spll_obj->cos[0]=(float)(-1);
	}
//	sinewave[s_index]=spll_obj->sin[0];
//	s_index = (s_index + 1) % (1000);
	new_Mag=((spll_obj->sin[0])+1)/2;
	//new_Mag=((newSin)+1)/2;
	inputValue=(int)(3300*(new_Mag));
//	coswave[c_index]=inputValue;
//	c_index = (c_index + 1) % (1000);
//	if(c_index==0){
//		bool fun=false;
//	}
//compute theta value (uncessesary for notch)
/*	spll_obj->theta[0]=spll_obj->theta[1]+(spll_obj->wo*spll_obj->delta_t); //*(float)(0.159154943)
	if(spll_obj->sin[0]>(float)(0.0) && spll_obj->sin[1] <=(float) (0.0) ){
		spll_obj->theta[0]=(float)(0.0); //-3.14159
	}*/
//	spll_obj->sin[0]=sinf (spll_obj->theta[0]); //spll_obj->sin=sinf (spll_obj->theta[0]*(float)2.0*(float)3.1415926);
//	spll_obj->cos[0]=cosf (spll_obj->theta[0]); 


	
	//Time to Update Coefficients
	// update the Upd array for future
	spll_obj->Upd[2]=spll_obj->Upd[1];
	spll_obj->Upd[1]=spll_obj->Upd[0];
		//update ynotch and ylf history for future use
	spll_obj->ynotch[2]=spll_obj->ynotch[1];
	spll_obj->ynotch[1]=spll_obj->ynotch[0];
	spll_obj->ylf[1]=spll_obj->ylf[0];
	
	//update theta
	spll_obj->theta[1]=spll_obj->theta[0];
	//update the history of the sin and cos
	spll_obj->sin[1]=spll_obj->sin[0];
	spll_obj->cos[1]=spll_obj->cos[0];

	//	sinewave[s_index]=new_Mag;
	//s_index = (s_index + 1) % (100);

}

//*********** Structure Coeff Update *****/
 
void SPLL_1ph_notch_coeff_update(float delta_T, float wn,float c2, float c1, SPLL_1ph_SOGI *spll_obj)
{
	// Note c2<<c1 for the notch to work
	float x,y,z;
	x=(float)(2.0*c2*wn*delta_T);
	y=(float)(2.0*c1*wn*delta_T);
	z=(float)(wn*delta_T*wn*delta_T);
	spll_obj->notch_coeff.A1_notch=(y-2);
	spll_obj->notch_coeff.A2_notch=(z-y+1);
	spll_obj->notch_coeff.B0_notch=(1.0);
	spll_obj->notch_coeff.B1_notch=(x-2);
}


void TIMER0AIntHandler(void){
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
	PLLRun(&PLLSync);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);
	
		//IntDisable(INT_TIMER0A);
}