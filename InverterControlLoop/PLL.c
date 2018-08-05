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
#define PI (3.14159)
#define Q3PI2  4.7124
#define QPI2   1.5708
#define Q2PI   6.2832
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
double degreeDesired=0;
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
		PLLCoeffUpdate(((float)(1.0/SAMPLING_FREQ)),(float)(Q2PI*60),&spll2);*/
	//shouldn't change unless grid freq changes 
	

}
SPLL_1ph_SOGI VSync;


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
spll_obj->wn=(Q2PI*Grid_freq);
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
	//PLLCoeffUpdate((double)1/(double)SAMPLING_FREQ,(float)(Q2PI*(float)60),&VSync);
	//SPLL_1ph_notch_coeff_update((double)1/(double)SAMPLING_FREQ,(float)(Q2PI*(float)60),0.1,0.00001,&VSync);
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
float sinewave[100 ];
uint16_t s_index = 0;
float oldValue=-1;
void PLLRun(SPLL_1ph_SOGI *spll_obj) {
// Update the spll_obj->u[0] with the grid value before calling this routine
	
	//status ^=GPIO_PIN_3;
//	count++;
	//Frequency Update
//	if(oldValue>0 && spll_obj->AC_input<=0){ //zero crossing 
//            if(coswave[c_index] !=60){
							// coswave[c_index]=6000/count;
//						}            
//						count=0;
//		}
	oldValue=spll_obj->AC_input;
	//*****************************************************************************
	//Phase Detect
	//*****************************************************************
	spll_obj->Upd[0]=spll_obj->AC_input*spll_obj->cos[1];
/*	coswave[c_index]= spll_obj->AC_input;
	c_index = (c_index + 1) % (100);*/
	
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
	//spll_obj->ylf[0]=0;
	spll_obj->wo=spll_obj->wn+spll_obj->ylf[0]; //update the output frequency in w (Q2PI*f)
	
	//---------------------------------//
	// VCO 
	// Internal voltage oscillator that changes based on the error from PLL
	//---------------------------------//
	
	
	// Put in the phase shift: Phase angle (deg) ? = time delay ? t × frequency f × 360
	degreeDesired=13.5; //changed in var control //the board introduces a 13.5 phase shift. it's been corrected to the best of my abilities, but still
	float newTimeShift=(degreeDesired*PI)/180;
	
	
	//compute theta value 
	spll_obj->theta[0]=spll_obj->theta[1]+(spll_obj->wo*spll_obj->delta_t); //*(float)(0.159154943)
	if(spll_obj->theta[0]>(float)(6.2832)){ //greater than 2 pi
		spll_obj->theta[0]=(float)(0.0);
	}
//	if(spll_obj->theta[0]<(float)(-6.2832)){ //greater than 2 pi
//		spll_obj->theta[0]=(float)(0.0);
//	}
	
	

//	%% Convert theta into an index to read table, goes from 0 to 2 pi. i go from 0 to 1 or 0 to QPI2 
//  % Update phase accumulator and extract the sine table index from it
//   % Mysin(1)=sin(theta(1));
//   %  Mycos(1)=cos(theta(1));
//Looking up Sine values
  float sinTheta=spll_obj->theta[0];
	
	
//	if(s_index==99){
//			bool fun=false;
//	}
	int8_t sineSign=1;
    if(spll_obj->theta[0] > (QPI2) && spll_obj->theta[0]< (PI)){ //Q2
       sinTheta=PI-spll_obj->theta[0];
		}
    
    if(spll_obj->theta[0]> PI && spll_obj->theta[0]< (Q3PI2)){ //Q3
        sinTheta=spll_obj->theta[0]-PI;
       sineSign=-1;
    }
    if(spll_obj->theta[0]> (Q3PI2) && spll_obj->theta[0] < (Q2PI)){ //Q4
        sinTheta=(Q2PI)-spll_obj->theta[0];
        sineSign=-1;
    }
		
    sinTheta=sinTheta*(2/PI); // sinIndex=sinTheta*(2/pi);
    int sinIndex=(int)ceil(sinTheta*(1023)); 
		spll_obj->sin[0]= sineSign*sineTable[sinIndex];
		

		
		//		new_Mag=sineSign*(sineTable[sinIndex]);

		
	// Looking up the phase shifted VCO output
		
 
		sineSign=1;
    float cosIndex=spll_obj->theta[0]+(QPI2); //changing the theta for cos
    if(cosIndex>(Q2PI)){ //Q1
        cosIndex=cosIndex-(Q2PI);
    }
    if(cosIndex> (QPI2) && cosIndex< (PI)) {//Q2
       cosIndex=PI-cosIndex;
    }
    if(cosIndex> PI && cosIndex< (Q3PI2)){//Q3
        cosIndex=cosIndex-PI;
           sineSign=-1;
    }
    if(cosIndex> (Q3PI2) && cosIndex < (Q2PI)){ //Q4
        cosIndex=(Q2PI)-cosIndex;
           sineSign=-1;
    }
    cosIndex=cosIndex*(2/PI);
		int cosTheta=(int)ceil(cosIndex*(1023)+1);
		spll_obj->cos[0]=sineSign*(sineTable[cosTheta]);
		
		//Do the same thing again for phase shifted VCO
		sinTheta=spll_obj->theta[0]+newTimeShift;
		sineSign=1;
    if(sinTheta>(Q2PI)){ //Q1
        sinTheta=sinTheta-(Q2PI);
    }
    if(sinTheta> (QPI2) && sinTheta< (PI)){
       sinTheta=PI-sinTheta;
		}
    
    if(sinTheta> PI && sinTheta< (Q3PI2)){ //Q3
        sinTheta=sinTheta-PI;
       sineSign=-1;
    }
    if(sinTheta> (Q3PI2) && sinTheta < (Q2PI)){ //Q4
        sinTheta=(Q2PI)-sinTheta;
        sineSign=-1;
    }
    sinTheta=sinTheta*(2/PI);
    sinIndex=(int)ceil(sinTheta*(1023));
		new_Mag=sineSign*(sineTable[sinIndex]);
		new_Mag=(new_Mag+1)/2;
		new_Mag=(1-new_Mag);
		inputValue=(int)(3300*(new_Mag)); //modulate the size of the sine wave here with ma 
		
////		sinewave[s_index]=sinIndex;
////		s_index = (s_index + 1) % (100);
////		if(s_index==999){
////			bool fun=false;
////		}



	
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
	if(spll_obj->theta[1]<=-6.2832){
		bool fun=false;
	}
	//update the history of the sin and cos
	spll_obj->sin[1]=spll_obj->sin[0];
	spll_obj->cos[1]=spll_obj->cos[0];


}

void TIMER0AIntHandler(void){
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
	PLLRun(&VSync);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);
	
		//IntDisable(INT_TIMER0A);
}