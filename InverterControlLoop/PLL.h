#include "IQmathLib.h"

#ifndef SPLL_1ph_SOGI_H_
#define SPLL_1ph_SOGI_H_

//*********** Structure Definition ********//
typedef struct{
	float osg_k;
	float osg_x;
	float osg_y;
	float osg_b0;
	float osg_b2;
	float osg_a1;
	float osg_a2;
	float osg_qb0;
	float osg_qb1;
	float osg_qb2;
}OSG_COEFF;
typedef struct{
	float B1_lf;
	float B0_lf;
	float A1_lf;
}LPF_COEFF;
typedef struct{
	float u[3]; // Ac Input
	float osg_u[3];
	float osg_qu[3];
	float u_Q[2];
	float u_D[2];
	float ylf[2];
	float fo; // output frequency of PLL
	float fn; //nominal frequency
	float theta[2];
	float cos;
	float sin;
	float delta_T;
	OSG_COEFF osg_coeff;
	LPF_COEFF lpf_coeff;
}SPLL_1ph_SOGI;
//*********** Function Declarations *******//
void PLLTask(void);
void PLLTaskInit(uint16_t Grid_freq, double DELTA_T,volatile LPF_COEFF lpf_coeff, SPLL_1ph_SOGI *spll_obj);
static inline void PLLRun(SPLL_1ph_SOGI *spll_obj); //pass in a pofloater 
void PLLCoeffUpdate(double delta_T, float wn, volatile SPLL_1ph_SOGI *spll);
void Timer2AIntHandler(void); //PLL ISR
void configureTimer2A(void); //ISR Init
#endif 

//*********** Macro Definition ***********//
//A macro is like a text shortcut, whenver you see "SPLL_1ph_SOGI_run_MACRO, this code runs instead
/*#define SPLL_1ph_SOGI_run_MACRO(v) \
v.osg_u[0]=SPLL_SOGI_Qmpy(v.osg_coeff.osg_b0,(v.u[0]-
v.u[2]))+SPLL_SOGI_Qmpy(v.osg_coeff.osg_a1,v.osg_u[1])+SPLL_SOGI_Qmpy(v.osg_coeff.osg_a2,v.osg_u[2
]);\
v.osg_u[2]=v.osg_u[1];\
v.osg_u[1]=v.osg_u[10];\
v.osg_qu[0]=SPLL_SOGI_Qmpy(v.osg_coeff.osg_qb0,v.u[0])+SPLL_SOGI_Qmpyu(v.osg_coeff.osg_qb1,v.u[1])
+SPLL_SOGI_Qmpy(v.osg_coeff.osg_qb2,v.u[2])+SPLL_SOGI_Qmpy(v.osg_coeff.osg_a1,v.osg_qu[1])+
SPLL_SOGI_Qmpy(v.osg_coeff.osg_a2.v.osg_qu[2]);
\
v.osg_qu[2]=v.osg_qu[1]; \
v.osg_qu[1]=v.osg_qu[0]; \
v.u[2]=v.u[1]; \
v.u[1]=v.u[0]; \
v.u_Q[0]=SPLL_SOGI_Qmpy(v.cos,v.osg_u[0])+SPLL_SOGI_Qmpy(v.sin,v.osg_qu[0]); \
v.u_D[0]=SPLL_SOGI_Qmpy(v.cos,v.osg_qu[0])-SPLL_SOGI_Qmpy(v.sin,v.osg_u[0]); \
v.ylf[0]=v.ylf[1]+SPLL_SOGI_Qmpy(v.lpf_coeff.B0_lf,v.u_Q[0])+SPLL_SOGI_Qmpy(v.lpf_coeff.B1_lf,v.u_
Q[1]); \
v.ylf[1]=v.ylf[0]; \
v.u_Q[1]=v.u_Q[0]; \
v.fo=v.fn+v.ylf[0]; \
v.theta[0]=v.theta[1]+SPLL_SOGI_Qmpy(SPLL_SOGI_Qmpy(v.fov.v.delta_T),SPLL_Q(2*3.1415926)); \
if(v.theta[0]>SPLL_SOGI_Q(2*3.1415926)) \
v.theta[0]=SPLL_SOGI_Q(0.0); \
v.theta[1]=v.theta[0]; \
v.sin=SPLL_SOGI_SINE(v.theta[0]); \
v.cos=SPLL_SOGI_COS(v.theta[0]);
#endif */
