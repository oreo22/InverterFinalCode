#include "IQmathLib.h"

#ifndef SPLL_1ph_SOGI_H_
#define SPLL_1ph_SOGI_H_

#define SPLL_SOGI_Q _IQ23
#define SPLL_SOGI_Qmpy _IQ23mpy(A, B) 
#define SPLL_SOGI_SINE _IQ23sin
#define SPLL_SOGI_COS _IQ23cos
//*********** Structure Definition ********//
typedef struct{
	uint32_t osg_k;
	uint32_t osg_x;
	uint32_t osg_y;
	uint32_t osg_b0;
	uint32_t osg_b2;
	uint32_t osg_a1;
	uint32_t osg_a2;
	uint32_t osg_qb0;
	uint32_t osg_qb1;
	uint32_t osg_qb2;
}OSG_COEFF;
typedef struct{
	uint32_t B1_lf;
	uint32_t B0_lf;
	uint32_t A1_lf;
}LPF_COEFF;
typedef struct{
	uint32_t u[3]; // Ac Input
	uint32_t osg_u[3];
	uint32_t osg_qu[3];
	uint32_t u_Q[2];
	uint32_t u_D[2];
	uint32_t ylf[2];
	uint32_t fo; // output frequency of PLL
	uint32_t fn; //nominal frequency
	uint32_t theta[2];
	uint32_t cos;
	uint32_t sin;
	uint32_t delta_T;
	OSG_COEFF osg_coeff;
	LPF_COEFF lpf_coeff;
}SPLL_1ph_SOGI;
//*********** Function Declarations *******//
void PLLTask(void);
uint32_t PLLTaskInit(uint16_t Grid_freq, long DELTA_T,volatile LPF_COEFF lpf_coeff, SPLL_1ph_SOGI *spll_obj);
void PLLRun(SPLL_1ph_SOGI *spll_obj); //pass in a pointer 
void PLLCoeffUpdate(float delta_T, float wn, volatile SPLL_1ph_SOGI *spll);
void Timer0BIntHandler(void); //PLL ISR
void configureTimer0B(void); //ISR Init
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
