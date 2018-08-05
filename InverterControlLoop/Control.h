#include "adc_task.h"
struct PIArray{
	double inst_err;
//	double run_err;
	double Ki;
	double Kp;
};
typedef struct PIArray PIArr;


void VarControl(ACPower_t *Sbus, ACPower_t *Sctrl,acValues *Vpcc, acValues *Pgrid);
void ctrlInit(void);
void Timer2AIntHandler(void);
void configureTimer2A(void);

