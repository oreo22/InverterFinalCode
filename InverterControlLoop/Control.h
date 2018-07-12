
struct PIArray{
	double inst_err;
//	double run_err;
	double Ki;
	double Kp;
};
typedef struct PIArray PIArr;


void VarControl(void);
void ctrlInit(void);
void Timer2AIntHandler(void);
void configureTimer2A(void);

