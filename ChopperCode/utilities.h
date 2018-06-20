#include <stdint.h>

#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))	
	
//PortA	0x40004000  
//PortB	0x40005000
//PortC	0x40006000
//PortD	0x40007000
//PortE	0x40024000
//PortF	0x40025000

/*
7 0x0200
6 0x0100
5 0x0080
4 0x0040
3 0x0020
2 0x0010
1 0x0008
0 0x0004
*/

uint32_t summer_duty[24] = {968,901,866,919,1000,1061,1173,1275,1346,1414,1458,1521,1620,1696,1732,1750,1768,1732,1677,1581,1458,1323,1146,1031};
uint32_t winter_duty[24] = {1350,1240,1200,1240,1350,1500,1650,1760,1800,1760,1650,1500,1350,1240,1200,1240,1350,1500,1650,1760,1800,1760,1650,1500};
uint32_t spring_duty[24] = {552,470,400,346,312,300,312,346,400,470,552,640,728,810,880,934,968,980,968,934,880,810,728,640};



void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

void DelayWait10ms(uint32_t n);
void DelayWait1ms(uint32_t n);
void heartbeat(void);
