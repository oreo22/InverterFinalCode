#include <stdint.h>
#include "PLL.h"
#include "PWM.h"
#include "../inc/tm4c123gh6pm.h"

void	EnableInterrupts(void);

#define PE0       (*((volatile uint32_t *)0x40024004))
#define PE1       (*((volatile uint32_t *)0x40024008))

#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))	


//array

//uint32_t summer_duty[24] = {968,901,866,919,1000,1061,1173,1275,1346,1414,1458,1521,1620,1696,1732,1750,1768,1732,1677,1581,1458,1323,1146,1031};
//uint32_t summer_duty[24] = {1101,1039,1006,1055,1131,1187,1292,1387,1453,1517,1557,1616,1708,1779,1812,1829,1846,1812,1761,1672,1557,1432,1267,1160};
uint32_t summer_duty[24] = {1201,1139,1106,1155,1231,1287,1392,1487,1553,1617,1657,1716,1808,1879,1912,1929,1946,1912,1861,1772,1657,1532,1367,1260};
uint32_t winter_duty[24] = {1350,1240,1200,1240,1350,1500,1650,1760,1800,1760,1650,1500,1350,1240,1200,1240,1350,1500,1650,1760,1800,1760,1650,1500};
uint32_t spring_duty[24] = {552,470,400,346,312,300,312,346,400,470,552,640,728,810,880,934,968,980,968,934,880,810,728,640};


void DelayWait10ms(uint32_t n){uint32_t volatile time;
  while(n){
    time = 727240*2/91;  // 10msec
    while(time){
      time--;
    }
    n--;
  }
}

uint8_t flag;
uint16_t count;
void Timer_15s_Init(void){
		NVIC_ST_CTRL_R = 0;                 // disable SysTick during setup
    NVIC_ST_RELOAD_R = 10000000;      // reload value (will interupt once a second for 80Mhz clock) 0x04C4B400
    NVIC_ST_CURRENT_R = 0;            // any write to current clears it
    NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2          
	  NVIC_ST_CTRL_R = 0x07; // enable SysTick with core clock and interrupts
		count = 0;
	  flag = 0;
}

void SysTick_Handler(void){ //change the value of this to determine the time between changing duty cycles 
	//count = (count+1)%8; //if i divide by 8, i get 0.5 second changes which will really test the system 
	count = (count+1)%240; 
	//if (count == 7){
	if (count == 239){
		flag = 1;
	}
}

void heartbeat(void){
	 SYSCTL_RCGCGPIO_R |= 0x00000020;         // activate port F
  GPIO_PORTF_DIR_R |= 0x0C;                // make PF2 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x0C;             // disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x0C;                // enable digital I/O on PF2
                                           // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFF00FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;                  // disable analog functionality on PF
  PF2 ^= 0x04;              // turn off LED
	PF3 ^= 0x08;
	//PF2 = 0;
	//PF3 = 0;
	
}




int main(void){
	PLL_Init(Bus80MHz);               // bus clock at 80 MHz
  //PWM0A_Init(13333, 6666);         // initialize PWM0, 6000 Hz, 50% duty
	//PWM0B_Init(13333, 6666);
	
	//PWM0A_Init(2000, 1000);
	//PWM0B_Init(2000, 1000);
	
	//PB6 and PB7
	PWM0A_Init(2000, summer_duty[0]);	//50% duty cycle, change second number to % of first
	PWM0B_Init(2000, summer_duty[0]);
	
	//PB4 and PB5
	PWM0A2_Init(2000, 1500);	//75% duty cycle
 	PWM0B2_Init(2000, summer_duty[0]);
	
	//PE4 and PE5
	//PWM0A3_Init(8000, 2000);	//25% duty cycle, change second number to % of first
	//PWM0B3_Init(8000, 2000);
	
	Timer_15s_Init();
	heartbeat();
	EnableInterrupts();
	uint8_t index = 0;
	
	/*SYSCTL_RCGCGPIO_R |= 0x00000020;         // activate port F

	GPIO_PORTF_DIR_R |= 0x04;                // make PF2 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x04;             // disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x04;                // enable digital I/O on PF2
                                           // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;                  // disable analog functionality on PF
	
	
  GPIO_PORTF_DATA_R &= ~0x18;              // turn off LED
	*/
	
	while(1){
		//DelayWait10ms(1500);
		if (flag){
			flag = 0;
			index = (index +1)%24;
			PWM0A2_Duty(1500);
			PWM0B2_Duty(summer_duty[index]);
		}
		
		if (index == 0){
			PF3 ^= 0x08;
			PF2 = 0;
		} else{
			PF2 ^= 0x04;
			PF3 = 0;
		}
		//GPIO_PORTF_DATA_R ^= 0x04;             // toggle LED
		
	}
}
