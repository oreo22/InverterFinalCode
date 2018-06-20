#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "utilities.h"

void DelayWait10ms(uint32_t n){
	uint32_t volatile time;
  while(n){
    time = 727240*2/91;  // 10msec
    while(time){
      time--;
    }
    n--;
  }
}

void DelayWait1ms(uint32_t n){
	uint32_t volatile time;
  while(n){
    time = (727240*2/91)/10;  // 1 msec //ACTION ITEM: ASK VALVANO ABOUT THIS EQUATION
    while(time){
      time--;
    }
    n--;
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
  PF2 |= 0x04;              // turn off LED
	PF3 |= 0x08;
	
}


