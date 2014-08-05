// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))
	
#define SENSOR              (*((volatile unsigned long *)0x40004070)) //acesses PA4-PA2
#define VEHICLE_LIGHTS       (*((volatile unsigned long *)0x400050FC))	//accesses PB5–PB0
#define PEDESTRIAN_LIGHTS    (*((volatile unsigned long *)0x40025028))	//accesses PF3 and PF1	
	
#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) // bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
	
#define GPIO_PORTA_IN           (*((volatile unsigned long *)0x40004070)) // bits 2-0
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))	
	
#define GPIO_PORTF_OUT          (*((volatile unsigned long *)0x40025028))// bits 1 and 3
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))

struct State {
  unsigned long Vehicle_Lights;
	unsigned long Pedestrian_Lights;
  unsigned long Time;
  unsigned long Next_State[10];};
typedef const struct State STyp;
	
#define goW      0 //Vehicle_Light: 0x0C, Pedestrian_Lights:0x02
#define waitW    1 //Vehicle_Light: 0x14, Pedestrian_Lights:0x02
#define goS      2 //Vehicle_Light: 0x21, Pedestrian_Lights:0x02
#define waitS    3 //Vehicle_Light: 0x22, Pedestrian_Lights:0x02
#define walk     4 //Vehicle_Light: 0x24, Pedestrian_Lights:0x08
#define walkOn1  5 //Vehicle_Light: 0x24, Pedestrian_Lights:0x02
#define walkOff1 6 //Vehicle_Light: 0x24, Pedestrian_Lights:0x00
#define walkOn2  7 //Vehicle_Light: 0x24, Pedestrian_Lights:0x02
#define walkOff2 8 //Vehicle_Light: 0x24, Pedestrian_Lights:0x00
#define allR   9 //Vehicle_Light: 0x24, Pedestrian_Lights:0x02
	
STyp FSM[10]={
 {0x0C, 0x02, 300,{goW,goW,waitW,waitW,waitW, waitW, waitW, waitW}}, 
 {0x14, 0x02, 200,{allR,allR,goS,goS,walk,walk,goS,goS}},
 {0x21, 0x02, 300,{goS,waitS,goS,waitS,waitS,waitS,waitS,waitS}},
 {0x22, 0x02, 200,{allR,goW,allR,goW,walk,walk,walk,walk}},
 {0x24, 0x08, 300,{walk,walkOn1,walkOn1,walkOn1,walk,walkOn1,walkOn1,walkOn1}}, 
 {0x24, 0x02, 50,{walkOff1,walkOff1,walkOff1,walkOff1,walkOff1,walkOff1,walkOff1,walkOff1}},
 {0x24, 0x00, 50,{walkOn2,walkOn2,walkOn2,walkOn2,walkOn2,walkOn2,walkOn2,walkOn2}},
 {0x24, 0x02, 50,{walkOff2,walkOff2,walkOff2,walkOff2,walkOff2,walkOff2,walkOff2,walkOff2}},
 {0x24, 0x00, 50,{allR,goW,goS,goW,allR,goW,goS,goW}}, 
 {0x24, 0x02, 200,{allR,goW,goS,goS,walk,walk,walk,walk}}};

unsigned long curState;  // index to the current state 
unsigned long input; 	
// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void SysTick_Init(void);
void SysTick_Wait(unsigned long delay);
void SysTick_Wait10ms(unsigned long delay);
void Ports_Init(void);
// ***** 3. Subroutines Section *****

int main(void){ 
  TExaS_Init(SW_PIN_PA432, LED_PIN_PB543210); // activate grader and set system clock to 80 MHz
  SysTick_Init();   
  Ports_Init();
  EnableInterrupts();
	curState = goW;
  while(1){
    VEHICLE_LIGHTS = FSM[curState].Vehicle_Lights;  
		PEDESTRIAN_LIGHTS = FSM[curState].Pedestrian_Lights;
    SysTick_Wait10ms(FSM[curState].Time);
    input = SENSOR >> 2;     // read sensors
    curState = FSM[curState].Next_State[input];  
  }
}

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 800000*12.5ns equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}

void Ports_Init(void){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x23;      // 1) A B F ports
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  
	GPIO_PORTA_AMSEL_R &= ~0x1C; // 3) disable analog function on PA4-2
  GPIO_PORTA_PCTL_R &= ~0x000FFF00; // 4) enable regular GPIO
  GPIO_PORTA_DIR_R &= ~0x1C;   // 5) inputs on PA4-2
  GPIO_PORTA_AFSEL_R &= ~0x1C; // 6) regular function on PA4-2
  GPIO_PORTA_DEN_R |= 0x1C;    // 7) enable digital on PA4-2
  
	GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 7) enable digital on PB5-0
	
	GPIO_PORTF_AMSEL_R &= ~0x0A;        // 3) disable analog function PF1 and PF3
  GPIO_PORTF_PCTL_R &= ~0x0000F0F0;   // 4) GPIO clear bit PCTL  PF1 and PF3
  GPIO_PORTF_DIR_R |= 0x0A;          // 5.2) PF3,PF1 as outputs  
  GPIO_PORTF_AFSEL_R &= ~0x0A;        // 6) no alternate function PF1 and PF3
	GPIO_PORTF_DEN_R |= 0x0A;    // 7) enable digital on PF1 and PF3
}
