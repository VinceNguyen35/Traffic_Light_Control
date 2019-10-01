// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Vince Nguyen, Ethan Nguyen
// October 12, 2018

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE0 (1=pedestrian present)
// east/west car detector connected to PE1 (1=car present)
// north/south car detector connected to PE2 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "tm4c123gh6pm.h"

#define SENSOR				          (*((volatile unsigned long *)0x4002401C))
#define LIGHT					          (*((volatile unsigned long *)0x400050FC))
#define P_LIGHT									(*((volatile unsigned long *)0x40025028))

//Port B Declaration
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_PUR_R        (*((volatile unsigned long *)0x40005510))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))

//Port E Declaration
#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))

//Port E Interrupt Declaration
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
#define NVIC_PRI1_R             (*((volatile unsigned long *)0xE000E404))  // IRQ 7 to 5 Priority Register
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define GPIO_PORTE_IS_R         (*((volatile unsigned long *)0x40024404))
#define GPIO_PORTE_IBE_R        (*((volatile unsigned long *)0x40024408))
#define GPIO_PORTE_IEV_R        (*((volatile unsigned long *)0x4002440C))
#define GPIO_PORTE_IM_R         (*((volatile unsigned long *)0x40024410))
#define GPIO_PORTE_RIS_R        (*((volatile unsigned long *)0x40024414))
#define GPIO_PORTE_ICR_R        (*((volatile unsigned long *)0x4002441C))
#define GPIO_PORTE_PDR_R        (*((volatile unsigned long *)0x40024514))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port F Clock Gating Control

//Port F Declaration
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))

// 2. Declarations Section
//   Global Variables
unsigned long In;  		// input from PE2,PE1,PE0
unsigned long CarOut; // outputs to PB5,PB4,PB3,PB2,PB1,PB0 (Car LED)
unsigned long PedOut;	// outputs to PF3,PF1 (Pedestrian LED)
volatile unsigned long Counts = 0;
unsigned int CS;									// current state
unsigned int Input;								// takes input from buttons

//   Function Prototypes
void PortB_Init(void);
void PortE_Init(void);
void PortF_Init(void);
void Delay(void);
void EnableInterrupts(void);
void GPIOPortE_Handler(void);

struct State {
  unsigned long CarOut;		// outputs car LED
	unsigned long PedOut;		// outputs pedestrian LED
  unsigned long Time;			// determines duration of state
  unsigned long Next[8];	// determines next state
}; 

typedef const struct State STyp;

#define goN   			0
#define waitN 			1
#define goE   			2
#define waitE 			3
#define goP					4
#define waitPOn1		5
#define waitPOff1		6
#define waitPOn2		7
#define waitPOff2		8

STyp FSM[9]={
 {0x21,0x02,12,{goN,waitN,waitN,waitN,goN,waitN,waitN,waitN}},																			// state 1: north LED on green,		east LED on red,		ped LED on red
 {0x22,0x02, 4,{goE,goP,goE,goE,goE,goP,goE,goE}},																									// state 2: north LED on yellow,	east LED on red,		ped LED on red
 {0x0C,0x02,12,{goE,waitE,goE,waitE,waitE,waitE,waitE,waitE}},																			// state 3: north LED on red,			east LED on green,	ped LED on red
 {0x14,0x02, 4,{goP,goP,goP,goP,goN,goP,goN,goP}},																									// state 4: north LED on red,			east LED on yellow,	ped LED on red
 {0x24,0x08,12,{goP,goP,waitPOn1,waitPOn1,waitPOn1,waitPOn1,waitPOn1,waitPOn1}},										// state 5: north LED on red,			east LED on red,		ped LED on green
 {0x24,0x02, 1,{waitPOff1,waitPOff1,waitPOff1,waitPOff1,waitPOff1,waitPOff1,waitPOff1,waitPOff1}},	// state 6: north LED on red,			east LED on red,		ped LED on red
 {0x24,0x00, 1,{waitPOn2,waitPOn2,waitPOn2,waitPOn2,waitPOn2,waitPOn2,waitPOn2,waitPOn2}},					// state 7: north LED on red,			east LED on red,		ped LED on blank
 {0x24,0x02, 1,{waitPOff2,waitPOff2,waitPOff2,waitPOff2,waitPOff2,waitPOff2,waitPOff2,waitPOff2}},	// state 8: north LED on red,			east LED on red,		ped LED on red
 {0x24,0x00, 1,{goP,goP,goE,goE,goN,goN,goN,goN}}																									// state 9: north LED on red,			east LED on red,		ped LED on blank
};

 
// ***** 3. Subroutines Section *****

void PortB_Init(void){ volatile unsigned long Delay;
  SYSCTL_RCGC2_R |= 0x00000002;    	 // 1) B clock
  Delay = SYSCTL_RCGC2_R;            // delay   
  GPIO_PORTB_CR_R |= 0xFF;           // allow changes to PB4-0       
  GPIO_PORTB_AMSEL_R &= ~0xFF;        // 3) disable analog function
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF;   // 4) GPIO clear bit PCTL  
  GPIO_PORTB_DIR_R |= 0xFF;          // 5) PB4,PB3,PB2,PB1,PB0 output   
  GPIO_PORTB_AFSEL_R &= ~0xFF;        // 6) no alternate function
  GPIO_PORTB_PUR_R &= 0xFF;          // do not enable pull up resistors     
  GPIO_PORTB_DEN_R |= 0xFF;          // 7) enable digital pins PB7-PB0        
}

void PortE_Init(void){ volatile unsigned long Delay;
  SYSCTL_RCGC2_R |= 0x00000010;     // 1) E clock
  Delay = SYSCTL_RCGC2_R;           // delay    
  GPIO_PORTE_CR_R |= 0x3F;           // allow changes to PE5-0       
  GPIO_PORTE_AMSEL_R &= ~0x3F;        // 3) disable analog function
  GPIO_PORTE_PCTL_R &= ~0x00000FFF;   // 4) GPIO clear bit PCTL  
  GPIO_PORTE_DIR_R |= 0x00;          // 5) PE2,PE1,PE0 input   
  GPIO_PORTE_AFSEL_R &= ~0x3F;        // 6) no alternate function
  GPIO_PORTE_PUR_R &= ~0x3F;          // do not enable pull up resistors     
  GPIO_PORTE_DEN_R |= 0x3F;          // 7) enable digital pins PE5-PE0 

	GPIO_PORTE_IS_R &= ~0x07;     											// (a) PE0,PE1,PE2 are edge-sensitive
  GPIO_PORTE_IBE_R &= ~0x07;    											//     PE0,PE1,PE2 is not both edges
  GPIO_PORTE_IEV_R &= ~0x07;   		  									//     PE0,PE1,PE2 falling edge event
  GPIO_PORTE_ICR_R = 0x07;      											// (b) clear flag4
  GPIO_PORTE_IM_R |= 0x07;      											// (c) arm interrupt on PE0,PE1,PE2
  NVIC_PRI3_R = (NVIC_PRI3_R&0xFF00FFFF)|0x00A00FFF;  // (d) priority 5
  NVIC_EN0_R = 0x40000000;     	    									// (e) enable interrupt 30 in NVIC
}

void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R |= 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R |= 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R &= ~0x0A;        // 3) disable analog function
  GPIO_PORTF_PCTL_R &= ~0x0000F0F0;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R |= 0x1F;          // 5) PF4,PF3,PF2,PF1,PF0 output   
  GPIO_PORTF_AFSEL_R &= ~0x0A;        // 6) no alternate function
  GPIO_PORTF_PUR_R |= 0x0A;          // enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R |= 0x0A;          // 7) enable digital pins PF4-PF0        
}

void GPIOPortE_Handler(void){
  GPIO_PORTF_ICR_R = 0x07;      // acknowledge flag4
	Input = SENSOR;									// read sensors
}

void SysTick_Init(unsigned long period){
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
  NVIC_ST_CTRL_R = 0x07;			// enable SysTick with core clock and interrupts
}
// Interrupt service routine
// Executed every 62.5ns*(period)
void SysTick_Handler(void){
  LIGHT = FSM[CS].CarOut;  				// set car lights
	P_LIGHT = FSM[CS].PedOut;				// set pedestrian light
  CS = FSM[CS].Next[Input];  			// go to next state
  Counts = Counts + 1;
}

int main(void){ 
	PortB_Init();											// outputs car LED
	PortE_Init();											// inputs buttons
	PortF_Init();											// outputs ped LED
	SysTick_Init(8000000);        // initialize SysTick timer
  EnableInterrupts();  // The grader uses interrupts
	
	CS = goN;													// project starts with goN
	
  while(1){	
	}
}
