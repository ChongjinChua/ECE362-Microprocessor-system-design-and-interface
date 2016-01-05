/*
************************************************************************
 ECE 362 - Mini-Project C Source File - Fall 2015
***********************************************************************
      
 Team ID: < 14 >

 Project Name: < MAZE RUNNER >

 Team Members:

   - Team/Doc Leader: < ? >      Signature: ______________________
   
   - Software Leader: < Chongjin Chua >      Signature: CHONG JIN CHUA

   - Interface Leader: < ? >     Signature: ______________________

   - Peripheral Leader: < ? >    Signature: ______________________


 Academic Honesty Statement:  In signing above, we hereby certify that we 
 are the individuals who created this HC(S)12 source file and that we have
 not copied the work of any other student (past or present) while completing 
 it. We understand that if we fail to honor this agreement, we will receive 
 a grade of ZERO and be subject to possible disciplinary action.

***********************************************************************

 The objective of this Mini-Project is to .... 
 
 Create an autonomous maze exploring vehicle. The vehicle would have the ability
 to explore different kinds of mazes, and display the data on a screen upon exit.


***********************************************************************

 List of project-specific success criteria (functionality that will be
 demonstrated):

 1. Motor rotation using PWM

 2. Distance measurement of infrared sensors using ATD

 3. Data transmittion with radio frequency sensors using SCI

 4.

 5.

***********************************************************************

  Date code started: < 25th November 2015 (Wednesday) >

  Update history (add an entry every time a significant change is made):

  Date: < 25th November 2015 (Wednesday) >  Name: < ChongJin Chua >   Update: < vehicle forward moving for 2 seconds >

  Date: < 26th November 2015 (Thursday) >  Name: < ChongJin Chua >   Update: < vehicle left/right/180 degrees turn, and configured infrared sensors(TIM and ATD module)  >
  
  Date: < 27th November 2015 (Friday) >  Name: < ChongJin Chua >   Update: < organized code into their specific funcitons, added in vehicle self-aligning code  >  

  Date: < 25th November 2015 (Saturday) >  Name: < ChongJin Chua >   Update: < made the code recursive by calling the recursion() function during run-time  >
  
  Date: < 5th December 2015 (Saturday) >  Name: < ChongJin Chua >   Update: < Decided to scale down the project, got rid of the recursive part. Configured RF sensor on a separate file  >


***********************************************************************

*****
self_align():
left right IR sensors first executes ATD conversion, and then aligns itself to prevent collision with the wall. 
- TURN DURATION DOESN'T INVOLVE TIM MODULE, for loop is being used

timed_movement();  
reset timer to the desired turn duration, and execute task(could be left/right turn, move forward or even calling self_align() function)
- Key Components: setting TCNT=0 and TC7=0xFFFF, and then set the desired number of times for interrupt to occur
- TIM MODULE NEEDS TO BE OFF BEFORE CALLING THIS FUNCTION
- TC7 will be set to original value(15000) before the function ends
- TIM MODULE and PWME remains off upon the end of function

pwm_config():
instruct vehicle to move in desired direction
- channel 0 = right motor forward, channel 3 = left motor forward
- ONLY ATD channel 0 is being used, NO INTERRUPTS INVOLVED
*****

*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>

/* All functions after main should be initialized here */
char inchar(void);
void outchar(char x);
void recursion(const char current_direction, const char previous_direction, short unsigned int stack, char did_junction);
void pwm_config(char pin0, char pin1, char pin2, char pin3);
void timed_movement(char *timer_flag, char *task_completed_flag, char pin0, char pin1, char pin2, char pin3, unsigned int turn_duration_count, char mode);
void self_align();
void force_delay(short unsigned int x);
const char flip_direction(const char direction);
void original_direction(const char prev_direction,const char rever_direction);
const char left_turn_direction(const char direction);
const char right_turn_direction(const char direction);

/* Variable declarations */
const char NORTH = 0;
const char SOUTH = 1;
const char WEST = 2;
const char EAST = 3;
const short unsigned int TURN_180_DURATION = 24;
const short unsigned int TURN_LEFT_DURATION = 14;
const short unsigned int TURN_RIGHT_DURATION = 15;
const short unsigned int JUNCTION_ENTER = 18;
const short unsigned int JUNCTION_EXIT = 24;
const char SELF_ALIGN_MODE = 1;
const char MOTOR_ON = 1;
const short unsigned int DEADEND_THRESH = 150;
const short unsigned int SIDEWALL_THRESH = 140;
const short unsigned int JUNCTION_THRESH = 115;
char prevl = 0;
char prevr = 0;
char leftpb = 0;
char rghtpb = 0;
short int tencnt = 0;
short int onecnt = 0;
char tenths = 0;
char onesec = 0;
unsigned int i = 0; 
unsigned int tcnt_count = 0;
unsigned int turn_duration = 0;
char left_sample_switch = 1;
char right_sample_switch = 1;
char turned_flag = 0;
char timer_turn_flag = 0;
char returned_junction_flag = 0;
char timer_return_junction_flag = 0;
char deadend_flag = 0;
char left_explore_flag = 0;
char right_explore_flag = 0;
short unsigned int left_ir_data = 0;
short unsigned int right_ir_data = 0;
short unsigned int front_ir_data = 0;
long unsigned int global_elapsed = 0;
              

/* Special ASCII characters */
#define CR 0x0D // ASCII return 
#define LF 0x0A // ASCII new line 

/* LCD COMMUNICATION BIT MASKS (note - different than previous labs) */
#define RS 0x10 // RS pin mask (PTT[4])
#define RW 0x20 // R/W pin mask (PTT[5])
#define LCDCLK 0x40 // LCD EN/CLK pin mask (PTT[6])

/* LCD INSTRUCTION CHARACTERS */
#define LCDON 0x0F // LCD initialization command
#define LCDCLR 0x01 // LCD clear display command
#define TWOLINE 0x38 // LCD 2-line enable command
#define CURMOV 0xFE // LCD cursor move instruction
#define LINE1 0x80 // LCD line 1 cursor position
#define LINE2 0xC0 // LCD line 2 cursor position

 
/*  
***********************************************************************
 Initializations
***********************************************************************
*/

void  initializations(void) {

/* Set the PLL speed (bus clock = 24 MHz) */
  CLKSEL = CLKSEL & 0x80; //; disengage PLL from system
  PLLCTL = PLLCTL | 0x40; //; turn on PLL
  SYNR = 0x02;            //; set PLL multiplier
  REFDV = 0;              //; set PLL divider
  while (!(CRGFLG & 0x08)){  }
  CLKSEL = CLKSEL | 0x80; //; engage PLL

/* Disable watchdog timer (COPCTL register) */
  COPCTL = 0x40   ; //COP off; RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, interrupts off initially */
  SCIBDH =  0x00; //set baud rate to 9600
  SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)  
  SCICR1 =  0x00; //$9C = 156
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port

/* Initialize peripherals */
  DDRT = 0xFF; 

/* Initialize interrupts */
  CRGINT = 0x80;
  RTICTL = 0x1F;
  
TSCR1 = 0x80;
  TSCR2 = 0x0C;
  TIOS  = 0x80;
  TIE   = 0x00; // = 0x80 to turn on
  TC7   = 15000; //set interrupt every 10ms      
MODRR  = 0x0F;
  PWME   = 0x00;  //
  PWMPOL = 0x0F;
  PWMCTL = 0x00;
  PWMCAE = 0x00;
  
  PWMPER0 = 0xFF;  
  PWMPER1 = 0xFF;  
  PWMPER2 = 0xFF;
  PWMPER3 = 0xFF;

  PWMDTY0 = 0x00;  
  PWMDTY1 = 0x00;
  PWMDTY2 = 0x00; //initially stoppeed
  PWMDTY3 = 0x00;
  
  PWMCLK = 0x0F;
  PWMPRCLK = 0x11;
  
  PWMSCLA = 0xEB;
  PWMSCLB = 0xEB;     
  
  ATDDIEN = 0xC0;     
  ATDCTL2 = 0xC0; // power up ATD
  ATDCTL3 = 0x28; //set conversion sequence length to two
  ATDCTL4 = 0x85;  
}

    
/*     
***********************************************************************
Main
***********************************************************************
*/
void main(void) {
  DisableInterrupts;
  initializations();     
  EnableInterrupts;

  
/* < start of your main loop > */ 
  while(!rghtpb) {}
  rghtpb = 0;
  PTT_PTT4 = 0;
  PTT_PTT5 = 0;  
  PTT_PTT6 = 0;
  PTT_PTT7 = 0;

  recursion(NORTH,NORTH,0,0);
    
    /* loop forever */
  for(;;) {}   
}   /* do not leave main */

//*****************************RECURSIVE FUNCTION*************************************************************************************
void recursion(const char current_direction, const char previous_direction, short unsigned int stack, char at_junction){
  long unsigned int elapsed = 0;
  const char reversed_direction = flip_direction(current_direction);

  TCNT = 0;
  TIE = 0x80;
  for(;;){
    //emergency stop
    if(leftpb){
     leftpb = 0;
     pwm_config(0,0,0,0);
     TIE = 0;
     for(;;){}
    }
    //if route available on the left
    if(left_explore_flag){

      //stop timer and store time
      TIE = 0;
      elapsed = global_elapsed;
      global_elapsed = 0;
      
      at_junction = 1;
      left_explore_flag = 0;      
      PTT_PTT4 = 1;
      PTT_PTT5 = 0;
      PTT_PTT6 = 0;
      PTT_PTT7 = 0;            

      //enter junction 
      timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,0,MOTOR_ON,JUNCTION_ENTER,!SELF_ALIGN_MODE);
      force_delay(20);

      //left turn
      timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,MOTOR_ON,0,TURN_LEFT_DURATION,!SELF_ALIGN_MODE);      
      force_delay(10);

      //exit junction
      timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,0,MOTOR_ON,JUNCTION_EXIT,!SELF_ALIGN_MODE);
      force_delay(20);
      
      PWME = 0;
      recursion(left_turn_direction(current_direction),current_direction,stack+1,0);
      //just returned from left route, don't check left direction again!
      left_sample_switch = 0;
      //reset and start timer
      TCNT = 0;
      TIE = 0x80; 
    }
    //if route available on the right
    if(right_explore_flag){
            
      TIE = 0;
      elapsed = global_elapsed;
      global_elapsed = 0;
      
      at_junction = 1;
      right_explore_flag = 0;      

      PTT_PTT5 = 1;
      PTT_PTT4 = 0;
      PTT_PTT6 = 0;
      PTT_PTT7 = 0;      
      
      timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,0,MOTOR_ON,JUNCTION_ENTER,!SELF_ALIGN_MODE);
      force_delay(20);
      
      timed_movement(&timer_turn_flag,&turned_flag,0,MOTOR_ON,0,MOTOR_ON,TURN_RIGHT_DURATION,!SELF_ALIGN_MODE);
      force_delay(10);      
      
      timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,0,MOTOR_ON,JUNCTION_EXIT,!SELF_ALIGN_MODE);
      force_delay(20);            
      
      PWME = 0;
      recursion(right_turn_direction(current_direction),current_direction,stack+1,0);
      right_sample_switch = 0;
      TCNT = 0;
      TIE = 0x80; 
    }
    //if arrived at deadend
    if(deadend_flag){ 

      TIE = 0;
      elapsed = global_elapsed;
      global_elapsed = 0;
      TCNT = 0;
      PWME = 0;
      deadend_flag = 0;
      break; 
    }else{ //if forward is the only available route

      if(PWME == 0){ //if vehicle is stationary
        PTT_PTT4 = 1;
        PTT_PTT5 = 1;
        PTT_PTT6 = 1;
        PTT_PTT7 = 1;            
        pwm_config(1,0,0,1); //forward
        force_delay(4);
        left_sample_switch = 1;
        right_sample_switch = 1;
      }else{  //if vehicle is moving
        PTT_PTT4 = 0;
        PTT_PTT5 = 0;
        self_align();
      }
    }
  }
  global_elapsed = elapsed;
  timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,MOTOR_ON,0,TURN_180_DURATION,!SELF_ALIGN_MODE); //180 degrees turn
  if(at_junction){
    at_junction = 0;
    timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,0,MOTOR_ON,JUNCTION_EXIT,!SELF_ALIGN_MODE);      
  }
  //travel back to however long it took you to get here, self aligning along the way
  timed_movement(&timer_return_junction_flag,&returned_junction_flag,0,0,0,0,0,SELF_ALIGN_MODE);/* motor configuration and turn duration not needed when in self align mode*/
  timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,0,MOTOR_ON,JUNCTION_ENTER,!SELF_ALIGN_MODE);
  force_delay(10);
  pwm_config(0,0,0,0);
  if(stack != 0){
    original_direction(previous_direction,reversed_direction);
    self_align();
    pwm_config(0,0,0,0);
  }
}
//****************************************************************************************************************************************
void original_direction(const char prev_direction, const char rever_direction){
  switch(prev_direction){
    case NORTH: if(rever_direction == EAST) timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,MOTOR_ON,0,TURN_LEFT_DURATION,!SELF_ALIGN_MODE);
                else if(rever_direction == WEST) timed_movement(&timer_turn_flag,&turned_flag,0,MOTOR_ON,0,MOTOR_ON,TURN_RIGHT_DURATION,!SELF_ALIGN_MODE);
                else if(rever_direction == SOUTH) timed_movement(&timer_turn_flag,&turned_flag,0,MOTOR_ON,0,MOTOR_ON,TURN_180_DURATION,!SELF_ALIGN_MODE);
                break;
    case SOUTH: if(rever_direction == EAST) timed_movement(&timer_turn_flag,&turned_flag,0,MOTOR_ON,0,MOTOR_ON,TURN_RIGHT_DURATION,!SELF_ALIGN_MODE);
                else if(rever_direction == WEST) timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,MOTOR_ON,0,TURN_LEFT_DURATION,!SELF_ALIGN_MODE);
                else if(rever_direction == NORTH) timed_movement(&timer_turn_flag,&turned_flag,0,MOTOR_ON,0,MOTOR_ON,TURN_180_DURATION,!SELF_ALIGN_MODE);
                break;
    case EAST: if(rever_direction == NORTH) timed_movement(&timer_turn_flag,&turned_flag,0,MOTOR_ON,0,MOTOR_ON,TURN_RIGHT_DURATION,!SELF_ALIGN_MODE);
                else if(rever_direction == WEST) timed_movement(&timer_turn_flag,&turned_flag,0,MOTOR_ON,0,MOTOR_ON,TURN_180_DURATION,!SELF_ALIGN_MODE);
                else if(rever_direction == SOUTH) timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,MOTOR_ON,0,TURN_LEFT_DURATION,!SELF_ALIGN_MODE);
                break;
    case WEST: if(rever_direction == EAST) timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,MOTOR_ON,0,TURN_180_DURATION,!SELF_ALIGN_MODE);
                else if(rever_direction == NORTH) timed_movement(&timer_turn_flag,&turned_flag,MOTOR_ON,0,MOTOR_ON,0,TURN_LEFT_DURATION,!SELF_ALIGN_MODE);
                else if(rever_direction == SOUTH) timed_movement(&timer_turn_flag,&turned_flag,0,MOTOR_ON,0,MOTOR_ON,TURN_RIGHT_DURATION,!SELF_ALIGN_MODE);                                                
                break;
  }
}

const char left_turn_direction(const char direction){
  if(direction == NORTH) return WEST;
  else if(direction == SOUTH) return EAST;
  else if(direction == EAST) return NORTH;
  else return SOUTH;
}

const char right_turn_direction(const char direction){
  if(direction == NORTH) return EAST;
  else if(direction == SOUTH) return WEST;
  else if(direction == EAST) return SOUTH;
  else return NORTH;
}

const char flip_direction(const char direction){
  if(direction == NORTH) return SOUTH;
  else if(direction == SOUTH) return NORTH;
  else if(direction == WEST) return EAST;
  else return WEST;  
}

void force_delay(short unsigned int x){
  while(x != 0){
    for(i = 0; i < 63000; i++){}
    x--;
  }
}

void self_align(){
  ATDCTL5 = 0x10;  
  while((!ATDSTAT0_SCF)) {}
  if((ATDDR3H) > SIDEWALL_THRESH){ //left  
    PTT_PTT6 = 1;
    PTT_PTT7 = 0;
    pwm_config(0,0,0,1);
    force_delay(2);  
  } 
  
  if((ATDDR4H) > SIDEWALL_THRESH){ //right       
    PTT_PTT6 = 0;
    PTT_PTT7 = 1;
    pwm_config(1,0,0,0);
    force_delay(2);
  }
  pwm_config(1,0,0,1);
}

//*****************************TIMED MOVEMENT FUNCTION************************************************************************************
void timed_movement(char *timer_flag, char *task_completed_flag, char pin0, char pin1, char pin2, char pin3, unsigned int turn_duration_count, char mode){
  TCNT = 0;
  *timer_flag = 1;
  TC7 = 0xFFFF;
  turn_duration = turn_duration_count;
  
  PWME = 0x0F;
  if(!mode){//mode is zero, turning movement
    pwm_config(pin0,pin1,pin2,pin3);
    TIE = 0x80;
    while(!(*task_completed_flag)){}
  }else{
    TIE = 0x80;
    while(!(*task_completed_flag)){
      self_align();
    }
  } 
  pwm_config(0,0,0,0);
  TIE = 0;
  TCNT = 0;
  *timer_flag = 0;
  *task_completed_flag = 0;
  tcnt_count = 0;
  
  TC7 = 15000;
}
//****************************************************************************************************************************************

//*****************************MOTOR FORWARD, LEFT, RIGHT, REVERSE FUNCTION***************************************************************
void pwm_config(char pin0, char pin1, char pin2, char pin3){
  //turn on 0 and 3 to move forward, 0 == right, 3 == left
  ATDCTL5 = 0x10;  
  while((!ATDSTAT0_SCF)) {}
  PWME = (pin0 || pin1 || pin2 || pin3) ? 0x0F : 0;  
  PWMDTY0 = pin0 ? ATDDR0H : 0;
  PWMDTY1 = pin1 ? ATDDR0H : 0;  
  PWMDTY2 = pin2 ? ATDDR0H : 0;  
  PWMDTY3 = pin3 ? ATDDR1H : 0;  
}
//****************************************************************************************************************************************

/*
***********************************************************************                       
 RTI interrupt service routine: RTI_ISR
************************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  // clear RTI interrupt flagt 
  CRGFLG = CRGFLG | 0x80; 
 
    if(PORTAD0_PTAD7 == 0) { // check left pushbutton
      if(prevl == 1) {
        leftpb = 1;
      }
    }
    prevl = PORTAD0_PTAD7;
    
    if(PORTAD0_PTAD6 == 0) { // check right pushbutton
      if(prevr == 1) {
        rghtpb = 1;
      }
    }
    prevr = PORTAD0_PTAD6;         
}

/*
***********************************************************************                       
  TIM interrupt service routine   
***********************************************************************
*/

interrupt 15 void TIM_ISR(void)
{
  // clear TIM CH 7 interrupt flag 
  TFLG1 = TFLG1 | 0x80; 
   
  if(timer_turn_flag){ //turn 90 degrees left
    if(tcnt_count == turn_duration){
      turned_flag = 1;
    }else{
      tcnt_count++;
    }
  }else if(timer_return_junction_flag){//returning to the most recently visited junction
    if(tcnt_count == (global_elapsed * 1000 / 0xFFFF)){
      returned_junction_flag = 1;  
    }else{
      tcnt_count++; 
    }
   
  }else{//normal interrupt, operate sensors
     global_elapsed += 15;//15 because of the value of TC7, will multiply 1000 later
   
     if(((++tencnt) % 11) == 10){
       tenths = 1;  
     }
   
     if(tenths){//sample data every tenths second
       tenths = 0;                    
       ATDCTL5 = 0x10;  
       while((!ATDSTAT0_SCF)) {}
       left_ir_data = ATDDR3H;
       right_ir_data = ATDDR4H;     
       front_ir_data = ATDDR2H;

       if(left_sample_switch && ((left_ir_data) < JUNCTION_THRESH)){
         pwm_config(0,0,0,0);
         left_explore_flag = 1; 
       }else if(right_sample_switch && ((right_ir_data) < JUNCTION_THRESH)){
         pwm_config(0,0,0,0);
         right_explore_flag = 1; 
       }
       else if(front_ir_data > DEADEND_THRESH){
         pwm_config(0,0,0,0);
         deadend_flag = 1; 
       }
     }
  }
}

/*
***********************************************************************                       
  SCI interrupt service routine   
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{
}

/*
***********************************************************************
 Character I/O Library Routines for 9S12C32 
***********************************************************************
 Name:         inchar
 Description:  inputs ASCII character from SCI serial port and returns it
 Example:      char ch1 = inchar();
***********************************************************************
*/

char inchar(void) {
  /* receives character from the terminal channel */
        while (!(SCISR1 & 0x20)); /* wait for input */
    return SCIDRL;
}

/*
***********************************************************************
 Name:         outchar    (use only for DEBUGGING purposes)
 Description:  outputs ASCII character x to SCI serial port
 Example:      outchar('x');
***********************************************************************
*/

void outchar(char x) {
  /* sends a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for output buffer empty */
    SCIDRL = x;
}
