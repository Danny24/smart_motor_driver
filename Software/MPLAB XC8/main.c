/* 
 * File:   main.c
 * Author: dannimakes
 *
 * Created on 8 de julio de 2018, 02:50 PM
 */
// PIC12LF1840 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#define _XTAL_FREQ = 32000000

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
//#include <pic12f1822.h>


// Global variables declation

unsigned short counter = 0;
unsigned int gear = 150;
int rpm = 0;
float kp = 0.4;        //0.1
float kd = 0.0001;     //0.001
float ki = 0.002;      //0.04
float accumulator = 0;
float lasterror = 0;



PWM_Init(void) // start the PWM at 50,000 Hz with a 0% duty
{
PR2 = 39; //load value to archieve desired frequency (check the excel file for calculations)
CCP1CON = 0b00001100; //configure CCP1 for PWM operation
CCPR1L = 0b00000000; //set duty to 0    
PIR1bits.TMR2IF = 0; //clear the interrupt flag
T2CON = 0b00000101; //start timer 2 and select prescaler as 4
}


PWM_set_duty(int duty) // change the duty of PWM
{
  if(duty<1024)
  {
    //CCPR1L = (unsigned int)duty>>2; //discard 2 LSB and write the rest to register
    //CCP1CON = 48 & (((unsigned int)duty) & 3); //filter the reamaining 2 LSB and then write it to DC1B (bit 5,4 in the register)
    CCPR1L = (0xFF & ((unsigned int)duty));
  }
}

// Interrupts

void  __interrupt all_of_them (void)
{
 if (INTCONbits.IOCIF == 1 && IOCAFbits.IOCAF4 == 1) //interrupt on change on RA4 triggered
 {
  INTCONbits.IOCIE = 0; //disable on change interrupts
  counter ++;  //increment counter one in one
  IOCAFbits.IOCAF4 = 0; //clear interrupt flags
  INTCONbits.IOCIE = 1; //enable on change interrupts
 }
 if(PIR1bits.TMR1IF == 1) //timer1 interrupt, called every 65.536ms
 {
  INTCONbits.IOCIE = 0; //disable on change interrupts
  T1CONbits.TMR1ON =  0; //stop timer1
  rpm = (counter * 300)/gear;  //calculate rpm  (multiplied 15 interrupts in 1 second, divided 3 encoder interrupts per lap, multiplied by 60 to convert to minutes, divided by gear ratio)
  counter = 0;  //clear counter
  if(LATAbits.LATA0 == 0)
  {
    rpm = rpm *-1;
  }
  INTCONbits.IOCIE = 1; //enable on change interrupts
  PIR1bits.TMR1IF = 0; //clear interrutp flag
  T1CONbits.TMR1ON =  1; //start timer1
 }
}

// Functions

void M_control(int ctr) //motor control function
{
   if (ctr == 0) //stop the motor
   {
     PWM_set_duty(ctr);
   }
   if (ctr > 0)  //clockwise turn set and set the pwm duty
   {
     LATAbits.LATA0 = 1;
     PWM_set_duty(ctr);
   }
   if (ctr < 0)  //counter clockwise turn set and set the pwm duty
   {
     LATAbits.LATA0 = 0;
     ctr = ctr * -1; //turn value positive before send in to PWM
     PWM_set_duty(ctr);
   }
}

void PID(int set) //PID calculation function
{
  float error = 0;
  float pid = 0;
  int rpm2 = rpm + 600;
  set = set + 600;
  error = set-rpm2; //calculate actual error
  pid = error*kp;     // calculate proportional gain
  accumulator += error;  // calculate accumulator, is sum of errors
  pid += ki*accumulator; // add integral gain and error accumulator
  pid += kd*(error-lasterror); //add differential gain
  lasterror = error; //save the error to the next iteration
  if(pid >= 511)   //next we guarantee that the PID value is in range
  {
    pid = 511;
  }
  if(pid <= 0)
  {
    pid = 0;
  }
  pid = (-255+((510)*((pid)/(511)))); //scale PID result from 0,511 to -255,255
  if(set < 600)
  {
   if(pid > 0)
   {
   pid = 0;
   }
  }
  if(set > 600)
  {
   if(pid < 0)
   {
   pid = 0;
   }
  }
  M_control((int)PID);
}


// Main program

int main(int argc, char** argv) 
{
OSCCON = 0b11110000; //configure internal oscilator for 32Mhz
TRISA = 0b00011000;  //configure IO
ANSELA = 0b00000000; //analog functions of pins disabled
WPUA = 0b00011110;   //configure weak pull-ups on input pins
OPTION_REGbits.nWPUEN = 0;   //enable weak pull-ups
APFCONbits.CCP1SEL = 1;       //select RA5 as CCP output pin
LATAbits.LATA0 = 0;         //put motor direction pin to low
PWM_Init();    //start pwm
PWM_set_duty(0);    //put duty of pwm to 0
IOCANbits.IOCAN4 = 1;        //configure interrupt on falling edge for rpm meter
INTCON = 0b01001000; //enables interrupts
T1CON = 0b00110100;  //configure timer1 to run at 1 MHz
PIE1bits.TMR1IE =  1;        //enable timer1 interrupt
T1CONbits.TMR1ON =  1;       //start timer1
INTCONbits.IOCIF = 1;       //run interrupts



while(1)
{
for(int x=0;x<10;x++)
{
PID(50);
_delay(1);
}
for(int x=0;x<5;x++)
{
PID(0);
_delay(1);
}
for(int x=0;x<10;x++)
{
PID(-150);
_delay(1);
}
for(int x=0;x<5;x++)
{
PID(0);
_delay(1);
}
}

    return (EXIT_SUCCESS);
}


