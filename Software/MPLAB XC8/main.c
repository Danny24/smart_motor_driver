/* 
 * File:   main.c
 * Author: dannimakes
 *
 * Created on July 8, 2018.
 */
// PIC12LF1840 Configuration Bit Settings

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

// Includes and definitions

#define _XTAL_FREQ 32000000

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

// Global variables declation

unsigned short counter = 0;
const unsigned short gear = 150;
int rpm = 0;
int lset = 0;
bit zero_cross = 0;
const float kp = 0.1; //0.3
const float kd = 0.01; //0.001
const float ki = 0.04; //0.1
float accumulator = 0;
float lasterror = 0;

// Interrupts

void __interrupt isr() 
{
    if (INTCONbits.IOCIF == 1 && IOCAFbits.IOCAF4 == 1) //interrupt on change on RA4 triggered
    {
        INTCONbits.IOCIE = 0; //disable on change interrupts
        counter++; //increment counter one in one
        IOCAFbits.IOCAF4 = 0; //clear interrupt flag
        INTCONbits.IOCIE = 1; //enable on change interrupts
    }
    if (PIR1bits.TMR1IF == 1) //timer1 interrupt, called every 65.536ms
    {
        INTCONbits.IOCIE = 0; //disable on change interrupts
        T1CONbits.TMR1ON = 0; //stop timer1
        rpm = (counter * 300) / gear; //calculate rpm  (multiplied 15 interrupts in 1 second, divided 3 encoder interrupts per lap, multiplied by 60 to convert to minutes, divided by gear ratio)
        counter = 0; //clear counter
        if (LATAbits.LATA0 == 0) 
        {
            rpm = rpm *-1;
        }
        INTCONbits.IOCIE = 1; //enable on change interrupts
        PIR1bits.TMR1IF = 0; //clear interrutp flag
        T1CONbits.TMR1ON = 1; //start timer1
    }
}

// Functions

PWM_Init(void) // start the PWM with a 0% duty
{
    /*
    //10 bit mode PWM at 1.95kHz
    PR2 = 0xFF; //load PR2 register
    CCP1CON = 0b00001100; //configure CCP1 for PWM operation
    CCPR1L = 0b00000000; //set duty to 0    
    PIR1bits.TMR2IF = 0; //clear the interrupt flag
    T2CON = 0b00000110; //start timer 2 and select prescaler as 4
     */

    /*    
    //10 bit mode PWM at 7.81kHz
    PR2 = 0xFF; //load PR2 register
    CCP1CON = 0b00001100; //configure CCP1 for PWM operation
    CCPR1L = 0b00000000; //set duty to 0    
    PIR1bits.TMR2IF = 0; //clear the interrupt flag
    T2CON = 0b00000101; //start timer 2 and select prescaler as 4
     */

    //10 bit mode PWM at 31.25kHz
    PR2 = 0xFF; //load PR2 register
    CCP1CON = 0b00001100; //configure CCP1 for PWM operation
    CCPR1L = 0b00000000; //set duty to 0    
    PIR1bits.TMR2IF = 0; //clear the interrupt flag
    T2CON = 0b00000100; //start timer 2 and select prescaler as 1

    /*
    //8 bit mode PWM at 125kHz
    PR2 = 0x3F; //load PR2 register
    CCP1CON = 0b00001100; //configure CCP1 for PWM operation
    CCPR1L = 0b00000000; //set duty to 0    
    PIR1bits.TMR2IF = 0; //clear the interrupt flag
    T2CON = 0b00000100; //start timer 2 and select prescaler as 1
     * */
}

PWM_set_duty(int duty) // change the duty of PWM
{
    if (duty < 1024) {
        CCPR1L = (0xFF & ((unsigned int) duty >> 2)); //discard 2 LSB and write the rest to register, 10 bit mode
        CCP1CON = (0x0C | (0x30 & ((unsigned int) duty << 4))); //filter the reamaining 2 LSB and then write it to DC1B (bit 5,4 in the register), 10 bit mode
        //CCPR1L = (0xFF & ((unsigned int)duty)); //to be used in 8 bit mode
    }
}

void M_control(int ctr) //motor control function
{
    if (ctr == 0) //stop the motor
    {
        PWM_set_duty(ctr);
    }
    if (ctr > 0) //clockwise turn set and set the pwm duty
    {
        LATAbits.LATA0 = 1;
        PWM_set_duty(ctr);
    }
    if (ctr < 0) //counter clockwise turn set and set the pwm duty
    {
        LATAbits.LATA0 = 0;
        ctr = ctr * -1; //turn value positive before send in to PWM
        PWM_set_duty(ctr);
    }
}

void calculate_pid(int set) //PID calculation function
{
    float error = 0;
    float pid = 0;
    int rpm2 = rpm + 600;
    set = set + 600;
    error = set - rpm2; //calculate actual error
    pid = error*kp; // calculate proportional gain
    accumulator += error; // calculate accumulator, is sum of errors
    pid += ki*accumulator; // add integral gain and error accumulator
    pid += kd * (error - lasterror); //add differential gain
    lasterror = error; //save the error to the next iteration
    if (pid >= 2047) //next we guarantee that the PID value is in range
    {
        pid = 2047;
    }
    if (pid <= 0) 
    {
        pid = 0;
    }
    pid = (-1023 + ((2046)*((pid) / (2047)))); //scale PID result from 0,2047 to -1023,1023 
    if (set < 600) //ignore half fordward values for backward
    {
        if (pid > 0) 
        {
            pid = 0;
        }
    }
    if (set > 600) //and viceversa, this prevents unwanted direction toggle at start-up
    {
        if (pid < 0) 
        {
            pid = 0;
        }
    }
    M_control((int) pid);
}

void PID(int set) //pre PID
{
    zero_cross = ((lset^set) < 0); //test if a set value has change direction wihout stoping at zero cross
    if (((zero_cross == 1) &(set <= 0) & (lset <= 0))) //discard some extra true cases when trabnsition from negative to posite direction
    {
        zero_cross = 0;
    }
    if (zero_cross == 1) 
    {
        do 
        {
            calculate_pid(0);
            __delay_ms(10);
        } 
        while ((rpm != 0)); //wait until the motor stops completely with PID before ressuming direction change
    }
    lset = set;
    calculate_pid(set);
}

// Main program

int main(int argc, char** argv) {
    OSCCON = 0b11110000; //configure internal oscilator for 32Mhz
    TRISA = 0b00011000; //configure IO
    ANSELA = 0b00000000; //analog functions of pins disabled
    WPUA = 0b00011110; //configure weak pull-ups on input pins
    OPTION_REGbits.nWPUEN = 0; //enable weak pull-ups
    APFCONbits.CCP1SEL = 1; //select RA5 as CCP output pin
    LATAbits.LATA0 = 0; //put motor direction pin to low
    PWM_Init(); //start pwm
    PWM_set_duty(0); //put duty of pwm to 0
    IOCANbits.IOCAN4 = 1; //configure interrupt on falling edge for rpm meter
    INTCON = 0b01001000; //enables interrupts
    T1CON = 0b00110100; //configure timer1 to run at 1 MHz
    PIE1bits.TMR1IE = 1; //enable timer1 interrupt
    T1CONbits.TMR1ON = 1; //start timer1
    INTCONbits.GIE = 1; //run interrupts

    while (1) 
    {
        for (int x = 0; x < 1000; x++)
        {
            PID(150);
            __delay_ms(10);
        }
        for (int x = 0; x < 1000; x++) 
        {
            PID(-150);
            __delay_ms(10);
        }
    }

    return (EXIT_SUCCESS);
}


