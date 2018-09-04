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

#define DEVICE_ID 0xF3
#define I2C_slave_address 0x24 // any value from 0 to 127
 
 volatile union _I2C_buffer 
 {
    struct _data 
    {
        unsigned char ID;
        unsigned char ADDRESS;
        unsigned char START_STOP;
        unsigned char IOWPU;
        unsigned char MODE;
        unsigned char SAVE;
        unsigned char RESET;
        unsigned int GEAR_RATIO;
        signed int DIAMETER;
        signed int RPM;
        signed int SPEED;
        long DISTANCE;
        float RPM_PID_KP;
        float RPM_PID_KD;
        float RPM_PID_KI;
        float ATS_PID_KP;
        float ATS_PID_KD;
        float ATS_PID_KI;
    } data; 
 unsigned char byte[];
 } I2C_buffer;

 
unsigned char first_i2c = 1;     
unsigned char index_i2c = 0; 
const unsigned char RX_ELMNTS = sizeof(I2C_buffer);

// Interrupts

void __interrupt isr() 
{
   static unsigned char junk = 0;  
   
   if (PIR1bits.SSP1IF)                              // check to see if SSP interrupt
    {
        if (SSP1STATbits.R_nW)               // Master read (R_nW = 1)
        {
            if (!SSP1STATbits.D_nA)        // Last byte was an address (D_nA = 0)
            {
                SSP1BUF = I2C_buffer.byte[index_i2c++]; // load with value from array
                SSP1CON1bits.CKP = 1;             // Release CLK
            }
            if (SSP1STATbits.D_nA)               // Last byte was data (D_nA = 1)
            {
                SSP1BUF = I2C_buffer.byte[index_i2c++]; // load with value from array
                SSP1CON1bits.CKP = 1;             // Release CLK
            }

        }
        if (!SSP1STATbits.R_nW) //  Master write (R_nW = 0)
        {
            if (!SSP1STATbits.D_nA) // Last byte was an address (D_nA = 0)
            {
                first_i2c = 1; //last byte was address, next will be data location
                junk = SSP1BUF;                  // read buffer to clear BF
                SSP1CON1bits.CKP = 1;            // Release CLK
            }
            if (SSP1STATbits.D_nA)               // Last byte was data (D_nA = 1)
            {
                if (first_i2c) 
                {
                    index_i2c = SSP1BUF;      // load index with array location
                    first_i2c = 0;               // now clear this since we have 
                }                            //location to read from/write to
                
                else
                {
                    if (index_i2c < RX_ELMNTS)       // make sure index is not
                    {                                //out of range of array
                        I2C_buffer.byte[index_i2c++] = SSP1BUF; //load array with data
                    } 
                    else
                    {
                        junk = SSP1BUF; //array location not valid, discard data
                    }
                }
                if (SSP1CON1bits.WCOL)           // Did a write collision occur?
                {
                    SSP1CON1bits.WCOL = 0;       //  clear WCOL
                    junk = SSP1BUF;              // dummy read to clear BF bit
                }
                SSP1CON1bits.CKP = 1;            // Release CLK
            }
        }
      PIR1bits.SSP1IF = 0;                              // clear SSPIF flag bit
    }
    if (PIR2bits.BCL1IF)                                  // Did a bus collision occur?
    {
        junk = SSP1BUF;                      // dummy read SSPBUF to clear BF bit
        PIR2bits.BCL1IF = 0;                          // clear bus collision Int Flag bit
        SSP1CON1bits.CKP = 1;                // Release CLK
        PIR1bits.SSP1IF = 0;                              // clear SSPIF flag bit
    }
}

void init_I2C_buffer()
{
    I2C_buffer.data.ID = DEVICE_ID;
    I2C_buffer.data.ADDRESS = I2C_slave_address << 1;
    I2C_buffer.data.START_STOP = 0;
    I2C_buffer.data.IOWPU = 1;
    I2C_buffer.data.MODE = 5;
    I2C_buffer.data.SAVE = 0;
    I2C_buffer.data.RESET = 0;
    I2C_buffer.data.GEAR_RATIO = 150;
    I2C_buffer.data.DIAMETER = 42;
    I2C_buffer.data.RPM = 1200;
    I2C_buffer.data.SPEED = 1150;
    I2C_buffer.data.DISTANCE = 5555;
    I2C_buffer.data.RPM_PID_KP = 0.1;
    I2C_buffer.data.RPM_PID_KD = 0.01;
    I2C_buffer.data.RPM_PID_KI = 0.04;
    I2C_buffer.data.ATS_PID_KP = 0.15;
    I2C_buffer.data.ATS_PID_KD = 0.08;
    I2C_buffer.data.ATS_PID_KI = 0.03;
}

// Main program

void main() 
{
    OSCCON = 0b11110000; //configure internal oscilator for 32Mhz
    init_I2C_buffer();
    TRISA = 0b00011110; //configure IO
    ANSELA = 0b00000000; //analog functions of pins disabled
    WPUA = 0b00011110; //configure weak pull-ups on input pins
    OPTION_REGbits.nWPUEN = 0; //enable weak pull-ups
    APFCONbits.CCP1SEL = 1; //select RA5 as CCP output pin
    LATAbits.LATA0 = 0; //put motor direction pin to low
    SSP1STAT = 0b10000000;       // Slew rate control disabled for standardspeed mode (100 kHz and 1 MHz)
    SSP1CON1 = 0b00110110; 		// Enable serial port, I2C slave mode, 7-bit address
    SSP1CON2bits.SEN = 1;        // Clock stretching is enabled
    SSP1CON3bits.BOEN = 1;       // SSPBUF is updated and NACK is generated for a received address/data byte, ignoring the state of the SSPOV bit only if the BF bit = 0
    SSP1CON3bits.SDAHT = 1;		// Minimum of 300 ns hold time on SDA after the falling edge of SCL
    SSP1CON3bits.SBCDE = 1;		// Enable slave bus collision detect interrupts
    SSP1ADD = I2C_buffer.data.ADDRESS; // Load the slave address
    PIR1bits.SSP1IF = 0;                  // Clear the serial port interrupt flag
    PIR2bits.BCL1IF = 0;                  // Clear the bus collision interrupt flag
    PIE2bits.BCL1IE = 1;                  // Enable bus collision interrupts
    PIE1bits.SSP1IE = 1;                  // Enable serial port interrupts
    INTCONbits.PEIE = 1;                   // Enable peripheral interrupts
    //IOCANbits.IOCAN4 = 1; //configure interrupt on falling edge for rpm meter
    INTCON = 0b01001000; //enables interrupts
    T1CON = 0b00110100; //configure timer1 to run at 1 MHz
    //PIE1bits.TMR1IE = 1; //enable timer1 interrupt
    //T1CONbits.TMR1ON = 1; //start timer1
    INTCONbits.GIE = 1; //run interrupts 
        
    while(1)                    // main while() loop
    {                           // program will wait here for ISR to be called
        //asm("CLRWDT");          // clear WDT
    }

}


