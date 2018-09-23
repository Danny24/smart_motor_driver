/* 
 * File:   main.c
 * Author: dannimakes
 *
 * Created on July 8, 2018.
 */
// PIC12LF1840 Configuration Bit Settings

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
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
#define DEVICE_ID 0xF3
#define time_lapse 5 //time in ms between updates in control loop
#define I2C_slave_address 0x24 // any value from 0 to 127, this will be the default
#define ATS_tolerance 10 //this tolerance is +/- from desired distance CPR, it prevents unwanted shakes and stuck up conditions
#define stable_time 50 //used for ATS, when distance reach it should be there this value multiplied by time_lapse value

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

//EEPROM default values and function prototypes for memory access on program

__EEPROM_DATA(I2C_slave_address, 0x01, 0x01, 0x00, 0x01, 0x00, 0xFF, 0xFF); //set i2c_address, IOWPU, Gear L-H, diameter L-H, 2 blank bytes
__EEPROM_DATA(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00); //PID_KP, PID_KD
__EEPROM_DATA(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00); //PID_KI, ATS_KP
__EEPROM_DATA(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00); //ATS_KD, ATS_KI
unsigned char eeprom_read(unsigned char address);
void eeprom_write(unsigned char address, unsigned char value);

// Global variables declation

volatile union _I2C_buffer {

    struct _data {
        unsigned char ID;
        unsigned char ADDRESS;
        unsigned char START_STOP;
        unsigned char IOWPU;
        unsigned char MODE;
        unsigned char SAVE;
        unsigned char RESET;
        unsigned int GEAR_RATIO;
        unsigned int DIAMETER;
        int RPM;
        int SPEED;
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

const unsigned char RX_ELMNTS = sizeof (I2C_buffer);
unsigned char first_i2c = 1;
unsigned char index_i2c = 0;
unsigned short counter = 0;
int lset = 0;
bit zero_cross = 0;
float accumulatorM = 0;
float lasterrorM = 0;
float accumulatorA = 0;
float lasterrorA = 0;
long auxDistance = 0;
bit loadDistance = 0;
int stable = 0;

// Interrupts

void __interrupt isr() {

    if (INTCONbits.IOCIF == 1 && IOCAFbits.IOCAF4 == 1) //interrupt on change on RA4 triggered
    {
        INTCONbits.IOCIE = 0; //disable on change interrupts
        counter++; //increment counter one in one
        if (LATAbits.LATA0 == 0) {
            I2C_buffer.data.DISTANCE--;
        } else {
            I2C_buffer.data.DISTANCE++;
        }
        IOCAFbits.IOCAF4 = 0; //clear interrupt flag
        INTCONbits.IOCIE = 1; //enable on change interrupts
    }

    if (PIR1bits.TMR1IF == 1) //timer1 interrupt, called every 65.536ms
    {
        INTCONbits.IOCIE = 0; //disable on change interrupts
        T1CONbits.TMR1ON = 0; //stop timer1
        I2C_buffer.data.RPM = (counter * 300) / I2C_buffer.data.GEAR_RATIO; //calculate rpm  (multiplied 15 interrupts in 1 second, divided 3 encoder interrupts per lap, multiplied by 60 to convert to minutes, divided by gear ratio)
        counter = 0; //clear counter
        if (LATAbits.LATA0 == 0) {
            I2C_buffer.data.RPM = I2C_buffer.data.RPM *-1;
        }
        INTCONbits.IOCIE = 1; //enable on change interrupts
        PIR1bits.TMR1IF = 0; //clear interrutp flag
        T1CONbits.TMR1ON = 1; //start timer1
    }

    static unsigned char junk = 0;

    if (PIR1bits.SSP1IF) // check to see if SSP interrupt
    {
        INTCONbits.IOCIE = 0; //disable on change interrupts
        PIE1bits.TMR1IE = 0; //disable timer1 interrupt
        if (SSP1STATbits.R_nW) // Master read (R_nW = 1)
        {
            if (!SSP1STATbits.D_nA) // Last byte was an address (D_nA = 0)
            {
                SSP1BUF = I2C_buffer.byte[index_i2c++]; // load with value from array
                SSP1CON1bits.CKP = 1; // Release CLK
            }
            if (SSP1STATbits.D_nA) // Last byte was data (D_nA = 1)
            {
                SSP1BUF = I2C_buffer.byte[index_i2c++]; // load with value from array
                SSP1CON1bits.CKP = 1; // Release CLK
            }
        }
        if (!SSP1STATbits.R_nW) //  Master write (R_nW = 0)
        {
            if (!SSP1STATbits.D_nA) // Last byte was an address (D_nA = 0)
            {
                first_i2c = 1; //last byte was address, next will be data location
                junk = SSP1BUF; // read buffer to clear BF
                SSP1CON1bits.CKP = 1; // Release CLK
            }
            if (SSP1STATbits.D_nA) // Last byte was data (D_nA = 1)
            {
                if (first_i2c) {
                    index_i2c = SSP1BUF; // load index with array location
                    first_i2c = 0; // now clear this since we have 
                }//location to read from/write to

                else {
                    if (index_i2c < RX_ELMNTS) // make sure index is not
                    { //out of range of array
                        I2C_buffer.byte[index_i2c++] = SSP1BUF; //load array with data
                    } else {
                        junk = SSP1BUF; //array location not valid, discard data
                    }
                }
                if (SSP1CON1bits.WCOL) // Did a write collision occur?
                {
                    SSP1CON1bits.WCOL = 0; //  clear WCOL
                    junk = SSP1BUF; // dummy read to clear BF bit
                }
                SSP1CON1bits.CKP = 1; // Release CLK
            }
        }
        PIR1bits.SSP1IF = 0; // clear SSPIF flag bit
        INTCONbits.IOCIE = 1; //enable on change interrupts
        PIE1bits.TMR1IE = 1; //enable timer1 interrupt
    }

    if (PIR2bits.BCL1IF) // Did a bus collision occur?
    {
        junk = SSP1BUF; // dummy read SSPBUF to clear BF bit
        PIR2bits.BCL1IF = 0; // clear bus collision Int Flag bit
        SSP1CON1bits.CKP = 1; // Release CLK
        PIR1bits.SSP1IF = 0; // clear SSPIF flag bit
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

int calculate_pidA(long set) //PID calculation function
{
    float error = 0;
    float pid = 0;
    error = set - I2C_buffer.data.DISTANCE; //calculate actual error
    pid = error * I2C_buffer.data.ATS_PID_KP; // calculate proportional gain
    accumulatorA += error; // calculate accumulator, is sum of errors
    pid += I2C_buffer.data.ATS_PID_KI*accumulatorA; // add integral gain and error accumulator
    pid += I2C_buffer.data.ATS_PID_KD * (error - lasterrorA); //add differential gain
    lasterrorA = error; //save the error to the next iteration
    if (pid >= abs(I2C_buffer.data.SPEED)) //next we guarantee that the PID value is in range
    {
        pid = abs(I2C_buffer.data.SPEED);
    }
    if (pid <= abs(I2C_buffer.data.SPEED)*-1) {
        pid = abs(I2C_buffer.data.SPEED)*-1;
    }
    return ((int) pid);
}

void calculate_pidM(int set) //PID calculation function
{
    float error = 0;
    float pid = 0;
    error = set - I2C_buffer.data.RPM; //calculate actual error
    pid = error * I2C_buffer.data.RPM_PID_KP; // calculate proportional gain
    accumulatorM += error; // calculate accumulator, is sum of errors
    pid += I2C_buffer.data.RPM_PID_KI*accumulatorM; // add integral gain and error accumulator
    pid += I2C_buffer.data.RPM_PID_KD * (error - lasterrorM); //add differential gain
    lasterrorM = error; //save the error to the next iteration
    if (pid >= 1023) //next we guarantee that the PID value is in range
    {
        pid = 1023;
    }
    if (pid <= -1023) {
        pid = -1023;
    }
    M_control((int) pid);
}

void pre_pidM(int set) //pre PID, allows detect zero cross for stoping motor before direction change
{
    zero_cross = ((lset^set) < 0); //test if a set value has change direction wihout stoping at zero cross
    if (((zero_cross == 1) &(set <= 0) & (lset <= 0))) //discard some extra true cases when trabnsition from negative to posite direction
    {
        zero_cross = 0;
    }
    if (zero_cross == 1) {
        do {
            calculate_pidM(0);
            asm("CLRWDT");
            __delay_ms(time_lapse);
        } while ((I2C_buffer.data.RPM != 0)); //wait until the motor stops completely with PID before ressuming direction change
    }
    lset = set;
    calculate_pidM(set);
}

void init_I2C_buffer() { //load default values of vars
    I2C_buffer.data.ID = DEVICE_ID;
    I2C_buffer.data.ADDRESS = eeprom_read(0);
    I2C_buffer.data.START_STOP = 0;
    I2C_buffer.data.IOWPU = eeprom_read(1);
    I2C_buffer.data.MODE = 0;
    I2C_buffer.data.SAVE = 0;
    I2C_buffer.data.RESET = 0;
    //I2C_buffer.data.GEAR_RATIO = 0;
    I2C_buffer.byte[0x07] = eeprom_read(2); //Recover GEAR_RATIO from EEPROM
    I2C_buffer.byte[0x08] = eeprom_read(3);
    //I2C_buffer.data.DIAMETER = 0;
    I2C_buffer.byte[0x09] = eeprom_read(4); //Recover DIAMETER from EEPROM
    I2C_buffer.byte[0x0A] = eeprom_read(5);
    I2C_buffer.data.RPM = 0;
    I2C_buffer.data.SPEED = 0;
    I2C_buffer.data.DISTANCE = 0;
    //I2C_buffer.data.RPM_PID_KP = 0; //0.3, 0.1
    I2C_buffer.byte[0x13] = eeprom_read(8); //Recover RPM_PID_KP from EEPROM
    I2C_buffer.byte[0x14] = eeprom_read(9);
    I2C_buffer.byte[0x15] = eeprom_read(10);
    I2C_buffer.byte[0x16] = eeprom_read(11);
    //I2C_buffer.data.RPM_PID_KD = 0; //0.001, 0.01
    I2C_buffer.byte[0x17] = eeprom_read(12); //Recover RPM_PID_KD from EEPROM
    I2C_buffer.byte[0x18] = eeprom_read(13);
    I2C_buffer.byte[0x19] = eeprom_read(14);
    I2C_buffer.byte[0x1A] = eeprom_read(15);
    //I2C_buffer.data.RPM_PID_KI = 0; //0.1, 0.04
    I2C_buffer.byte[0x1B] = eeprom_read(16); //Recover RPM_PID_KI from EEPROM
    I2C_buffer.byte[0x1C] = eeprom_read(17);
    I2C_buffer.byte[0x1D] = eeprom_read(18);
    I2C_buffer.byte[0x1E] = eeprom_read(19);
    //I2C_buffer.data.ATS_PID_KP = 0;
    I2C_buffer.byte[0x1F] = eeprom_read(20); //Recover ATS_PID_KP from EEPROM
    I2C_buffer.byte[0x20] = eeprom_read(21);
    I2C_buffer.byte[0x21] = eeprom_read(22);
    I2C_buffer.byte[0x22] = eeprom_read(23);
    //I2C_buffer.data.ATS_PID_KD = 0;
    I2C_buffer.byte[0x23] = eeprom_read(24); //Recover ATS_PID_KD from EEPROM
    I2C_buffer.byte[0x24] = eeprom_read(25);
    I2C_buffer.byte[0x25] = eeprom_read(26);
    I2C_buffer.byte[0x26] = eeprom_read(27);
    //I2C_buffer.data.ATS_PID_KI = 0;
    I2C_buffer.byte[0x27] = eeprom_read(28); //Recover ATS_PID_KI from EEPROM
    I2C_buffer.byte[0x28] = eeprom_read(29);
    I2C_buffer.byte[0x29] = eeprom_read(30);
    I2C_buffer.byte[0x2A] = eeprom_read(31);
}

// Main program

void main() {
    OSCCON = 0b11110000; //configure internal oscilator for 32Mhz
    init_I2C_buffer(); //load default values and configurations
    TRISA = 0b00011110; //configure IO
    ANSELA = 0b00000000; //analog functions of pins disabled
    WPUA = 0b00011110; //configure weak pull-ups on input pins
    OPTION_REGbits.nWPUEN = (char) !(I2C_buffer.data.IOWPU & 0x01); //enable/disable weak pull-ups
    APFCONbits.CCP1SEL = 1; //select RA5 as CCP output pin
    LATAbits.LATA0 = 0; //put motor direction pin to low
    SSP1STAT = 0b10000000; // Slew rate control disabled for standardspeed mode (100 kHz and 1 MHz)
    SSP1CON1 = 0b00110110; // Enable serial port, I2C slave mode, 7-bit address
    SSP1CON2bits.SEN = 1; // Clock stretching is enabled
    SSP1CON3bits.BOEN = 1; // SSPBUF is updated and NACK is generated for a received address/data byte, ignoring the state of the SSPOV bit only if the BF bit = 0
    SSP1CON3bits.SDAHT = 1; // Minimum of 300 ns hold time on SDA after the falling edge of SCL
    SSP1CON3bits.SBCDE = 1; // Enable slave bus collision detect interrupts
    SSP1ADD = (char) (I2C_buffer.data.ADDRESS << 1); // Load the slave address
    PIR1bits.SSP1IF = 0; // Clear the serial port interrupt flag
    PIR2bits.BCL1IF = 0; // Clear the bus collision interrupt flag
    PIE2bits.BCL1IE = 1; // Enable bus collision interrupts
    PIE1bits.SSP1IE = 1; // Enable serial port interrupts
    INTCONbits.PEIE = 1; // Enable peripheral interrupts
    PWM_Init(); //start pwm
    PWM_set_duty(0); //put duty of pwm to 0
    IOCANbits.IOCAN4 = 1; //configure interrupt on falling edge for rpm meter
    INTCON = 0b01001000; //enables interrupts
    T1CON = 0b00110100; //configure timer1 to run at 1 MHz
    PIE1bits.TMR1IE = 1; //enable timer1 interrupt
    T1CONbits.TMR1ON = 1; //start timer1
    INTCONbits.GIE = 1; //run interrupts

    __delay_ms(time_lapse);

    if (PORTAbits.RA3 == 0) { //if reset pin has trigger at boot the reset to default I2C address, this is required to unbrick the device         
        eeprom_write(0, I2C_slave_address);
        __delay_ms(time_lapse);
    }

    while (1) {
        asm("CLRWDT");
        if (I2C_buffer.data.START_STOP == 1) {
            switch (I2C_buffer.data.MODE) {
                case 0:
                    M_control(0);
                    break;
                case 1:
                    M_control(I2C_buffer.data.SPEED);
                    break;
                case 2:
                    calculate_pidM(I2C_buffer.data.SPEED);
                    break;
                case 3:
                    if (I2C_buffer.data.DISTANCE != 0 && loadDistance == 0) {
                        loadDistance = 1;
                        auxDistance = I2C_buffer.data.DISTANCE;
                        I2C_buffer.data.DISTANCE = 0;
                    } else {
                        if (auxDistance > 0) {
                            calculate_pidM(abs(I2C_buffer.data.SPEED));
                            if (auxDistance <= I2C_buffer.data.DISTANCE) {
                                loadDistance = 0;
                                M_control(0);
                                I2C_buffer.data.START_STOP = 0;
                            }
                        }
                        if (auxDistance < 0) {
                            calculate_pidM(abs(I2C_buffer.data.SPEED)*-1);
                            if (auxDistance >= I2C_buffer.data.DISTANCE) {
                                loadDistance = 0;
                                M_control(0);
                                I2C_buffer.data.START_STOP = 0;
                            }
                        }
                    }
                    break;
                case 4:
                    if (I2C_buffer.data.DISTANCE != 0 && loadDistance == 0) {
                        loadDistance = 1;
                        auxDistance = I2C_buffer.data.DISTANCE;
                        I2C_buffer.data.DISTANCE = 0;
                        stable = 0;
                    } else {
                        calculate_pidM(calculate_pidA(auxDistance));
                        if (auxDistance >= I2C_buffer.data.DISTANCE - ATS_tolerance && auxDistance <= I2C_buffer.data.DISTANCE + ATS_tolerance) {
                            stable++;
                            if (stable > stable_time) //travel distance stable at desired value for at least some time
                            {
                                loadDistance = 0;
                                M_control(0);
                                I2C_buffer.data.START_STOP = 0;
                            }
                        } else {
                            stable = 0;
                        }
                    }
                    break;
                case 5:
                    pre_pidM(I2C_buffer.data.SPEED);
                    break;
                case 6:
                    if (I2C_buffer.data.DISTANCE != 0 && loadDistance == 0) {
                        loadDistance = 1;
                        auxDistance = I2C_buffer.data.DISTANCE;
                        I2C_buffer.data.DISTANCE = 0;
                    } else {
                        if (auxDistance > 0) {
                            pre_pidM(abs(I2C_buffer.data.SPEED));
                            if (auxDistance <= I2C_buffer.data.DISTANCE) {
                                loadDistance = 0;
                                M_control(0);
                                I2C_buffer.data.START_STOP = 0;
                            }
                        }
                        if (auxDistance < 0) {
                            pre_pidM(abs(I2C_buffer.data.SPEED)*-1);
                            if (auxDistance >= I2C_buffer.data.DISTANCE) {
                                loadDistance = 0;
                                M_control(0);
                                I2C_buffer.data.START_STOP = 0;
                            }
                        }
                    }
                    break;
                case 7:
                    if (I2C_buffer.data.DISTANCE != 0 && loadDistance == 0) {
                        loadDistance = 1;
                        auxDistance = I2C_buffer.data.DISTANCE;
                        I2C_buffer.data.DISTANCE = 0;
                        stable = 0;
                    } else {
                        pre_pidM(calculate_pidA(auxDistance));
                        if (auxDistance >= I2C_buffer.data.DISTANCE - ATS_tolerance && auxDistance <= I2C_buffer.data.DISTANCE + ATS_tolerance) {
                            stable++;
                            if (stable > stable_time) //travel distance stable at desired value for at least some time
                            {
                                loadDistance = 0;
                                M_control(0);
                                I2C_buffer.data.START_STOP = 0;
                            }
                        } else {
                            stable = 0;
                        }
                    }
                    break;
                default:
                    M_control(0);
                    break;
            }
            __delay_ms(time_lapse);
        } else {
            M_control(0);
            accumulatorM = 0;
            lasterrorM = 0;
            accumulatorA = 0;
            lasterrorA = 0;
        }
        if (I2C_buffer.data.RESET == 1) {
            asm("RESET");
        }
        if (I2C_buffer.data.SAVE == 1) { //Save non-volatile data into the EEPROM
            eeprom_write(0, I2C_buffer.data.ADDRESS);
            eeprom_write(1, I2C_buffer.data.IOWPU);
            eeprom_write(2, I2C_buffer.byte[0x07]);
            eeprom_write(3, I2C_buffer.byte[0x08]);
            eeprom_write(4, I2C_buffer.byte[0x09]);
            eeprom_write(5, I2C_buffer.byte[0x0A]);
            eeprom_write(8, I2C_buffer.byte[0x13]);
            eeprom_write(9, I2C_buffer.byte[0x14]);
            eeprom_write(10, I2C_buffer.byte[0x15]);
            eeprom_write(11, I2C_buffer.byte[0x16]);
            eeprom_write(12, I2C_buffer.byte[0x17]);
            eeprom_write(13, I2C_buffer.byte[0x18]);
            eeprom_write(14, I2C_buffer.byte[0x19]);
            eeprom_write(15, I2C_buffer.byte[0x1A]);
            eeprom_write(16, I2C_buffer.byte[0x1B]);
            eeprom_write(17, I2C_buffer.byte[0x1C]);
            eeprom_write(18, I2C_buffer.byte[0x1D]);
            eeprom_write(19, I2C_buffer.byte[0x1E]);
            eeprom_write(20, I2C_buffer.byte[0x1F]);
            eeprom_write(21, I2C_buffer.byte[0x20]);
            eeprom_write(22, I2C_buffer.byte[0x21]);
            eeprom_write(23, I2C_buffer.byte[0x22]);
            eeprom_write(24, I2C_buffer.byte[0x23]);
            eeprom_write(25, I2C_buffer.byte[0x24]);
            eeprom_write(26, I2C_buffer.byte[0x25]);
            eeprom_write(27, I2C_buffer.byte[0x26]);
            eeprom_write(28, I2C_buffer.byte[0x27]);
            eeprom_write(29, I2C_buffer.byte[0x28]);
            eeprom_write(30, I2C_buffer.byte[0x29]);
            eeprom_write(31, I2C_buffer.byte[0x2A]);
            __delay_ms(time_lapse);
            I2C_buffer.data.SAVE = 0;
        }
    }
}


