#line 1 "C:/Users/ciro_/Documents/Smart_motor_driver/SmartMotorDriver.c"
unsigned short counter = 0;
unsigned int gear = 150;
int rpm = 0;
float kp = 0.1;
float kd = 0.001;
float ki = 0.04;
float accumulator = 0;
float lasterror = 0;

unsigned short asp = 0;

void interrupt() iv 0x0004 ics ICS_AUTO
{
 if (INTCON.f0 == 1 && IOCAF.f4 == 1)
 {
 INTCON.f3 = 0;
 counter ++;
 IOCAF.f4 = 0;
 INTCON.f3 = 1;
 }
 if(PIR1.f0 == 1)
 {
 INTCON.f3 = 0;
 T1CON.f0 = 0;
 rpm = (counter * 300)/gear;
 counter = 0;
 if(LATA.f0 == 0)
 {
 rpm = rpm *-1;
 }
 INTCON.f3 = 1;
 PIR1.f0 = 0;
 T1CON.f0 = 1;
 }
}

void M_control(int ctr);
void PID(int ctr);

void main()
{
OSCCON = 0b11110000;
TRISA = 0b00011000;
ANSELA = 0b00000000;
WPUA = 0b00011110;
OPTION_REG.f7 = 0;
APFCON.f0 = 1;
LATA.f0 = 0;
PWM1_init(50000);
PWM1_start();
PWM1_set_duty(0);
IOCAN.f4 = 1;
INTCON = 0b01001000;
T1CON = 0b00110100;
PIE1.f0 = 1;
T1CON.f0 = 1;
INTCON.f7 = 1;

M_control(0);


while(1)
{
int x = 0;
for(x=0;x<1000;x++)
{
PID(100);
delay_ms(10);
}
for(x=0;x<1000;x++)
{
PID(-100);
delay_ms(10);
}
}


}

void PID(int set)
{
 float error = 0;
 float PID = 0;
 int rpm2 = rpm + 600;
 set = set + 600;
 error = set-rpm2;
 PID = error*kp;
 accumulator += error;
 PID += ki*accumulator;
 PID += kd*(error-lasterror);
 lasterror = error;
 if(PID>=511)
 {
 PID = 511;
 }
 if(PID<=0)
 {
 PID = 0;
 }
 PID = (-255+((510)*((PID)/(511))));
 M_control((int)PID);

 Ow_reset(&PORTA, 1);
 Ow_Write(&PORTA, 1, 0xCC);
 Ow_Write(&PORTA, 1, (((int)set) >>8));
 Ow_Write(&PORTA, 1, (((int)set)&0xFF));
 Ow_Write(&PORTA, 1, (((int)rpm2) >>8));
 Ow_Write(&PORTA, 1, (((int)rpm2)&0xFF));
 Ow_Write(&PORTA, 1, (((int)PID) >>8));
 Ow_Write(&PORTA, 1, (((int)PID)&0xFF));
}

void M_control(int ctr)
{
 if (ctr == 0)
 {
 PWM1_set_duty(ctr);
 }
 if (ctr > 0)
 {
 LATA.f0 = 1;
 PWM1_set_duty(ctr);
 }
 if (ctr < 0)
 {
 LATA.f0 = 0;
 ctr = ctr * -1;
 PWM1_set_duty(ctr);
 }
}
