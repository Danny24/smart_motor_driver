#line 1 "C:/Users/ciro_/Documents/Smart_motor_driver/SmartMotorDriver.c"
unsigned short counter = 0;
unsigned int rpm = 0;
unsigned int gear = 150;
short kp = 1;
short kd = 1;
short ki = 1;
int accumulator = 0;
int lasterror = 0;
char txt[10];
int i,x;


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
TRISA = 0b00011100;
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
Soft_UART_Init(&PORTA, 2, 1, 9600, 0);

while(1)
{
PID(100);
IntToStr(rpm, txt);
for (i=0;i<strlen(txt);i++)
{
Soft_UART_Write(txt[i]);
}
Soft_UART_Write(10);
Soft_UART_Write(13);
delay_ms(100);
}
}

void PID(int ctr)
{
 int error = ctr-rpm;
 int PID = error*kp;
 accumulator += error;
 PID += ki*accumulator;
 PID += kd*(error-lasterror);
 lasterror = error;
 if(PID>=255)
 {
 PID = 255;
 }
 if(PID<=-255)
 {
 PID = -255;
 }
 M_control(PID);
}

void M_control(int ctr)
{
 if(abs(ctr) > 255)
 {
 return;
 }
 else
 {
 if (ctr == 0)
 {
 PWM1_set_duty(ctr);
 }
 if (ctr < 0)
 {
 LATA.f0 = 0;
 PWM1_set_duty(ctr);
 }
 if (ctr > 0)
 {
 LATA.f0 = 1;
 PWM1_set_duty(ctr);
 }
 }
}
