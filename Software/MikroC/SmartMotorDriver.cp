#line 1 "C:/Users/ciro_/Documents/Smart_motor_driver/SmartMotorDriver.c"
unsigned int counter = 0;
unsigned int rpm = 0;
char txt[30];
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
 rpm = (counter * 5 * 60)/150;
 counter = 0;
 INTCON.f3 = 1;
 PIR1.f0 = 0;
 T1CON.f0 = 1;
 }
}

void M_control(int ctr);

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

M_control(255);
Soft_UART_Init(&PORTA, 2, 1, 9600, 0);

while(1)
{
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
