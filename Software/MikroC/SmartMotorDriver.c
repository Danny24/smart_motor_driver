unsigned int counter = 0;
unsigned int rpm = 0;
char txt[30];
int i,x;

void interrupt() iv 0x0004 ics ICS_AUTO //interrupts
{
 if (INTCON.f0 == 1 && IOCAF.f4 == 1) //interrupt on change on RA4 trigered
 {
  INTCON.f3 = 0; //disable on change interrupts
  counter ++;  //increment counter one in one
  IOCAF.f4 = 0; //clear interrupt flags
  INTCON.f3 = 1; //enable on change interrupts
 }
 if(PIR1.f0 == 1) //timer1 interrupt, called every 65.536ms
 {
  INTCON.f3 = 0; //disable on change interrupts
  T1CON.f0 =  0; //stop timer1
  rpm = (counter * 5 * 60)/150;  //calculate rpm
  counter = 0;  //clear counter
  INTCON.f3 = 1; //enable on change interrupts
  PIR1.f0 = 0; //clear interrutp flag
  T1CON.f0 =  1; //start timer1
 }
}

void M_control(int ctr); //function prototype

void main() 
{
OSCCON = 0b11110000; //configure internal oscilator fro 32Mhz
TRISA = 0b00011100;  //configure IO
ANSELA = 0b00000000; //analog functions of pins disabled
WPUA = 0b00011110;   //configure weak pull-ups on input pins
OPTION_REG.f7 = 0;   //enable weak pull-ups
APFCON.f0 = 1;       //select RA5 as CCP output pin
LATA.f0 = 0;         //put motor direction pin to low
PWM1_init(50000);    //confifure pwm frecuency
PWM1_start();        //start pwm module
PWM1_set_duty(0);    //put duty of pwm to 0
IOCAN.f4 = 1;        //configure interrupt on falling edge for rpm meter
INTCON = 0b01001000; //enables interrupts
T1CON = 0b00110100;  //configure timer1 to run at 1 MHz
PIE1.f0 =  1;        //enable timer1 interrupt
T1CON.f0 =  1;       //start timer1
INTCON.f7 = 1;       //run interrupts

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

void M_control(int ctr) //motor control function
{
  if(abs(ctr) > 255)  //if the value is bigger than 8 bits...
  {
  return;   //exit function
  }
  else
  {
   if (ctr == 0) //stop the motor
   {
     PWM1_set_duty(ctr);
   }
   if (ctr < 0)  //clockwise turn set and set the pwm duty
   {
     LATA.f0 = 0;
     PWM1_set_duty(ctr);
   }
   if (ctr > 0)  //counter clockwise turn set and set the pwm duty
   {
     LATA.f0 = 1;
     PWM1_set_duty(ctr);
   }
  }
}