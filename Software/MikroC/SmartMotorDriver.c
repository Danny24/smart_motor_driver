unsigned short counter = 0;
unsigned int gear = 150;
int rpm = 0;
float kp = 0.1;        //0.1
float kd = 0.001;     //0.001
float ki = 0.04;      //0.04
float accumulator = 0;
float lasterror = 0;

unsigned short asp = 0;

void interrupt() iv 0x0004 ics ICS_AUTO //interrupts
{
 if (INTCON.f0 == 1 && IOCAF.f4 == 1) //interrupt on change on RA4 triggered
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
  rpm = (counter * 300)/gear;  //calculate rpm  (multiplied 15 interrupts in 1 second, divided 3 encoder interrupts per lap, multiplied by 60 to convert to minutes, divided by gear ratio)
  counter = 0;  //clear counter
  if(LATA.f0 == 0)
  {
    rpm = rpm *-1;
  }
  INTCON.f3 = 1; //enable on change interrupts
  PIR1.f0 = 0; //clear interrutp flag
  T1CON.f0 =  1; //start timer1
 }
}

void M_control(int ctr); //function prototype
void PID(int ctr); //function prototype

void main() 
{
OSCCON = 0b11110000; //configure internal oscilator fro 32Mhz
TRISA = 0b00011000;  //configure IO
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

void PID(int set) //PID calculation function
{
  float error = 0;
  float PID = 0;
  int rpm2 = rpm + 600;
  set = set + 600;
  error = set-rpm2; //calculate actual error
  PID = error*kp;     // calculate proportional gain
  accumulator += error;  // calculate accumulator, is sum of errors
  PID += ki*accumulator; // add integral gain and error accumulator
  PID += kd*(error-lasterror); //add differential gain
  lasterror = error; //save the error to the next iteration
  if(PID>=511)   //next we guarantee that the PID value is in PWM range
  {
    PID = 511;
  }
  if(PID<=0)
  {
    PID = 0;
  }
  PID = (-255+((510)*((PID)/(511)))); //scale PID result from 0,511 to -255,255
  M_control((int)PID);

  Ow_reset(&PORTA, 1); //debug over onewire protocol
  Ow_Write(&PORTA, 1, 0xCC);
  Ow_Write(&PORTA, 1, (((int)set) >>8));
  Ow_Write(&PORTA, 1, (((int)set)&0xFF));
  Ow_Write(&PORTA, 1, (((int)rpm2) >>8));
  Ow_Write(&PORTA, 1, (((int)rpm2)&0xFF));
  Ow_Write(&PORTA, 1, (((int)PID) >>8));
  Ow_Write(&PORTA, 1, (((int)PID)&0xFF));
}

void M_control(int ctr) //motor control function
{
   if (ctr == 0) //stop the motor
   {
     PWM1_set_duty(ctr);
   }
   if (ctr > 0)  //clockwise turn set and set the pwm duty
   {
     LATA.f0 = 1;
     PWM1_set_duty(ctr);
   }
   if (ctr < 0)  //counter clockwise turn set and set the pwm duty
   {
     LATA.f0 = 0;
     ctr = ctr * -1; //turn value positive before send in to PWM
     PWM1_set_duty(ctr);
   }
}