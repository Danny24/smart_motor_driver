#include <Wire.h>

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

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
}

void loop() {
  Wire.beginTransmission(0x24);
  Wire.write(0);
  Wire.endTransmission(false);
  Wire.requestFrom( (uint8_t) 0x24, (uint8_t) 32 , (uint8_t) true );
  int index = 0;
  while (Wire.available())
  {
    I2C_buffer.byte[index] = Wire.read();
    index++;
  }
  //Wire.endTransmission();
  Wire.beginTransmission(0x24);
  Wire.write(32);
  Wire.endTransmission(false);
  Wire.requestFrom( (uint8_t) 0x24, (uint8_t) 12 , (uint8_t) true );
  while (Wire.available())
  {
    I2C_buffer.byte[index] = Wire.read();
    index++;
  }
  //Wire.endTransmission();

  Serial.print("ID: "); Serial.println(I2C_buffer.data.ID);
  Serial.print("ADDRESS: "); Serial.println(I2C_buffer.data.ADDRESS);
  Serial.print("START_STOP: "); Serial.println(I2C_buffer.data.START_STOP);
  Serial.print("IOWPU: "); Serial.println(I2C_buffer.data.IOWPU);
  Serial.print("MODE: "); Serial.println(I2C_buffer.data.MODE);
  Serial.print("SAVE: "); Serial.println(I2C_buffer.data.SAVE);
  Serial.print("RESET: "); Serial.println(I2C_buffer.data.RESET);
  Serial.print("GEAR_RATIO: "); Serial.println(I2C_buffer.data.GEAR_RATIO);
  Serial.print("DIAMETER: "); Serial.println(I2C_buffer.data.DIAMETER);
  Serial.print("RPM: "); Serial.println(I2C_buffer.data.RPM);
  Serial.print("SPEED: "); Serial.println(I2C_buffer.data.SPEED);
  Serial.print("DISTANCE: "); Serial.println(I2C_buffer.data.DISTANCE);
  Serial.print("RPM_PID_KP: "); Serial.println(I2C_buffer.data.RPM_PID_KP);
  Serial.print("RPM_PID_KD: "); Serial.println(I2C_buffer.data.RPM_PID_KD);
  Serial.print("RPM_PID_KI: "); Serial.println(I2C_buffer.data.RPM_PID_KI);
  Serial.print("ATS_PID_KP: "); Serial.println(I2C_buffer.data.ATS_PID_KP);
  Serial.print("ATS_PID_KD: "); Serial.println(I2C_buffer.data.ATS_PID_KD);
  Serial.print("ATS_PID_KI: "); Serial.println(I2C_buffer.data.ATS_PID_KI);
  Serial.println(" ");

  if (Serial.available()) {
    I2C_buffer.data.SPEED = Serial.parseInt();
    Wire.beginTransmission(0x24);
    Wire.write(13);
    Wire.write((I2C_buffer.data.SPEED)&0xFF); //checar orden de los bytes, aqui hay algo que no cuadra
    Wire.write((I2C_buffer.data.SPEED>>8)&0xFF);
    Wire.endTransmission();
  }
    delay(500);
}
