#include <Wire.h>

int mod = 5;
float fsr1 = 0.0;
float pos1 = 0.0;
byte i2cAddress1 = 0x10;
short fsr1_raw, fsr1_mm;
short fsr1_min = 310, fsr1_max = 1000;
int pos1_raw, pos1_mm;
int pos1_min = 60, pos1_max = 400;
float pos1_des;

float max_force = 2.0;
float stop_pos1 = 0.5;

int AI1 =   23;
int AI2 =   25;
int PA =    2;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(AI1,OUTPUT);
  pinMode(AI2,OUTPUT);
  pinMode(PA,OUTPUT); 
  digitalWrite(AI1,LOW);
  digitalWrite(AI2,LOW);
  digitalWrite(PA,LOW);
}

void loop() {
  fsr1_raw = readDataFromSensor(i2cAddress1);

  pos1_raw = analogRead(A1);

  fsr1_mm = fsr1_raw;
  pos1_mm = pos1_raw;

  if (fsr1_mm < fsr1_min)       fsr1_mm = fsr1_min;
  else if (fsr1_mm > fsr1_max)  fsr1_mm = fsr1_max;

  if (pos1_mm < pos1_min)       pos1_mm = pos1_min;
  else if (pos1_mm > pos1_max)  pos1_mm = pos1_max;

  pos1 = ((float)pos1_mm - pos1_min) / (pos1_max - pos1_min) * 10.0;          // position in mm
  fsr1 = ((float)fsr1_mm - fsr1_min) / (fsr1_max - fsr1_min) * 10.0;          // force in newton
    
  char m = Serial.read();

  if (m == '1')
  {
    stop_pos1 = 0.5;
  }
  else if (m == '2')
  {
    stop_pos1 = 5.0;
  }
  else if (m == '3')
  {
    stop_pos1 = 9.5;
  }

  pos1_des = (fsr1) / (max_force) * (-10.0) + 10;          // 0 ~ 10.0 N -> 10mm ~ 0 mm

  if (mod == 5)
  {
    if (pos1 > stop_pos1)   pos1_des = (fsr1) / (max_force) * (stop_pos1 - 10.0) + 10.0;  // 0 ~ max -> 10mm ~ stop_pos mm
    else                    pos1_des = (fsr1) / (500.0) * (-(stop_pos1 + 0.5)) + (stop_pos1 + 0.5); // max ~ 500 -> 5.5mm ~ 0 mm
  }

  if (pos1_des < 0.0)       pos1_des = 0.0;
  else if (pos1_des > 10.0) pos1_des = 10.0;

  Serial.print(fsr1_raw);
  Serial.print("\t");
  Serial.print(fsr1_mm);
  Serial.print("\t");
  Serial.print(fsr1);
  Serial.print("\t");
  Serial.print(pos1_raw);
  Serial.print("\t");
  Serial.print(pos1_mm);
  Serial.print("\t");
  Serial.print(pos1);
  Serial.print("\t");
  Serial.print(pos1_des);
  Serial.print("\n");

  pcont1(pos1_des);
}

void pcont1(float p_tar1) {
  float dif_mar1 = 0.5;
  float dif1 = p_tar1 - pos1;
  int duty_min1 = 80;
  int duty_max1 = 255;

  int p_duty1 = (abs(dif1) - 0.0) / (2 - 0.0) * (duty_max1 - duty_min1) + duty_min1;
  if (p_duty1 < duty_min1)        p_duty1 = duty_min1;
  else if (p_duty1 > duty_max1)   p_duty1 = duty_max1;
  if (dif1 < -dif_mar1)       motor_con1(1,p_duty1);
  else if (dif1 > dif_mar1)   motor_con1(3,p_duty1);
  else                        motor_con1(2,0);    
}

void motor_con1(int n1, int m1) {    // 1: back, 2: stop, 3: extrude
  if (n1 == 1) {
    digitalWrite(AI1,HIGH);
    digitalWrite(AI2,LOW);
    analogWrite(PA,m1);
  }
  else if (n1 == 2) {
    digitalWrite(AI1,HIGH);
    digitalWrite(AI2,HIGH);
    analogWrite(PA,m1);
  }
  else if (n1 == 3) {
    digitalWrite(AI1,LOW);
    digitalWrite(AI2,HIGH);
    analogWrite(PA,m1);
  }
}

short readDataFromSensor(short address)
{
  byte i2cPacketLength = 6;//i2c packet length. Just need 6 bytes from each slave
  byte outgoingI2CBuffer[3];//outgoing array buffer
  byte incomingI2CBuffer[6];//incoming array buffer

  outgoingI2CBuffer[0] = 0x01;//I2c read command
  outgoingI2CBuffer[1] = 128;//Slave data offset
  outgoingI2CBuffer[2] = i2cPacketLength;//require 6 bytes

  Wire.beginTransmission(address); // transmit to device 
  Wire.write(outgoingI2CBuffer, 3);// send out command
  byte error = Wire.endTransmission(); // stop transmitting and check slave status
  if (error != 0) return -1; //if slave not exists or has error, return -1
  Wire.requestFrom(address, i2cPacketLength);//require 6 bytes from slave

  byte incomeCount = 0;
  while (incomeCount < i2cPacketLength)    // slave may send less than requested
  {
    if (Wire.available())
    {
      incomingI2CBuffer[incomeCount] = Wire.read(); // receive a byte as character
      incomeCount++;
    }
    else
    {
      delayMicroseconds(10); //Wait 10us 
    }
  }

  short rawData = (incomingI2CBuffer[4] << 8) + incomingI2CBuffer[5]; //get the raw data

  return rawData;
}
