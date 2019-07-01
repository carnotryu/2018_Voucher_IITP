#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define VIB 7 // vibration motor pwm pin number

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     36 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int mod = 5;
float fsr[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float pos[] = {0.0, 0.0, 0.0, 0.0, 0.0};
byte i2cAddress[] = {0x06, 0x07, 0x08, 0x09, 0x10};
short fsr_raw[5], fsr_mm[5], fsr_raw_f[5];
byte fsr_tx[] = {0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe};
short fsr_min[] = {250, 250, 250, 250, 250};
short fsr_max[] = {850, 950, 850, 750, 750};
short fsr_range[] = {600, 700, 600, 500, 500};
int pos_raw[5], pos_mm[5];
int pos_min[] = {40, 40, 40, 40, 40};
int pos_max[] = {400, 400, 400, 400, 400};
byte pos_received[] = {0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe};
float pos_des[] = {0.0, 0.0, 0.0, 0.0, 0.0};
int bData = 0;
int init_flag = 1;
int vib_int_prev = 0;

float max_force = 0.5;

int AI1 =   23;
int AI2 =   25;
int PA =    2;
int BI1 =   27;
int BI2 =   29;
int PB =    3;
int CI1 =   31;
int CI2 =   33;
int PC =    4;
int DI1 =   35;
int DI2 =   37;
int PD =    5;
int EI1 =   39;
int EI2 =   41;
int PE =    6;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(AI1,OUTPUT);
  pinMode(AI2,OUTPUT);
  pinMode(PA,OUTPUT);
  pinMode(BI1,OUTPUT);
  pinMode(BI2,OUTPUT);
  pinMode(PB,OUTPUT);
  pinMode(CI1,OUTPUT);
  pinMode(CI2,OUTPUT);
  pinMode(PC,OUTPUT);
  pinMode(DI1,OUTPUT);
  pinMode(DI2,OUTPUT);
  pinMode(PD,OUTPUT);  
  pinMode(EI1,OUTPUT);
  pinMode(EI2,OUTPUT);
  pinMode(PE,OUTPUT);
  pinMode(VIB,OUTPUT);
  analogWrite(VIB,0);
  digitalWrite(AI1,LOW);
  digitalWrite(AI2,LOW);
  digitalWrite(PA,LOW);
  digitalWrite(BI1,LOW);
  digitalWrite(BI2,LOW);
  digitalWrite(PB,LOW);
  digitalWrite(CI1,LOW);
  digitalWrite(CI2,LOW);
  digitalWrite(PC,LOW);  
  digitalWrite(DI1,LOW);
  digitalWrite(DI2,LOW);
  digitalWrite(PD,LOW);
  digitalWrite(EI1,LOW);
  digitalWrite(EI2,LOW);
  digitalWrite(PE,LOW);

  //init_fsr();

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.println("Cal.");
  display.println("required");
  display.display();    

}

void init_fsr() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Cal.");
  display.println("ing...");
  display.display();

  motor_stop();
  // FSR initialization
  delay(300);
  for (int i = 0; i < 5; i++) {
    fsr_raw[i] = 0;
  }
  for (int i = 0; i < 100; i++)
  {
    for (int j = 0; j < 5; j++) {
      fsr_raw[j] = fsr_raw[j] + readDataFromSensor(i2cAddress[j]);
    }
    delay(1);
  }
  for (int i = 0; i < 5; i++) {
    fsr_min[i] = (short)fsr_raw[i] / 100 + 30;
    fsr_max[i] = fsr_min[i] + fsr_range[i];
    fsr_raw_f[i] = readDataFromSensor(i2cAddress[i]);
  }
  byte cal_complete[] = {0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd};
  Serial.write(cal_complete,7);
  Serial.write("\n");
  init_flag = 0;
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Cal.");
  display.println("completed");
  display.display();
}

void loop() {
  if (init_flag == 0) {
    for (int i = 0; i < 5; i++) {
      fsr_raw[i] = readDataFromSensor(i2cAddress[i]);
      fsr_raw_f[i] = 0.8000 * fsr_raw_f[i] + 0.2000 * fsr_raw[i];
      pos_raw[i] = analogRead(i+1);
      fsr_mm[i] = fsr_raw_f[i];
      pos_mm[i] = pos_raw[i];
      if (fsr_mm[i] < fsr_min[i])       fsr_mm[i] = fsr_min[i];
      else if (fsr_mm[i] > fsr_max[i])  fsr_mm[i] = fsr_max[i];
      if (pos_mm[i] < pos_min[i])       pos_mm[i] = pos_min[i];
      else if (pos_mm[i] > pos_max[i])  pos_mm[i] = pos_max[i];
      pos[i] = ((float)pos_mm[i] - pos_min[i]) / (pos_max[i] - pos_min[i]) * 10.0;  // position in mm ( 0 - 10.0 )
  
    }
  
    for (int i = 0; i < 5; i++)
    {
      fsr_tx[i+1] = (byte)map(fsr_mm[i], fsr_min[i], fsr_max[i], 0x00, 0xfa);
    }
  
    if (init_flag == 0) {
      Serial.write(fsr_tx,7);
      Serial.print("\n");    
    }
  
    if ( (pos_raw[0] > 450) || (pos_raw[1] > 450) || (pos_raw[2] > 450) || (pos_raw[3] > 450) || (pos_raw[4] > 450) )    motor_stop();
  
    delayMicroseconds(500);    
  }

}

void serialEvent() {
  while (Serial.available()) {
    byte inData = Serial.read();
    if ( (bData == 0) && (inData == 0xff) ) {
      bData = 1;
    }
    else if ( (bData == 1) && (inData < 0xfe) ) {
      pos_received[1] = inData;
      bData = 2;
    }
    else if ( (bData == 2) && (inData < 0xfe) ) {
      pos_received[2] = inData;
      bData = 3;
    }
    else if ( (bData == 3) && (inData < 0xfe) ) {
      pos_received[3] = inData;
      bData = 4;
    }
    else if ( (bData == 4) && (inData < 0xfe) ) {
      pos_received[4] = inData;
      bData = 5;
    }
    else if ( (bData == 5) && (inData < 0xfe) ) {
      pos_received[5] = inData;
      bData = 6;
    }
    else if ( (bData == 6) && (inData < 0xfe) ) {
      pos_received[6] = inData;
      bData = 7;
    }
    else if ( bData == 7 ) {
      if ( inData == 0xfe ) {
        motor_cont(pos_received);
        bData = 0;   
      }
      else if ( inData == 0xfd) {
        init_flag = 1;
        init_fsr();
        bData = 0;
      }
      else if ( inData == 0xfc) {
        motor_stop();
        bData = 0;
      }
      else    bData = 0;
    }
  }
}

void motor_cont(byte pos_tar[8]) {
  for (int i = 0; i < 5; i++) {
    pos_des[i] = (float)map(pos_tar[i+1],0,255,10.0,0.0);
    if (pos_des[i] < 0.0)         pos_des[i] = 0.0;
    else if (pos_des[i] > 10.0)   pos_des[i] = 10.0;    
  }
  pcont1(pos_des[0]);
  pcont2(pos_des[1]);
  pcont3(pos_des[2]);
  pcont4(pos_des[3]);
  pcont5(pos_des[4]);
  if (pos_tar[6] != vib_int_prev)   analogWrite(VIB, int(pos_tar[6]));
  vib_int_prev = pos_tar[6];
}

void motor_stop() {
  motor_con1(2,0);
  motor_con2(2,0);
  motor_con3(2,0);
  motor_con4(2,0);
  motor_con5(2,0);
}

void pcont1(float p_tar1) {
  float dif_mar1 = 0.05;
  float dif1 = p_tar1 - pos[0];
  int duty_min1 = 50;
  int duty_max1 = 255;

  int p_duty1 = (abs(dif1) - 0.0) / (2 - 0.0) * (duty_max1 - duty_min1) + duty_min1;
  if (p_duty1 < duty_min1)        p_duty1 = duty_min1;
  else if (p_duty1 > duty_max1)   p_duty1 = duty_max1;
  if (dif1 < -dif_mar1)       motor_con1(1,p_duty1);
  else if (dif1 > dif_mar1)   motor_con1(3,p_duty1);
  else                        motor_con1(2,0);    
}

void pcont2(float p_tar2) {
  float dif_mar2 = 0.05;
  float dif2 = p_tar2 - pos[1];
  int duty_min2 = 50;
  int duty_max2 = 255;

  int p_duty2 = (abs(dif2) - 0.0) / (2 - 0.0) * (duty_max2 - duty_min2) + duty_min2;
  if (p_duty2 < duty_min2)        p_duty2 = duty_min2;
  else if (p_duty2 > duty_max2)   p_duty2 = duty_max2;
  if (dif2 < -dif_mar2)       motor_con2(1,p_duty2);
  else if (dif2 > dif_mar2)   motor_con2(3,p_duty2);
  else                        motor_con2(2,0);    
}

void pcont3(float p_tar3) {
  float dif_mar3 = 0.05;
  float dif3 = p_tar3 - pos[2];
  int duty_min3 = 50;
  int duty_max3 = 255;

  int p_duty3 = (abs(dif3) - 0.0) / (2 - 0.0) * (duty_max3 - duty_min3) + duty_min3;
  if (p_duty3 < duty_min3)        p_duty3 = duty_min3;
  else if (p_duty3 > duty_max3)   p_duty3 = duty_max3;
  if (dif3 < -dif_mar3)       motor_con3(1,p_duty3);
  else if (dif3 > dif_mar3)   motor_con3(3,p_duty3);
  else                        motor_con3(2,0);    
}

void pcont4(float p_tar4) {
  float dif_mar4 = 0.05;
  float dif4 = p_tar4 - pos[3];
  int duty_min4 = 50;
  int duty_max4 = 255;

  int p_duty4 = (abs(dif4) - 0.0) / (2 - 0.0) * (duty_max4 - duty_min4) + duty_min4;
  if (p_duty4 < duty_min4)        p_duty4 = duty_min4;
  else if (p_duty4 > duty_max4)   p_duty4 = duty_max4;
  if (dif4 < -dif_mar4)       motor_con4(1,p_duty4);
  else if (dif4 > dif_mar4)   motor_con4(3,p_duty4);
  else                        motor_con4(2,0);    
}

void pcont5(float p_tar5) {
  float dif_mar5 = 0.05;
  float dif5 = p_tar5 - pos[4];
  int duty_min5 = 50;
  int duty_max5 = 255;

  int p_duty5 = (abs(dif5) - 0.0) / (2 - 0.0) * (duty_max5 - duty_min5) + duty_min5;
  if (p_duty5 < duty_min5)        p_duty5 = duty_min5;
  else if (p_duty5 > duty_max5)   p_duty5 = duty_max5;
  if (dif5 < -dif_mar5)       motor_con5(1,p_duty5);
  else if (dif5 > dif_mar5)   motor_con5(3,p_duty5);
  else                        motor_con5(2,0);    
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

void motor_con2(int n2, int m2) {    // 1: back, 2: stop, 3: extrude
  if (n2 == 1) {
    digitalWrite(BI1,HIGH);
    digitalWrite(BI2,LOW);
    analogWrite(PB,m2);
  }
  else if (n2 == 2) {
    digitalWrite(BI1,HIGH);
    digitalWrite(BI2,HIGH);
    analogWrite(PB,m2);
  }
  else if (n2 == 3) {
    digitalWrite(BI1,LOW);
    digitalWrite(BI2,HIGH);
    analogWrite(PB,m2);
  }
}

void motor_con3(int n3, int m3) {    // 1: back, 2: stop, 3: extrude
  if (n3 == 1) {
    digitalWrite(CI1,HIGH);
    digitalWrite(CI2,LOW);
    analogWrite(PC,m3);
  }
  else if (n3 == 2) {
    digitalWrite(CI1,HIGH);
    digitalWrite(CI2,HIGH);
    analogWrite(PC,m3);
  }
  else if (n3 == 3) {
    digitalWrite(CI1,LOW);
    digitalWrite(CI2,HIGH);
    analogWrite(PC,m3);
  }
}

void motor_con4(int n4, int m4) {    // 1: back, 2: stop, 3: extrude
  if (n4 == 1) {
    digitalWrite(DI1,HIGH);
    digitalWrite(DI2,LOW);
    analogWrite(PD,m4);
  }
  else if (n4 == 2) {
    digitalWrite(DI1,HIGH);
    digitalWrite(DI2,HIGH);
    analogWrite(PD,m4);
  }
  else if (n4 == 3) {
    digitalWrite(DI1,LOW);
    digitalWrite(DI2,HIGH);
    analogWrite(PD,m4);
  }
}

void motor_con5(int n5, int m5) {    // 1: back, 2: stop, 3: extrude
  if (n5 == 1) {
    digitalWrite(EI1,HIGH);
    digitalWrite(EI2,LOW);
    analogWrite(PE,m5);
  }
  else if (n5 == 2) {
    digitalWrite(EI1,HIGH);
    digitalWrite(EI2,HIGH);
    analogWrite(PE,m5);
  }
  else if (n5 == 3) {
    digitalWrite(EI1,LOW);
    digitalWrite(EI2,HIGH);
    analogWrite(PE,m5);
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
