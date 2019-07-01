void setup()
{
  Serial.begin(9600);
  Serial1.begin(38400);    // 19: MEGA_RX1 <-> BT TX, 18: MEGA_TX1 <-> BT RX
}

void loop()
{
  
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  

  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
}
