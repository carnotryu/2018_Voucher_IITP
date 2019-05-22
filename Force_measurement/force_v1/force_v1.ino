float pos1 = 0.0;
int pos1_raw, pos1_mm;
int pos1_min = 40, pos1_max = 800;
int dir = 2, duty = 250;
int moving = 0;

float max_force = 0.5;

int AI1 =   23;
int AI2 =   25;
int PA =    2;

void setup() {
  Serial.begin(9600);
  pinMode(AI1,OUTPUT);
  pinMode(AI2,OUTPUT);
  pinMode(PA,OUTPUT);  
  digitalWrite(AI1,LOW);
  digitalWrite(AI2,LOW);
  digitalWrite(PA,LOW);
  
}

void loop() {
  
  pos1_raw = analogRead(A1);

  pos1_mm = pos1_raw;

  if (pos1_mm < pos1_min)       pos1_mm = pos1_min;
  else if (pos1_mm > pos1_max)  pos1_mm = pos1_max;

  pos1 = ((float)pos1_mm - pos1_min) / (pos1_max - pos1_min) * 10.0;          // position in mm
    
  char m = Serial.read();

  if (m == '1')
  {
    dir = 3;      // extrude
    moving = 1;
  }
  else if (m == '2')
  {
    dir = 1;      // back
    moving = 2;
  }
  else if (m == '3')
  {
    if (duty > 30)   duty -= 30;
  }
  else if (m == '4')
  {
    if (duty < 250) duty += 30;
  }
  else if (m == '5')
  {
    dir = 2;
  }

  Serial.print(dir);

  Serial.print("\t");
  Serial.print(pos1_mm);

  Serial.print("\t");
  Serial.println(duty);

  if (moving == 2 && pos1_mm < 600)
  {
    dir = 2;
  }
  motor_con1(dir, duty);

  delay(20);
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

