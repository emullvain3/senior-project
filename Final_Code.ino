#include <Servo.h>
#include <Wire.h>

#define address 0x60

const int pingPin1 = 40;
const int pingPin2 = 38;
const int pingPin3 = 36;
const int genFlamePin = 52;
const int preciseFlamePin = A8;
const int navLED = 48;
const int fireLED = 50;

const int wallClearance = 20;
const int leftTurnDelay = 585;
const int rightTurnDelay = 602;

const int initBasePos = map(125, 0, 270, 0, 180);
const int rightSweepMax = map(30, 0, 270, 0, 180);
const int leftSweepMax = map(230, 0, 270, 0, 180);

Servo baseServo;
Servo latchServo;
int basePos = initBasePos;
int state = 1;
int initCompPos;
int firePos = -1;
bool gen_fire_state = false;

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  Wire.begin();

  pinMode(genFlamePin, INPUT);
  pinMode(preciseFlamePin, INPUT);
  pinMode(navLED, OUTPUT);
  pinMode(fireLED, OUTPUT);

  baseServo.attach(3);
  latchServo.attach(4);
  baseServo.write(initBasePos);
  digitalWrite(navLED, HIGH);

  //initCompPos = readCompass();
  //Serial.println(initCompPos);
  delay(3000);
}

void loop() {
    while(gen_fire_state == false) {
      moveWithUltrasonic();   
      //readCompass();
    } 
    digitalWrite(navLED, LOW);
    digitalWrite(fireLED, HIGH);
    stopRobot();
    delay(500);
    //turnTowardsFire();
    gen_fire_state = extinguishFire();
    delay(500);
    baseServo.write(initBasePos);
    basePos = initBasePos;
    delay(1000);
}

void moveWithUltrasonic() {
  int distance;
  int i = pingPin1;
  
      //Left Wall Sensor Check
      distance = measureDistance(i);
      if (distance <= wallClearance) {
        stopRobot();
        turnRight();
      }
      else {
        goForward(); 
        firePos = genFireScan(basePos, state);
     
        if(firePos != -1) {
          return;
        }
      }
      i-=2;

      //Forward Wall Sensor Check
      distance = measureDistance(i);
      if (distance <= wallClearance) {
        stopRobot();
        turnRight();
      }
      else{
        goForward();
        firePos = genFireScan(basePos, state);
        if(firePos != -1) {
          return;
        }
      }
      i-=2;
                                     
      distance = measureDistance(i);
      if (distance <= wallClearance) {
        stopRobot();
        turnLeft();
      }
      else {
        goForward();
        firePos = genFireScan(basePos, state);
        if(firePos != -1) {
          return;
        }
      }
      i-=2;
}

int genFireScan(int angle, int s) {
  if(s == 1) {
    if(angle == rightSweepMax) {
      s = 2;
    }else {
      baseServo.write(angle);
      angle--;
    }
    basePos = angle;
    state = s;
    gen_fire_state = precFlameRead();
    if(gen_fire_state == true) {
      return basePos;
    }else {
      return -1;
    }
  }
  
  if(s == 2) {
    if(angle == leftSweepMax) {
      s = 3;
    }else {
      baseServo.write(angle);
      angle++;
    }
    basePos = angle;
    state = s;
    gen_fire_state = precFlameRead();
    if(gen_fire_state == true) {
      return basePos;
    }else {
      return -1;
    }
  }
  
  if(s == 3) {
    if(angle == initBasePos) {
      s = 1;
    }else {
      baseServo.write(angle);
      angle--;
    }
    basePos = angle;
    state = s;
    gen_fire_state = precFlameRead();
    if(gen_fire_state == true) {
      return basePos;
    }else {
      return -1;
    }
  }
}

bool genFlameRead() {

  int fire = digitalRead(genFlamePin);

  if (fire == HIGH)
  {
    return true;
  } else {
    return false;
  }
}

bool precFlameRead() {
  float exact_fire;
  exact_fire = analogRead(preciseFlamePin) * (5.0 / 1023.0);
  //Serial.println(exact_fire, 4);
  if (exact_fire < 2.5){
    return true; 
  } else return false;
}

bool extinguishFire() {
  latchServo.write(0);
  delay(750);
  latchServo.write(90);
  delay(100);
  latchServo.write(180);
  delay(750);
  latchServo.write(90);
  delay(2000);
  if(precFlameRead() == true) {
    extinguishFire();
  }else{
    return false;
  }
  
}

int measureDistance(int pin) { 

  long duration, cm;
  pinMode(pin, OUTPUT);

  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);

  cm = duration / 29 / 2;

  delay(15);
  return cm;
}

/*
int readCompass() {

}
*/

void goForward() {
  Serial3.write(92);
  Serial3.write(220);
}

void stopRobot() {
  Serial3.write(64);
  Serial3.write(192);
  delay(200);
}

void stopRobotForever() {
  Serial3.write(64);
  Serial3.write(192);
  delay(20000);
}

/*
void compturnLeft() {

  int c_val = readCompass();
  int left_limit = ((c_val - 90) + 365) % 365;
  Serial.print("Left limit is "); Serial.println(left_limit);
  delay(2000);
  while(c_val != left_limit) {
    Serial3.write(255);    //Motor 1 - FORWARD
    Serial3.write(1);      //Motor 2 - REVERSE
    c_val = readCompass();
    Serial.println(c_val);
  }
  
  Serial3.write(64);     //Motor 1 - STOP
  Serial3.write(192);    //Motor 2 - STOP
  delay(1000);
}
*/

void turnLeft() {
  Serial3.write(255);
  Serial3.write(1);
  delay(leftTurnDelay);
  Serial3.write(64);
  Serial3.write(192);
  delay(1000);
}

void turnRight() {
  
  Serial3.write(127);    //Motor 1 - FORWARD
  Serial3.write(129);    //Motor 2 - REVERSE
  delay(rightTurnDelay);
  Serial3.write(64);     //Motor 1 - STOP
  Serial3.write(192);    //Motor 2 - STOP
  delay(1000);
}

void turnTowardsFire() {
  if (firePos < initBasePos && firePos != -1){
    exactTurnRight();
    
  }
  if (firePos >= initBasePos && firePos != -1) {
    exactTurnLeft();
    
  }
}

void exactTurnRight() {
  int newDelay = map(firePos, rightSweepMax, initBasePos - 1, rightTurnDelay, 0);
  Serial.println(firePos);
  Serial.println(newDelay);
  Serial3.write(127);    //Motor 1 - FORWARD
  Serial3.write(129);    //Motor 2 - REVERSE
  delay(newDelay);
  Serial3.write(64);     //Motor 1 - STOP
  Serial3.write(192);    //Motor 2 - STOP
  delay(1000);
  firePos = -1;

}

void exactTurnLeft() {
  int newDelay = map(firePos, initBasePos, leftSweepMax , 0, leftTurnDelay);
  Serial3.write(255);
  Serial3.write(1);
  delay(newDelay);
  Serial3.write(64);
  Serial3.write(192);
  delay(1000);
  firePos = -1;
}

  /*
    Motor 1 = Right Motors (1-127)
    Motor 2 = Left Motors (255-128)
    Motor 1 Forward = 65-127
    Motor 2 Forward = 193-255
    Motor 1 Stop = 64
    Motor 2 Stop = 192
    Motor 1 Reverse = 63-1
    Motor 2 Reverse = 191-128
  */
