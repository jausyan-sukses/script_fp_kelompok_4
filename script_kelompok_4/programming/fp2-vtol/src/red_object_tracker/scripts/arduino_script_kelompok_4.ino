#include <Arduino.h>
#include <Servo.h>

Servo servo;

const int dirPin1 = 5;  
const int stepPin1 = 4; 
const int dirPin2 = 7;  
const int stepPin2 = 6; 
const int dirPin3 = 13; 
const int stepPin3 = 12; 

const int stepsPerRevolution = 200; 
const int rotations3 = 8; 
const int totalSteps3 = stepsPerRevolution * rotations3;

bool motor1 = false; 
bool arah1 = LOW;    
bool motor2 = false; 
bool arah2 = LOW;    

bool isGripperClosed = false;
bool isGripperOpened = false;

unsigned long lastCommandTime = 0; 
const unsigned long timeoutInterval = 500;

void setup() {
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(stepPin3, OUTPUT);

  servo.attach(3);
  servo.write(90); 

  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); 
    command.trim();

    lastCommandTime = millis(); 

    if (command.indexOf("gerak atas") != -1) {
      motor2 = true;
      arah2 = HIGH;
    } else if (command.indexOf("gerak bawah") != -1) {
      motor2 = true;
      arah2 = LOW;
    } else if (command.indexOf("tetap y") != -1) {
      motor2 = false;
    }

    if (command.indexOf("gerak kiri") != -1) {
      motor1 = true;
      arah1 = LOW;
    } else if (command.indexOf("gerak kanan") != -1) {
      motor1 = true;
      arah1 = HIGH;
    } else if (command.indexOf("tetap x") != -1) {
      motor1 = false;
    }

    if (command.indexOf("tutup gripper") != -1 && !isGripperClosed) {
      tutupGripper();
      isGripperClosed = true;
      isGripperOpened = false;
    }

    if (command.indexOf("buka gripper") != -1 && !isGripperOpened) {
      bukaGripper();
      isGripperOpened = true;
      isGripperClosed = false;
    }
  }

  if (millis() - lastCommandTime > timeoutInterval) {
    motor1 = false;
    motor2 = false;
  }

  if (motor1) {
    digitalWrite(dirPin1, arah1);
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(500);
  }

  if (motor2) {
    digitalWrite(dirPin2, arah2);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(2000);
  }
}

void tutupGripper() {
  motor1 = false; 
  motor2 = false;

  servo.write(0);
  delay(1000);   

  digitalWrite(dirPin3, HIGH);
  for (int stepCount = 0; stepCount < totalSteps3; stepCount++) {
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(1000);
  }

  delay(500); 

  servo.write(90); 
  delay(1000);

  digitalWrite(dirPin3, LOW); 
  for (int stepCount = 0; stepCount < totalSteps3; stepCount++) {
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(1000);
  }
      digitalWrite(dirPin1, LOW); 
    for (int stepCount = 0; stepCount < stepsPerRevolution * 4; stepCount++) {
      digitalWrite(stepPin1, HIGH);
      delayMicroseconds(2000);
      digitalWrite(stepPin1, LOW);
      delayMicroseconds(2000);
    }

    delay(500); 
    digitalWrite(dirPin2, LOW); 
    for (int stepCount = 0; stepCount < stepsPerRevolution * 7; stepCount++) {
      digitalWrite(stepPin2, HIGH);
      delayMicroseconds(2000);
      digitalWrite(stepPin2, LOW);
      delayMicroseconds(2000);
    }
}

void bukaGripper() {
  motor1 = false; 
  motor2 = false; 

  digitalWrite(dirPin3, HIGH); 
  for (int stepCount = 0; stepCount < totalSteps3; stepCount++) {
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(1000);
  }

  delay(500); 

  servo.write(0); 
  delay(1000);

  digitalWrite(dirPin3, LOW); 
  for (int stepCount = 0; stepCount < totalSteps3; stepCount++) {
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(1000);
  }

  servo.write(90); 
  delay(1000);
}