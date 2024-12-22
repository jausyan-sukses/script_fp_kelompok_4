#include <arduino.h>
#include <Servo.h>

Servo servo;

int dirPin1 = 5; 
int stepPin1 = 4; 
int dirPin2 = 7;  
int stepPin2 = 6;  
int dirPin3 = 13; 
int stepPin3 = 12;

int stepsPerRevolution = 200;     
int rotations1 = 5;
int rotations2 = 5;
int rotations3 = 5;
int totalSteps1 = stepsPerRevolution * rotations1;
int totalSteps2 = stepsPerRevolution * rotations2;
int totalSteps3 = stepsPerRevolution * rotations3;

bool motor1 = false; 
bool arah1 = LOW;   
bool motor2 = false; 
bool arah2 = LOW;   
bool motor3 = false; 
bool arah3 = LOW;

unsigned long lastCommandTime = 0;   
unsigned long timeoutInterval = 500;  

void setup() {
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  servo.attach(2);
  Serial.begin(9600); // Mulai komunikasi serial
}

void loop() {
  // Periksa apakah ada data serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); 
    command.trim(); 
    motor3 = false;
    lastCommandTime = millis(); 
    servo.write(0);
    delay(2000);
    servo.write(90);
    delay(2000);

   
    if (command == "gerak atas") {                   
      motor2 = true;
      arah2 = HIGH;
    } 
    else if (command == "gerak bawah") {
      motor2 = true;
      arah2 = LOW;
    } 
    else if (command == "tetap y") {
      motor2 = false;
    }

    
    if (command == "gerak kanan") {
      motor1 = true;
      arah1 = LOW;
    } 
    else if (command == "gerak kiri") {
      motor1 = true;
      arah1 = HIGH;
    } 
    else if (command == "tetap x") {
      motor1 = false;
    } 
    else if (command == "tutup gripper") {
      servo.write(0);
      delay(2000);

      for (int stepCount = 0; stepCount < totalSteps3; stepCount++) {
        motor3 = true;
        arah3 = LOW;
        delay(2000);
      }

      servo.write(90);
      delay(2000);

      for (int stepCount = 0; stepCount < totalSteps3; stepCount++) {
        motor3 = true;
        arah3 = HIGH;
        delay(2000);
      }

      for (int stepCount = 0; stepCount < totalSteps2; stepCount++) {
        motor2 = true;
        arah2 = LOW;
        delay(2000);
      }
      for (int stepCount = 0; stepCount < totalSteps1; stepCount++) {
        motor2 = true;
        arah2 = HIGH;
        delay(2000);
      }
    }
    else if (command == "buka gripper") {

      for (int stepCount = 0; stepCount < totalSteps3; stepCount++) {
        motor3 = true;
        arah3 = LOW;
        delay(2000);
      }

      servo.write(0);
      delay(2000);

      for (int stepCount = 0; stepCount < totalSteps3; stepCount++) {
        motor3 = true;
        arah3 = HIGH;
        delay(2000);
      }
      servo.write(0);
      delay(2000);
      
    }
  }

  // Jika tidak ada pesan baru dalam waktu timeout, matikan motor
  if (millis() - lastCommandTime > timeoutInterval) {
    motor1 = false;
    motor2 = false;
    motor3 = false; // Tambahkan agar motor3 juga dimatikan
  }

  // Jalankan motor 1 jika aktif
  if (motor1) {
    digitalWrite(dirPin1, arah1);
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(20000);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(20000);
  }

  // Jalankan motor 2 jika aktif
  if (motor2) {
    digitalWrite(dirPin2, arah2);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(2000);
  }

  // Jalankan motor 3 jika aktif
  if (motor3) {
    digitalWrite(dirPin3, arah3);
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(2000);
  }
}