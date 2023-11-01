#include <Servo.h>          //Servo motor library. This is standard library
#include <NewPing.h>        //Ultrasonic sensor function library. You must install this library

//our L298N control pins
//const int LeftMotorForward = 7;
//const int LeftMotorBackward = 6;
const int RightMotorForward = 4;
const int RightMotorBackward = 5;
#define R_EN 8
#define R_PWM 11
//#define L_IS 7
#define L_EN 9
#define L_PWM 12

//sensor pins
#define trig_pin A1 //analog input 1
#define echo_pin A2 //analog input 2

#define maximum_distance 200
boolean goesForward = false;
int distance = 100;

NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //our servo name


void setup(){
  Serial.begin(9600);

  pinMode(RightMotorForward, OUTPUT);
  // pinMode(LeftMotorForward, OUTPUT);
  //pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  //pinMode(L_IS, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  //digitalWrite(R_IS, LOW);
  //digitalWrite(L_IS, LOW);
 digitalWrite(R_EN, HIGH);
 digitalWrite(L_EN, HIGH);
  
  servo_motor.attach(10); //our servo pin

  servo_motor.write(115);
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop(){
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "steer_left"){
      turnLeft();
      delay(1500);
      steermoveStop();
      moveForward();
      delay(1500);
      turnRight();
      delay(650);
      steermoveStop();
    }
    else if (command == "steer_right"){
      turnRight();
      delay(1500);
      steermoveStop();
      moveForward();
      delay(1500);
      turnLeft();
      delay(650);
      steermoveStop();
    }
    else if (command == "steer_straight"){
      moveForward();
    }
  }
  main_function();
    
}

int lookRight(){  
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft(){
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing(){
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
}

void steermoveStop(){
    
  digitalWrite(RightMotorForward, LOW);
  //digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  //digitalWrite(LeftMotorBackward, LOW);
}

void moveForward(){

  if(!goesForward){

    goesForward=true;
    
  analogWrite(R_PWM, 200);
  analogWrite(L_PWM, 0); 
  }
}

void moveBackward(){

  goesForward=false;

  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 200);
  
}

void turnRight(){

  //digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  //digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  //delay(1500);
  //digitalWrite(LeftMotorForward, HIGH);
  //digitalWrite(RightMotorForward, HIGH);
  
  //digitalWrite(LeftMotorBackward, LOW);
  //digitalWrite(RightMotorBackward, LOW);
  
}

void turnLeft(){

  //digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  //digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  //delay(1500);
}

void moveStop(){
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

void main_function(){

  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  if (distance <= 50){
    moveStop();
    delay(300);
    moveBackward();
    delay(800);
    moveStop();
    delay(300);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distance >= distanceLeft){
      turnRight();
      delay(1500);
      steermoveStop();
      moveForward();
      delay(1500);
      turnLeft();
      delay(650);
      steermoveStop();      
    }
    else{
      turnLeft();
      delay(1500);
      steermoveStop();
      moveForward();
      delay(1500);
      turnRight();
      delay(650);
      steermoveStop();
    }
  }
  else{
    moveForward();
     
  }
  distance = readPing();
}
 