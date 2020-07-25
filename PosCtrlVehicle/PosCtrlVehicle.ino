#include "PID.h"

// Initialize pin assignments
#define Lpwm_pin 5            // pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin 10           // pin of controlling speed---- ENB of motor driver board
int pinLB = 2;                // pin of controlling turning---- IN1 of motor driver board
int pinLF = 4;                // pin of controlling turning---- IN2 of motor driver board
int pinRB = 7;                // pin of controlling turning---- IN3 of motor driver board
int pinRF = 8;                // pin of controlling turning---- IN4 of motor driver board
unsigned char Lpwm_val = 200; // initialized left wheel speed at 200 (try if max is at 255)
unsigned char Rpwm_val = 200; // initialized right wheel speed at 200 (try if max is at 255)
int Car_state = 0;            // the working state of car
int servopin = 3;             // defining digital port pin 3, connecting to signal line of servo motor
int myangle;                  // defining variable of angle
int pulsewidth;               // defining variable of pulse width
unsigned char DuoJiao = 90;   // initialized angle of motor at 90°
int inputPin = A0;            // ultrasonic module ECHO to A0
int outputPin= A1;            // ultrasonic module TRIG to A1

/*      kp | ki | kd | i_lo | i_hi | out_lo | out_hi */
PID pid(255., 0.,  0.,  -1.,    1.,  100.,    255.); // lower output limit has to be 80 since below 80 cannot overcome necessary torque to move the vehicle
float outPWM{};
float refDist{};
float duration, distance;
float getDistance;
float error, dt;
float deltaSpeed;

unsigned long startTime;
unsigned long lastTime;


void M_Control_IO_config(void) {
  pinMode(pinLB,OUTPUT);    // pin 2
  pinMode(pinLF,OUTPUT);    // pin 4
  pinMode(pinRB,OUTPUT);    // pin 7
  pinMode(pinRF,OUTPUT);    // pin 8
  pinMode(Lpwm_pin,OUTPUT); // pin  5(PWM)
  pinMode(Rpwm_pin,OUTPUT); // pin 10(PWM)
}
void Set_Speed(unsigned char Left,unsigned char Right) { //function of setting speed
  analogWrite(Lpwm_pin,Left);
  analogWrite(Rpwm_pin,Right);
}
void advance() { // going forward
 digitalWrite(pinRB,LOW);  // making motor move towards right rear
 digitalWrite(pinRF,HIGH);
 digitalWrite(pinLB,LOW);  // making motor move towards left rear
 digitalWrite(pinLF,HIGH); 
 Car_state = 1;   
}
void halt() { // stop
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,HIGH);
  Car_state = 5;
}
void reverse() { // back up
  digitalWrite(pinRB,HIGH); //making motor move towards right rear     
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH); //making motor move towards left rear
  digitalWrite(pinLF,LOW);
  Car_state = 2;  
}

float distSensor()
{
  digitalWrite(outputPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(outputPin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(outputPin, LOW);    
  duration = pulseIn(inputPin, HIGH); // reading the duration of high level
  distance = (duration*.0343)/2;      // Transform pulse time to distance in cm
  return distance;
}

void setup() { 
  M_Control_IO_config();        //motor controlling the initialization of IO
  Set_Speed(Lpwm_val,Rpwm_val); //setting initialized speed
  // Set_servopulse(DuoJiao);   //setting initialized motor angle
  /* Ultrasonic distance sensor */
  pinMode(inputPin, INPUT);     // IO of ultrasonic module ECHO
  pinMode(outputPin, OUTPUT);   // IO of ultrasonic module TRIG
  Serial.begin(9600);           // initialized serial port , using Bluetooth as serial port, setting baud 
  halt();                       // stop
  refDist = 30.0f;              // set the ref. dist.
  startTime = micros();         // initialize the start time to get dt (time step)
}
void loop() {
  getDistance = distSensor();                // get the distance of an object
  error = (getDistance - refDist) / refDist; // normalize the error to get the output between 0 and 1
  startTime = micros();
  if (abs(error) < 0.03f) {                  // within the range of error tolerance, stop the vehicle
    halt();
    lastTime = micros();
  } else {                                   // need to run speed control
    dt = startTime - lastTime;
    deltaSpeed = pid.run(abs(error), dt);    // pid.run(float error, float dt)
    lastTime = micros();
    /* decide the direction of the vehicle */
    if (error < 0.0f) {                      // vehicle needs to back up
      Set_Speed(deltaSpeed, deltaSpeed);
      reverse();
    }
    else if (error > 0.0f) {                 // vehicle needs to go forward
      Set_Speed(deltaSpeed, deltaSpeed);
      advance();
    }
  }
  //Serial.println(error);
}
