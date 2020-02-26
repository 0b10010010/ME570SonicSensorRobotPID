// Initialize pin assignments
#define PWM_LeftPin 5            // pin of controlling speed---- ENA of motor driver board
#define PWM_RightPin 10           // pin of controlling speed---- ENB of motor driver board
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
unsigned char servoAngle = 90;   // initialized angle of servo at 90°
int ECHO_Pin = A0;            // ultrasonic module ECHO to A0
int TRIG_Pin= A1;            // ultrasonic module TRIG to A1

void servopulse(int servopin,int myangle) { //defining a function of pulse 
  pulsewidth=(myangle*11)+500; //converting angle into pulse width value at 500-2480 
  digitalWrite(servopin,HIGH); //increasing the level of motor interface to upmost
  delayMicroseconds(pulsewidth); //delaying microsecond of pulse width value
  digitalWrite(servopin,LOW); //decreasing the level of motor interface to the least
  delay(20-pulsewidth/1000);
}
void Set_servopulse(int set_val) {
 for(int i=0;i<=10;i++)  //giving motor enough time to turn to assigning point
   servopulse(servopin,set_val); //invokimg pulse function
}
void M_Control_IO_config(void) {
  pinMode(pinLB,OUTPUT);    // pin 2
  pinMode(pinLF,OUTPUT);    // pin 4
  pinMode(pinRB,OUTPUT);    // pin 7
  pinMode(pinRF,OUTPUT);    // pin 8
  pinMode(PWM_LeftPin,OUTPUT); // pin  5(PWM)
  pinMode(PWM_RightPin,OUTPUT); // pin 10(PWM)
}
void Set_Speed(unsigned char Left,unsigned char Right) { //function of setting speed
  analogWrite(PWM_LeftPin,Left);
  analogWrite(PWM_RightPin,Right);
}
void advance() { // going forward
 digitalWrite(pinRB,LOW);  // making motor move towards right rear
 digitalWrite(pinRF,HIGH);
 digitalWrite(pinLB,LOW);  // making motor move towards left rear
 digitalWrite(pinLF,HIGH); 
 Car_state = 1;   
}
void turnR() { // turning right(dual wheel)
  digitalWrite(pinRB,LOW);  //making motor move towards right rear
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW);  //making motor move towards left front
  Car_state = 4;
}
void turnL() { // turning left(dual wheel)
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW ); //making motor move towards right front
  digitalWrite(pinLB,LOW);  //making motor move towards left rear
  digitalWrite(pinLF,HIGH);
  Car_state = 3;
}    
void stopp() { // stop
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,HIGH);
  Car_state = 5;
}
void back() { // back up
  digitalWrite(pinRB,HIGH); //making motor move towards right rear     
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH); //making motor move towards left rear
  digitalWrite(pinLF,LOW);
  Car_state = 2;  
}



void Self_Control(void) { // self-going, ultrasonic obstacle avoidance
  int H;
  
  Set_servopulse(servoAngle);
  
  H = Ultrasonic_Ranging(1);
  delay(300);   
  if(Ultrasonic_Ranging(1) < 35) {
    stopp();
    delay(100);
    back();
    delay(50);
  }
         
  if(Ultrasonic_Ranging(1) < 60) {
    stopp();  
    delay(100);            
    Set_servopulse(5);
    int L = ask_pin_L(2);
    delay(300);      
    Set_servopulse(177);
    int R = ask_pin_R(3);
    delay(300);      
  
    if(ask_pin_L(2) > ask_pin_R(3)) {
      back();
      delay(100);
      turnL();
      delay(400);
      stopp();
      delay(50);
      Set_servopulse(servoAngle);
      H = Ultrasonic_Ranging(1);
      delay(500);
    }
    if(ask_pin_L(2)  <= ask_pin_R(3)) {
      back();
      delay(100);
      turnR();
      delay(400);
      stopp();
      delay(50);
      Set_servopulse(servoAngle);
      H = Ultrasonic_Ranging(1);
      delay(300);
    }
    if (ask_pin_L(2)  < 35 && ask_pin_R(3)< 35) {
      stopp();
      delay(50);
      back();
      delay(50);
    }  
  }
  else {
    advance();                
  }              
}

int Ultrasonic_Ranging(unsigned char Mode) { // function of ultrasonic distance detecting ，MODE=1，displaying，no displaying under other situation
  int old_distance;
  digitalWrite(TRIG_Pin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(TRIG_Pin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(TRIG_Pin, LOW);    
  int distance = pulseIn(ECHO_Pin, HIGH);  // reading the duration of high level
  distance= distance/58;   // Transform pulse time to distance   
  if(Mode==1) {
    Serial.print("\n H = ");
    Serial.print(distance,DEC); 
    return distance;
  }
  else return distance;
}
int ask_pin_L(unsigned char Mode) { 
  int old_Ldistance;
  digitalWrite(TRIG_Pin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(TRIG_Pin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(TRIG_Pin, LOW);    
  int Ldistance = pulseIn(ECHO_Pin, HIGH); 
  Ldistance= Ldistance/58;   // Transform pulse time to distance   
  if(Mode==2) {
    Serial.print("\n L = ");
    Serial.print(Ldistance,DEC); 
    return Ldistance;
  }
  else return Ldistance;
}
int ask_pin_R(unsigned char Mode) { 
  int old_Rdistance;
  digitalWrite(TRIG_Pin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(TRIG_Pin, HIGH); // 
  delayMicroseconds(10); 
  digitalWrite(TRIG_Pin, LOW);    
  int Rdistance = pulseIn(ECHO_Pin, HIGH); 
  Rdistance= Rdistance/58;   // Transform pulse time to distance   
  if(Mode==3) {
    Serial.print("\n R = ");
    Serial.print(Rdistance,DEC); 
    return Rdistance;
  }
  else return Rdistance;
}

void setup() { 
  pinMode(servopin,OUTPUT);  //setting motor interface as output
  M_Control_IO_config();     //motor controlling the initialization of IO
  Set_Speed(Lpwm_val,Rpwm_val);  //setting initialized speed
  Set_servopulse(servoAngle);       //setting initialized motor angle
  pinMode(ECHO_Pin, INPUT);      //IO of ultrasonic module ECHO
  pinMode(TRIG_Pin, OUTPUT);    //IO of ultrasonic module TRIG
  Serial.begin(9600);            //initialized serial port , using Bluetooth as serial port, setting baud 
  stopp();                       //stop
} 
void loop() {
  Self_Control();
  // PID pid(kp, ki, kd)
  // error = H - DistRef;
  // deltaSpeed = pid.calculate(error); // calculate PID using ultrasonic sensor distance reading
  // Set_Speed(deltaSpeed, deltaSpeed); // use Set_Speed to set output speed from PID controller output
}

