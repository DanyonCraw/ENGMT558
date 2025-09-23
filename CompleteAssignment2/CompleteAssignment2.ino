//importing librarys
#include <util/atomic.h>

//setting interupt pins from motor encoder to arduino
#define ENCA 3
#define ENCB 2
//srtting pins from motordriver to arduino
#define IN1 7
#define IN2 8
#define PWM 9
#define EN 6
//initilising variables
int target = 0;
char colour = 'n';

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
 
//seting up pins and serial
void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
 
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  digitalWrite(EN, HIGH)
}
 
void loop() {
 
  //checks to see if colour is being sent via serial from python script
  if (Serial.available() > 0) {
    colour = Serial.read();
    Serial.print("arduino got: ");
    Serial.println(colour);
  }
  
  //sets motor target based on recieved colour
  //r and b (red and blue) = flag raised position
  //y and g (red and green) = flaglowered position
  if (colour == 'r' || colour == 'b'){
    target = 1300;
  }
  else if (colour == 'y' || colour == 'g'){
    target = 0;
  }
 
 
  // PID constants
  float kp = 10;
  float kd = 0.025;
  float ki = 0.5;
 
 
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
 
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
 
  // error
  int e = pos - target;
 
  // derivative
  float dedt = (e-eprev)/(deltaT);
 
  // integral
  eintegral = eintegral + e*deltaT;
 
  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;
 
  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
 
  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
 
  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);
 
  // store previous error
  eprev = e;
 
  Serial.print("target: ");
  Serial.print(target);
  Serial.print(" ");
  Serial.print("position: ");
  Serial.print(pos);
  Serial.println();
}

//runs motor until desired target is reached calculated from PID control
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}
//reads position of the motor
void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}