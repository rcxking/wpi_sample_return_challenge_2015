#include <string.h> //strtok_r

#define MOTOR0FWD   22 
#define MOTOR0REV   23
#define MOTOR0EN    2

#define MOTOR1FWD   24
#define MOTOR1REV   25
#define MOTOR1EN    3

#define MOTOR2FWD   26
#define MOTOR2REV   27
#define MOTOR2EN    4

#define MOTOR3FWD   28
#define MOTOR3REV   29
#define MOTOR3EN    5

#define MOTOR4FWD   30
#define MOTOR4REV   31
#define MOTOR4EN    6

#define MOTOR5FWD   32
#define MOTOR5REV   33
#define MOTOR5EN    7

#define MOTOR6FWD   34
#define MOTOR6REV   35
#define MOTOR6EN    8

#define MOTOR7FWD   36
#define MOTOR7REV   37
#define MOTOR7EN    9

int motors[8]; //Array which holds all the motors statuses.
void SetMotorPorts()
{
   pinMode(MOTOR0FWD,OUTPUT);
   pinMode(MOTOR0REV,OUTPUT);
   pinMode(MOTOR0EN,OUTPUT);
   pinMode(MOTOR1FWD,OUTPUT);
   pinMode(MOTOR1REV,OUTPUT);
   pinMode(MOTOR1EN,OUTPUT);
   pinMode(MOTOR2FWD,OUTPUT);
   pinMode(MOTOR2REV,OUTPUT);
   pinMode(MOTOR2EN,OUTPUT);
   pinMode(MOTOR3FWD,OUTPUT);
   pinMode(MOTOR3REV,OUTPUT);
   pinMode(MOTOR3EN,OUTPUT);
   pinMode(MOTOR4FWD,OUTPUT);
   pinMode(MOTOR4REV,OUTPUT);
   pinMode(MOTOR4EN,OUTPUT);
   pinMode(MOTOR5FWD,OUTPUT);
   pinMode(MOTOR5REV,OUTPUT);
   pinMode(MOTOR5EN,OUTPUT);
   pinMode(MOTOR6FWD,OUTPUT);
   pinMode(MOTOR6REV,OUTPUT);
   pinMode(MOTOR6EN,OUTPUT);
   pinMode(MOTOR7FWD,OUTPUT);
   pinMode(MOTOR7REV,OUTPUT);
   pinMode(MOTOR7EN,OUTPUT);
}

void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  for(int i=0;i<8;i++) motors[i]=0; //Initialize motors speed to 0
  SetMotorPorts();  //Set digital outs and PWM outs
  establishContact();  // send a byte to establish contact until receiver responds 
}

void loop()
{
  if (Serial.available() > 0) {
    // get incoming byte:
    char data[33];
    int dc = 0;
    dc=Serial.readBytes(data,32);
    parseSerial(data,dc);
  }
}


void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('?');   // send a ?
    delay(300);
  }
}

void parseSerial(char *data,int datasz)
{
  data[datasz]='\0';  //Make sure it's null terminated.
  char *itr;
  char *name= strtok_r(data,"-",&itr); //Motor is anything up to the first dash.
  char *val= strtok_r(NULL,"\0",&itr); //Speed is anything after the first dash.
  int value = atoi(val);
  int motor= atoi(name);
  if(value==256)
  {
    Serial.print("Motor ");
    Serial.print(motor);
    Serial.print(" is set to speed ");
    Serial.print(motors[motor]);
  }
  else if(value<256&&value>-256) //If it's an acceptable value
  {
    motors[motor]=value;
    Serial.print("Setting motor ");
    Serial.print(motor);
    Serial.print(" to speed ");
    Serial.print(value);
    Serial.print('\n');
  }
}

void SetMotor(int motor,int value) //To be modified when other actuators are added.
{
  bool fwd=true;
  if(value<0) 
  {
    fwd=false;
    value=value*-1;
  }
  switch(motor)
  {
    case 0:
      analogWrite(MOTOR0EN,value);
      digitalWrite(MOTOR0FWD,fwd); 
      digitalWrite(MOTOR0REV,!fwd);
      break;
    case 1:
      analogWrite(MOTOR1EN,value);
      digitalWrite(MOTOR1FWD,fwd); 
      digitalWrite(MOTOR1REV,!fwd);
      break;
    case 2:
      analogWrite(MOTOR2EN,value);
      digitalWrite(MOTOR2FWD,fwd); 
      digitalWrite(MOTOR2REV,!fwd);
      break;
    case 3:
      analogWrite(MOTOR3EN,value);
      digitalWrite(MOTOR3FWD,fwd); 
      digitalWrite(MOTOR3REV,!fwd);
      break;
    case 4:
      analogWrite(MOTOR4EN,value);
      digitalWrite(MOTOR4FWD,fwd); 
      digitalWrite(MOTOR4REV,!fwd);
      break;
    case 5:
      analogWrite(MOTOR5EN,value);
      digitalWrite(MOTOR5FWD,fwd); 
      digitalWrite(MOTOR5REV,!fwd);
      break;
    case 6:      
      analogWrite(MOTOR6EN,value);
      digitalWrite(MOTOR6FWD,fwd); 
      digitalWrite(MOTOR6REV,!fwd);
      break;
    case 7:
      analogWrite(MOTOR7EN,value);
      digitalWrite(MOTOR7FWD,fwd); 
      digitalWrite(MOTOR7REV,!fwd);
      break;
    break;
  }
}
