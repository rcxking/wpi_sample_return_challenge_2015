#include <string.h> //strtok_r

<<<<<<< HEAD
#define MOTOR0PWM   2
#define MOTOR0FWD   22 
#define MOTOR0REV   24
#define MOTOR0EN    26

#define MOTOR1PWM   3
#define MOTOR1FWD   23
#define MOTOR1REV   25
#define MOTOR1EN    27

#define MOTOR2PWM   4
#define MOTOR2FWD   28
#define MOTOR2REV   30
#define MOTOR2EN    32
=======
#define MOTOR0PWM   3
#define MOTOR0FWD   10
#define MOTOR0REV   9
#define MOTOR0EN    8

#define MOTOR1PWM   3
#define MOTOR1FWD   7
#define MOTOR1REV   6
#define MOTOR1EN    5

#define MOTOR2PWM   3
#define MOTOR2FWD   4
#define MOTOR2REV   3
#define MOTOR2EN    2
>>>>>>> f7a700d3de4f3f6dc0f899e4912f4ac29953cfce

#define MOTOR3PWM   5
#define MOTOR3FWD   31
#define MOTOR3REV   32
#define MOTOR3EN    33

#define MOTOR4PWM   6
#define MOTOR4FWD   34
#define MOTOR4REV   35
#define MOTOR4EN    36

#define MOTOR5PWM   7
#define MOTOR5FWD   37
#define MOTOR5REV   38
#define MOTOR5EN    39

#define MOTOR6PWM   8
#define MOTOR6FWD   40
#define MOTOR6REV   41
#define MOTOR6EN    42

#define MOTOR7PWM   9
#define MOTOR7FWD   43
#define MOTOR7REV   44
#define MOTOR7EN    45

#define SOLENOID1 12
#define SOLENOID2 13

int motors[8]; //Array which holds all the motors statuses.
void SetMotorPorts()
{
   pinMode(MOTOR0FWD,OUTPUT);
   pinMode(MOTOR0REV,OUTPUT);
   pinMode(MOTOR0EN,OUTPUT);
   pinMode(MOTOR0PWM,OUTPUT);
   
   pinMode(MOTOR1FWD,OUTPUT);
   pinMode(MOTOR1REV,OUTPUT);
   pinMode(MOTOR1EN, OUTPUT);
   pinMode(MOTOR1PWM,OUTPUT);
   
   pinMode(MOTOR2FWD,OUTPUT);
   pinMode(MOTOR2REV,OUTPUT);
   pinMode(MOTOR2EN,OUTPUT);
   pinMode(MOTOR2PWM,OUTPUT);
   
   pinMode(MOTOR3FWD,OUTPUT);
   pinMode(MOTOR3REV,OUTPUT);
   pinMode(MOTOR3EN,OUTPUT);
   pinMode(MOTOR3PWM,OUTPUT);
   
   pinMode(MOTOR4FWD,OUTPUT);
   pinMode(MOTOR4REV,OUTPUT);
   pinMode(MOTOR4EN,OUTPUT);
   pinMode(MOTOR4PWM,OUTPUT);
   
   pinMode(MOTOR5FWD,OUTPUT);
   pinMode(MOTOR5REV,OUTPUT);
   pinMode(MOTOR5EN,OUTPUT);
   pinMode(MOTOR5PWM,OUTPUT);

   pinMode(MOTOR6FWD,OUTPUT);
   pinMode(MOTOR6REV,OUTPUT);
   pinMode(MOTOR6EN,OUTPUT);
   pinMode(MOTOR6PWM,OUTPUT);   
   
   pinMode(MOTOR7FWD,OUTPUT);
   pinMode(MOTOR7REV,OUTPUT);
   pinMode(MOTOR7EN,OUTPUT);
   pinMode(MOTOR7PWM,OUTPUT);
   
   pinMode(SOLENOID1, OUTPUT);
   pinMode(SOLENOID2, OUTPUT);
}

void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(115200);
  
  /*
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }*/
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
  
  // Variables for string tokenization:
  const char commaDelim[2] = ",";
  char *commaToken;
  
  // Get the next motor command:
  commaToken = strtok(data, commaDelim);
  
  while(commaToken != NULL) {
    
    Serial.print("Next command: ");
    Serial.println(commaToken);
    
    // Next parse the commands by commaToken:
    char *itr;
    char *name= strtok_r(commaToken,"-",&itr); //Motor is anything up to the first dash.
    char *val= strtok_r(NULL,"\0",&itr); //Speed is anything after the first dash.
    int value = atoi(val);
    int motor= atoi(name);
    
    if(value >= 50) {
      value = 50;
    }
    if(value <= -50) {
      value = -50;
    }
    
    // Handle Solenoids 1 and 2:
    if(motor == 9 || motor == 10) {
      digitalWrite(motor, HIGH);
      
      Serial.print("Solenoid ");
      if(motor == 9) {
        Serial.print(1);
      } else {
        Serial.print(2);
      }
      Serial.print(" is turned on\n");
    }
    else if(value==256)
    {
      Serial.print("Motor ");
      Serial.print(motor);
      Serial.print(" is set to speed ");
      Serial.print(motors[motor]);
      Serial.print('\n');
    }
    else if(value<256&&value>-256) //If it's an acceptable value
    {
      if(value > 25) {
        value = 25;
      }
      
      if(value < -25) {
        value = -25;
      }
      
      motors[motor]=value;
      SetMotor(motor,value);
      Serial.print("Setting motor ");
      Serial.print(motor);
      Serial.print(" to speed ");
      Serial.print(value);
      Serial.print('\n');
    }
    
    commaToken = strtok(NULL, commaDelim);
  } // End while
}

void SetMotor(int motor,int value) //To be modified when other actuators are added.
{
  bool fwd=true;
  if(value<0) 
  {
    fwd=false;
    value=value*-1;
  }
  
  // DON'T BREAK THE MOTORS
  if(value > 50) {
    value = 50;
  }
  switch(motor)
  {
    case 0:
      analogWrite(MOTOR0FWD,fwd*value); 
      analogWrite(MOTOR0REV,!fwd*value);
      digitalWrite(MOTOR0EN,1);
      //analogWrite(MOTOR0PWM,value);
      break;
    case 1:
      analogWrite(MOTOR1FWD,fwd*value); 
      analogWrite(MOTOR1REV,!fwd*value);
      digitalWrite(MOTOR1EN,1);
      //analogWrite(MOTOR1PWM,value);
      break;
    case 2:
      analogWrite(MOTOR2FWD,fwd*value); 
      analogWrite(MOTOR2REV,!fwd*value);
      digitalWrite(MOTOR2EN,1);
      //analogWrite(MOTOR2PWM,value);
      break;
    /*
    case 3:
      digitalWrite(MOTOR3FWD,fwd); 
      digitalWrite(MOTOR3REV,!fwd);
      digitalWrite(MOTOR3EN,1);
      analogWrite(MOTOR3PWM,value);
      break;
    case 4:
      digitalWrite(MOTOR4FWD,fwd); 
      digitalWrite(MOTOR4REV,!fwd);
      digitalWrite(MOTOR4EN,1);
      analogWrite(MOTOR4PWM,value);
      break;
    case 5:
      digitalWrite(MOTOR5FWD,fwd); 
      digitalWrite(MOTOR5REV,!fwd);
      digitalWrite(MOTOR5EN,1);
      analogWrite(MOTOR5PWM,value);
      break;
    case 6:      
      digitalWrite(MOTOR6FWD,fwd); 
      digitalWrite(MOTOR6REV,!fwd);
      digitalWrite(MOTOR6EN,1);
      analogWrite(MOTOR6PWM,value);
      break;
    case 7:
      digitalWrite(MOTOR7FWD,fwd); 
      digitalWrite(MOTOR7REV,!fwd);
      digitalWrite(MOTOR7EN,1);
      analogWrite(MOTOR7PWM,value);
      break;
    */
    break;
  }
}
