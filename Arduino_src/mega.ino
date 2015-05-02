#include <string.h> //strtok_r
int motors[8]; //Array which holds all the motors statuses.

void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  for(int i=0;i<8;i++) motors[i]=0; //Initialize motors speed to 0
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
//    analogWrite();
    Serial.print("Setting motor ");
    Serial.print(motor);
    Serial.print(" to speed ");
    Serial.print(value);
    Serial.print('\n');
  }
}
