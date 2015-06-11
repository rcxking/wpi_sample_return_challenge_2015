#include "TimerOne.h"

// Define limit switch pins
#define  RF  8
#define  RR  9
#define  LF  10
#define  LR  11

// Define light pins
#define  APIN  3
#define  RPIN  4
#define  GPIN  5

// Define request protocol
// Send this number to the arduino for that command
#define NONE   0
#define LIMITS 42
#define AMBERP 43
#define AMBERR 44
#define GREENH 45
#define GREENL 46
#define REDH   47
#define REDL   48

#define RESPONSE (char)100

char request = NONE;
char state = 0;

bool paused = true;

void setup() 
{
  // Define serial connection as 9600 baud
  Serial.begin(9600);
  
  // Set limit switch inputs to pullup
  pinMode(RF, INPUT);
  pinMode(RR, INPUT);
  pinMode(LF, INPUT);
  pinMode(LR, INPUT);
  
  // Set light outputs
  pinMode(APIN, OUTPUT);
  pinMode(RPIN, OUTPUT);
  pinMode(GPIN, OUTPUT);
  
  // Start in paused state
  digitalWrite(APIN, HIGH);
  
  Timer1.initialize(1000000);         // initialize timer1, and set a 1 second period
  Timer1.attachInterrupt(amberBlink);  // attaches callback() as a timer overflow interrupt
}

void loop() 
{
  // If data requested
  if(request)
  {
    switch(request)
    {
      case LIMITS:
      // Calculate state char
      state = digitalRead(RF)*0x8 + digitalRead(RR)*0x4 + digitalRead(LF)*0x2 + digitalRead(LR)*0x1;
      // Send state to computer
      Serial.print(state);
      break;
      
      case AMBERP:
      // Set pause to true
      paused = true;
      digitalWrite(APIN, HIGH);
      Serial.print(RESPONSE);
      break;
      
      case AMBERR:
      // Set pause to false
      paused = false;
      Serial.print(RESPONSE);
      break;
      
      case GREENH:
      // Turn on green light
      digitalWrite(GPIN, HIGH);
      Serial.print(RESPONSE);
      break;
      
      case GREENL:
      // Turn off green light
      digitalWrite(GPIN, LOW);
      Serial.print(RESPONSE);
      break;
      
      case REDH:
      // Turn on red light
      digitalWrite(RPIN, HIGH);
      Serial.print(RESPONSE);
      break;
      
      case REDL:
      // Turn off green light
      digitalWrite(RPIN, LOW);
      Serial.print(RESPONSE);
      break;
    }
    
    // Reset the request flag
    request = NONE;
  }
}

// Serial callback
void serialEvent() 
{
  // Check for available data
  while (Serial.available()) 
  {
    // Store incoming data
    request = (char)Serial.read();
  }
}

void amberBlink()
{
  if(!paused)
  {
    // Toggle the amber light
    digitalWrite(APIN, !digitalRead(APIN));
  }
}
