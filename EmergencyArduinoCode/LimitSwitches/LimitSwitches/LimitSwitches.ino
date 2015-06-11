// Define limit switch pins
#define  RF  2
#define  RR  3
#define  LF  4
#define  LR  5

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

char request = NONE;
char state = 0;

bool paused = true;

void setup() 
{
  // Define serial connection as 9600 baud
  Serial.begin(9600);
  
  // Set limit switch inputs to pullup
  pinMode(RF, INPUT_PULLUP);
  pinMode(RR, INPUT_PULLUP);
  pinMode(LF, INPUT_PULLUP);
  pinMode(LR, INPUT_PULLUP);
  
  // Set light outputs
  pinMode(APIN, OUTPUT);
  pinMode(RPIN, OUTPUT);
  pinMode(GPIN, OUTPUT);
  
  // Start in paused state
  digitalWrite(APIN, HIGH);
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
      
      case AMBERR:
      // Set pause to true
      break;
      
      case GREENH:
      break;
      
      case GREENL:
      break;
      
      case REDH:
      break;
      
      case REDL:
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
    char request = (char)Serial.read();
  }
}

void amberBlink()
{
  // Toggle the amber light
  digitalWrite(APIN, !digitalRead(APIN));
}
