//Code wot does the stuff for the interface thing.
#include <project.h>
#include <stdlib.h>
#include <stdio.h>
uint8 line[128];
uint8 linesz=0;
int motors[12];
void SetMotor(int motor, int value);
void DisableAll();
void ReallyDisableEverything(); //AKA 'Pause'
void SerialCB()
{
    //This happens when PCComms interupt is called. Lego Batman.
    uint8 c = PCComms_UartGetChar();
    while(c!=0)
    {
        line[linesz++]=c;
        c = PCComms_UartGetChar();
    }
    PCComms_ClearRxInterruptSource(0xffff);
}
void parseSerial(uint8 *data,uint8 datasz);
int ScaleVal(int val);  //Scales a value to match our schema.
int main()
{
    PCComms_SetCustomInterruptHandler(&SerialCB);

    PCComms_Start();
    CyGlobalIntEnable;
    OpenExhaust_Write(1);
    CloseExhaust_Write(1);
    for(;;)
    {
        CyDelay(100);
        if(linesz !=0)
        {
            if(line[linesz-1]=='\n' || line[linesz-1]=='\r') //Assume that the computer isn't doing anything funky. 
            //Q. What is our computer's favorite dance?
            //A. The Robot!
            {
                parseSerial(line,linesz);
                line[linesz]='\0';
                linesz=0;                
            } 
        }
    }
}

void parseSerial(uint8 *data,uint8 datasz)
{
  data[datasz]='\0';  //Make sure it's null terminated. Arnold would approve.
    char *itr;
    char *name= strtok_r((char*)data,"-",&itr); //Motor is anything up to the first dash.
    char *val= strtok_r(NULL,"\0",&itr); //Speed is anything after the first dash.
    int value = atoi(val);
    int motor= atoi(name);
    char buff[255];
    if(motor==0&&value==0)//SHUT. DOWN. EVERYTHING..
    {
        ReallyDisableEverything();
        return;
    }
    if(motor==10) //THE CLAW!
    {
        if(value==1) //OPEN
        {
            CloseExhaust_Write(0);
            ClosePressure_Write(0);
            OpenExhaust_Write(1);
            OpenExhaust_Write(1);
        }
        else if(value==-1) //CLOSE
        {
            CloseExhaust_Write(1);
            ClosePressure_Write(1);
            OpenExhaust_Write(0);
            OpenExhaust_Write(0);
        }
        return;   
    }
    if(motor ==11 || motor ==12)//Red or Green light
    {
        if(motor==11)
            Red_Write(value);
        else if(motor==12)
            Green_Write(value);
        return;
    }
    if(value==255)
    {
      snprintf(buff,255,"Motor %d is set to speed %d\r\n",motor,motors[motor]);
      PCComms_UartPutString(buff);
    }
    else if(value<128&&value>-128) //If it's an acceptable value
    {      
      motors[motor]=value;
      SetMotor(motor,ScaleVal(value));
      snprintf(buff,255,"Setting motor %d to speed %d\r\n",motor,value);
      PCComms_UartPutString(buff);
    }
    else
    {
       snprintf(buff,255,"Bad data. Got: motor:%s value:%s\r\n",motor,value);
       PCComms_UartPutString(buff);
    }
}

void DisableAll()   //This is the function which cripples all the EN pins. Reminds me of a fire at an orphanage.
{
    MotorControl1EN_Write(0);
    MotorControl2EN_Write(0);
    MotorControl3EN_Write(0);
    MotorControl4EN_Write(0);
    MotorControl5EN_Write(0);
    FPGA_EN_Write(0);
}

void SetMotor(int motor, int value) //Sets the speed on the motors. TODO: Remove 'Kill All Humans' subroutine.
{
    switch(motor)
    {
        case 1: //Rear Right motor, MC1-1
            DisableAll();
            MotorControl1EN_Write(1);
            MotorComms_UartPutChar(value);
        break;
        case 2: //Front Right motor, MC1-2
            DisableAll();
            MotorControl1EN_Write(1);
            MotorComms_UartPutChar(value+127);
        break;
        case 3: //Rear Left motor, MC2-1
            DisableAll();
            MotorControl2EN_Write(1);
            MotorComms_UartPutChar(value);
        break;
        case 4: //Front Left motor, MC2-2
            DisableAll();
            MotorControl2EN_Write(1);
            MotorComms_UartPutChar(value+127);
        break;
        case 5: //Arm Rotation motor, MC3-1
            DisableAll();
            MotorControl3EN_Write(1);
            MotorComms_UartPutChar(value);
        break;
        case 6: //Hand Rotation motor, MC3-2
            DisableAll();
            MotorControl3EN_Write(1);
            MotorComms_UartPutChar(value+127);
        break;
        case 7: //Arm Z Axis motor, MC4-1
            DisableAll();
            MotorControl4EN_Write(1);
            MotorComms_UartPutChar(value);
        break;
        case 8: //Right Actuator motor, MC5-1
            DisableAll();
            MotorControl5EN_Write(1);
            MotorComms_UartPutChar(value);
        break;
        case 9: //Left Actuator motor, MC5-2
            DisableAll();
            MotorControl5EN_Write(1);
            MotorComms_UartPutChar(value+127);
    }
    
}
int ScaleVal(int val)
{
  if(val>=64) val=63;
  if(val<=-64) val=-63;
  return val+64;
}
void ReallyDisableEverything() //For the pause mode.
{
    DisableAll();
    MotorControl1EN_Write(1);
    MotorComms_UartPutChar(0);
    MotorControl1EN_Write(0);
    
    MotorControl2EN_Write(1);
    MotorComms_UartPutChar(0);
    MotorControl2EN_Write(0);
    
    MotorControl3EN_Write(1);
    MotorComms_UartPutChar(0);
    MotorControl3EN_Write(0);
    
    MotorControl4EN_Write(1);
    MotorComms_UartPutChar(0);
    MotorControl4EN_Write(0);
    
    MotorControl5EN_Write(1);
    MotorComms_UartPutChar(0);
    MotorControl5EN_Write(0);
}
/* [] END OF FILE */
