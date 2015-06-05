#include <project.h>
#include <stdlib.h>
uint8 line[128];
uint8 linesz=0;
int motors[12];
void SerialCB()
{
    //This happens when PCComms interupt is called
    uint8 c = PCComms_UartGetChar();
    while(c!=0)
    {
        line[linesz++]=c;
        c = PCComms_UartGetChar();
    }
    PCComms_ClearRxInterruptSource(0xffff);
}
void parseSerial(uint8 *data,uint8 datasz);
int main()
{
    PCComms_SetCustomInterruptHandler(&SerialCB);

    PCComms_Start();
    CyGlobalIntEnable;
    for(;;)
    {
        PCComms_UartPutString("TEST");
        /*if(linesz !=0)
        {
            if(line[linesz]=='\n')
            {
                parseSerial(line,linesz);
                line[linesz]='\0';
                linesz=0;
                
            }   
        }
        PCComms_UartPutChar(line[linesz]);*/
    }
}

void parseSerial(uint8 *data,uint8 datasz)
{
  data[datasz]='\0';  //Make sure it's null terminated.  
    char *itr;
    char *name= strtok_r((char*)data,"-",&itr); //Motor is anything up to the first dash.
    char *val= strtok_r(NULL,"\0",&itr); //Speed is anything after the first dash.
    int value = atoi(val);
    int motor= atoi(name);
    if(value==255)
    {
      PCComms_UartPutString("Motor ");
      PCComms_UartPutChar(motor);
      PCComms_UartPutString(" is set to speed ");
      PCComms_UartPutChar(motors[motor]);
      PCComms_UartPutCRLF(1);
    }
    else if(value<128&&value>-128) //If it's an acceptable value
    {      
      motors[motor]=value;
      //char buff[128];
      //sprintf(buff,"Setting motor %d to speed %d\r\n",motor,value);
      //SetMotor(motor,value);
      PCComms_UartPutString("Setting motor ");
      PCComms_UartPutChar(motor);
      PCComms_UartPutString(" to speed ");
      PCComms_UartPutChar(value);
      PCComms_UartPutCRLF(1);
    }
}

/* [] END OF FILE */
