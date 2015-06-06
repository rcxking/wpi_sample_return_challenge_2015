#include <project.h>
#include <stdlib.h>
#include <stdio.h>
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
        CyDelay(100);
        if(linesz !=0)
        {
            if(line[linesz-1]=='\n' || line[linesz-1]=='\r')
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
  data[datasz]='\0';  //Make sure it's null terminated.  
    char *itr;
    char *name= strtok_r((char*)data,"-",&itr); //Motor is anything up to the first dash.
    char *val= strtok_r(NULL,"\0",&itr); //Speed is anything after the first dash.
    int value = atoi(val);
    int motor= atoi(name);
    char buff[255];
    if(value==255)
    {
      snprintf(buff,255,"Motor %d is set to speed %d\r\n",motor,motors[motor]);
      PCComms_UartPutString(buff);
    }
    else if(value<128&&value>-128) //If it's an acceptable value
    {      
      motors[motor]=value;
      //SetMotor(motor,value);
      snprintf(buff,255,"Setting motor %d to speed %d\r\n",motor,value);
      PCComms_UartPutString(buff);
    }
    else
    {
       snprintf(buff,255,"Bad data. Got: motor:%s value:%s\r\n",motor,value);
       PCComms_UartPutString(buff);
    }
}

/* [] END OF FILE */
