/*******************************************************************************
* File Name: MotorControl3EN.c  
* Version 2.10
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "MotorControl3EN.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        MotorControl3EN_PC =   (MotorControl3EN_PC & \
                                (uint32)(~(uint32)(MotorControl3EN_DRIVE_MODE_IND_MASK << (MotorControl3EN_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (MotorControl3EN_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: MotorControl3EN_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None 
*  
*******************************************************************************/
void MotorControl3EN_Write(uint8 value) 
{
    uint8 drVal = (uint8)(MotorControl3EN_DR & (uint8)(~MotorControl3EN_MASK));
    drVal = (drVal | ((uint8)(value << MotorControl3EN_SHIFT) & MotorControl3EN_MASK));
    MotorControl3EN_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: MotorControl3EN_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  MotorControl3EN_DM_STRONG     Strong Drive 
*  MotorControl3EN_DM_OD_HI      Open Drain, Drives High 
*  MotorControl3EN_DM_OD_LO      Open Drain, Drives Low 
*  MotorControl3EN_DM_RES_UP     Resistive Pull Up 
*  MotorControl3EN_DM_RES_DWN    Resistive Pull Down 
*  MotorControl3EN_DM_RES_UPDWN  Resistive Pull Up/Down 
*  MotorControl3EN_DM_DIG_HIZ    High Impedance Digital 
*  MotorControl3EN_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void MotorControl3EN_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(MotorControl3EN__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: MotorControl3EN_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro MotorControl3EN_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 MotorControl3EN_Read(void) 
{
    return (uint8)((MotorControl3EN_PS & MotorControl3EN_MASK) >> MotorControl3EN_SHIFT);
}


/*******************************************************************************
* Function Name: MotorControl3EN_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 MotorControl3EN_ReadDataReg(void) 
{
    return (uint8)((MotorControl3EN_DR & MotorControl3EN_MASK) >> MotorControl3EN_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(MotorControl3EN_INTSTAT) 

    /*******************************************************************************
    * Function Name: MotorControl3EN_ClearInterrupt
    ********************************************************************************
    *
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 MotorControl3EN_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(MotorControl3EN_INTSTAT & MotorControl3EN_MASK);
		MotorControl3EN_INTSTAT = maskedStatus;
        return maskedStatus >> MotorControl3EN_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
