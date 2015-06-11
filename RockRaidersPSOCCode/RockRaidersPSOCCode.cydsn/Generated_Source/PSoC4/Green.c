/*******************************************************************************
* File Name: Green.c  
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
#include "Green.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        Green_PC =   (Green_PC & \
                                (uint32)(~(uint32)(Green_DRIVE_MODE_IND_MASK << (Green_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (Green_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: Green_Write
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
void Green_Write(uint8 value) 
{
    uint8 drVal = (uint8)(Green_DR & (uint8)(~Green_MASK));
    drVal = (drVal | ((uint8)(value << Green_SHIFT) & Green_MASK));
    Green_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: Green_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Green_DM_STRONG     Strong Drive 
*  Green_DM_OD_HI      Open Drain, Drives High 
*  Green_DM_OD_LO      Open Drain, Drives Low 
*  Green_DM_RES_UP     Resistive Pull Up 
*  Green_DM_RES_DWN    Resistive Pull Down 
*  Green_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Green_DM_DIG_HIZ    High Impedance Digital 
*  Green_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Green_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(Green__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: Green_Read
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
*  Macro Green_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Green_Read(void) 
{
    return (uint8)((Green_PS & Green_MASK) >> Green_SHIFT);
}


/*******************************************************************************
* Function Name: Green_ReadDataReg
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
uint8 Green_ReadDataReg(void) 
{
    return (uint8)((Green_DR & Green_MASK) >> Green_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Green_INTSTAT) 

    /*******************************************************************************
    * Function Name: Green_ClearInterrupt
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
    uint8 Green_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(Green_INTSTAT & Green_MASK);
		Green_INTSTAT = maskedStatus;
        return maskedStatus >> Green_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
