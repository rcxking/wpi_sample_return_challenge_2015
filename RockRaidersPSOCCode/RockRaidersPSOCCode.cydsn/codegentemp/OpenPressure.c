/*******************************************************************************
* File Name: OpenPressure.c  
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
#include "OpenPressure.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        OpenPressure_PC =   (OpenPressure_PC & \
                                (uint32)(~(uint32)(OpenPressure_DRIVE_MODE_IND_MASK << (OpenPressure_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (OpenPressure_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: OpenPressure_Write
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
void OpenPressure_Write(uint8 value) 
{
    uint8 drVal = (uint8)(OpenPressure_DR & (uint8)(~OpenPressure_MASK));
    drVal = (drVal | ((uint8)(value << OpenPressure_SHIFT) & OpenPressure_MASK));
    OpenPressure_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: OpenPressure_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  OpenPressure_DM_STRONG     Strong Drive 
*  OpenPressure_DM_OD_HI      Open Drain, Drives High 
*  OpenPressure_DM_OD_LO      Open Drain, Drives Low 
*  OpenPressure_DM_RES_UP     Resistive Pull Up 
*  OpenPressure_DM_RES_DWN    Resistive Pull Down 
*  OpenPressure_DM_RES_UPDWN  Resistive Pull Up/Down 
*  OpenPressure_DM_DIG_HIZ    High Impedance Digital 
*  OpenPressure_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void OpenPressure_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(OpenPressure__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: OpenPressure_Read
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
*  Macro OpenPressure_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 OpenPressure_Read(void) 
{
    return (uint8)((OpenPressure_PS & OpenPressure_MASK) >> OpenPressure_SHIFT);
}


/*******************************************************************************
* Function Name: OpenPressure_ReadDataReg
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
uint8 OpenPressure_ReadDataReg(void) 
{
    return (uint8)((OpenPressure_DR & OpenPressure_MASK) >> OpenPressure_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(OpenPressure_INTSTAT) 

    /*******************************************************************************
    * Function Name: OpenPressure_ClearInterrupt
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
    uint8 OpenPressure_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(OpenPressure_INTSTAT & OpenPressure_MASK);
		OpenPressure_INTSTAT = maskedStatus;
        return maskedStatus >> OpenPressure_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
