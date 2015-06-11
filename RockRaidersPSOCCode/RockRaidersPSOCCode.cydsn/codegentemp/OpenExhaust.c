/*******************************************************************************
* File Name: OpenExhaust.c  
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
#include "OpenExhaust.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        OpenExhaust_PC =   (OpenExhaust_PC & \
                                (uint32)(~(uint32)(OpenExhaust_DRIVE_MODE_IND_MASK << (OpenExhaust_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (OpenExhaust_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: OpenExhaust_Write
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
void OpenExhaust_Write(uint8 value) 
{
    uint8 drVal = (uint8)(OpenExhaust_DR & (uint8)(~OpenExhaust_MASK));
    drVal = (drVal | ((uint8)(value << OpenExhaust_SHIFT) & OpenExhaust_MASK));
    OpenExhaust_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: OpenExhaust_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  OpenExhaust_DM_STRONG     Strong Drive 
*  OpenExhaust_DM_OD_HI      Open Drain, Drives High 
*  OpenExhaust_DM_OD_LO      Open Drain, Drives Low 
*  OpenExhaust_DM_RES_UP     Resistive Pull Up 
*  OpenExhaust_DM_RES_DWN    Resistive Pull Down 
*  OpenExhaust_DM_RES_UPDWN  Resistive Pull Up/Down 
*  OpenExhaust_DM_DIG_HIZ    High Impedance Digital 
*  OpenExhaust_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void OpenExhaust_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(OpenExhaust__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: OpenExhaust_Read
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
*  Macro OpenExhaust_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 OpenExhaust_Read(void) 
{
    return (uint8)((OpenExhaust_PS & OpenExhaust_MASK) >> OpenExhaust_SHIFT);
}


/*******************************************************************************
* Function Name: OpenExhaust_ReadDataReg
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
uint8 OpenExhaust_ReadDataReg(void) 
{
    return (uint8)((OpenExhaust_DR & OpenExhaust_MASK) >> OpenExhaust_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(OpenExhaust_INTSTAT) 

    /*******************************************************************************
    * Function Name: OpenExhaust_ClearInterrupt
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
    uint8 OpenExhaust_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(OpenExhaust_INTSTAT & OpenExhaust_MASK);
		OpenExhaust_INTSTAT = maskedStatus;
        return maskedStatus >> OpenExhaust_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
