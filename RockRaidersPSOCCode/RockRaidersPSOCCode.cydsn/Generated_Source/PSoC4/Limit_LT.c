/*******************************************************************************
* File Name: Limit_LT.c  
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
#include "Limit_LT.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        Limit_LT_PC =   (Limit_LT_PC & \
                                (uint32)(~(uint32)(Limit_LT_DRIVE_MODE_IND_MASK << (Limit_LT_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (Limit_LT_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: Limit_LT_Write
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
void Limit_LT_Write(uint8 value) 
{
    uint8 drVal = (uint8)(Limit_LT_DR & (uint8)(~Limit_LT_MASK));
    drVal = (drVal | ((uint8)(value << Limit_LT_SHIFT) & Limit_LT_MASK));
    Limit_LT_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: Limit_LT_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Limit_LT_DM_STRONG     Strong Drive 
*  Limit_LT_DM_OD_HI      Open Drain, Drives High 
*  Limit_LT_DM_OD_LO      Open Drain, Drives Low 
*  Limit_LT_DM_RES_UP     Resistive Pull Up 
*  Limit_LT_DM_RES_DWN    Resistive Pull Down 
*  Limit_LT_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Limit_LT_DM_DIG_HIZ    High Impedance Digital 
*  Limit_LT_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Limit_LT_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(Limit_LT__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: Limit_LT_Read
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
*  Macro Limit_LT_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Limit_LT_Read(void) 
{
    return (uint8)((Limit_LT_PS & Limit_LT_MASK) >> Limit_LT_SHIFT);
}


/*******************************************************************************
* Function Name: Limit_LT_ReadDataReg
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
uint8 Limit_LT_ReadDataReg(void) 
{
    return (uint8)((Limit_LT_DR & Limit_LT_MASK) >> Limit_LT_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Limit_LT_INTSTAT) 

    /*******************************************************************************
    * Function Name: Limit_LT_ClearInterrupt
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
    uint8 Limit_LT_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(Limit_LT_INTSTAT & Limit_LT_MASK);
		Limit_LT_INTSTAT = maskedStatus;
        return maskedStatus >> Limit_LT_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
