/*******************************************************************************
* File Name: Limit_RR.c  
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
#include "Limit_RR.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        Limit_RR_PC =   (Limit_RR_PC & \
                                (uint32)(~(uint32)(Limit_RR_DRIVE_MODE_IND_MASK << (Limit_RR_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (Limit_RR_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: Limit_RR_Write
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
void Limit_RR_Write(uint8 value) 
{
    uint8 drVal = (uint8)(Limit_RR_DR & (uint8)(~Limit_RR_MASK));
    drVal = (drVal | ((uint8)(value << Limit_RR_SHIFT) & Limit_RR_MASK));
    Limit_RR_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: Limit_RR_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Limit_RR_DM_STRONG     Strong Drive 
*  Limit_RR_DM_OD_HI      Open Drain, Drives High 
*  Limit_RR_DM_OD_LO      Open Drain, Drives Low 
*  Limit_RR_DM_RES_UP     Resistive Pull Up 
*  Limit_RR_DM_RES_DWN    Resistive Pull Down 
*  Limit_RR_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Limit_RR_DM_DIG_HIZ    High Impedance Digital 
*  Limit_RR_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Limit_RR_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(Limit_RR__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: Limit_RR_Read
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
*  Macro Limit_RR_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Limit_RR_Read(void) 
{
    return (uint8)((Limit_RR_PS & Limit_RR_MASK) >> Limit_RR_SHIFT);
}


/*******************************************************************************
* Function Name: Limit_RR_ReadDataReg
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
uint8 Limit_RR_ReadDataReg(void) 
{
    return (uint8)((Limit_RR_DR & Limit_RR_MASK) >> Limit_RR_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Limit_RR_INTSTAT) 

    /*******************************************************************************
    * Function Name: Limit_RR_ClearInterrupt
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
    uint8 Limit_RR_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(Limit_RR_INTSTAT & Limit_RR_MASK);
		Limit_RR_INTSTAT = maskedStatus;
        return maskedStatus >> Limit_RR_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
