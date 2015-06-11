/*******************************************************************************
* File Name: Limit_RL.c  
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
#include "Limit_RL.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        Limit_RL_PC =   (Limit_RL_PC & \
                                (uint32)(~(uint32)(Limit_RL_DRIVE_MODE_IND_MASK << (Limit_RL_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (Limit_RL_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: Limit_RL_Write
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
void Limit_RL_Write(uint8 value) 
{
    uint8 drVal = (uint8)(Limit_RL_DR & (uint8)(~Limit_RL_MASK));
    drVal = (drVal | ((uint8)(value << Limit_RL_SHIFT) & Limit_RL_MASK));
    Limit_RL_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: Limit_RL_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Limit_RL_DM_STRONG     Strong Drive 
*  Limit_RL_DM_OD_HI      Open Drain, Drives High 
*  Limit_RL_DM_OD_LO      Open Drain, Drives Low 
*  Limit_RL_DM_RES_UP     Resistive Pull Up 
*  Limit_RL_DM_RES_DWN    Resistive Pull Down 
*  Limit_RL_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Limit_RL_DM_DIG_HIZ    High Impedance Digital 
*  Limit_RL_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Limit_RL_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(Limit_RL__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: Limit_RL_Read
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
*  Macro Limit_RL_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Limit_RL_Read(void) 
{
    return (uint8)((Limit_RL_PS & Limit_RL_MASK) >> Limit_RL_SHIFT);
}


/*******************************************************************************
* Function Name: Limit_RL_ReadDataReg
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
uint8 Limit_RL_ReadDataReg(void) 
{
    return (uint8)((Limit_RL_DR & Limit_RL_MASK) >> Limit_RL_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Limit_RL_INTSTAT) 

    /*******************************************************************************
    * Function Name: Limit_RL_ClearInterrupt
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
    uint8 Limit_RL_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(Limit_RL_INTSTAT & Limit_RL_MASK);
		Limit_RL_INTSTAT = maskedStatus;
        return maskedStatus >> Limit_RL_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
