/*******************************************************************************
* File Name: phiA_Out.c  
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
#include "phiA_Out.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        phiA_Out_PC =   (phiA_Out_PC & \
                                (uint32)(~(uint32)(phiA_Out_DRIVE_MODE_IND_MASK << (phiA_Out_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (phiA_Out_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: phiA_Out_Write
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
void phiA_Out_Write(uint8 value) 
{
    uint8 drVal = (uint8)(phiA_Out_DR & (uint8)(~phiA_Out_MASK));
    drVal = (drVal | ((uint8)(value << phiA_Out_SHIFT) & phiA_Out_MASK));
    phiA_Out_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: phiA_Out_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  phiA_Out_DM_STRONG     Strong Drive 
*  phiA_Out_DM_OD_HI      Open Drain, Drives High 
*  phiA_Out_DM_OD_LO      Open Drain, Drives Low 
*  phiA_Out_DM_RES_UP     Resistive Pull Up 
*  phiA_Out_DM_RES_DWN    Resistive Pull Down 
*  phiA_Out_DM_RES_UPDWN  Resistive Pull Up/Down 
*  phiA_Out_DM_DIG_HIZ    High Impedance Digital 
*  phiA_Out_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void phiA_Out_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(phiA_Out__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: phiA_Out_Read
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
*  Macro phiA_Out_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 phiA_Out_Read(void) 
{
    return (uint8)((phiA_Out_PS & phiA_Out_MASK) >> phiA_Out_SHIFT);
}


/*******************************************************************************
* Function Name: phiA_Out_ReadDataReg
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
uint8 phiA_Out_ReadDataReg(void) 
{
    return (uint8)((phiA_Out_DR & phiA_Out_MASK) >> phiA_Out_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(phiA_Out_INTSTAT) 

    /*******************************************************************************
    * Function Name: phiA_Out_ClearInterrupt
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
    uint8 phiA_Out_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(phiA_Out_INTSTAT & phiA_Out_MASK);
		phiA_Out_INTSTAT = maskedStatus;
        return maskedStatus >> phiA_Out_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
