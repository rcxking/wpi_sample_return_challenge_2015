/*******************************************************************************
* File Name: Steering_Register_PM.c
* Version 1.80
*
* Description:
*  This file contains the setup, control, and status commands to support 
*  the component operation in the low power mode. 
*
* Note:
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "Steering_Register.h"

/* Check for removal by optimization */
#if !defined(Steering_Register_Sync_ctrl_reg__REMOVED)

static Steering_Register_BACKUP_STRUCT  Steering_Register_backup = {0u};

    
/*******************************************************************************
* Function Name: Steering_Register_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the control register value.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Steering_Register_SaveConfig(void) 
{
    Steering_Register_backup.controlState = Steering_Register_Control;
}


/*******************************************************************************
* Function Name: Steering_Register_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the control register value.
*
* Parameters:
*  None
*
* Return:
*  None
*
*
*******************************************************************************/
void Steering_Register_RestoreConfig(void) 
{
     Steering_Register_Control = Steering_Register_backup.controlState;
}


/*******************************************************************************
* Function Name: Steering_Register_Sleep
********************************************************************************
*
* Summary:
*  Prepares the component for entering the low power mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Steering_Register_Sleep(void) 
{
    Steering_Register_SaveConfig();
}


/*******************************************************************************
* Function Name: Steering_Register_Wakeup
********************************************************************************
*
* Summary:
*  Restores the component after waking up from the low power mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Steering_Register_Wakeup(void)  
{
    Steering_Register_RestoreConfig();
}

#endif /* End check for removal by optimization */


/* [] END OF FILE */
