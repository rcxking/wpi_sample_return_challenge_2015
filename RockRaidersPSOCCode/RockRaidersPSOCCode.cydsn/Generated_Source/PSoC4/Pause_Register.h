/*******************************************************************************
* File Name: Pause_Register.h  
* Version 1.80
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CONTROL_REG_Pause_Register_H) /* CY_CONTROL_REG_Pause_Register_H */
#define CY_CONTROL_REG_Pause_Register_H

#include "cytypes.h"

    
/***************************************
*     Data Struct Definitions
***************************************/

/* Sleep Mode API Support */
typedef struct
{
    uint8 controlState;

} Pause_Register_BACKUP_STRUCT;


/***************************************
*         Function Prototypes 
***************************************/

void    Pause_Register_Write(uint8 control) ;
uint8   Pause_Register_Read(void) ;

void Pause_Register_SaveConfig(void) ;
void Pause_Register_RestoreConfig(void) ;
void Pause_Register_Sleep(void) ; 
void Pause_Register_Wakeup(void) ;


/***************************************
*            Registers        
***************************************/

/* Control Register */
#define Pause_Register_Control        (* (reg8 *) Pause_Register_Sync_ctrl_reg__CONTROL_REG )
#define Pause_Register_Control_PTR    (  (reg8 *) Pause_Register_Sync_ctrl_reg__CONTROL_REG )

#endif /* End CY_CONTROL_REG_Pause_Register_H */


/* [] END OF FILE */
