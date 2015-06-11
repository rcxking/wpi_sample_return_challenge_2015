/*******************************************************************************
* File Name: MotorComms_tx.h  
* Version 2.10
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_MotorComms_tx_H) /* Pins MotorComms_tx_H */
#define CY_PINS_MotorComms_tx_H

#include "cytypes.h"
#include "cyfitter.h"
#include "MotorComms_tx_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    MotorComms_tx_Write(uint8 value) ;
void    MotorComms_tx_SetDriveMode(uint8 mode) ;
uint8   MotorComms_tx_ReadDataReg(void) ;
uint8   MotorComms_tx_Read(void) ;
uint8   MotorComms_tx_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define MotorComms_tx_DRIVE_MODE_BITS        (3)
#define MotorComms_tx_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - MotorComms_tx_DRIVE_MODE_BITS))

#define MotorComms_tx_DM_ALG_HIZ         (0x00u)
#define MotorComms_tx_DM_DIG_HIZ         (0x01u)
#define MotorComms_tx_DM_RES_UP          (0x02u)
#define MotorComms_tx_DM_RES_DWN         (0x03u)
#define MotorComms_tx_DM_OD_LO           (0x04u)
#define MotorComms_tx_DM_OD_HI           (0x05u)
#define MotorComms_tx_DM_STRONG          (0x06u)
#define MotorComms_tx_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define MotorComms_tx_MASK               MotorComms_tx__MASK
#define MotorComms_tx_SHIFT              MotorComms_tx__SHIFT
#define MotorComms_tx_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define MotorComms_tx_PS                     (* (reg32 *) MotorComms_tx__PS)
/* Port Configuration */
#define MotorComms_tx_PC                     (* (reg32 *) MotorComms_tx__PC)
/* Data Register */
#define MotorComms_tx_DR                     (* (reg32 *) MotorComms_tx__DR)
/* Input Buffer Disable Override */
#define MotorComms_tx_INP_DIS                (* (reg32 *) MotorComms_tx__PC2)


#if defined(MotorComms_tx__INTSTAT)  /* Interrupt Registers */

    #define MotorComms_tx_INTSTAT                (* (reg32 *) MotorComms_tx__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define MotorComms_tx_DRIVE_MODE_SHIFT       (0x00u)
#define MotorComms_tx_DRIVE_MODE_MASK        (0x07u << MotorComms_tx_DRIVE_MODE_SHIFT)


#endif /* End Pins MotorComms_tx_H */


/* [] END OF FILE */
