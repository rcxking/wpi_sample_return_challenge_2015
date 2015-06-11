/*******************************************************************************
* File Name: MotorControl4EN.h  
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

#if !defined(CY_PINS_MotorControl4EN_H) /* Pins MotorControl4EN_H */
#define CY_PINS_MotorControl4EN_H

#include "cytypes.h"
#include "cyfitter.h"
#include "MotorControl4EN_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    MotorControl4EN_Write(uint8 value) ;
void    MotorControl4EN_SetDriveMode(uint8 mode) ;
uint8   MotorControl4EN_ReadDataReg(void) ;
uint8   MotorControl4EN_Read(void) ;
uint8   MotorControl4EN_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define MotorControl4EN_DRIVE_MODE_BITS        (3)
#define MotorControl4EN_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - MotorControl4EN_DRIVE_MODE_BITS))

#define MotorControl4EN_DM_ALG_HIZ         (0x00u)
#define MotorControl4EN_DM_DIG_HIZ         (0x01u)
#define MotorControl4EN_DM_RES_UP          (0x02u)
#define MotorControl4EN_DM_RES_DWN         (0x03u)
#define MotorControl4EN_DM_OD_LO           (0x04u)
#define MotorControl4EN_DM_OD_HI           (0x05u)
#define MotorControl4EN_DM_STRONG          (0x06u)
#define MotorControl4EN_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define MotorControl4EN_MASK               MotorControl4EN__MASK
#define MotorControl4EN_SHIFT              MotorControl4EN__SHIFT
#define MotorControl4EN_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define MotorControl4EN_PS                     (* (reg32 *) MotorControl4EN__PS)
/* Port Configuration */
#define MotorControl4EN_PC                     (* (reg32 *) MotorControl4EN__PC)
/* Data Register */
#define MotorControl4EN_DR                     (* (reg32 *) MotorControl4EN__DR)
/* Input Buffer Disable Override */
#define MotorControl4EN_INP_DIS                (* (reg32 *) MotorControl4EN__PC2)


#if defined(MotorControl4EN__INTSTAT)  /* Interrupt Registers */

    #define MotorControl4EN_INTSTAT                (* (reg32 *) MotorControl4EN__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define MotorControl4EN_DRIVE_MODE_SHIFT       (0x00u)
#define MotorControl4EN_DRIVE_MODE_MASK        (0x07u << MotorControl4EN_DRIVE_MODE_SHIFT)


#endif /* End Pins MotorControl4EN_H */


/* [] END OF FILE */
