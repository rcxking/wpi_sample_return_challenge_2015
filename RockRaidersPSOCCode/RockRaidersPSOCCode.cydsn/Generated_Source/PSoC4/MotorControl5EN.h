/*******************************************************************************
* File Name: MotorControl5EN.h  
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

#if !defined(CY_PINS_MotorControl5EN_H) /* Pins MotorControl5EN_H */
#define CY_PINS_MotorControl5EN_H

#include "cytypes.h"
#include "cyfitter.h"
#include "MotorControl5EN_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    MotorControl5EN_Write(uint8 value) ;
void    MotorControl5EN_SetDriveMode(uint8 mode) ;
uint8   MotorControl5EN_ReadDataReg(void) ;
uint8   MotorControl5EN_Read(void) ;
uint8   MotorControl5EN_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define MotorControl5EN_DRIVE_MODE_BITS        (3)
#define MotorControl5EN_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - MotorControl5EN_DRIVE_MODE_BITS))

#define MotorControl5EN_DM_ALG_HIZ         (0x00u)
#define MotorControl5EN_DM_DIG_HIZ         (0x01u)
#define MotorControl5EN_DM_RES_UP          (0x02u)
#define MotorControl5EN_DM_RES_DWN         (0x03u)
#define MotorControl5EN_DM_OD_LO           (0x04u)
#define MotorControl5EN_DM_OD_HI           (0x05u)
#define MotorControl5EN_DM_STRONG          (0x06u)
#define MotorControl5EN_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define MotorControl5EN_MASK               MotorControl5EN__MASK
#define MotorControl5EN_SHIFT              MotorControl5EN__SHIFT
#define MotorControl5EN_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define MotorControl5EN_PS                     (* (reg32 *) MotorControl5EN__PS)
/* Port Configuration */
#define MotorControl5EN_PC                     (* (reg32 *) MotorControl5EN__PC)
/* Data Register */
#define MotorControl5EN_DR                     (* (reg32 *) MotorControl5EN__DR)
/* Input Buffer Disable Override */
#define MotorControl5EN_INP_DIS                (* (reg32 *) MotorControl5EN__PC2)


#if defined(MotorControl5EN__INTSTAT)  /* Interrupt Registers */

    #define MotorControl5EN_INTSTAT                (* (reg32 *) MotorControl5EN__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define MotorControl5EN_DRIVE_MODE_SHIFT       (0x00u)
#define MotorControl5EN_DRIVE_MODE_MASK        (0x07u << MotorControl5EN_DRIVE_MODE_SHIFT)


#endif /* End Pins MotorControl5EN_H */


/* [] END OF FILE */
