/*******************************************************************************
* File Name: MotorControl3EN.h  
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

#if !defined(CY_PINS_MotorControl3EN_H) /* Pins MotorControl3EN_H */
#define CY_PINS_MotorControl3EN_H

#include "cytypes.h"
#include "cyfitter.h"
#include "MotorControl3EN_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    MotorControl3EN_Write(uint8 value) ;
void    MotorControl3EN_SetDriveMode(uint8 mode) ;
uint8   MotorControl3EN_ReadDataReg(void) ;
uint8   MotorControl3EN_Read(void) ;
uint8   MotorControl3EN_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define MotorControl3EN_DRIVE_MODE_BITS        (3)
#define MotorControl3EN_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - MotorControl3EN_DRIVE_MODE_BITS))

#define MotorControl3EN_DM_ALG_HIZ         (0x00u)
#define MotorControl3EN_DM_DIG_HIZ         (0x01u)
#define MotorControl3EN_DM_RES_UP          (0x02u)
#define MotorControl3EN_DM_RES_DWN         (0x03u)
#define MotorControl3EN_DM_OD_LO           (0x04u)
#define MotorControl3EN_DM_OD_HI           (0x05u)
#define MotorControl3EN_DM_STRONG          (0x06u)
#define MotorControl3EN_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define MotorControl3EN_MASK               MotorControl3EN__MASK
#define MotorControl3EN_SHIFT              MotorControl3EN__SHIFT
#define MotorControl3EN_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define MotorControl3EN_PS                     (* (reg32 *) MotorControl3EN__PS)
/* Port Configuration */
#define MotorControl3EN_PC                     (* (reg32 *) MotorControl3EN__PC)
/* Data Register */
#define MotorControl3EN_DR                     (* (reg32 *) MotorControl3EN__DR)
/* Input Buffer Disable Override */
#define MotorControl3EN_INP_DIS                (* (reg32 *) MotorControl3EN__PC2)


#if defined(MotorControl3EN__INTSTAT)  /* Interrupt Registers */

    #define MotorControl3EN_INTSTAT                (* (reg32 *) MotorControl3EN__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define MotorControl3EN_DRIVE_MODE_SHIFT       (0x00u)
#define MotorControl3EN_DRIVE_MODE_MASK        (0x07u << MotorControl3EN_DRIVE_MODE_SHIFT)


#endif /* End Pins MotorControl3EN_H */


/* [] END OF FILE */
