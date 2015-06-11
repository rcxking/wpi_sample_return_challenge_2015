/*******************************************************************************
* File Name: ClosePressure.h  
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

#if !defined(CY_PINS_ClosePressure_H) /* Pins ClosePressure_H */
#define CY_PINS_ClosePressure_H

#include "cytypes.h"
#include "cyfitter.h"
#include "ClosePressure_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    ClosePressure_Write(uint8 value) ;
void    ClosePressure_SetDriveMode(uint8 mode) ;
uint8   ClosePressure_ReadDataReg(void) ;
uint8   ClosePressure_Read(void) ;
uint8   ClosePressure_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define ClosePressure_DRIVE_MODE_BITS        (3)
#define ClosePressure_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - ClosePressure_DRIVE_MODE_BITS))

#define ClosePressure_DM_ALG_HIZ         (0x00u)
#define ClosePressure_DM_DIG_HIZ         (0x01u)
#define ClosePressure_DM_RES_UP          (0x02u)
#define ClosePressure_DM_RES_DWN         (0x03u)
#define ClosePressure_DM_OD_LO           (0x04u)
#define ClosePressure_DM_OD_HI           (0x05u)
#define ClosePressure_DM_STRONG          (0x06u)
#define ClosePressure_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define ClosePressure_MASK               ClosePressure__MASK
#define ClosePressure_SHIFT              ClosePressure__SHIFT
#define ClosePressure_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define ClosePressure_PS                     (* (reg32 *) ClosePressure__PS)
/* Port Configuration */
#define ClosePressure_PC                     (* (reg32 *) ClosePressure__PC)
/* Data Register */
#define ClosePressure_DR                     (* (reg32 *) ClosePressure__DR)
/* Input Buffer Disable Override */
#define ClosePressure_INP_DIS                (* (reg32 *) ClosePressure__PC2)


#if defined(ClosePressure__INTSTAT)  /* Interrupt Registers */

    #define ClosePressure_INTSTAT                (* (reg32 *) ClosePressure__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define ClosePressure_DRIVE_MODE_SHIFT       (0x00u)
#define ClosePressure_DRIVE_MODE_MASK        (0x07u << ClosePressure_DRIVE_MODE_SHIFT)


#endif /* End Pins ClosePressure_H */


/* [] END OF FILE */
