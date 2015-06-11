/*******************************************************************************
* File Name: PCComms_tx.h  
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

#if !defined(CY_PINS_PCComms_tx_H) /* Pins PCComms_tx_H */
#define CY_PINS_PCComms_tx_H

#include "cytypes.h"
#include "cyfitter.h"
#include "PCComms_tx_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    PCComms_tx_Write(uint8 value) ;
void    PCComms_tx_SetDriveMode(uint8 mode) ;
uint8   PCComms_tx_ReadDataReg(void) ;
uint8   PCComms_tx_Read(void) ;
uint8   PCComms_tx_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define PCComms_tx_DRIVE_MODE_BITS        (3)
#define PCComms_tx_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - PCComms_tx_DRIVE_MODE_BITS))

#define PCComms_tx_DM_ALG_HIZ         (0x00u)
#define PCComms_tx_DM_DIG_HIZ         (0x01u)
#define PCComms_tx_DM_RES_UP          (0x02u)
#define PCComms_tx_DM_RES_DWN         (0x03u)
#define PCComms_tx_DM_OD_LO           (0x04u)
#define PCComms_tx_DM_OD_HI           (0x05u)
#define PCComms_tx_DM_STRONG          (0x06u)
#define PCComms_tx_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define PCComms_tx_MASK               PCComms_tx__MASK
#define PCComms_tx_SHIFT              PCComms_tx__SHIFT
#define PCComms_tx_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define PCComms_tx_PS                     (* (reg32 *) PCComms_tx__PS)
/* Port Configuration */
#define PCComms_tx_PC                     (* (reg32 *) PCComms_tx__PC)
/* Data Register */
#define PCComms_tx_DR                     (* (reg32 *) PCComms_tx__DR)
/* Input Buffer Disable Override */
#define PCComms_tx_INP_DIS                (* (reg32 *) PCComms_tx__PC2)


#if defined(PCComms_tx__INTSTAT)  /* Interrupt Registers */

    #define PCComms_tx_INTSTAT                (* (reg32 *) PCComms_tx__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define PCComms_tx_DRIVE_MODE_SHIFT       (0x00u)
#define PCComms_tx_DRIVE_MODE_MASK        (0x07u << PCComms_tx_DRIVE_MODE_SHIFT)


#endif /* End Pins PCComms_tx_H */


/* [] END OF FILE */
