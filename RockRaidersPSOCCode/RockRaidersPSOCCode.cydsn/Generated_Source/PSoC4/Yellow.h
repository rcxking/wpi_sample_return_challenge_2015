/*******************************************************************************
* File Name: Yellow.h  
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

#if !defined(CY_PINS_Yellow_H) /* Pins Yellow_H */
#define CY_PINS_Yellow_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Yellow_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    Yellow_Write(uint8 value) ;
void    Yellow_SetDriveMode(uint8 mode) ;
uint8   Yellow_ReadDataReg(void) ;
uint8   Yellow_Read(void) ;
uint8   Yellow_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Yellow_DRIVE_MODE_BITS        (3)
#define Yellow_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Yellow_DRIVE_MODE_BITS))

#define Yellow_DM_ALG_HIZ         (0x00u)
#define Yellow_DM_DIG_HIZ         (0x01u)
#define Yellow_DM_RES_UP          (0x02u)
#define Yellow_DM_RES_DWN         (0x03u)
#define Yellow_DM_OD_LO           (0x04u)
#define Yellow_DM_OD_HI           (0x05u)
#define Yellow_DM_STRONG          (0x06u)
#define Yellow_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define Yellow_MASK               Yellow__MASK
#define Yellow_SHIFT              Yellow__SHIFT
#define Yellow_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Yellow_PS                     (* (reg32 *) Yellow__PS)
/* Port Configuration */
#define Yellow_PC                     (* (reg32 *) Yellow__PC)
/* Data Register */
#define Yellow_DR                     (* (reg32 *) Yellow__DR)
/* Input Buffer Disable Override */
#define Yellow_INP_DIS                (* (reg32 *) Yellow__PC2)


#if defined(Yellow__INTSTAT)  /* Interrupt Registers */

    #define Yellow_INTSTAT                (* (reg32 *) Yellow__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define Yellow_DRIVE_MODE_SHIFT       (0x00u)
#define Yellow_DRIVE_MODE_MASK        (0x07u << Yellow_DRIVE_MODE_SHIFT)


#endif /* End Pins Yellow_H */


/* [] END OF FILE */
