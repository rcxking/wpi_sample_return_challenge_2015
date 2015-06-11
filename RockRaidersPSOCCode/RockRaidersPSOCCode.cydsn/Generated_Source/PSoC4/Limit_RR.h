/*******************************************************************************
* File Name: Limit_RR.h  
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

#if !defined(CY_PINS_Limit_RR_H) /* Pins Limit_RR_H */
#define CY_PINS_Limit_RR_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Limit_RR_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    Limit_RR_Write(uint8 value) ;
void    Limit_RR_SetDriveMode(uint8 mode) ;
uint8   Limit_RR_ReadDataReg(void) ;
uint8   Limit_RR_Read(void) ;
uint8   Limit_RR_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Limit_RR_DRIVE_MODE_BITS        (3)
#define Limit_RR_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Limit_RR_DRIVE_MODE_BITS))

#define Limit_RR_DM_ALG_HIZ         (0x00u)
#define Limit_RR_DM_DIG_HIZ         (0x01u)
#define Limit_RR_DM_RES_UP          (0x02u)
#define Limit_RR_DM_RES_DWN         (0x03u)
#define Limit_RR_DM_OD_LO           (0x04u)
#define Limit_RR_DM_OD_HI           (0x05u)
#define Limit_RR_DM_STRONG          (0x06u)
#define Limit_RR_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define Limit_RR_MASK               Limit_RR__MASK
#define Limit_RR_SHIFT              Limit_RR__SHIFT
#define Limit_RR_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Limit_RR_PS                     (* (reg32 *) Limit_RR__PS)
/* Port Configuration */
#define Limit_RR_PC                     (* (reg32 *) Limit_RR__PC)
/* Data Register */
#define Limit_RR_DR                     (* (reg32 *) Limit_RR__DR)
/* Input Buffer Disable Override */
#define Limit_RR_INP_DIS                (* (reg32 *) Limit_RR__PC2)


#if defined(Limit_RR__INTSTAT)  /* Interrupt Registers */

    #define Limit_RR_INTSTAT                (* (reg32 *) Limit_RR__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define Limit_RR_DRIVE_MODE_SHIFT       (0x00u)
#define Limit_RR_DRIVE_MODE_MASK        (0x07u << Limit_RR_DRIVE_MODE_SHIFT)


#endif /* End Pins Limit_RR_H */


/* [] END OF FILE */
