/*******************************************************************************
* File Name: Limit_LS.h  
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

#if !defined(CY_PINS_Limit_LS_H) /* Pins Limit_LS_H */
#define CY_PINS_Limit_LS_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Limit_LS_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    Limit_LS_Write(uint8 value) ;
void    Limit_LS_SetDriveMode(uint8 mode) ;
uint8   Limit_LS_ReadDataReg(void) ;
uint8   Limit_LS_Read(void) ;
uint8   Limit_LS_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Limit_LS_DRIVE_MODE_BITS        (3)
#define Limit_LS_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Limit_LS_DRIVE_MODE_BITS))

#define Limit_LS_DM_ALG_HIZ         (0x00u)
#define Limit_LS_DM_DIG_HIZ         (0x01u)
#define Limit_LS_DM_RES_UP          (0x02u)
#define Limit_LS_DM_RES_DWN         (0x03u)
#define Limit_LS_DM_OD_LO           (0x04u)
#define Limit_LS_DM_OD_HI           (0x05u)
#define Limit_LS_DM_STRONG          (0x06u)
#define Limit_LS_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define Limit_LS_MASK               Limit_LS__MASK
#define Limit_LS_SHIFT              Limit_LS__SHIFT
#define Limit_LS_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Limit_LS_PS                     (* (reg32 *) Limit_LS__PS)
/* Port Configuration */
#define Limit_LS_PC                     (* (reg32 *) Limit_LS__PC)
/* Data Register */
#define Limit_LS_DR                     (* (reg32 *) Limit_LS__DR)
/* Input Buffer Disable Override */
#define Limit_LS_INP_DIS                (* (reg32 *) Limit_LS__PC2)


#if defined(Limit_LS__INTSTAT)  /* Interrupt Registers */

    #define Limit_LS_INTSTAT                (* (reg32 *) Limit_LS__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define Limit_LS_DRIVE_MODE_SHIFT       (0x00u)
#define Limit_LS_DRIVE_MODE_MASK        (0x07u << Limit_LS_DRIVE_MODE_SHIFT)


#endif /* End Pins Limit_LS_H */


/* [] END OF FILE */
