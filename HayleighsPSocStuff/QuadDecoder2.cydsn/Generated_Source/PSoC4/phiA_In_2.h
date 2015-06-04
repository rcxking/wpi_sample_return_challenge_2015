/*******************************************************************************
* File Name: phiA_In_2.h  
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

#if !defined(CY_PINS_phiA_In_2_H) /* Pins phiA_In_2_H */
#define CY_PINS_phiA_In_2_H

#include "cytypes.h"
#include "cyfitter.h"
#include "phiA_In_2_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    phiA_In_2_Write(uint8 value) ;
void    phiA_In_2_SetDriveMode(uint8 mode) ;
uint8   phiA_In_2_ReadDataReg(void) ;
uint8   phiA_In_2_Read(void) ;
uint8   phiA_In_2_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define phiA_In_2_DRIVE_MODE_BITS        (3)
#define phiA_In_2_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - phiA_In_2_DRIVE_MODE_BITS))

#define phiA_In_2_DM_ALG_HIZ         (0x00u)
#define phiA_In_2_DM_DIG_HIZ         (0x01u)
#define phiA_In_2_DM_RES_UP          (0x02u)
#define phiA_In_2_DM_RES_DWN         (0x03u)
#define phiA_In_2_DM_OD_LO           (0x04u)
#define phiA_In_2_DM_OD_HI           (0x05u)
#define phiA_In_2_DM_STRONG          (0x06u)
#define phiA_In_2_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define phiA_In_2_MASK               phiA_In_2__MASK
#define phiA_In_2_SHIFT              phiA_In_2__SHIFT
#define phiA_In_2_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define phiA_In_2_PS                     (* (reg32 *) phiA_In_2__PS)
/* Port Configuration */
#define phiA_In_2_PC                     (* (reg32 *) phiA_In_2__PC)
/* Data Register */
#define phiA_In_2_DR                     (* (reg32 *) phiA_In_2__DR)
/* Input Buffer Disable Override */
#define phiA_In_2_INP_DIS                (* (reg32 *) phiA_In_2__PC2)


#if defined(phiA_In_2__INTSTAT)  /* Interrupt Registers */

    #define phiA_In_2_INTSTAT                (* (reg32 *) phiA_In_2__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define phiA_In_2_DRIVE_MODE_SHIFT       (0x00u)
#define phiA_In_2_DRIVE_MODE_MASK        (0x07u << phiA_In_2_DRIVE_MODE_SHIFT)


#endif /* End Pins phiA_In_2_H */


/* [] END OF FILE */
