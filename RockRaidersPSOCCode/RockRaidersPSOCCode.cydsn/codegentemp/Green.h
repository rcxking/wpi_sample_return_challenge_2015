/*******************************************************************************
* File Name: Green.h  
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

#if !defined(CY_PINS_Green_H) /* Pins Green_H */
#define CY_PINS_Green_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Green_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    Green_Write(uint8 value) ;
void    Green_SetDriveMode(uint8 mode) ;
uint8   Green_ReadDataReg(void) ;
uint8   Green_Read(void) ;
uint8   Green_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Green_DRIVE_MODE_BITS        (3)
#define Green_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Green_DRIVE_MODE_BITS))

#define Green_DM_ALG_HIZ         (0x00u)
#define Green_DM_DIG_HIZ         (0x01u)
#define Green_DM_RES_UP          (0x02u)
#define Green_DM_RES_DWN         (0x03u)
#define Green_DM_OD_LO           (0x04u)
#define Green_DM_OD_HI           (0x05u)
#define Green_DM_STRONG          (0x06u)
#define Green_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define Green_MASK               Green__MASK
#define Green_SHIFT              Green__SHIFT
#define Green_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Green_PS                     (* (reg32 *) Green__PS)
/* Port Configuration */
#define Green_PC                     (* (reg32 *) Green__PC)
/* Data Register */
#define Green_DR                     (* (reg32 *) Green__DR)
/* Input Buffer Disable Override */
#define Green_INP_DIS                (* (reg32 *) Green__PC2)


#if defined(Green__INTSTAT)  /* Interrupt Registers */

    #define Green_INTSTAT                (* (reg32 *) Green__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define Green_DRIVE_MODE_SHIFT       (0x00u)
#define Green_DRIVE_MODE_MASK        (0x07u << Green_DRIVE_MODE_SHIFT)


#endif /* End Pins Green_H */


/* [] END OF FILE */
