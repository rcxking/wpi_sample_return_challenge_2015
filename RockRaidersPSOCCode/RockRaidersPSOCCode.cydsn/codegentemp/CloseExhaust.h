/*******************************************************************************
* File Name: CloseExhaust.h  
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

#if !defined(CY_PINS_CloseExhaust_H) /* Pins CloseExhaust_H */
#define CY_PINS_CloseExhaust_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CloseExhaust_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    CloseExhaust_Write(uint8 value) ;
void    CloseExhaust_SetDriveMode(uint8 mode) ;
uint8   CloseExhaust_ReadDataReg(void) ;
uint8   CloseExhaust_Read(void) ;
uint8   CloseExhaust_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define CloseExhaust_DRIVE_MODE_BITS        (3)
#define CloseExhaust_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - CloseExhaust_DRIVE_MODE_BITS))

#define CloseExhaust_DM_ALG_HIZ         (0x00u)
#define CloseExhaust_DM_DIG_HIZ         (0x01u)
#define CloseExhaust_DM_RES_UP          (0x02u)
#define CloseExhaust_DM_RES_DWN         (0x03u)
#define CloseExhaust_DM_OD_LO           (0x04u)
#define CloseExhaust_DM_OD_HI           (0x05u)
#define CloseExhaust_DM_STRONG          (0x06u)
#define CloseExhaust_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define CloseExhaust_MASK               CloseExhaust__MASK
#define CloseExhaust_SHIFT              CloseExhaust__SHIFT
#define CloseExhaust_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CloseExhaust_PS                     (* (reg32 *) CloseExhaust__PS)
/* Port Configuration */
#define CloseExhaust_PC                     (* (reg32 *) CloseExhaust__PC)
/* Data Register */
#define CloseExhaust_DR                     (* (reg32 *) CloseExhaust__DR)
/* Input Buffer Disable Override */
#define CloseExhaust_INP_DIS                (* (reg32 *) CloseExhaust__PC2)


#if defined(CloseExhaust__INTSTAT)  /* Interrupt Registers */

    #define CloseExhaust_INTSTAT                (* (reg32 *) CloseExhaust__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define CloseExhaust_DRIVE_MODE_SHIFT       (0x00u)
#define CloseExhaust_DRIVE_MODE_MASK        (0x07u << CloseExhaust_DRIVE_MODE_SHIFT)


#endif /* End Pins CloseExhaust_H */


/* [] END OF FILE */
