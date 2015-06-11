/*******************************************************************************
* File Name: OpenExhaust.h  
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

#if !defined(CY_PINS_OpenExhaust_H) /* Pins OpenExhaust_H */
#define CY_PINS_OpenExhaust_H

#include "cytypes.h"
#include "cyfitter.h"
#include "OpenExhaust_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    OpenExhaust_Write(uint8 value) ;
void    OpenExhaust_SetDriveMode(uint8 mode) ;
uint8   OpenExhaust_ReadDataReg(void) ;
uint8   OpenExhaust_Read(void) ;
uint8   OpenExhaust_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define OpenExhaust_DRIVE_MODE_BITS        (3)
#define OpenExhaust_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - OpenExhaust_DRIVE_MODE_BITS))

#define OpenExhaust_DM_ALG_HIZ         (0x00u)
#define OpenExhaust_DM_DIG_HIZ         (0x01u)
#define OpenExhaust_DM_RES_UP          (0x02u)
#define OpenExhaust_DM_RES_DWN         (0x03u)
#define OpenExhaust_DM_OD_LO           (0x04u)
#define OpenExhaust_DM_OD_HI           (0x05u)
#define OpenExhaust_DM_STRONG          (0x06u)
#define OpenExhaust_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define OpenExhaust_MASK               OpenExhaust__MASK
#define OpenExhaust_SHIFT              OpenExhaust__SHIFT
#define OpenExhaust_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define OpenExhaust_PS                     (* (reg32 *) OpenExhaust__PS)
/* Port Configuration */
#define OpenExhaust_PC                     (* (reg32 *) OpenExhaust__PC)
/* Data Register */
#define OpenExhaust_DR                     (* (reg32 *) OpenExhaust__DR)
/* Input Buffer Disable Override */
#define OpenExhaust_INP_DIS                (* (reg32 *) OpenExhaust__PC2)


#if defined(OpenExhaust__INTSTAT)  /* Interrupt Registers */

    #define OpenExhaust_INTSTAT                (* (reg32 *) OpenExhaust__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define OpenExhaust_DRIVE_MODE_SHIFT       (0x00u)
#define OpenExhaust_DRIVE_MODE_MASK        (0x07u << OpenExhaust_DRIVE_MODE_SHIFT)


#endif /* End Pins OpenExhaust_H */


/* [] END OF FILE */
