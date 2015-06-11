/*******************************************************************************
* File Name: OpenPressure.h  
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

#if !defined(CY_PINS_OpenPressure_H) /* Pins OpenPressure_H */
#define CY_PINS_OpenPressure_H

#include "cytypes.h"
#include "cyfitter.h"
#include "OpenPressure_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    OpenPressure_Write(uint8 value) ;
void    OpenPressure_SetDriveMode(uint8 mode) ;
uint8   OpenPressure_ReadDataReg(void) ;
uint8   OpenPressure_Read(void) ;
uint8   OpenPressure_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define OpenPressure_DRIVE_MODE_BITS        (3)
#define OpenPressure_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - OpenPressure_DRIVE_MODE_BITS))

#define OpenPressure_DM_ALG_HIZ         (0x00u)
#define OpenPressure_DM_DIG_HIZ         (0x01u)
#define OpenPressure_DM_RES_UP          (0x02u)
#define OpenPressure_DM_RES_DWN         (0x03u)
#define OpenPressure_DM_OD_LO           (0x04u)
#define OpenPressure_DM_OD_HI           (0x05u)
#define OpenPressure_DM_STRONG          (0x06u)
#define OpenPressure_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define OpenPressure_MASK               OpenPressure__MASK
#define OpenPressure_SHIFT              OpenPressure__SHIFT
#define OpenPressure_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define OpenPressure_PS                     (* (reg32 *) OpenPressure__PS)
/* Port Configuration */
#define OpenPressure_PC                     (* (reg32 *) OpenPressure__PC)
/* Data Register */
#define OpenPressure_DR                     (* (reg32 *) OpenPressure__DR)
/* Input Buffer Disable Override */
#define OpenPressure_INP_DIS                (* (reg32 *) OpenPressure__PC2)


#if defined(OpenPressure__INTSTAT)  /* Interrupt Registers */

    #define OpenPressure_INTSTAT                (* (reg32 *) OpenPressure__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define OpenPressure_DRIVE_MODE_SHIFT       (0x00u)
#define OpenPressure_DRIVE_MODE_MASK        (0x07u << OpenPressure_DRIVE_MODE_SHIFT)


#endif /* End Pins OpenPressure_H */


/* [] END OF FILE */
