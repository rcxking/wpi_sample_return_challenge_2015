/*******************************************************************************
* File Name: .h
* Version 2.0
*
* Description:
*  This private file provides constants and parameter values for the
*  SCB Component.
*  Please do not use this file or its content in your project.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PVT_PCComms_H)
#define CY_SCB_PVT_PCComms_H

#include "PCComms.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define PCComms_SetI2CExtClkInterruptMode(interruptMask) PCComms_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define PCComms_ClearI2CExtClkInterruptSource(interruptMask) PCComms_CLEAR_INTR_I2C_EC(interruptMask)
#define PCComms_GetI2CExtClkInterruptSource()                (PCComms_INTR_I2C_EC_REG)
#define PCComms_GetI2CExtClkInterruptMode()                  (PCComms_INTR_I2C_EC_MASK_REG)
#define PCComms_GetI2CExtClkInterruptSourceMasked()          (PCComms_INTR_I2C_EC_MASKED_REG)

#if (!PCComms_CY_SCBIP_V1)
    /* APIs to service INTR_SPI_EC register */
    #define PCComms_SetSpiExtClkInterruptMode(interruptMask) \
                                                                PCComms_WRITE_INTR_SPI_EC_MASK(interruptMask)
    #define PCComms_ClearSpiExtClkInterruptSource(interruptMask) \
                                                                PCComms_CLEAR_INTR_SPI_EC(interruptMask)
    #define PCComms_GetExtSpiClkInterruptSource()                 (PCComms_INTR_SPI_EC_REG)
    #define PCComms_GetExtSpiClkInterruptMode()                   (PCComms_INTR_SPI_EC_MASK_REG)
    #define PCComms_GetExtSpiClkInterruptSourceMasked()           (PCComms_INTR_SPI_EC_MASKED_REG)
#endif /* (!PCComms_CY_SCBIP_V1) */

#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void PCComms_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask);
#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Vars with External Linkage
***************************************/

#if !defined (CY_REMOVE_PCComms_CUSTOM_INTR_HANDLER)
    extern cyisraddress PCComms_customIntrHandler;
#endif /* !defined (CY_REMOVE_PCComms_CUSTOM_INTR_HANDLER) */

extern PCComms_BACKUP_STRUCT PCComms_backup;

#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    extern uint8 PCComms_scbMode;
    extern uint8 PCComms_scbEnableWake;
    extern uint8 PCComms_scbEnableIntr;

    /* I2C configuration variables */
    extern uint8 PCComms_mode;
    extern uint8 PCComms_acceptAddr;

    /* SPI/UART configuration variables */
    extern volatile uint8 * PCComms_rxBuffer;
    extern uint8   PCComms_rxDataBits;
    extern uint32  PCComms_rxBufferSize;

    extern volatile uint8 * PCComms_txBuffer;
    extern uint8   PCComms_txDataBits;
    extern uint32  PCComms_txBufferSize;

    /* EZI2C configuration variables */
    extern uint8 PCComms_numberOfAddr;
    extern uint8 PCComms_subAddrSize;
#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*        Conditional Macro
****************************************/

#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Defines run time operation mode */
    #define PCComms_SCB_MODE_I2C_RUNTM_CFG     (PCComms_SCB_MODE_I2C      == PCComms_scbMode)
    #define PCComms_SCB_MODE_SPI_RUNTM_CFG     (PCComms_SCB_MODE_SPI      == PCComms_scbMode)
    #define PCComms_SCB_MODE_UART_RUNTM_CFG    (PCComms_SCB_MODE_UART     == PCComms_scbMode)
    #define PCComms_SCB_MODE_EZI2C_RUNTM_CFG   (PCComms_SCB_MODE_EZI2C    == PCComms_scbMode)
    #define PCComms_SCB_MODE_UNCONFIG_RUNTM_CFG \
                                                        (PCComms_SCB_MODE_UNCONFIG == PCComms_scbMode)

    /* Defines wakeup enable */
    #define PCComms_SCB_WAKE_ENABLE_CHECK       (0u != PCComms_scbEnableWake)
#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Defines maximum number of SCB pins */
#if (!PCComms_CY_SCBIP_V1)
    #define PCComms_SCB_PINS_NUMBER    (7u)
#else
    #define PCComms_SCB_PINS_NUMBER    (2u)
#endif /* (!PCComms_CY_SCBIP_V1) */

#endif /* (CY_SCB_PVT_PCComms_H) */


/* [] END OF FILE */
