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

#if !defined(CY_SCB_PVT_MotorComms_H)
#define CY_SCB_PVT_MotorComms_H

#include "MotorComms.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define MotorComms_SetI2CExtClkInterruptMode(interruptMask) MotorComms_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define MotorComms_ClearI2CExtClkInterruptSource(interruptMask) MotorComms_CLEAR_INTR_I2C_EC(interruptMask)
#define MotorComms_GetI2CExtClkInterruptSource()                (MotorComms_INTR_I2C_EC_REG)
#define MotorComms_GetI2CExtClkInterruptMode()                  (MotorComms_INTR_I2C_EC_MASK_REG)
#define MotorComms_GetI2CExtClkInterruptSourceMasked()          (MotorComms_INTR_I2C_EC_MASKED_REG)

#if (!MotorComms_CY_SCBIP_V1)
    /* APIs to service INTR_SPI_EC register */
    #define MotorComms_SetSpiExtClkInterruptMode(interruptMask) \
                                                                MotorComms_WRITE_INTR_SPI_EC_MASK(interruptMask)
    #define MotorComms_ClearSpiExtClkInterruptSource(interruptMask) \
                                                                MotorComms_CLEAR_INTR_SPI_EC(interruptMask)
    #define MotorComms_GetExtSpiClkInterruptSource()                 (MotorComms_INTR_SPI_EC_REG)
    #define MotorComms_GetExtSpiClkInterruptMode()                   (MotorComms_INTR_SPI_EC_MASK_REG)
    #define MotorComms_GetExtSpiClkInterruptSourceMasked()           (MotorComms_INTR_SPI_EC_MASKED_REG)
#endif /* (!MotorComms_CY_SCBIP_V1) */

#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void MotorComms_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask);
#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Vars with External Linkage
***************************************/

#if !defined (CY_REMOVE_MotorComms_CUSTOM_INTR_HANDLER)
    extern cyisraddress MotorComms_customIntrHandler;
#endif /* !defined (CY_REMOVE_MotorComms_CUSTOM_INTR_HANDLER) */

extern MotorComms_BACKUP_STRUCT MotorComms_backup;

#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    extern uint8 MotorComms_scbMode;
    extern uint8 MotorComms_scbEnableWake;
    extern uint8 MotorComms_scbEnableIntr;

    /* I2C configuration variables */
    extern uint8 MotorComms_mode;
    extern uint8 MotorComms_acceptAddr;

    /* SPI/UART configuration variables */
    extern volatile uint8 * MotorComms_rxBuffer;
    extern uint8   MotorComms_rxDataBits;
    extern uint32  MotorComms_rxBufferSize;

    extern volatile uint8 * MotorComms_txBuffer;
    extern uint8   MotorComms_txDataBits;
    extern uint32  MotorComms_txBufferSize;

    /* EZI2C configuration variables */
    extern uint8 MotorComms_numberOfAddr;
    extern uint8 MotorComms_subAddrSize;
#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*        Conditional Macro
****************************************/

#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Defines run time operation mode */
    #define MotorComms_SCB_MODE_I2C_RUNTM_CFG     (MotorComms_SCB_MODE_I2C      == MotorComms_scbMode)
    #define MotorComms_SCB_MODE_SPI_RUNTM_CFG     (MotorComms_SCB_MODE_SPI      == MotorComms_scbMode)
    #define MotorComms_SCB_MODE_UART_RUNTM_CFG    (MotorComms_SCB_MODE_UART     == MotorComms_scbMode)
    #define MotorComms_SCB_MODE_EZI2C_RUNTM_CFG   (MotorComms_SCB_MODE_EZI2C    == MotorComms_scbMode)
    #define MotorComms_SCB_MODE_UNCONFIG_RUNTM_CFG \
                                                        (MotorComms_SCB_MODE_UNCONFIG == MotorComms_scbMode)

    /* Defines wakeup enable */
    #define MotorComms_SCB_WAKE_ENABLE_CHECK       (0u != MotorComms_scbEnableWake)
#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Defines maximum number of SCB pins */
#if (!MotorComms_CY_SCBIP_V1)
    #define MotorComms_SCB_PINS_NUMBER    (7u)
#else
    #define MotorComms_SCB_PINS_NUMBER    (2u)
#endif /* (!MotorComms_CY_SCBIP_V1) */

#endif /* (CY_SCB_PVT_MotorComms_H) */


/* [] END OF FILE */
