/*******************************************************************************
* File Name: MotorComms_BOOT.h
* Version 2.0
*
* Description:
*  This file provides constants and parameter values for the bootloader
*  communication interface of SCB component.
*
* Note:
*
********************************************************************************
* Copyright 2014, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_BOOT_MotorComms_H)
#define CY_SCB_BOOT_MotorComms_H

#include "MotorComms_PVT.h"

#if (MotorComms_SCB_MODE_I2C_INC)
    #include "MotorComms_I2C.h"
#endif /* (MotorComms_SCB_MODE_I2C_INC) */

#if (MotorComms_SCB_MODE_EZI2C_INC)
    #include "MotorComms_EZI2C.h"
#endif /* (MotorComms_SCB_MODE_EZI2C_INC) */

#if (MotorComms_SCB_MODE_SPI_INC || MotorComms_SCB_MODE_UART_INC)
    #include "MotorComms_SPI_UART.h"
#endif /* (MotorComms_SCB_MODE_SPI_INC || MotorComms_SCB_MODE_UART_INC) */


/***************************************
*        Function Prototypes
***************************************/

/* Bootloader communication interface enable */
#define MotorComms_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_MotorComms) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

#if (MotorComms_SCB_MODE_I2C_INC)

    #define MotorComms_I2C_BTLDR_COMM_ENABLED     (MotorComms_BTLDR_COMM_ENABLED && \
                                                            (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             MotorComms_I2C_SLAVE_CONST))

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_I2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void MotorComms_I2CCyBtldrCommStart(void);
    void MotorComms_I2CCyBtldrCommStop (void);
    void MotorComms_I2CCyBtldrCommReset(void);
    cystatus MotorComms_I2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus MotorComms_I2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Size of Read/Write buffers for I2C bootloader  */
    #define MotorComms_I2C_BTLDR_SIZEOF_READ_BUFFER   (64u)
    #define MotorComms_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (64u)
    #define MotorComms_I2C_MIN_UINT16(a, b)           ( ((uint16)(a) < (uint16) (b)) ? \
                                                                    ((uint32) (a)) : ((uint32) (b)) )
    #define MotorComms_WAIT_1_MS                      (1u)
#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_I2C_BTLDR_COMM_ENABLED) */

#endif /* (MotorComms_SCB_MODE_I2C_INC) */


#if (MotorComms_SCB_MODE_EZI2C_INC)

    /* Provide EMPTY bootloader communication functions. EZI2C is NOT supported yet */
    #define MotorComms_EZI2C_BTLDR_COMM_ENABLED   (MotorComms_BTLDR_COMM_ENABLED && \
                                                         MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_EZI2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void MotorComms_EzI2CCyBtldrCommStart(void);
    void MotorComms_EzI2CCyBtldrCommStop (void);
    void MotorComms_EzI2CCyBtldrCommReset(void);
    cystatus MotorComms_EzI2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus MotorComms_EzI2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_EZI2C_BTLDR_COMM_ENABLED) */

#endif /* (MotorComms_SCB_MODE_EZI2C_INC) */

#if (MotorComms_SCB_MODE_SPI_INC || MotorComms_SCB_MODE_UART_INC)
    /* Provide EMPTY bootloader communication functions. SPI and UART is NOT supported yet */
    #define MotorComms_SPI_BTLDR_COMM_ENABLED     (MotorComms_BTLDR_COMM_ENABLED && \
                                                        MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)

    #define MotorComms_UART_BTLDR_COMM_ENABLED    (MotorComms_BTLDR_COMM_ENABLED && \
                                                        MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void MotorComms_SpiCyBtldrCommStart(void);
    void MotorComms_SpiCyBtldrCommStop (void);
    void MotorComms_SpiCyBtldrCommReset(void);
    cystatus MotorComms_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus MotorComms_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void MotorComms_UartCyBtldrCommStart(void);
    void MotorComms_UartCyBtldrCommStop (void);
    void MotorComms_UartCyBtldrCommReset(void);
    cystatus MotorComms_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus MotorComms_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_UART_BTLDR_COMM_ENABLED) */

#endif /* (MotorComms_SCB_MODE_SPI_INC || MotorComms_SCB_MODE_UART_INC) */

#if !defined (MotorComms_I2C_BTLDR_COMM_ENABLED)
    #define MotorComms_I2C_BTLDR_COMM_ENABLED     (0u)
#endif /* (MotorComms_I2C_BTLDR_COMM_ENABLED) */

#if !defined (MotorComms_EZI2C_BTLDR_COMM_ENABLED)
    #define MotorComms_EZI2C_BTLDR_COMM_ENABLED   (0u)
#endif /* (MotorComms_EZI2C_BTLDR_COMM_ENABLED) */

#if !defined (MotorComms_SPI_BTLDR_COMM_ENABLED)
    #define MotorComms_SPI_BTLDR_COMM_ENABLED     (0u)
#endif /* (MotorComms_SPI_BTLDR_COMM_ENABLED) */

#if !defined (MotorComms_UART_BTLDR_COMM_ENABLED)
    #define MotorComms_UART_BTLDR_COMM_ENABLED    (0u)
#endif /* (MotorComms_UART_BTLDR_COMM_ENABLED) */

/* Bootloader enabled condition for each mode */
#define MotorComms_BTLDR_COMM_MODE_ENABLED    (MotorComms_I2C_BTLDR_COMM_ENABLED   || \
                                                     MotorComms_EZI2C_BTLDR_COMM_ENABLED || \
                                                     MotorComms_SPI_BTLDR_COMM_ENABLED   || \
                                                     MotorComms_UART_BTLDR_COMM_ENABLED)

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_BTLDR_COMM_ENABLED)
    #if (MotorComms_BTLDR_COMM_MODE_ENABLED)
        /* Bootloader physical layer functions */
        void MotorComms_CyBtldrCommStart(void);
        void MotorComms_CyBtldrCommStop (void);
        void MotorComms_CyBtldrCommReset(void);
        cystatus MotorComms_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
        cystatus MotorComms_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    #endif /* (MotorComms_BTLDR_COMM_MODE_ENABLED) */

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_MotorComms)
        #define CyBtldrCommStart    MotorComms_CyBtldrCommStart
        #define CyBtldrCommStop     MotorComms_CyBtldrCommStop
        #define CyBtldrCommReset    MotorComms_CyBtldrCommReset
        #define CyBtldrCommWrite    MotorComms_CyBtldrCommWrite
        #define CyBtldrCommRead     MotorComms_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_MotorComms) */
#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_BTLDR_COMM_ENABLED) */

#endif /* (CY_SCB_BOOT_MotorComms_H) */

/* [] END OF FILE */
