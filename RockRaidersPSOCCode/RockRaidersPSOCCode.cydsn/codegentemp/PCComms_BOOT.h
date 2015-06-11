/*******************************************************************************
* File Name: PCComms_BOOT.h
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

#if !defined(CY_SCB_BOOT_PCComms_H)
#define CY_SCB_BOOT_PCComms_H

#include "PCComms_PVT.h"

#if (PCComms_SCB_MODE_I2C_INC)
    #include "PCComms_I2C.h"
#endif /* (PCComms_SCB_MODE_I2C_INC) */

#if (PCComms_SCB_MODE_EZI2C_INC)
    #include "PCComms_EZI2C.h"
#endif /* (PCComms_SCB_MODE_EZI2C_INC) */

#if (PCComms_SCB_MODE_SPI_INC || PCComms_SCB_MODE_UART_INC)
    #include "PCComms_SPI_UART.h"
#endif /* (PCComms_SCB_MODE_SPI_INC || PCComms_SCB_MODE_UART_INC) */


/***************************************
*        Function Prototypes
***************************************/

/* Bootloader communication interface enable */
#define PCComms_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PCComms) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

#if (PCComms_SCB_MODE_I2C_INC)

    #define PCComms_I2C_BTLDR_COMM_ENABLED     (PCComms_BTLDR_COMM_ENABLED && \
                                                            (PCComms_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             PCComms_I2C_SLAVE_CONST))

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_I2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void PCComms_I2CCyBtldrCommStart(void);
    void PCComms_I2CCyBtldrCommStop (void);
    void PCComms_I2CCyBtldrCommReset(void);
    cystatus PCComms_I2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus PCComms_I2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Size of Read/Write buffers for I2C bootloader  */
    #define PCComms_I2C_BTLDR_SIZEOF_READ_BUFFER   (64u)
    #define PCComms_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (64u)
    #define PCComms_I2C_MIN_UINT16(a, b)           ( ((uint16)(a) < (uint16) (b)) ? \
                                                                    ((uint32) (a)) : ((uint32) (b)) )
    #define PCComms_WAIT_1_MS                      (1u)
#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_I2C_BTLDR_COMM_ENABLED) */

#endif /* (PCComms_SCB_MODE_I2C_INC) */


#if (PCComms_SCB_MODE_EZI2C_INC)

    /* Provide EMPTY bootloader communication functions. EZI2C is NOT supported yet */
    #define PCComms_EZI2C_BTLDR_COMM_ENABLED   (PCComms_BTLDR_COMM_ENABLED && \
                                                         PCComms_SCB_MODE_UNCONFIG_CONST_CFG)

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_EZI2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void PCComms_EzI2CCyBtldrCommStart(void);
    void PCComms_EzI2CCyBtldrCommStop (void);
    void PCComms_EzI2CCyBtldrCommReset(void);
    cystatus PCComms_EzI2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus PCComms_EzI2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_EZI2C_BTLDR_COMM_ENABLED) */

#endif /* (PCComms_SCB_MODE_EZI2C_INC) */

#if (PCComms_SCB_MODE_SPI_INC || PCComms_SCB_MODE_UART_INC)
    /* Provide EMPTY bootloader communication functions. SPI and UART is NOT supported yet */
    #define PCComms_SPI_BTLDR_COMM_ENABLED     (PCComms_BTLDR_COMM_ENABLED && \
                                                        PCComms_SCB_MODE_UNCONFIG_CONST_CFG)

    #define PCComms_UART_BTLDR_COMM_ENABLED    (PCComms_BTLDR_COMM_ENABLED && \
                                                        PCComms_SCB_MODE_UNCONFIG_CONST_CFG)

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void PCComms_SpiCyBtldrCommStart(void);
    void PCComms_SpiCyBtldrCommStop (void);
    void PCComms_SpiCyBtldrCommReset(void);
    cystatus PCComms_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus PCComms_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void PCComms_UartCyBtldrCommStart(void);
    void PCComms_UartCyBtldrCommStop (void);
    void PCComms_UartCyBtldrCommReset(void);
    cystatus PCComms_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus PCComms_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_UART_BTLDR_COMM_ENABLED) */

#endif /* (PCComms_SCB_MODE_SPI_INC || PCComms_SCB_MODE_UART_INC) */

#if !defined (PCComms_I2C_BTLDR_COMM_ENABLED)
    #define PCComms_I2C_BTLDR_COMM_ENABLED     (0u)
#endif /* (PCComms_I2C_BTLDR_COMM_ENABLED) */

#if !defined (PCComms_EZI2C_BTLDR_COMM_ENABLED)
    #define PCComms_EZI2C_BTLDR_COMM_ENABLED   (0u)
#endif /* (PCComms_EZI2C_BTLDR_COMM_ENABLED) */

#if !defined (PCComms_SPI_BTLDR_COMM_ENABLED)
    #define PCComms_SPI_BTLDR_COMM_ENABLED     (0u)
#endif /* (PCComms_SPI_BTLDR_COMM_ENABLED) */

#if !defined (PCComms_UART_BTLDR_COMM_ENABLED)
    #define PCComms_UART_BTLDR_COMM_ENABLED    (0u)
#endif /* (PCComms_UART_BTLDR_COMM_ENABLED) */

/* Bootloader enabled condition for each mode */
#define PCComms_BTLDR_COMM_MODE_ENABLED    (PCComms_I2C_BTLDR_COMM_ENABLED   || \
                                                     PCComms_EZI2C_BTLDR_COMM_ENABLED || \
                                                     PCComms_SPI_BTLDR_COMM_ENABLED   || \
                                                     PCComms_UART_BTLDR_COMM_ENABLED)

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_BTLDR_COMM_ENABLED)
    #if (PCComms_BTLDR_COMM_MODE_ENABLED)
        /* Bootloader physical layer functions */
        void PCComms_CyBtldrCommStart(void);
        void PCComms_CyBtldrCommStop (void);
        void PCComms_CyBtldrCommReset(void);
        cystatus PCComms_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
        cystatus PCComms_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    #endif /* (PCComms_BTLDR_COMM_MODE_ENABLED) */

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PCComms)
        #define CyBtldrCommStart    PCComms_CyBtldrCommStart
        #define CyBtldrCommStop     PCComms_CyBtldrCommStop
        #define CyBtldrCommReset    PCComms_CyBtldrCommReset
        #define CyBtldrCommWrite    PCComms_CyBtldrCommWrite
        #define CyBtldrCommRead     PCComms_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PCComms) */
#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_BTLDR_COMM_ENABLED) */

#endif /* (CY_SCB_BOOT_PCComms_H) */

/* [] END OF FILE */
