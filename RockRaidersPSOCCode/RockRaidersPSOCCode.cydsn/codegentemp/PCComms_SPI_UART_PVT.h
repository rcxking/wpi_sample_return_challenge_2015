/*******************************************************************************
* File Name: PCComms_SPI_UART_PVT.h
* Version 2.0
*
* Description:
*  This private file provides constants and parameter values for the
*  SCB Component in SPI and UART modes.
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

#if !defined(CY_SCB_SPI_UART_PVT_PCComms_H)
#define CY_SCB_SPI_UART_PVT_PCComms_H

#include "PCComms_SPI_UART.h"


/***************************************
*     Internal Global Vars
***************************************/

#if(PCComms_INTERNAL_RX_SW_BUFFER_CONST)
    extern volatile uint32  PCComms_rxBufferHead;
    extern volatile uint32  PCComms_rxBufferTail;
    extern volatile uint8   PCComms_rxBufferOverflow;
#endif /* (PCComms_INTERNAL_RX_SW_BUFFER_CONST) */

#if(PCComms_INTERNAL_TX_SW_BUFFER_CONST)
    extern volatile uint32  PCComms_txBufferHead;
    extern volatile uint32  PCComms_txBufferTail;
#endif /* (PCComms_INTERNAL_TX_SW_BUFFER_CONST) */

#if(PCComms_INTERNAL_RX_SW_BUFFER)
    extern volatile uint8 PCComms_rxBufferInternal[PCComms_RX_BUFFER_SIZE];
#endif /* (PCComms_INTERNAL_RX_SW_BUFFER) */

#if(PCComms_INTERNAL_TX_SW_BUFFER)
    extern volatile uint8 PCComms_txBufferInternal[PCComms_TX_BUFFER_SIZE];
#endif /* (PCComms_INTERNAL_TX_SW_BUFFER) */


/***************************************
*     Private Function Prototypes
***************************************/

#if(PCComms_SCB_MODE_SPI_CONST_CFG)
    void PCComms_SpiInit(void);
#endif /* (PCComms_SCB_MODE_SPI_CONST_CFG) */

#if(PCComms_SPI_WAKE_ENABLE_CONST)
    void PCComms_SpiSaveConfig(void);
    void PCComms_SpiRestoreConfig(void);
#endif /* (PCComms_SPI_WAKE_ENABLE_CONST) */

#if(PCComms_SCB_MODE_UART_CONST_CFG)
    void PCComms_UartInit(void);
#endif /* (PCComms_SCB_MODE_UART_CONST_CFG) */

#if(PCComms_UART_WAKE_ENABLE_CONST)
    void PCComms_UartSaveConfig(void);
    void PCComms_UartRestoreConfig(void);
    #define PCComms_UartStop() \
        do{                             \
            PCComms_UART_RX_CTRL_REG &= ~PCComms_UART_RX_CTRL_SKIP_START; \
        }while(0)
#else
        #define PCComms_UartStop() do{ /* Does nothing */ }while(0)

#endif /* (PCComms_UART_WAKE_ENABLE_CONST) */

/* Interrupt processing */
#define PCComms_SpiUartEnableIntRx(intSourceMask)  PCComms_SetRxInterruptMode(intSourceMask)
#define PCComms_SpiUartEnableIntTx(intSourceMask)  PCComms_SetTxInterruptMode(intSourceMask)
uint32  PCComms_SpiUartDisableIntRx(void);
uint32  PCComms_SpiUartDisableIntTx(void);


/***************************************
*         UART API Constants
***************************************/

/* UART RX and TX position to be used in PCComms_SetPins() */
#define PCComms_UART_RX_PIN_ENABLE    (PCComms_UART_RX)
#define PCComms_UART_TX_PIN_ENABLE    (PCComms_UART_TX)

/* UART RTS and CTS position to be used in  PCComms_SetPins() */
#define PCComms_UART_RTS_PIN_ENABLE    (0x10u)
#define PCComms_UART_CTS_PIN_ENABLE    (0x20u)

#endif /* (CY_SCB_SPI_UART_PVT_PCComms_H) */


/* [] END OF FILE */
