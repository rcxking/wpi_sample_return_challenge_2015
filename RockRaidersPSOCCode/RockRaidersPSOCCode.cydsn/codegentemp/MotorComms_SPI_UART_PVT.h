/*******************************************************************************
* File Name: MotorComms_SPI_UART_PVT.h
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

#if !defined(CY_SCB_SPI_UART_PVT_MotorComms_H)
#define CY_SCB_SPI_UART_PVT_MotorComms_H

#include "MotorComms_SPI_UART.h"


/***************************************
*     Internal Global Vars
***************************************/

#if(MotorComms_INTERNAL_RX_SW_BUFFER_CONST)
    extern volatile uint32  MotorComms_rxBufferHead;
    extern volatile uint32  MotorComms_rxBufferTail;
    extern volatile uint8   MotorComms_rxBufferOverflow;
#endif /* (MotorComms_INTERNAL_RX_SW_BUFFER_CONST) */

#if(MotorComms_INTERNAL_TX_SW_BUFFER_CONST)
    extern volatile uint32  MotorComms_txBufferHead;
    extern volatile uint32  MotorComms_txBufferTail;
#endif /* (MotorComms_INTERNAL_TX_SW_BUFFER_CONST) */

#if(MotorComms_INTERNAL_RX_SW_BUFFER)
    extern volatile uint8 MotorComms_rxBufferInternal[MotorComms_RX_BUFFER_SIZE];
#endif /* (MotorComms_INTERNAL_RX_SW_BUFFER) */

#if(MotorComms_INTERNAL_TX_SW_BUFFER)
    extern volatile uint8 MotorComms_txBufferInternal[MotorComms_TX_BUFFER_SIZE];
#endif /* (MotorComms_INTERNAL_TX_SW_BUFFER) */


/***************************************
*     Private Function Prototypes
***************************************/

#if(MotorComms_SCB_MODE_SPI_CONST_CFG)
    void MotorComms_SpiInit(void);
#endif /* (MotorComms_SCB_MODE_SPI_CONST_CFG) */

#if(MotorComms_SPI_WAKE_ENABLE_CONST)
    void MotorComms_SpiSaveConfig(void);
    void MotorComms_SpiRestoreConfig(void);
#endif /* (MotorComms_SPI_WAKE_ENABLE_CONST) */

#if(MotorComms_SCB_MODE_UART_CONST_CFG)
    void MotorComms_UartInit(void);
#endif /* (MotorComms_SCB_MODE_UART_CONST_CFG) */

#if(MotorComms_UART_WAKE_ENABLE_CONST)
    void MotorComms_UartSaveConfig(void);
    void MotorComms_UartRestoreConfig(void);
    #define MotorComms_UartStop() \
        do{                             \
            MotorComms_UART_RX_CTRL_REG &= ~MotorComms_UART_RX_CTRL_SKIP_START; \
        }while(0)
#else
        #define MotorComms_UartStop() do{ /* Does nothing */ }while(0)

#endif /* (MotorComms_UART_WAKE_ENABLE_CONST) */

/* Interrupt processing */
#define MotorComms_SpiUartEnableIntRx(intSourceMask)  MotorComms_SetRxInterruptMode(intSourceMask)
#define MotorComms_SpiUartEnableIntTx(intSourceMask)  MotorComms_SetTxInterruptMode(intSourceMask)
uint32  MotorComms_SpiUartDisableIntRx(void);
uint32  MotorComms_SpiUartDisableIntTx(void);


/***************************************
*         UART API Constants
***************************************/

/* UART RX and TX position to be used in MotorComms_SetPins() */
#define MotorComms_UART_RX_PIN_ENABLE    (MotorComms_UART_RX)
#define MotorComms_UART_TX_PIN_ENABLE    (MotorComms_UART_TX)

/* UART RTS and CTS position to be used in  MotorComms_SetPins() */
#define MotorComms_UART_RTS_PIN_ENABLE    (0x10u)
#define MotorComms_UART_CTS_PIN_ENABLE    (0x20u)

#endif /* (CY_SCB_SPI_UART_PVT_MotorComms_H) */


/* [] END OF FILE */
