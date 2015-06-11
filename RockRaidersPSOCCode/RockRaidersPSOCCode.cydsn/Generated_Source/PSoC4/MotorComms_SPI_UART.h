/*******************************************************************************
* File Name: MotorComms_SPI_UART.h
* Version 2.0
*
* Description:
*  This file provides constants and parameter values for the SCB Component in
*  SPI and UART modes.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_SPI_UART_MotorComms_H)
#define CY_SCB_SPI_UART_MotorComms_H

#include "MotorComms.h"


/***************************************
*   SPI Initial Parameter Constants
****************************************/

#define MotorComms_SPI_MODE                   (0u)
#define MotorComms_SPI_SUB_MODE               (0u)
#define MotorComms_SPI_CLOCK_MODE             (0u)
#define MotorComms_SPI_OVS_FACTOR             (16u)
#define MotorComms_SPI_MEDIAN_FILTER_ENABLE   (0u)
#define MotorComms_SPI_LATE_MISO_SAMPLE_ENABLE (0u)
#define MotorComms_SPI_RX_DATA_BITS_NUM       (8u)
#define MotorComms_SPI_TX_DATA_BITS_NUM       (8u)
#define MotorComms_SPI_WAKE_ENABLE            (0u)
#define MotorComms_SPI_BITS_ORDER             (1u)
#define MotorComms_SPI_TRANSFER_SEPARATION    (1u)
#define MotorComms_SPI_NUMBER_OF_SS_LINES     (1u)
#define MotorComms_SPI_RX_BUFFER_SIZE         (8u)
#define MotorComms_SPI_TX_BUFFER_SIZE         (8u)

#define MotorComms_SPI_INTERRUPT_MODE         (0u)

#define MotorComms_SPI_INTR_RX_MASK           (0u)
#define MotorComms_SPI_INTR_TX_MASK           (0u)

#define MotorComms_SPI_RX_TRIGGER_LEVEL       (7u)
#define MotorComms_SPI_TX_TRIGGER_LEVEL       (0u)

#define MotorComms_SPI_BYTE_MODE_ENABLE       (0u)
#define MotorComms_SPI_FREE_RUN_SCLK_ENABLE   (0u)
#define MotorComms_SPI_SS0_POLARITY           (0u)
#define MotorComms_SPI_SS1_POLARITY           (0u)
#define MotorComms_SPI_SS2_POLARITY           (0u)
#define MotorComms_SPI_SS3_POLARITY           (0u)


/***************************************
*   UART Initial Parameter Constants
****************************************/

#define MotorComms_UART_SUB_MODE              (0u)
#define MotorComms_UART_DIRECTION             (3u)
#define MotorComms_UART_DATA_BITS_NUM         (8u)
#define MotorComms_UART_PARITY_TYPE           (2u)
#define MotorComms_UART_STOP_BITS_NUM         (2u)
#define MotorComms_UART_OVS_FACTOR            (12u)
#define MotorComms_UART_IRDA_LOW_POWER        (0u)
#define MotorComms_UART_MEDIAN_FILTER_ENABLE  (0u)
#define MotorComms_UART_RETRY_ON_NACK         (0u)
#define MotorComms_UART_IRDA_POLARITY         (0u)
#define MotorComms_UART_DROP_ON_FRAME_ERR     (0u)
#define MotorComms_UART_DROP_ON_PARITY_ERR    (0u)
#define MotorComms_UART_WAKE_ENABLE           (0u)
#define MotorComms_UART_RX_BUFFER_SIZE        (8u)
#define MotorComms_UART_TX_BUFFER_SIZE        (8u)
#define MotorComms_UART_MP_MODE_ENABLE        (0u)
#define MotorComms_UART_MP_ACCEPT_ADDRESS     (0u)
#define MotorComms_UART_MP_RX_ADDRESS         (2u)
#define MotorComms_UART_MP_RX_ADDRESS_MASK    (255u)

#define MotorComms_UART_INTERRUPT_MODE        (0u)

#define MotorComms_UART_INTR_RX_MASK          (0u)
#define MotorComms_UART_INTR_TX_MASK          (0u)

#define MotorComms_UART_RX_TRIGGER_LEVEL      (7u)
#define MotorComms_UART_TX_TRIGGER_LEVEL      (0u)

#define MotorComms_UART_BYTE_MODE_ENABLE      (0u)
#define MotorComms_UART_CTS_ENABLE            (0u)
#define MotorComms_UART_CTS_POLARITY          (0u)
#define MotorComms_UART_RTS_POLARITY          (0u)
#define MotorComms_UART_RTS_FIFO_LEVEL        (4u)

/* SPI mode enum */
#define MotorComms_SPI_SLAVE  (0u)
#define MotorComms_SPI_MASTER (1u)

/* UART direction enum */
#define MotorComms_UART_RX    (1u)
#define MotorComms_UART_TX    (2u)
#define MotorComms_UART_TX_RX (3u)


/***************************************
*   Conditional Compilation Parameters
****************************************/

#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)

    /* Mode */
    #define MotorComms_SPI_MASTER_CONST       (1u)

    /* Direction */
    #define MotorComms_RX_DIRECTION           (1u)
    #define MotorComms_TX_DIRECTION           (1u)
    #define MotorComms_UART_RX_DIRECTION      (1u)
    #define MotorComms_UART_TX_DIRECTION      (1u)

    /* Only external RX and TX buffer for Uncofigured mode */
    #define MotorComms_INTERNAL_RX_SW_BUFFER   (0u)
    #define MotorComms_INTERNAL_TX_SW_BUFFER   (0u)

    /* Get RX and TX buffer size */
    #define MotorComms_RX_BUFFER_SIZE (MotorComms_rxBufferSize)
    #define MotorComms_TX_BUFFER_SIZE (MotorComms_txBufferSize)

    /* Return true if buffer is provided */
    #define MotorComms_CHECK_RX_SW_BUFFER (NULL != MotorComms_rxBuffer)
    #define MotorComms_CHECK_TX_SW_BUFFER (NULL != MotorComms_txBuffer)

    /* Always provide global variables to support RX and TX buffers */
    #define MotorComms_INTERNAL_RX_SW_BUFFER_CONST    (1u)
    #define MotorComms_INTERNAL_TX_SW_BUFFER_CONST    (1u)

    /* Get wakeup enable option */
    #define MotorComms_SPI_WAKE_ENABLE_CONST  (1u)
    #define MotorComms_CHECK_SPI_WAKE_ENABLE  (0u != MotorComms_scbEnableWake)
    #define MotorComms_UART_WAKE_ENABLE_CONST (1u)

    /* SPI/UART: TX or RX FIFO size */
    #if (MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
        #define MotorComms_SPI_UART_FIFO_SIZE (MotorComms_FIFO_SIZE)
    #else
        #define MotorComms_SPI_UART_FIFO_SIZE (MotorComms_GET_FIFO_SIZE(MotorComms_CTRL_REG & \
                                                                                    MotorComms_CTRL_BYTE_MODE))
    #endif /* (MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */

#else

    /* Internal RX and TX buffer: for SPI or UART */
    #if (MotorComms_SCB_MODE_SPI_CONST_CFG)

        /* SPI Direction */
        #define MotorComms_SPI_RX_DIRECTION (1u)
        #define MotorComms_SPI_TX_DIRECTION (1u)

        /* Get FIFO size */
        #if (MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
            #define MotorComms_SPI_UART_FIFO_SIZE    (MotorComms_FIFO_SIZE)
        #else
            #define MotorComms_SPI_UART_FIFO_SIZE \
                                           MotorComms_GET_FIFO_SIZE(MotorComms_SPI_BYTE_MODE_ENABLE)

        #endif /* (MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */

        /* SPI internal RX and TX buffers */
        #define MotorComms_INTERNAL_SPI_RX_SW_BUFFER  (MotorComms_SPI_RX_BUFFER_SIZE > \
                                                                MotorComms_SPI_UART_FIFO_SIZE)
        #define MotorComms_INTERNAL_SPI_TX_SW_BUFFER  (MotorComms_SPI_TX_BUFFER_SIZE > \
                                                                MotorComms_SPI_UART_FIFO_SIZE)

        /* Internal SPI RX and TX buffer */
        #define MotorComms_INTERNAL_RX_SW_BUFFER  (MotorComms_INTERNAL_SPI_RX_SW_BUFFER)
        #define MotorComms_INTERNAL_TX_SW_BUFFER  (MotorComms_INTERNAL_SPI_TX_SW_BUFFER)

        /* Internal SPI RX and TX buffer size */
        #define MotorComms_RX_BUFFER_SIZE         (MotorComms_SPI_RX_BUFFER_SIZE + 1u)
        #define MotorComms_TX_BUFFER_SIZE         (MotorComms_SPI_TX_BUFFER_SIZE)

        /* Get wakeup enable option */
        #define MotorComms_SPI_WAKE_ENABLE_CONST  (0u != MotorComms_SPI_WAKE_ENABLE)
        #define MotorComms_UART_WAKE_ENABLE_CONST (0u)

    #else

        /* UART Direction */
        #define MotorComms_UART_RX_DIRECTION (0u != (MotorComms_UART_DIRECTION & MotorComms_UART_RX))
        #define MotorComms_UART_TX_DIRECTION (0u != (MotorComms_UART_DIRECTION & MotorComms_UART_TX))

        /* Get FIFO size */
        #if (MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
            #define MotorComms_SPI_UART_FIFO_SIZE    (MotorComms_FIFO_SIZE)
        #else
            #define MotorComms_SPI_UART_FIFO_SIZE \
                                           MotorComms_GET_FIFO_SIZE(MotorComms_UART_BYTE_MODE_ENABLE)
        #endif /* (MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */

        /* UART internal RX and TX buffers */
        #define MotorComms_INTERNAL_UART_RX_SW_BUFFER  (MotorComms_UART_RX_BUFFER_SIZE > \
                                                                MotorComms_SPI_UART_FIFO_SIZE)
        #define MotorComms_INTERNAL_UART_TX_SW_BUFFER  (MotorComms_UART_TX_BUFFER_SIZE > \
                                                                    MotorComms_SPI_UART_FIFO_SIZE)

        /* Internal UART RX and TX buffer */
        #define MotorComms_INTERNAL_RX_SW_BUFFER  (MotorComms_INTERNAL_UART_RX_SW_BUFFER)
        #define MotorComms_INTERNAL_TX_SW_BUFFER  (MotorComms_INTERNAL_UART_TX_SW_BUFFER)

        /* Internal UART RX and TX buffer size */
        #define MotorComms_RX_BUFFER_SIZE         (MotorComms_UART_RX_BUFFER_SIZE + 1u)
        #define MotorComms_TX_BUFFER_SIZE         (MotorComms_UART_TX_BUFFER_SIZE)

        /* Get wakeup enable option */
        #define MotorComms_SPI_WAKE_ENABLE_CONST  (0u)
        #define MotorComms_UART_WAKE_ENABLE_CONST (0u != MotorComms_UART_WAKE_ENABLE)

    #endif /* (MotorComms_SCB_MODE_SPI_CONST_CFG) */

    /* Mode */
    #define MotorComms_SPI_MASTER_CONST   (MotorComms_SPI_MODE == MotorComms_SPI_MASTER)

    /* Direction */
    #define MotorComms_RX_DIRECTION ((MotorComms_SCB_MODE_SPI_CONST_CFG) ? \
                                            (MotorComms_SPI_RX_DIRECTION) : (MotorComms_UART_RX_DIRECTION))

    #define MotorComms_TX_DIRECTION ((MotorComms_SCB_MODE_SPI_CONST_CFG) ? \
                                            (MotorComms_SPI_TX_DIRECTION) : (MotorComms_UART_TX_DIRECTION))

    /* Internal RX and TX buffer: for SPI or UART. Used in conditional compilation check */
    #define MotorComms_CHECK_RX_SW_BUFFER (MotorComms_INTERNAL_RX_SW_BUFFER)
    #define MotorComms_CHECK_TX_SW_BUFFER (MotorComms_INTERNAL_TX_SW_BUFFER)

    /* Provide global variables to support RX and TX buffers */
    #define MotorComms_INTERNAL_RX_SW_BUFFER_CONST    (MotorComms_INTERNAL_RX_SW_BUFFER)
    #define MotorComms_INTERNAL_TX_SW_BUFFER_CONST    (MotorComms_INTERNAL_TX_SW_BUFFER)

    /* Wakeup for SPI */
    #define MotorComms_CHECK_SPI_WAKE_ENABLE  (MotorComms_SPI_WAKE_ENABLE_CONST)

#endif /* End (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*       Type Definitions
***************************************/

/* MotorComms_SPI_INIT_STRUCT */
typedef struct
{
    uint32 mode;
    uint32 submode;
    uint32 sclkMode;
    uint32 oversample;
    uint32 enableMedianFilter;
    uint32 enableLateSampling;
    uint32 enableWake;
    uint32 rxDataBits;
    uint32 txDataBits;
    uint32 bitOrder;
    uint32 transferSeperation;
    uint32 rxBufferSize;
    uint8* rxBuffer;
    uint32 txBufferSize;
    uint8* txBuffer;
    uint32 enableInterrupt;
    uint32 rxInterruptMask;
    uint32 rxTriggerLevel;
    uint32 txInterruptMask;
    uint32 txTriggerLevel;
    uint8 enableByteMode;
    uint8 enableFreeRunSclk;
    uint8 polaritySs;
} MotorComms_SPI_INIT_STRUCT;

/* MotorComms_UART_INIT_STRUCT */
typedef struct
{
    uint32 mode;
    uint32 direction;
    uint32 dataBits;
    uint32 parity;
    uint32 stopBits;
    uint32 oversample;
    uint32 enableIrdaLowPower;
    uint32 enableMedianFilter;
    uint32 enableRetryNack;
    uint32 enableInvertedRx;
    uint32 dropOnParityErr;
    uint32 dropOnFrameErr;
    uint32 enableWake;
    uint32 rxBufferSize;
    uint8* rxBuffer;
    uint32 txBufferSize;
    uint8* txBuffer;
    uint32 enableMultiproc;
    uint32 multiprocAcceptAddr;
    uint32 multiprocAddr;
    uint32 multiprocAddrMask;
    uint32 enableInterrupt;
    uint32 rxInterruptMask;
    uint32 rxTriggerLevel;
    uint32 txInterruptMask;
    uint32 txTriggerLevel;
    uint8 enableByteMode;
    uint8 enableCts;
    uint8 ctsPolarity;
    uint8 rtsRxFifoLevel;
    uint8 rtsPolarity;
} MotorComms_UART_INIT_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

/* SPI specific functions */
#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    void MotorComms_SpiInit(const MotorComms_SPI_INIT_STRUCT *config);
#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */

#if(MotorComms_SCB_MODE_SPI_INC)
    #define MotorComms_SpiIsBusBusy() ((uint32) (0u != (MotorComms_SPI_STATUS_REG & \
                                                              MotorComms_SPI_STATUS_BUS_BUSY)))

    #if (MotorComms_SPI_MASTER_CONST)
        void MotorComms_SpiSetActiveSlaveSelect(uint32 slaveSelect);
    #endif /*(MotorComms_SPI_MASTER_CONST) */

    #if !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
        void MotorComms_SpiSetSlaveSelectPolarity(uint32 slaveSelect, uint32 polarity);
    #endif /* !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */
#endif /* (MotorComms_SCB_MODE_SPI_INC) */

/* UART specific functions */
#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    void MotorComms_UartInit(const MotorComms_UART_INIT_STRUCT *config);
#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */

#if(MotorComms_SCB_MODE_UART_INC)
    void MotorComms_UartSetRxAddress(uint32 address);
    void MotorComms_UartSetRxAddressMask(uint32 addressMask);

    /* UART RX direction APIs */
    #if(MotorComms_UART_RX_DIRECTION)
        uint32 MotorComms_UartGetChar(void);
        uint32 MotorComms_UartGetByte(void);

        #if !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
            /* UART APIs for Flow Control */
            void MotorComms_UartSetRtsPolarity(uint32 polarity);
            void MotorComms_UartSetRtsFifoLevel(uint32 level);
        #endif /* !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */
    #endif /* (MotorComms_UART_RX_DIRECTION) */

    /* UART TX direction APIs */
    #if(MotorComms_UART_TX_DIRECTION)
        #define MotorComms_UartPutChar(ch)    MotorComms_SpiUartWriteTxData((uint32)(ch))
        void MotorComms_UartPutString(const char8 string[]);
        void MotorComms_UartPutCRLF(uint32 txDataByte);

        #if !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
            /* UART APIs for Flow Control */
            void MotorComms_UartEnableCts(void);
            void MotorComms_UartDisableCts(void);
            void MotorComms_UartSetCtsPolarity(uint32 polarity);
        #endif /* !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */
    #endif /* (MotorComms_UART_TX_DIRECTION) */
#endif /* (MotorComms_SCB_MODE_UART_INC) */

/* Common APIs RX direction */
#if(MotorComms_RX_DIRECTION)
    uint32 MotorComms_SpiUartReadRxData(void);
    uint32 MotorComms_SpiUartGetRxBufferSize(void);
    void   MotorComms_SpiUartClearRxBuffer(void);
#endif /* (MotorComms_RX_DIRECTION) */

/* Common APIs TX direction */
#if(MotorComms_TX_DIRECTION)
    void   MotorComms_SpiUartWriteTxData(uint32 txData);
    void   MotorComms_SpiUartPutArray(const uint8 wrBuf[], uint32 count);
    void   MotorComms_SpiUartClearTxBuffer(void);
    uint32 MotorComms_SpiUartGetTxBufferSize(void);
#endif /* (MotorComms_TX_DIRECTION) */

CY_ISR_PROTO(MotorComms_SPI_UART_ISR);

#if(MotorComms_UART_RX_WAKEUP_IRQ)
    CY_ISR_PROTO(MotorComms_UART_WAKEUP_ISR);
#endif /* (MotorComms_UART_RX_WAKEUP_IRQ) */


/***************************************
*     Buffer Access Macro Definitions
***************************************/

#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /* RX direction */
    void   MotorComms_PutWordInRxBuffer  (uint32 idx, uint32 rxDataByte);
    uint32 MotorComms_GetWordFromRxBuffer(uint32 idx);

    /* TX direction */
    void   MotorComms_PutWordInTxBuffer  (uint32 idx, uint32 txDataByte);
    uint32 MotorComms_GetWordFromTxBuffer(uint32 idx);

#else
    /* RX direction */
    #if(MotorComms_INTERNAL_RX_SW_BUFFER_CONST)
        #define MotorComms_PutWordInRxBuffer(idx, rxDataByte) \
                do{                                                 \
                    MotorComms_rxBufferInternal[(idx)] = ((uint8) (rxDataByte)); \
                }while(0)

        #define MotorComms_GetWordFromRxBuffer(idx) MotorComms_rxBufferInternal[(idx)]

    #endif /* (MotorComms_INTERNAL_RX_SW_BUFFER_CONST) */

    /* TX direction */
    #if(MotorComms_INTERNAL_TX_SW_BUFFER_CONST)
        #define MotorComms_PutWordInTxBuffer(idx, txDataByte) \
                    do{                                             \
                        MotorComms_txBufferInternal[(idx)] = ((uint8) (txDataByte)); \
                    }while(0)

        #define MotorComms_GetWordFromTxBuffer(idx) MotorComms_txBufferInternal[(idx)]

    #endif /* (MotorComms_INTERNAL_TX_SW_BUFFER_CONST) */

#endif /* (MotorComms_TX_SW_BUFFER_ENABLE) */


/***************************************
*         SPI API Constants
***************************************/

/* SPI sub mode enum */
#define MotorComms_SPI_MODE_MOTOROLA      (0x00u)
#define MotorComms_SPI_MODE_TI_COINCIDES  (0x01u)
#define MotorComms_SPI_MODE_TI_PRECEDES   (0x11u)
#define MotorComms_SPI_MODE_NATIONAL      (0x02u)
#define MotorComms_SPI_MODE_MASK          (0x03u)
#define MotorComms_SPI_MODE_TI_PRECEDES_MASK  (0x10u)
#define MotorComms_SPI_MODE_NS_MICROWIRE  (MotorComms_SPI_MODE_NATIONAL)

/* SPI phase and polarity mode enum */
#define MotorComms_SPI_SCLK_CPHA0_CPOL0   (0x00u)
#define MotorComms_SPI_SCLK_CPHA0_CPOL1   (0x02u)
#define MotorComms_SPI_SCLK_CPHA1_CPOL0   (0x01u)
#define MotorComms_SPI_SCLK_CPHA1_CPOL1   (0x03u)

/* SPI bits order enum */
#define MotorComms_BITS_ORDER_LSB_FIRST   (0u)
#define MotorComms_BITS_ORDER_MSB_FIRST   (1u)

/* SPI transfer separation enum */
#define MotorComms_SPI_TRANSFER_SEPARATED     (0u)
#define MotorComms_SPI_TRANSFER_CONTINUOUS    (1u)

/* SPI slave select constants */
#define MotorComms_SPI_SLAVE_SELECT0    (MotorComms_SCB__SS0_POSISTION)
#define MotorComms_SPI_SLAVE_SELECT1    (MotorComms_SCB__SS1_POSISTION)
#define MotorComms_SPI_SLAVE_SELECT2    (MotorComms_SCB__SS2_POSISTION)
#define MotorComms_SPI_SLAVE_SELECT3    (MotorComms_SCB__SS3_POSISTION)

/* SPI slave select polarity settings */
#define MotorComms_SPI_SS_ACTIVE_LOW  (0u)
#define MotorComms_SPI_SS_ACTIVE_HIGH (1u)


/***************************************
*         UART API Constants
***************************************/

/* UART sub-modes enum */
#define MotorComms_UART_MODE_STD          (0u)
#define MotorComms_UART_MODE_SMARTCARD    (1u)
#define MotorComms_UART_MODE_IRDA         (2u)

/* UART direction enum */
#define MotorComms_UART_RX    (1u)
#define MotorComms_UART_TX    (2u)
#define MotorComms_UART_TX_RX (3u)

/* UART parity enum */
#define MotorComms_UART_PARITY_EVEN   (0u)
#define MotorComms_UART_PARITY_ODD    (1u)
#define MotorComms_UART_PARITY_NONE   (2u)

/* UART stop bits enum */
#define MotorComms_UART_STOP_BITS_1   (2u)
#define MotorComms_UART_STOP_BITS_1_5 (3u)
#define MotorComms_UART_STOP_BITS_2   (4u)

/* UART IrDA low power OVS enum */
#define MotorComms_UART_IRDA_LP_OVS16     (16u)
#define MotorComms_UART_IRDA_LP_OVS32     (32u)
#define MotorComms_UART_IRDA_LP_OVS48     (48u)
#define MotorComms_UART_IRDA_LP_OVS96     (96u)
#define MotorComms_UART_IRDA_LP_OVS192    (192u)
#define MotorComms_UART_IRDA_LP_OVS768    (768u)
#define MotorComms_UART_IRDA_LP_OVS1536   (1536u)

/* Uart MP: mark (address) and space (data) bit definitions */
#define MotorComms_UART_MP_MARK       (0x100u)
#define MotorComms_UART_MP_SPACE      (0x000u)

/* UART CTS/RTS polarity settings */
#define MotorComms_UART_CTS_ACTIVE_LOW    (0u)
#define MotorComms_UART_CTS_ACTIVE_HIGH   (1u)
#define MotorComms_UART_RTS_ACTIVE_LOW    (0u)
#define MotorComms_UART_RTS_ACTIVE_HIGH   (1u)

/* Sources of RX errors */
#define MotorComms_INTR_RX_ERR        (MotorComms_INTR_RX_OVERFLOW    | \
                                             MotorComms_INTR_RX_UNDERFLOW   | \
                                             MotorComms_INTR_RX_FRAME_ERROR | \
                                             MotorComms_INTR_RX_PARITY_ERROR)

/* Shifted INTR_RX_ERR defines ONLY for MotorComms_UartGetByte() */
#define MotorComms_UART_RX_OVERFLOW       (MotorComms_INTR_RX_OVERFLOW << 8u)
#define MotorComms_UART_RX_UNDERFLOW      (MotorComms_INTR_RX_UNDERFLOW << 8u)
#define MotorComms_UART_RX_FRAME_ERROR    (MotorComms_INTR_RX_FRAME_ERROR << 8u)
#define MotorComms_UART_RX_PARITY_ERROR   (MotorComms_INTR_RX_PARITY_ERROR << 8u)
#define MotorComms_UART_RX_ERROR_MASK     (MotorComms_UART_RX_OVERFLOW    | \
                                                 MotorComms_UART_RX_UNDERFLOW   | \
                                                 MotorComms_UART_RX_FRAME_ERROR | \
                                                 MotorComms_UART_RX_PARITY_ERROR)


/***************************************
*     Vars with External Linkage
***************************************/

#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    extern const MotorComms_SPI_INIT_STRUCT  MotorComms_configSpi;
    extern const MotorComms_UART_INIT_STRUCT MotorComms_configUart;
#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*    Specific SPI Macro Definitions
***************************************/

#define MotorComms_GET_SPI_INTR_SLAVE_MASK(sourceMask)  ((sourceMask) & MotorComms_INTR_SLAVE_SPI_BUS_ERROR)
#define MotorComms_GET_SPI_INTR_MASTER_MASK(sourceMask) ((sourceMask) & MotorComms_INTR_MASTER_SPI_DONE)
#define MotorComms_GET_SPI_INTR_RX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~MotorComms_INTR_SLAVE_SPI_BUS_ERROR)

#define MotorComms_GET_SPI_INTR_TX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~MotorComms_INTR_MASTER_SPI_DONE)


/***************************************
*    Specific UART Macro Definitions
***************************************/

#define MotorComms_UART_GET_CTRL_OVS_IRDA_LP(oversample) \
        ((MotorComms_UART_IRDA_LP_OVS16   == (oversample)) ? MotorComms_CTRL_OVS_IRDA_LP_OVS16 : \
         ((MotorComms_UART_IRDA_LP_OVS32   == (oversample)) ? MotorComms_CTRL_OVS_IRDA_LP_OVS32 : \
          ((MotorComms_UART_IRDA_LP_OVS48   == (oversample)) ? MotorComms_CTRL_OVS_IRDA_LP_OVS48 : \
           ((MotorComms_UART_IRDA_LP_OVS96   == (oversample)) ? MotorComms_CTRL_OVS_IRDA_LP_OVS96 : \
            ((MotorComms_UART_IRDA_LP_OVS192  == (oversample)) ? MotorComms_CTRL_OVS_IRDA_LP_OVS192 : \
             ((MotorComms_UART_IRDA_LP_OVS768  == (oversample)) ? MotorComms_CTRL_OVS_IRDA_LP_OVS768 : \
              ((MotorComms_UART_IRDA_LP_OVS1536 == (oversample)) ? MotorComms_CTRL_OVS_IRDA_LP_OVS1536 : \
                                                                          MotorComms_CTRL_OVS_IRDA_LP_OVS16)))))))

#define MotorComms_GET_UART_RX_CTRL_ENABLED(direction) ((0u != (MotorComms_UART_RX & (direction))) ? \
                                                                     (MotorComms_RX_CTRL_ENABLED) : (0u))

#define MotorComms_GET_UART_TX_CTRL_ENABLED(direction) ((0u != (MotorComms_UART_TX & (direction))) ? \
                                                                     (MotorComms_TX_CTRL_ENABLED) : (0u))


/***************************************
*        SPI Register Settings
***************************************/

#define MotorComms_CTRL_SPI      (MotorComms_CTRL_MODE_SPI)
#define MotorComms_SPI_RX_CTRL   (MotorComms_RX_CTRL_ENABLED)
#define MotorComms_SPI_TX_CTRL   (MotorComms_TX_CTRL_ENABLED)


/***************************************
*       SPI Init Register Settings
***************************************/

#define MotorComms_SPI_SS_POLARITY \
             (((uint32) MotorComms_SPI_SS0_POLARITY << MotorComms_SPI_SLAVE_SELECT0) | \
              ((uint32) MotorComms_SPI_SS1_POLARITY << MotorComms_SPI_SLAVE_SELECT1) | \
              ((uint32) MotorComms_SPI_SS2_POLARITY << MotorComms_SPI_SLAVE_SELECT2) | \
              ((uint32) MotorComms_SPI_SS3_POLARITY << MotorComms_SPI_SLAVE_SELECT3))

#if(MotorComms_SCB_MODE_SPI_CONST_CFG)

    /* SPI Configuration */
    #define MotorComms_SPI_DEFAULT_CTRL \
                    (MotorComms_GET_CTRL_OVS(MotorComms_SPI_OVS_FACTOR) | \
                     MotorComms_GET_CTRL_BYTE_MODE (MotorComms_SPI_BYTE_MODE_ENABLE) | \
                     MotorComms_GET_CTRL_EC_AM_MODE(MotorComms_SPI_WAKE_ENABLE)      | \
                     MotorComms_CTRL_SPI)

    #define MotorComms_SPI_DEFAULT_SPI_CTRL \
                    (MotorComms_GET_SPI_CTRL_CONTINUOUS    (MotorComms_SPI_TRANSFER_SEPARATION)       | \
                     MotorComms_GET_SPI_CTRL_SELECT_PRECEDE(MotorComms_SPI_SUB_MODE &                   \
                                                                  MotorComms_SPI_MODE_TI_PRECEDES_MASK)     | \
                     MotorComms_GET_SPI_CTRL_SCLK_MODE     (MotorComms_SPI_CLOCK_MODE)                | \
                     MotorComms_GET_SPI_CTRL_LATE_MISO_SAMPLE(MotorComms_SPI_LATE_MISO_SAMPLE_ENABLE) | \
                     MotorComms_GET_SPI_CTRL_SCLK_CONTINUOUS(MotorComms_SPI_FREE_RUN_SCLK_ENABLE)     | \
                     MotorComms_GET_SPI_CTRL_SSEL_POLARITY (MotorComms_SPI_SS_POLARITY)               | \
                     MotorComms_GET_SPI_CTRL_SUB_MODE      (MotorComms_SPI_SUB_MODE)                  | \
                     MotorComms_GET_SPI_CTRL_MASTER_MODE   (MotorComms_SPI_MODE))

    /* RX direction */
    #define MotorComms_SPI_DEFAULT_RX_CTRL \
                    (MotorComms_GET_RX_CTRL_DATA_WIDTH(MotorComms_SPI_RX_DATA_BITS_NUM)     | \
                     MotorComms_GET_RX_CTRL_BIT_ORDER (MotorComms_SPI_BITS_ORDER)           | \
                     MotorComms_GET_RX_CTRL_MEDIAN    (MotorComms_SPI_MEDIAN_FILTER_ENABLE) | \
                     MotorComms_SPI_RX_CTRL)

    #define MotorComms_SPI_DEFAULT_RX_FIFO_CTRL \
                    MotorComms_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(MotorComms_SPI_RX_TRIGGER_LEVEL)

    /* TX direction */
    #define MotorComms_SPI_DEFAULT_TX_CTRL \
                    (MotorComms_GET_TX_CTRL_DATA_WIDTH(MotorComms_SPI_TX_DATA_BITS_NUM) | \
                     MotorComms_GET_TX_CTRL_BIT_ORDER (MotorComms_SPI_BITS_ORDER)       | \
                     MotorComms_SPI_TX_CTRL)

    #define MotorComms_SPI_DEFAULT_TX_FIFO_CTRL \
                    MotorComms_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(MotorComms_SPI_TX_TRIGGER_LEVEL)

    /* Interrupt sources */
    #define MotorComms_SPI_DEFAULT_INTR_SPI_EC_MASK   (MotorComms_NO_INTR_SOURCES)

    #define MotorComms_SPI_DEFAULT_INTR_I2C_EC_MASK   (MotorComms_NO_INTR_SOURCES)
    #define MotorComms_SPI_DEFAULT_INTR_SLAVE_MASK \
                    (MotorComms_SPI_INTR_RX_MASK & MotorComms_INTR_SLAVE_SPI_BUS_ERROR)

    #define MotorComms_SPI_DEFAULT_INTR_MASTER_MASK \
                    (MotorComms_SPI_INTR_TX_MASK & MotorComms_INTR_MASTER_SPI_DONE)

    #define MotorComms_SPI_DEFAULT_INTR_RX_MASK \
                    (MotorComms_SPI_INTR_RX_MASK & (uint32) ~MotorComms_INTR_SLAVE_SPI_BUS_ERROR)

    #define MotorComms_SPI_DEFAULT_INTR_TX_MASK \
                    (MotorComms_SPI_INTR_TX_MASK & (uint32) ~MotorComms_INTR_MASTER_SPI_DONE)

#endif /* (MotorComms_SCB_MODE_SPI_CONST_CFG) */


/***************************************
*        UART Register Settings
***************************************/

#define MotorComms_CTRL_UART      (MotorComms_CTRL_MODE_UART)
#define MotorComms_UART_RX_CTRL   (MotorComms_RX_CTRL_LSB_FIRST) /* LSB for UART goes first */
#define MotorComms_UART_TX_CTRL   (MotorComms_TX_CTRL_LSB_FIRST) /* LSB for UART goes first */


/***************************************
*      UART Init Register Settings
***************************************/

#if(MotorComms_SCB_MODE_UART_CONST_CFG)

    /* UART configuration */
    #if(MotorComms_UART_MODE_IRDA == MotorComms_UART_SUB_MODE)

        #define MotorComms_DEFAULT_CTRL_OVS   ((0u != MotorComms_UART_IRDA_LOW_POWER) ?              \
                                (MotorComms_UART_GET_CTRL_OVS_IRDA_LP(MotorComms_UART_OVS_FACTOR)) : \
                                (MotorComms_CTRL_OVS_IRDA_OVS16))

    #else

        #define MotorComms_DEFAULT_CTRL_OVS   MotorComms_GET_CTRL_OVS(MotorComms_UART_OVS_FACTOR)

    #endif /* (MotorComms_UART_MODE_IRDA == MotorComms_UART_SUB_MODE) */

    #define MotorComms_UART_DEFAULT_CTRL \
                                (MotorComms_GET_CTRL_BYTE_MODE  (MotorComms_UART_BYTE_MODE_ENABLE)  | \
                                 MotorComms_GET_CTRL_ADDR_ACCEPT(MotorComms_UART_MP_ACCEPT_ADDRESS) | \
                                 MotorComms_DEFAULT_CTRL_OVS                                              | \
                                 MotorComms_CTRL_UART)

    #define MotorComms_UART_DEFAULT_UART_CTRL \
                                    (MotorComms_GET_UART_CTRL_MODE(MotorComms_UART_SUB_MODE))

    /* RX direction */
    #define MotorComms_UART_DEFAULT_RX_CTRL_PARITY \
                                ((MotorComms_UART_PARITY_NONE != MotorComms_UART_PARITY_TYPE) ?      \
                                  (MotorComms_GET_UART_RX_CTRL_PARITY(MotorComms_UART_PARITY_TYPE) | \
                                   MotorComms_UART_RX_CTRL_PARITY_ENABLED) : (0u))

    #define MotorComms_UART_DEFAULT_UART_RX_CTRL \
                    (MotorComms_GET_UART_RX_CTRL_MODE(MotorComms_UART_STOP_BITS_NUM)                    | \
                     MotorComms_GET_UART_RX_CTRL_POLARITY(MotorComms_UART_IRDA_POLARITY)                | \
                     MotorComms_GET_UART_RX_CTRL_MP_MODE(MotorComms_UART_MP_MODE_ENABLE)                | \
                     MotorComms_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(MotorComms_UART_DROP_ON_PARITY_ERR) | \
                     MotorComms_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(MotorComms_UART_DROP_ON_FRAME_ERR)   | \
                     MotorComms_UART_DEFAULT_RX_CTRL_PARITY)

    #define MotorComms_UART_DEFAULT_RX_CTRL \
                                (MotorComms_GET_RX_CTRL_DATA_WIDTH(MotorComms_UART_DATA_BITS_NUM)        | \
                                 MotorComms_GET_RX_CTRL_MEDIAN    (MotorComms_UART_MEDIAN_FILTER_ENABLE) | \
                                 MotorComms_GET_UART_RX_CTRL_ENABLED(MotorComms_UART_DIRECTION))

    #define MotorComms_UART_DEFAULT_RX_FIFO_CTRL \
                                MotorComms_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(MotorComms_UART_RX_TRIGGER_LEVEL)

    #define MotorComms_UART_DEFAULT_RX_MATCH_REG  ((0u != MotorComms_UART_MP_MODE_ENABLE) ?          \
                                (MotorComms_GET_RX_MATCH_ADDR(MotorComms_UART_MP_RX_ADDRESS) | \
                                 MotorComms_GET_RX_MATCH_MASK(MotorComms_UART_MP_RX_ADDRESS_MASK)) : (0u))

    /* TX direction */
    #define MotorComms_UART_DEFAULT_TX_CTRL_PARITY (MotorComms_UART_DEFAULT_RX_CTRL_PARITY)

    #define MotorComms_UART_DEFAULT_UART_TX_CTRL \
                                (MotorComms_GET_UART_TX_CTRL_MODE(MotorComms_UART_STOP_BITS_NUM)       | \
                                 MotorComms_GET_UART_TX_CTRL_RETRY_NACK(MotorComms_UART_RETRY_ON_NACK) | \
                                 MotorComms_UART_DEFAULT_TX_CTRL_PARITY)

    #define MotorComms_UART_DEFAULT_TX_CTRL \
                                (MotorComms_GET_TX_CTRL_DATA_WIDTH(MotorComms_UART_DATA_BITS_NUM) | \
                                 MotorComms_GET_UART_TX_CTRL_ENABLED(MotorComms_UART_DIRECTION))

    #define MotorComms_UART_DEFAULT_TX_FIFO_CTRL \
                                MotorComms_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(MotorComms_UART_TX_TRIGGER_LEVEL)

    #define MotorComms_UART_DEFAULT_FLOW_CTRL \
                        (MotorComms_GET_UART_FLOW_CTRL_TRIGGER_LEVEL(MotorComms_UART_RTS_FIFO_LEVEL) | \
                         MotorComms_GET_UART_FLOW_CTRL_RTS_POLARITY (MotorComms_UART_RTS_POLARITY)   | \
                         MotorComms_GET_UART_FLOW_CTRL_CTS_POLARITY (MotorComms_UART_CTS_POLARITY)   | \
                         MotorComms_GET_UART_FLOW_CTRL_CTS_ENABLE   (MotorComms_UART_CTS_ENABLE))

    /* Interrupt sources */
    #define MotorComms_UART_DEFAULT_INTR_I2C_EC_MASK  (MotorComms_NO_INTR_SOURCES)
    #define MotorComms_UART_DEFAULT_INTR_SPI_EC_MASK  (MotorComms_NO_INTR_SOURCES)
    #define MotorComms_UART_DEFAULT_INTR_SLAVE_MASK   (MotorComms_NO_INTR_SOURCES)
    #define MotorComms_UART_DEFAULT_INTR_MASTER_MASK  (MotorComms_NO_INTR_SOURCES)
    #define MotorComms_UART_DEFAULT_INTR_RX_MASK      (MotorComms_UART_INTR_RX_MASK)
    #define MotorComms_UART_DEFAULT_INTR_TX_MASK      (MotorComms_UART_INTR_TX_MASK)

#endif /* (MotorComms_SCB_MODE_UART_CONST_CFG) */


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

#define MotorComms_SPIM_ACTIVE_SS0    (MotorComms_SPI_SLAVE_SELECT0)
#define MotorComms_SPIM_ACTIVE_SS1    (MotorComms_SPI_SLAVE_SELECT1)
#define MotorComms_SPIM_ACTIVE_SS2    (MotorComms_SPI_SLAVE_SELECT2)
#define MotorComms_SPIM_ACTIVE_SS3    (MotorComms_SPI_SLAVE_SELECT3)

#endif /* CY_SCB_SPI_UART_MotorComms_H */


/* [] END OF FILE */
