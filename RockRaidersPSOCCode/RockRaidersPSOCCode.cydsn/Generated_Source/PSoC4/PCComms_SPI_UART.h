/*******************************************************************************
* File Name: PCComms_SPI_UART.h
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

#if !defined(CY_SCB_SPI_UART_PCComms_H)
#define CY_SCB_SPI_UART_PCComms_H

#include "PCComms.h"


/***************************************
*   SPI Initial Parameter Constants
****************************************/

#define PCComms_SPI_MODE                   (0u)
#define PCComms_SPI_SUB_MODE               (0u)
#define PCComms_SPI_CLOCK_MODE             (0u)
#define PCComms_SPI_OVS_FACTOR             (16u)
#define PCComms_SPI_MEDIAN_FILTER_ENABLE   (0u)
#define PCComms_SPI_LATE_MISO_SAMPLE_ENABLE (0u)
#define PCComms_SPI_RX_DATA_BITS_NUM       (8u)
#define PCComms_SPI_TX_DATA_BITS_NUM       (8u)
#define PCComms_SPI_WAKE_ENABLE            (0u)
#define PCComms_SPI_BITS_ORDER             (1u)
#define PCComms_SPI_TRANSFER_SEPARATION    (1u)
#define PCComms_SPI_NUMBER_OF_SS_LINES     (1u)
#define PCComms_SPI_RX_BUFFER_SIZE         (8u)
#define PCComms_SPI_TX_BUFFER_SIZE         (8u)

#define PCComms_SPI_INTERRUPT_MODE         (0u)

#define PCComms_SPI_INTR_RX_MASK           (0u)
#define PCComms_SPI_INTR_TX_MASK           (0u)

#define PCComms_SPI_RX_TRIGGER_LEVEL       (7u)
#define PCComms_SPI_TX_TRIGGER_LEVEL       (0u)

#define PCComms_SPI_BYTE_MODE_ENABLE       (0u)
#define PCComms_SPI_FREE_RUN_SCLK_ENABLE   (0u)
#define PCComms_SPI_SS0_POLARITY           (0u)
#define PCComms_SPI_SS1_POLARITY           (0u)
#define PCComms_SPI_SS2_POLARITY           (0u)
#define PCComms_SPI_SS3_POLARITY           (0u)


/***************************************
*   UART Initial Parameter Constants
****************************************/

#define PCComms_UART_SUB_MODE              (0u)
#define PCComms_UART_DIRECTION             (3u)
#define PCComms_UART_DATA_BITS_NUM         (8u)
#define PCComms_UART_PARITY_TYPE           (2u)
#define PCComms_UART_STOP_BITS_NUM         (2u)
#define PCComms_UART_OVS_FACTOR            (12u)
#define PCComms_UART_IRDA_LOW_POWER        (0u)
#define PCComms_UART_MEDIAN_FILTER_ENABLE  (0u)
#define PCComms_UART_RETRY_ON_NACK         (0u)
#define PCComms_UART_IRDA_POLARITY         (0u)
#define PCComms_UART_DROP_ON_FRAME_ERR     (0u)
#define PCComms_UART_DROP_ON_PARITY_ERR    (0u)
#define PCComms_UART_WAKE_ENABLE           (0u)
#define PCComms_UART_RX_BUFFER_SIZE        (8u)
#define PCComms_UART_TX_BUFFER_SIZE        (8u)
#define PCComms_UART_MP_MODE_ENABLE        (0u)
#define PCComms_UART_MP_ACCEPT_ADDRESS     (0u)
#define PCComms_UART_MP_RX_ADDRESS         (2u)
#define PCComms_UART_MP_RX_ADDRESS_MASK    (255u)

#define PCComms_UART_INTERRUPT_MODE        (1u)

#define PCComms_UART_INTR_RX_MASK          (12u)
#define PCComms_UART_INTR_TX_MASK          (0u)

#define PCComms_UART_RX_TRIGGER_LEVEL      (7u)
#define PCComms_UART_TX_TRIGGER_LEVEL      (0u)

#define PCComms_UART_BYTE_MODE_ENABLE      (0u)
#define PCComms_UART_CTS_ENABLE            (0u)
#define PCComms_UART_CTS_POLARITY          (0u)
#define PCComms_UART_RTS_POLARITY          (0u)
#define PCComms_UART_RTS_FIFO_LEVEL        (4u)

/* SPI mode enum */
#define PCComms_SPI_SLAVE  (0u)
#define PCComms_SPI_MASTER (1u)

/* UART direction enum */
#define PCComms_UART_RX    (1u)
#define PCComms_UART_TX    (2u)
#define PCComms_UART_TX_RX (3u)


/***************************************
*   Conditional Compilation Parameters
****************************************/

#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)

    /* Mode */
    #define PCComms_SPI_MASTER_CONST       (1u)

    /* Direction */
    #define PCComms_RX_DIRECTION           (1u)
    #define PCComms_TX_DIRECTION           (1u)
    #define PCComms_UART_RX_DIRECTION      (1u)
    #define PCComms_UART_TX_DIRECTION      (1u)

    /* Only external RX and TX buffer for Uncofigured mode */
    #define PCComms_INTERNAL_RX_SW_BUFFER   (0u)
    #define PCComms_INTERNAL_TX_SW_BUFFER   (0u)

    /* Get RX and TX buffer size */
    #define PCComms_RX_BUFFER_SIZE (PCComms_rxBufferSize)
    #define PCComms_TX_BUFFER_SIZE (PCComms_txBufferSize)

    /* Return true if buffer is provided */
    #define PCComms_CHECK_RX_SW_BUFFER (NULL != PCComms_rxBuffer)
    #define PCComms_CHECK_TX_SW_BUFFER (NULL != PCComms_txBuffer)

    /* Always provide global variables to support RX and TX buffers */
    #define PCComms_INTERNAL_RX_SW_BUFFER_CONST    (1u)
    #define PCComms_INTERNAL_TX_SW_BUFFER_CONST    (1u)

    /* Get wakeup enable option */
    #define PCComms_SPI_WAKE_ENABLE_CONST  (1u)
    #define PCComms_CHECK_SPI_WAKE_ENABLE  (0u != PCComms_scbEnableWake)
    #define PCComms_UART_WAKE_ENABLE_CONST (1u)

    /* SPI/UART: TX or RX FIFO size */
    #if (PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
        #define PCComms_SPI_UART_FIFO_SIZE (PCComms_FIFO_SIZE)
    #else
        #define PCComms_SPI_UART_FIFO_SIZE (PCComms_GET_FIFO_SIZE(PCComms_CTRL_REG & \
                                                                                    PCComms_CTRL_BYTE_MODE))
    #endif /* (PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */

#else

    /* Internal RX and TX buffer: for SPI or UART */
    #if (PCComms_SCB_MODE_SPI_CONST_CFG)

        /* SPI Direction */
        #define PCComms_SPI_RX_DIRECTION (1u)
        #define PCComms_SPI_TX_DIRECTION (1u)

        /* Get FIFO size */
        #if (PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
            #define PCComms_SPI_UART_FIFO_SIZE    (PCComms_FIFO_SIZE)
        #else
            #define PCComms_SPI_UART_FIFO_SIZE \
                                           PCComms_GET_FIFO_SIZE(PCComms_SPI_BYTE_MODE_ENABLE)

        #endif /* (PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */

        /* SPI internal RX and TX buffers */
        #define PCComms_INTERNAL_SPI_RX_SW_BUFFER  (PCComms_SPI_RX_BUFFER_SIZE > \
                                                                PCComms_SPI_UART_FIFO_SIZE)
        #define PCComms_INTERNAL_SPI_TX_SW_BUFFER  (PCComms_SPI_TX_BUFFER_SIZE > \
                                                                PCComms_SPI_UART_FIFO_SIZE)

        /* Internal SPI RX and TX buffer */
        #define PCComms_INTERNAL_RX_SW_BUFFER  (PCComms_INTERNAL_SPI_RX_SW_BUFFER)
        #define PCComms_INTERNAL_TX_SW_BUFFER  (PCComms_INTERNAL_SPI_TX_SW_BUFFER)

        /* Internal SPI RX and TX buffer size */
        #define PCComms_RX_BUFFER_SIZE         (PCComms_SPI_RX_BUFFER_SIZE + 1u)
        #define PCComms_TX_BUFFER_SIZE         (PCComms_SPI_TX_BUFFER_SIZE)

        /* Get wakeup enable option */
        #define PCComms_SPI_WAKE_ENABLE_CONST  (0u != PCComms_SPI_WAKE_ENABLE)
        #define PCComms_UART_WAKE_ENABLE_CONST (0u)

    #else

        /* UART Direction */
        #define PCComms_UART_RX_DIRECTION (0u != (PCComms_UART_DIRECTION & PCComms_UART_RX))
        #define PCComms_UART_TX_DIRECTION (0u != (PCComms_UART_DIRECTION & PCComms_UART_TX))

        /* Get FIFO size */
        #if (PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
            #define PCComms_SPI_UART_FIFO_SIZE    (PCComms_FIFO_SIZE)
        #else
            #define PCComms_SPI_UART_FIFO_SIZE \
                                           PCComms_GET_FIFO_SIZE(PCComms_UART_BYTE_MODE_ENABLE)
        #endif /* (PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */

        /* UART internal RX and TX buffers */
        #define PCComms_INTERNAL_UART_RX_SW_BUFFER  (PCComms_UART_RX_BUFFER_SIZE > \
                                                                PCComms_SPI_UART_FIFO_SIZE)
        #define PCComms_INTERNAL_UART_TX_SW_BUFFER  (PCComms_UART_TX_BUFFER_SIZE > \
                                                                    PCComms_SPI_UART_FIFO_SIZE)

        /* Internal UART RX and TX buffer */
        #define PCComms_INTERNAL_RX_SW_BUFFER  (PCComms_INTERNAL_UART_RX_SW_BUFFER)
        #define PCComms_INTERNAL_TX_SW_BUFFER  (PCComms_INTERNAL_UART_TX_SW_BUFFER)

        /* Internal UART RX and TX buffer size */
        #define PCComms_RX_BUFFER_SIZE         (PCComms_UART_RX_BUFFER_SIZE + 1u)
        #define PCComms_TX_BUFFER_SIZE         (PCComms_UART_TX_BUFFER_SIZE)

        /* Get wakeup enable option */
        #define PCComms_SPI_WAKE_ENABLE_CONST  (0u)
        #define PCComms_UART_WAKE_ENABLE_CONST (0u != PCComms_UART_WAKE_ENABLE)

    #endif /* (PCComms_SCB_MODE_SPI_CONST_CFG) */

    /* Mode */
    #define PCComms_SPI_MASTER_CONST   (PCComms_SPI_MODE == PCComms_SPI_MASTER)

    /* Direction */
    #define PCComms_RX_DIRECTION ((PCComms_SCB_MODE_SPI_CONST_CFG) ? \
                                            (PCComms_SPI_RX_DIRECTION) : (PCComms_UART_RX_DIRECTION))

    #define PCComms_TX_DIRECTION ((PCComms_SCB_MODE_SPI_CONST_CFG) ? \
                                            (PCComms_SPI_TX_DIRECTION) : (PCComms_UART_TX_DIRECTION))

    /* Internal RX and TX buffer: for SPI or UART. Used in conditional compilation check */
    #define PCComms_CHECK_RX_SW_BUFFER (PCComms_INTERNAL_RX_SW_BUFFER)
    #define PCComms_CHECK_TX_SW_BUFFER (PCComms_INTERNAL_TX_SW_BUFFER)

    /* Provide global variables to support RX and TX buffers */
    #define PCComms_INTERNAL_RX_SW_BUFFER_CONST    (PCComms_INTERNAL_RX_SW_BUFFER)
    #define PCComms_INTERNAL_TX_SW_BUFFER_CONST    (PCComms_INTERNAL_TX_SW_BUFFER)

    /* Wakeup for SPI */
    #define PCComms_CHECK_SPI_WAKE_ENABLE  (PCComms_SPI_WAKE_ENABLE_CONST)

#endif /* End (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*       Type Definitions
***************************************/

/* PCComms_SPI_INIT_STRUCT */
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
} PCComms_SPI_INIT_STRUCT;

/* PCComms_UART_INIT_STRUCT */
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
} PCComms_UART_INIT_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

/* SPI specific functions */
#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    void PCComms_SpiInit(const PCComms_SPI_INIT_STRUCT *config);
#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */

#if(PCComms_SCB_MODE_SPI_INC)
    #define PCComms_SpiIsBusBusy() ((uint32) (0u != (PCComms_SPI_STATUS_REG & \
                                                              PCComms_SPI_STATUS_BUS_BUSY)))

    #if (PCComms_SPI_MASTER_CONST)
        void PCComms_SpiSetActiveSlaveSelect(uint32 slaveSelect);
    #endif /*(PCComms_SPI_MASTER_CONST) */

    #if !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
        void PCComms_SpiSetSlaveSelectPolarity(uint32 slaveSelect, uint32 polarity);
    #endif /* !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */
#endif /* (PCComms_SCB_MODE_SPI_INC) */

/* UART specific functions */
#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    void PCComms_UartInit(const PCComms_UART_INIT_STRUCT *config);
#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */

#if(PCComms_SCB_MODE_UART_INC)
    void PCComms_UartSetRxAddress(uint32 address);
    void PCComms_UartSetRxAddressMask(uint32 addressMask);

    /* UART RX direction APIs */
    #if(PCComms_UART_RX_DIRECTION)
        uint32 PCComms_UartGetChar(void);
        uint32 PCComms_UartGetByte(void);

        #if !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
            /* UART APIs for Flow Control */
            void PCComms_UartSetRtsPolarity(uint32 polarity);
            void PCComms_UartSetRtsFifoLevel(uint32 level);
        #endif /* !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */
    #endif /* (PCComms_UART_RX_DIRECTION) */

    /* UART TX direction APIs */
    #if(PCComms_UART_TX_DIRECTION)
        #define PCComms_UartPutChar(ch)    PCComms_SpiUartWriteTxData((uint32)(ch))
        void PCComms_UartPutString(const char8 string[]);
        void PCComms_UartPutCRLF(uint32 txDataByte);

        #if !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
            /* UART APIs for Flow Control */
            void PCComms_UartEnableCts(void);
            void PCComms_UartDisableCts(void);
            void PCComms_UartSetCtsPolarity(uint32 polarity);
        #endif /* !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */
    #endif /* (PCComms_UART_TX_DIRECTION) */
#endif /* (PCComms_SCB_MODE_UART_INC) */

/* Common APIs RX direction */
#if(PCComms_RX_DIRECTION)
    uint32 PCComms_SpiUartReadRxData(void);
    uint32 PCComms_SpiUartGetRxBufferSize(void);
    void   PCComms_SpiUartClearRxBuffer(void);
#endif /* (PCComms_RX_DIRECTION) */

/* Common APIs TX direction */
#if(PCComms_TX_DIRECTION)
    void   PCComms_SpiUartWriteTxData(uint32 txData);
    void   PCComms_SpiUartPutArray(const uint8 wrBuf[], uint32 count);
    void   PCComms_SpiUartClearTxBuffer(void);
    uint32 PCComms_SpiUartGetTxBufferSize(void);
#endif /* (PCComms_TX_DIRECTION) */

CY_ISR_PROTO(PCComms_SPI_UART_ISR);

#if(PCComms_UART_RX_WAKEUP_IRQ)
    CY_ISR_PROTO(PCComms_UART_WAKEUP_ISR);
#endif /* (PCComms_UART_RX_WAKEUP_IRQ) */


/***************************************
*     Buffer Access Macro Definitions
***************************************/

#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /* RX direction */
    void   PCComms_PutWordInRxBuffer  (uint32 idx, uint32 rxDataByte);
    uint32 PCComms_GetWordFromRxBuffer(uint32 idx);

    /* TX direction */
    void   PCComms_PutWordInTxBuffer  (uint32 idx, uint32 txDataByte);
    uint32 PCComms_GetWordFromTxBuffer(uint32 idx);

#else
    /* RX direction */
    #if(PCComms_INTERNAL_RX_SW_BUFFER_CONST)
        #define PCComms_PutWordInRxBuffer(idx, rxDataByte) \
                do{                                                 \
                    PCComms_rxBufferInternal[(idx)] = ((uint8) (rxDataByte)); \
                }while(0)

        #define PCComms_GetWordFromRxBuffer(idx) PCComms_rxBufferInternal[(idx)]

    #endif /* (PCComms_INTERNAL_RX_SW_BUFFER_CONST) */

    /* TX direction */
    #if(PCComms_INTERNAL_TX_SW_BUFFER_CONST)
        #define PCComms_PutWordInTxBuffer(idx, txDataByte) \
                    do{                                             \
                        PCComms_txBufferInternal[(idx)] = ((uint8) (txDataByte)); \
                    }while(0)

        #define PCComms_GetWordFromTxBuffer(idx) PCComms_txBufferInternal[(idx)]

    #endif /* (PCComms_INTERNAL_TX_SW_BUFFER_CONST) */

#endif /* (PCComms_TX_SW_BUFFER_ENABLE) */


/***************************************
*         SPI API Constants
***************************************/

/* SPI sub mode enum */
#define PCComms_SPI_MODE_MOTOROLA      (0x00u)
#define PCComms_SPI_MODE_TI_COINCIDES  (0x01u)
#define PCComms_SPI_MODE_TI_PRECEDES   (0x11u)
#define PCComms_SPI_MODE_NATIONAL      (0x02u)
#define PCComms_SPI_MODE_MASK          (0x03u)
#define PCComms_SPI_MODE_TI_PRECEDES_MASK  (0x10u)
#define PCComms_SPI_MODE_NS_MICROWIRE  (PCComms_SPI_MODE_NATIONAL)

/* SPI phase and polarity mode enum */
#define PCComms_SPI_SCLK_CPHA0_CPOL0   (0x00u)
#define PCComms_SPI_SCLK_CPHA0_CPOL1   (0x02u)
#define PCComms_SPI_SCLK_CPHA1_CPOL0   (0x01u)
#define PCComms_SPI_SCLK_CPHA1_CPOL1   (0x03u)

/* SPI bits order enum */
#define PCComms_BITS_ORDER_LSB_FIRST   (0u)
#define PCComms_BITS_ORDER_MSB_FIRST   (1u)

/* SPI transfer separation enum */
#define PCComms_SPI_TRANSFER_SEPARATED     (0u)
#define PCComms_SPI_TRANSFER_CONTINUOUS    (1u)

/* SPI slave select constants */
#define PCComms_SPI_SLAVE_SELECT0    (PCComms_SCB__SS0_POSISTION)
#define PCComms_SPI_SLAVE_SELECT1    (PCComms_SCB__SS1_POSISTION)
#define PCComms_SPI_SLAVE_SELECT2    (PCComms_SCB__SS2_POSISTION)
#define PCComms_SPI_SLAVE_SELECT3    (PCComms_SCB__SS3_POSISTION)

/* SPI slave select polarity settings */
#define PCComms_SPI_SS_ACTIVE_LOW  (0u)
#define PCComms_SPI_SS_ACTIVE_HIGH (1u)


/***************************************
*         UART API Constants
***************************************/

/* UART sub-modes enum */
#define PCComms_UART_MODE_STD          (0u)
#define PCComms_UART_MODE_SMARTCARD    (1u)
#define PCComms_UART_MODE_IRDA         (2u)

/* UART direction enum */
#define PCComms_UART_RX    (1u)
#define PCComms_UART_TX    (2u)
#define PCComms_UART_TX_RX (3u)

/* UART parity enum */
#define PCComms_UART_PARITY_EVEN   (0u)
#define PCComms_UART_PARITY_ODD    (1u)
#define PCComms_UART_PARITY_NONE   (2u)

/* UART stop bits enum */
#define PCComms_UART_STOP_BITS_1   (2u)
#define PCComms_UART_STOP_BITS_1_5 (3u)
#define PCComms_UART_STOP_BITS_2   (4u)

/* UART IrDA low power OVS enum */
#define PCComms_UART_IRDA_LP_OVS16     (16u)
#define PCComms_UART_IRDA_LP_OVS32     (32u)
#define PCComms_UART_IRDA_LP_OVS48     (48u)
#define PCComms_UART_IRDA_LP_OVS96     (96u)
#define PCComms_UART_IRDA_LP_OVS192    (192u)
#define PCComms_UART_IRDA_LP_OVS768    (768u)
#define PCComms_UART_IRDA_LP_OVS1536   (1536u)

/* Uart MP: mark (address) and space (data) bit definitions */
#define PCComms_UART_MP_MARK       (0x100u)
#define PCComms_UART_MP_SPACE      (0x000u)

/* UART CTS/RTS polarity settings */
#define PCComms_UART_CTS_ACTIVE_LOW    (0u)
#define PCComms_UART_CTS_ACTIVE_HIGH   (1u)
#define PCComms_UART_RTS_ACTIVE_LOW    (0u)
#define PCComms_UART_RTS_ACTIVE_HIGH   (1u)

/* Sources of RX errors */
#define PCComms_INTR_RX_ERR        (PCComms_INTR_RX_OVERFLOW    | \
                                             PCComms_INTR_RX_UNDERFLOW   | \
                                             PCComms_INTR_RX_FRAME_ERROR | \
                                             PCComms_INTR_RX_PARITY_ERROR)

/* Shifted INTR_RX_ERR defines ONLY for PCComms_UartGetByte() */
#define PCComms_UART_RX_OVERFLOW       (PCComms_INTR_RX_OVERFLOW << 8u)
#define PCComms_UART_RX_UNDERFLOW      (PCComms_INTR_RX_UNDERFLOW << 8u)
#define PCComms_UART_RX_FRAME_ERROR    (PCComms_INTR_RX_FRAME_ERROR << 8u)
#define PCComms_UART_RX_PARITY_ERROR   (PCComms_INTR_RX_PARITY_ERROR << 8u)
#define PCComms_UART_RX_ERROR_MASK     (PCComms_UART_RX_OVERFLOW    | \
                                                 PCComms_UART_RX_UNDERFLOW   | \
                                                 PCComms_UART_RX_FRAME_ERROR | \
                                                 PCComms_UART_RX_PARITY_ERROR)


/***************************************
*     Vars with External Linkage
***************************************/

#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    extern const PCComms_SPI_INIT_STRUCT  PCComms_configSpi;
    extern const PCComms_UART_INIT_STRUCT PCComms_configUart;
#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*    Specific SPI Macro Definitions
***************************************/

#define PCComms_GET_SPI_INTR_SLAVE_MASK(sourceMask)  ((sourceMask) & PCComms_INTR_SLAVE_SPI_BUS_ERROR)
#define PCComms_GET_SPI_INTR_MASTER_MASK(sourceMask) ((sourceMask) & PCComms_INTR_MASTER_SPI_DONE)
#define PCComms_GET_SPI_INTR_RX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~PCComms_INTR_SLAVE_SPI_BUS_ERROR)

#define PCComms_GET_SPI_INTR_TX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~PCComms_INTR_MASTER_SPI_DONE)


/***************************************
*    Specific UART Macro Definitions
***************************************/

#define PCComms_UART_GET_CTRL_OVS_IRDA_LP(oversample) \
        ((PCComms_UART_IRDA_LP_OVS16   == (oversample)) ? PCComms_CTRL_OVS_IRDA_LP_OVS16 : \
         ((PCComms_UART_IRDA_LP_OVS32   == (oversample)) ? PCComms_CTRL_OVS_IRDA_LP_OVS32 : \
          ((PCComms_UART_IRDA_LP_OVS48   == (oversample)) ? PCComms_CTRL_OVS_IRDA_LP_OVS48 : \
           ((PCComms_UART_IRDA_LP_OVS96   == (oversample)) ? PCComms_CTRL_OVS_IRDA_LP_OVS96 : \
            ((PCComms_UART_IRDA_LP_OVS192  == (oversample)) ? PCComms_CTRL_OVS_IRDA_LP_OVS192 : \
             ((PCComms_UART_IRDA_LP_OVS768  == (oversample)) ? PCComms_CTRL_OVS_IRDA_LP_OVS768 : \
              ((PCComms_UART_IRDA_LP_OVS1536 == (oversample)) ? PCComms_CTRL_OVS_IRDA_LP_OVS1536 : \
                                                                          PCComms_CTRL_OVS_IRDA_LP_OVS16)))))))

#define PCComms_GET_UART_RX_CTRL_ENABLED(direction) ((0u != (PCComms_UART_RX & (direction))) ? \
                                                                     (PCComms_RX_CTRL_ENABLED) : (0u))

#define PCComms_GET_UART_TX_CTRL_ENABLED(direction) ((0u != (PCComms_UART_TX & (direction))) ? \
                                                                     (PCComms_TX_CTRL_ENABLED) : (0u))


/***************************************
*        SPI Register Settings
***************************************/

#define PCComms_CTRL_SPI      (PCComms_CTRL_MODE_SPI)
#define PCComms_SPI_RX_CTRL   (PCComms_RX_CTRL_ENABLED)
#define PCComms_SPI_TX_CTRL   (PCComms_TX_CTRL_ENABLED)


/***************************************
*       SPI Init Register Settings
***************************************/

#define PCComms_SPI_SS_POLARITY \
             (((uint32) PCComms_SPI_SS0_POLARITY << PCComms_SPI_SLAVE_SELECT0) | \
              ((uint32) PCComms_SPI_SS1_POLARITY << PCComms_SPI_SLAVE_SELECT1) | \
              ((uint32) PCComms_SPI_SS2_POLARITY << PCComms_SPI_SLAVE_SELECT2) | \
              ((uint32) PCComms_SPI_SS3_POLARITY << PCComms_SPI_SLAVE_SELECT3))

#if(PCComms_SCB_MODE_SPI_CONST_CFG)

    /* SPI Configuration */
    #define PCComms_SPI_DEFAULT_CTRL \
                    (PCComms_GET_CTRL_OVS(PCComms_SPI_OVS_FACTOR) | \
                     PCComms_GET_CTRL_BYTE_MODE (PCComms_SPI_BYTE_MODE_ENABLE) | \
                     PCComms_GET_CTRL_EC_AM_MODE(PCComms_SPI_WAKE_ENABLE)      | \
                     PCComms_CTRL_SPI)

    #define PCComms_SPI_DEFAULT_SPI_CTRL \
                    (PCComms_GET_SPI_CTRL_CONTINUOUS    (PCComms_SPI_TRANSFER_SEPARATION)       | \
                     PCComms_GET_SPI_CTRL_SELECT_PRECEDE(PCComms_SPI_SUB_MODE &                   \
                                                                  PCComms_SPI_MODE_TI_PRECEDES_MASK)     | \
                     PCComms_GET_SPI_CTRL_SCLK_MODE     (PCComms_SPI_CLOCK_MODE)                | \
                     PCComms_GET_SPI_CTRL_LATE_MISO_SAMPLE(PCComms_SPI_LATE_MISO_SAMPLE_ENABLE) | \
                     PCComms_GET_SPI_CTRL_SCLK_CONTINUOUS(PCComms_SPI_FREE_RUN_SCLK_ENABLE)     | \
                     PCComms_GET_SPI_CTRL_SSEL_POLARITY (PCComms_SPI_SS_POLARITY)               | \
                     PCComms_GET_SPI_CTRL_SUB_MODE      (PCComms_SPI_SUB_MODE)                  | \
                     PCComms_GET_SPI_CTRL_MASTER_MODE   (PCComms_SPI_MODE))

    /* RX direction */
    #define PCComms_SPI_DEFAULT_RX_CTRL \
                    (PCComms_GET_RX_CTRL_DATA_WIDTH(PCComms_SPI_RX_DATA_BITS_NUM)     | \
                     PCComms_GET_RX_CTRL_BIT_ORDER (PCComms_SPI_BITS_ORDER)           | \
                     PCComms_GET_RX_CTRL_MEDIAN    (PCComms_SPI_MEDIAN_FILTER_ENABLE) | \
                     PCComms_SPI_RX_CTRL)

    #define PCComms_SPI_DEFAULT_RX_FIFO_CTRL \
                    PCComms_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(PCComms_SPI_RX_TRIGGER_LEVEL)

    /* TX direction */
    #define PCComms_SPI_DEFAULT_TX_CTRL \
                    (PCComms_GET_TX_CTRL_DATA_WIDTH(PCComms_SPI_TX_DATA_BITS_NUM) | \
                     PCComms_GET_TX_CTRL_BIT_ORDER (PCComms_SPI_BITS_ORDER)       | \
                     PCComms_SPI_TX_CTRL)

    #define PCComms_SPI_DEFAULT_TX_FIFO_CTRL \
                    PCComms_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(PCComms_SPI_TX_TRIGGER_LEVEL)

    /* Interrupt sources */
    #define PCComms_SPI_DEFAULT_INTR_SPI_EC_MASK   (PCComms_NO_INTR_SOURCES)

    #define PCComms_SPI_DEFAULT_INTR_I2C_EC_MASK   (PCComms_NO_INTR_SOURCES)
    #define PCComms_SPI_DEFAULT_INTR_SLAVE_MASK \
                    (PCComms_SPI_INTR_RX_MASK & PCComms_INTR_SLAVE_SPI_BUS_ERROR)

    #define PCComms_SPI_DEFAULT_INTR_MASTER_MASK \
                    (PCComms_SPI_INTR_TX_MASK & PCComms_INTR_MASTER_SPI_DONE)

    #define PCComms_SPI_DEFAULT_INTR_RX_MASK \
                    (PCComms_SPI_INTR_RX_MASK & (uint32) ~PCComms_INTR_SLAVE_SPI_BUS_ERROR)

    #define PCComms_SPI_DEFAULT_INTR_TX_MASK \
                    (PCComms_SPI_INTR_TX_MASK & (uint32) ~PCComms_INTR_MASTER_SPI_DONE)

#endif /* (PCComms_SCB_MODE_SPI_CONST_CFG) */


/***************************************
*        UART Register Settings
***************************************/

#define PCComms_CTRL_UART      (PCComms_CTRL_MODE_UART)
#define PCComms_UART_RX_CTRL   (PCComms_RX_CTRL_LSB_FIRST) /* LSB for UART goes first */
#define PCComms_UART_TX_CTRL   (PCComms_TX_CTRL_LSB_FIRST) /* LSB for UART goes first */


/***************************************
*      UART Init Register Settings
***************************************/

#if(PCComms_SCB_MODE_UART_CONST_CFG)

    /* UART configuration */
    #if(PCComms_UART_MODE_IRDA == PCComms_UART_SUB_MODE)

        #define PCComms_DEFAULT_CTRL_OVS   ((0u != PCComms_UART_IRDA_LOW_POWER) ?              \
                                (PCComms_UART_GET_CTRL_OVS_IRDA_LP(PCComms_UART_OVS_FACTOR)) : \
                                (PCComms_CTRL_OVS_IRDA_OVS16))

    #else

        #define PCComms_DEFAULT_CTRL_OVS   PCComms_GET_CTRL_OVS(PCComms_UART_OVS_FACTOR)

    #endif /* (PCComms_UART_MODE_IRDA == PCComms_UART_SUB_MODE) */

    #define PCComms_UART_DEFAULT_CTRL \
                                (PCComms_GET_CTRL_BYTE_MODE  (PCComms_UART_BYTE_MODE_ENABLE)  | \
                                 PCComms_GET_CTRL_ADDR_ACCEPT(PCComms_UART_MP_ACCEPT_ADDRESS) | \
                                 PCComms_DEFAULT_CTRL_OVS                                              | \
                                 PCComms_CTRL_UART)

    #define PCComms_UART_DEFAULT_UART_CTRL \
                                    (PCComms_GET_UART_CTRL_MODE(PCComms_UART_SUB_MODE))

    /* RX direction */
    #define PCComms_UART_DEFAULT_RX_CTRL_PARITY \
                                ((PCComms_UART_PARITY_NONE != PCComms_UART_PARITY_TYPE) ?      \
                                  (PCComms_GET_UART_RX_CTRL_PARITY(PCComms_UART_PARITY_TYPE) | \
                                   PCComms_UART_RX_CTRL_PARITY_ENABLED) : (0u))

    #define PCComms_UART_DEFAULT_UART_RX_CTRL \
                    (PCComms_GET_UART_RX_CTRL_MODE(PCComms_UART_STOP_BITS_NUM)                    | \
                     PCComms_GET_UART_RX_CTRL_POLARITY(PCComms_UART_IRDA_POLARITY)                | \
                     PCComms_GET_UART_RX_CTRL_MP_MODE(PCComms_UART_MP_MODE_ENABLE)                | \
                     PCComms_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(PCComms_UART_DROP_ON_PARITY_ERR) | \
                     PCComms_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(PCComms_UART_DROP_ON_FRAME_ERR)   | \
                     PCComms_UART_DEFAULT_RX_CTRL_PARITY)

    #define PCComms_UART_DEFAULT_RX_CTRL \
                                (PCComms_GET_RX_CTRL_DATA_WIDTH(PCComms_UART_DATA_BITS_NUM)        | \
                                 PCComms_GET_RX_CTRL_MEDIAN    (PCComms_UART_MEDIAN_FILTER_ENABLE) | \
                                 PCComms_GET_UART_RX_CTRL_ENABLED(PCComms_UART_DIRECTION))

    #define PCComms_UART_DEFAULT_RX_FIFO_CTRL \
                                PCComms_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(PCComms_UART_RX_TRIGGER_LEVEL)

    #define PCComms_UART_DEFAULT_RX_MATCH_REG  ((0u != PCComms_UART_MP_MODE_ENABLE) ?          \
                                (PCComms_GET_RX_MATCH_ADDR(PCComms_UART_MP_RX_ADDRESS) | \
                                 PCComms_GET_RX_MATCH_MASK(PCComms_UART_MP_RX_ADDRESS_MASK)) : (0u))

    /* TX direction */
    #define PCComms_UART_DEFAULT_TX_CTRL_PARITY (PCComms_UART_DEFAULT_RX_CTRL_PARITY)

    #define PCComms_UART_DEFAULT_UART_TX_CTRL \
                                (PCComms_GET_UART_TX_CTRL_MODE(PCComms_UART_STOP_BITS_NUM)       | \
                                 PCComms_GET_UART_TX_CTRL_RETRY_NACK(PCComms_UART_RETRY_ON_NACK) | \
                                 PCComms_UART_DEFAULT_TX_CTRL_PARITY)

    #define PCComms_UART_DEFAULT_TX_CTRL \
                                (PCComms_GET_TX_CTRL_DATA_WIDTH(PCComms_UART_DATA_BITS_NUM) | \
                                 PCComms_GET_UART_TX_CTRL_ENABLED(PCComms_UART_DIRECTION))

    #define PCComms_UART_DEFAULT_TX_FIFO_CTRL \
                                PCComms_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(PCComms_UART_TX_TRIGGER_LEVEL)

    #define PCComms_UART_DEFAULT_FLOW_CTRL \
                        (PCComms_GET_UART_FLOW_CTRL_TRIGGER_LEVEL(PCComms_UART_RTS_FIFO_LEVEL) | \
                         PCComms_GET_UART_FLOW_CTRL_RTS_POLARITY (PCComms_UART_RTS_POLARITY)   | \
                         PCComms_GET_UART_FLOW_CTRL_CTS_POLARITY (PCComms_UART_CTS_POLARITY)   | \
                         PCComms_GET_UART_FLOW_CTRL_CTS_ENABLE   (PCComms_UART_CTS_ENABLE))

    /* Interrupt sources */
    #define PCComms_UART_DEFAULT_INTR_I2C_EC_MASK  (PCComms_NO_INTR_SOURCES)
    #define PCComms_UART_DEFAULT_INTR_SPI_EC_MASK  (PCComms_NO_INTR_SOURCES)
    #define PCComms_UART_DEFAULT_INTR_SLAVE_MASK   (PCComms_NO_INTR_SOURCES)
    #define PCComms_UART_DEFAULT_INTR_MASTER_MASK  (PCComms_NO_INTR_SOURCES)
    #define PCComms_UART_DEFAULT_INTR_RX_MASK      (PCComms_UART_INTR_RX_MASK)
    #define PCComms_UART_DEFAULT_INTR_TX_MASK      (PCComms_UART_INTR_TX_MASK)

#endif /* (PCComms_SCB_MODE_UART_CONST_CFG) */


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

#define PCComms_SPIM_ACTIVE_SS0    (PCComms_SPI_SLAVE_SELECT0)
#define PCComms_SPIM_ACTIVE_SS1    (PCComms_SPI_SLAVE_SELECT1)
#define PCComms_SPIM_ACTIVE_SS2    (PCComms_SPI_SLAVE_SELECT2)
#define PCComms_SPIM_ACTIVE_SS3    (PCComms_SPI_SLAVE_SELECT3)

#endif /* CY_SCB_SPI_UART_PCComms_H */


/* [] END OF FILE */
