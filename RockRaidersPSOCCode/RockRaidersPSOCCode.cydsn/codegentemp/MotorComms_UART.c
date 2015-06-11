/*******************************************************************************
* File Name: MotorComms_UART.c
* Version 2.0
*
* Description:
*  This file provides the source code to the API for the SCB Component in
*  UART mode.
*
* Note:
*
*******************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "MotorComms_PVT.h"
#include "MotorComms_SPI_UART_PVT.h"


#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Configuration Structure Initialization
    ***************************************/

    const MotorComms_UART_INIT_STRUCT MotorComms_configUart =
    {
        MotorComms_UART_SUB_MODE,
        MotorComms_UART_DIRECTION,
        MotorComms_UART_DATA_BITS_NUM,
        MotorComms_UART_PARITY_TYPE,
        MotorComms_UART_STOP_BITS_NUM,
        MotorComms_UART_OVS_FACTOR,
        MotorComms_UART_IRDA_LOW_POWER,
        MotorComms_UART_MEDIAN_FILTER_ENABLE,
        MotorComms_UART_RETRY_ON_NACK,
        MotorComms_UART_IRDA_POLARITY,
        MotorComms_UART_DROP_ON_PARITY_ERR,
        MotorComms_UART_DROP_ON_FRAME_ERR,
        MotorComms_UART_WAKE_ENABLE,
        0u,
        NULL,
        0u,
        NULL,
        MotorComms_UART_MP_MODE_ENABLE,
        MotorComms_UART_MP_ACCEPT_ADDRESS,
        MotorComms_UART_MP_RX_ADDRESS,
        MotorComms_UART_MP_RX_ADDRESS_MASK,
        (uint32) MotorComms_SCB_IRQ_INTERNAL,
        MotorComms_UART_INTR_RX_MASK,
        MotorComms_UART_RX_TRIGGER_LEVEL,
        MotorComms_UART_INTR_TX_MASK,
        MotorComms_UART_TX_TRIGGER_LEVEL,
        (uint8) MotorComms_UART_BYTE_MODE_ENABLE,
        (uint8) MotorComms_UART_CTS_ENABLE,
        (uint8) MotorComms_UART_CTS_POLARITY,
        (uint8) MotorComms_UART_RTS_POLARITY,
        (uint8) MotorComms_UART_RTS_FIFO_LEVEL
    };


    /*******************************************************************************
    * Function Name: MotorComms_UartInit
    ********************************************************************************
    *
    * Summary:
    *  Configures the SCB for the UART operation.
    *
    * Parameters:
    *  config:  Pointer to a structure that contains the following ordered list of
    *           fields. These fields match the selections available in the
    *           customizer.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MotorComms_UartInit(const MotorComms_UART_INIT_STRUCT *config)
    {
        uint32 pinsConfig;

        if(NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due to bad function parameter */
        }
        else
        {
            /* Get direction to configure UART pins: TX, RX or TX+RX */
            pinsConfig  = config->direction;

        #if !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
            /* Add RTS and CTS pins to configure */
            pinsConfig |= (0u != config->rtsRxFifoLevel) ? (MotorComms_UART_RTS_PIN_ENABLE) : (0u);
            pinsConfig |= (0u != config->enableCts)         ? (MotorComms_UART_CTS_PIN_ENABLE) : (0u);
        #endif /* !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */

            /* Configure pins */
            MotorComms_SetPins(MotorComms_SCB_MODE_UART, config->mode, pinsConfig);

            /* Store internal configuration */
            MotorComms_scbMode       = (uint8) MotorComms_SCB_MODE_UART;
            MotorComms_scbEnableWake = (uint8) config->enableWake;
            MotorComms_scbEnableIntr = (uint8) config->enableInterrupt;

            /* Set RX direction internal variables */
            MotorComms_rxBuffer      =         config->rxBuffer;
            MotorComms_rxDataBits    = (uint8) config->dataBits;
            MotorComms_rxBufferSize  = (uint8) config->rxBufferSize;

            /* Set TX direction internal variables */
            MotorComms_txBuffer      =         config->txBuffer;
            MotorComms_txDataBits    = (uint8) config->dataBits;
            MotorComms_txBufferSize  = (uint8) config->txBufferSize;

            /* Configure UART interface */
            if(MotorComms_UART_MODE_IRDA == config->mode)
            {
                /* OVS settings: IrDA */
                MotorComms_CTRL_REG  = ((0u != config->enableIrdaLowPower) ?
                                                (MotorComms_UART_GET_CTRL_OVS_IRDA_LP(config->oversample)) :
                                                (MotorComms_CTRL_OVS_IRDA_OVS16));
            }
            else
            {
                /* OVS settings: UART and SmartCard */
                MotorComms_CTRL_REG  = MotorComms_GET_CTRL_OVS(config->oversample);
            }

            MotorComms_CTRL_REG     |= MotorComms_GET_CTRL_BYTE_MODE  (config->enableByteMode)      |
                                             MotorComms_GET_CTRL_ADDR_ACCEPT(config->multiprocAcceptAddr) |
                                             MotorComms_CTRL_UART;

            /* Configure sub-mode: UART, SmartCard or IrDA */
            MotorComms_UART_CTRL_REG = MotorComms_GET_UART_CTRL_MODE(config->mode);

            /* Configure RX direction */
            MotorComms_UART_RX_CTRL_REG = MotorComms_GET_UART_RX_CTRL_MODE(config->stopBits)              |
                                        MotorComms_GET_UART_RX_CTRL_POLARITY(config->enableInvertedRx)          |
                                        MotorComms_GET_UART_RX_CTRL_MP_MODE(config->enableMultiproc)            |
                                        MotorComms_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(config->dropOnParityErr) |
                                        MotorComms_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(config->dropOnFrameErr);

            if(MotorComms_UART_PARITY_NONE != config->parity)
            {
               MotorComms_UART_RX_CTRL_REG |= MotorComms_GET_UART_RX_CTRL_PARITY(config->parity) |
                                                    MotorComms_UART_RX_CTRL_PARITY_ENABLED;
            }

            MotorComms_RX_CTRL_REG      = MotorComms_GET_RX_CTRL_DATA_WIDTH(config->dataBits)       |
                                                MotorComms_GET_RX_CTRL_MEDIAN(config->enableMedianFilter) |
                                                MotorComms_GET_UART_RX_CTRL_ENABLED(config->direction);

            MotorComms_RX_FIFO_CTRL_REG = MotorComms_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(config->rxTriggerLevel);

            /* Configure MP address */
            MotorComms_RX_MATCH_REG     = MotorComms_GET_RX_MATCH_ADDR(config->multiprocAddr) |
                                                MotorComms_GET_RX_MATCH_MASK(config->multiprocAddrMask);

            /* Configure RX direction */
            MotorComms_UART_TX_CTRL_REG = MotorComms_GET_UART_TX_CTRL_MODE(config->stopBits) |
                                                MotorComms_GET_UART_TX_CTRL_RETRY_NACK(config->enableRetryNack);

            if(MotorComms_UART_PARITY_NONE != config->parity)
            {
               MotorComms_UART_TX_CTRL_REG |= MotorComms_GET_UART_TX_CTRL_PARITY(config->parity) |
                                                    MotorComms_UART_TX_CTRL_PARITY_ENABLED;
            }

            MotorComms_TX_CTRL_REG      = MotorComms_GET_TX_CTRL_DATA_WIDTH(config->dataBits)    |
                                                MotorComms_GET_UART_TX_CTRL_ENABLED(config->direction);

            MotorComms_TX_FIFO_CTRL_REG = MotorComms_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(config->txTriggerLevel);

        #if !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
            MotorComms_UART_FLOW_CTRL_REG = MotorComms_GET_UART_FLOW_CTRL_CTS_ENABLE(config->enableCts) | \
                                            MotorComms_GET_UART_FLOW_CTRL_CTS_POLARITY (config->ctsPolarity)  | \
                                            MotorComms_GET_UART_FLOW_CTRL_RTS_POLARITY(config->rtsPolarity)   | \
                                            MotorComms_GET_UART_FLOW_CTRL_TRIGGER_LEVEL(config->rtsRxFifoLevel);
        #endif /* !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */

            /* Configure interrupt with UART handler but do not enable it */
            CyIntDisable    (MotorComms_ISR_NUMBER);
            CyIntSetPriority(MotorComms_ISR_NUMBER, MotorComms_ISR_PRIORITY);
            (void) CyIntSetVector(MotorComms_ISR_NUMBER, &MotorComms_SPI_UART_ISR);

            /* Configure WAKE interrupt */
        #if(MotorComms_UART_RX_WAKEUP_IRQ)
            CyIntDisable    (MotorComms_RX_WAKE_ISR_NUMBER);
            CyIntSetPriority(MotorComms_RX_WAKE_ISR_NUMBER, MotorComms_RX_WAKE_ISR_PRIORITY);
            (void) CyIntSetVector(MotorComms_RX_WAKE_ISR_NUMBER, &MotorComms_UART_WAKEUP_ISR);
        #endif /* (MotorComms_UART_RX_WAKEUP_IRQ) */

            /* Configure interrupt sources */
            MotorComms_INTR_I2C_EC_MASK_REG = MotorComms_NO_INTR_SOURCES;
            MotorComms_INTR_SPI_EC_MASK_REG = MotorComms_NO_INTR_SOURCES;
            MotorComms_INTR_SLAVE_MASK_REG  = MotorComms_NO_INTR_SOURCES;
            MotorComms_INTR_MASTER_MASK_REG = MotorComms_NO_INTR_SOURCES;
            MotorComms_INTR_RX_MASK_REG     = config->rxInterruptMask;
            MotorComms_INTR_TX_MASK_REG     = config->txInterruptMask;

            /* Clear RX buffer indexes */
            MotorComms_rxBufferHead     = 0u;
            MotorComms_rxBufferTail     = 0u;
            MotorComms_rxBufferOverflow = 0u;

            /* Clear TX buffer indexes */
            MotorComms_txBufferHead = 0u;
            MotorComms_txBufferTail = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: MotorComms_UartInit
    ********************************************************************************
    *
    * Summary:
    *  Configures the SCB for the UART operation.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MotorComms_UartInit(void)
    {
        /* Configure UART interface */
        MotorComms_CTRL_REG = MotorComms_UART_DEFAULT_CTRL;

        /* Configure sub-mode: UART, SmartCard or IrDA */
        MotorComms_UART_CTRL_REG = MotorComms_UART_DEFAULT_UART_CTRL;

        /* Configure RX direction */
        MotorComms_UART_RX_CTRL_REG = MotorComms_UART_DEFAULT_UART_RX_CTRL;
        MotorComms_RX_CTRL_REG      = MotorComms_UART_DEFAULT_RX_CTRL;
        MotorComms_RX_FIFO_CTRL_REG = MotorComms_UART_DEFAULT_RX_FIFO_CTRL;
        MotorComms_RX_MATCH_REG     = MotorComms_UART_DEFAULT_RX_MATCH_REG;

        /* Configure TX direction */
        MotorComms_UART_TX_CTRL_REG = MotorComms_UART_DEFAULT_UART_TX_CTRL;
        MotorComms_TX_CTRL_REG      = MotorComms_UART_DEFAULT_TX_CTRL;
        MotorComms_TX_FIFO_CTRL_REG = MotorComms_UART_DEFAULT_TX_FIFO_CTRL;

    #if !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
        MotorComms_UART_FLOW_CTRL_REG = MotorComms_UART_DEFAULT_FLOW_CTRL;
    #endif /* !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */

        /* Configure interrupt with UART handler but do not enable it */
    #if(MotorComms_SCB_IRQ_INTERNAL)
        CyIntDisable    (MotorComms_ISR_NUMBER);
        CyIntSetPriority(MotorComms_ISR_NUMBER, MotorComms_ISR_PRIORITY);
        (void) CyIntSetVector(MotorComms_ISR_NUMBER, &MotorComms_SPI_UART_ISR);
    #endif /* (MotorComms_SCB_IRQ_INTERNAL) */

        /* Configure WAKE interrupt */
    #if(MotorComms_UART_RX_WAKEUP_IRQ)
        CyIntDisable    (MotorComms_RX_WAKE_ISR_NUMBER);
        CyIntSetPriority(MotorComms_RX_WAKE_ISR_NUMBER, MotorComms_RX_WAKE_ISR_PRIORITY);
        (void) CyIntSetVector(MotorComms_RX_WAKE_ISR_NUMBER, &MotorComms_UART_WAKEUP_ISR);
    #endif /* (MotorComms_UART_RX_WAKEUP_IRQ) */

        /* Configure interrupt sources */
        MotorComms_INTR_I2C_EC_MASK_REG = MotorComms_UART_DEFAULT_INTR_I2C_EC_MASK;
        MotorComms_INTR_SPI_EC_MASK_REG = MotorComms_UART_DEFAULT_INTR_SPI_EC_MASK;
        MotorComms_INTR_SLAVE_MASK_REG  = MotorComms_UART_DEFAULT_INTR_SLAVE_MASK;
        MotorComms_INTR_MASTER_MASK_REG = MotorComms_UART_DEFAULT_INTR_MASTER_MASK;
        MotorComms_INTR_RX_MASK_REG     = MotorComms_UART_DEFAULT_INTR_RX_MASK;
        MotorComms_INTR_TX_MASK_REG     = MotorComms_UART_DEFAULT_INTR_TX_MASK;

    #if(MotorComms_INTERNAL_RX_SW_BUFFER_CONST)
        MotorComms_rxBufferHead     = 0u;
        MotorComms_rxBufferTail     = 0u;
        MotorComms_rxBufferOverflow = 0u;
    #endif /* (MotorComms_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(MotorComms_INTERNAL_TX_SW_BUFFER_CONST)
        MotorComms_txBufferHead = 0u;
        MotorComms_txBufferTail = 0u;
    #endif /* (MotorComms_INTERNAL_TX_SW_BUFFER_CONST) */
    }
#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: MotorComms_UartSetRxAddress
********************************************************************************
*
* Summary:
*  Sets the hardware detectable receiver address for the UART in the
*  Multiprocessor mode.
*
* Parameters:
*  address: Address for hardware address detection.
*
* Return:
*  None
*
*******************************************************************************/
void MotorComms_UartSetRxAddress(uint32 address)
{
     uint32 matchReg;

    matchReg = MotorComms_RX_MATCH_REG;

    matchReg &= ((uint32) ~MotorComms_RX_MATCH_ADDR_MASK); /* Clear address bits */
    matchReg |= ((uint32)  (address & MotorComms_RX_MATCH_ADDR_MASK)); /* Set address  */

    MotorComms_RX_MATCH_REG = matchReg;
}


/*******************************************************************************
* Function Name: MotorComms_UartSetRxAddressMask
********************************************************************************
*
* Summary:
*  Sets the hardware address mask for the UART in the Multiprocessor mode.
*
* Parameters:
*  addressMask: Address mask.
*   0 - address bit does not care while comparison.
*   1 - address bit is significant while comparison.
*
* Return:
*  None
*
*******************************************************************************/
void MotorComms_UartSetRxAddressMask(uint32 addressMask)
{
    uint32 matchReg;

    matchReg = MotorComms_RX_MATCH_REG;

    matchReg &= ((uint32) ~MotorComms_RX_MATCH_MASK_MASK); /* Clear address mask bits */
    matchReg |= ((uint32) (addressMask << MotorComms_RX_MATCH_MASK_POS));

    MotorComms_RX_MATCH_REG = matchReg;
}


#if(MotorComms_UART_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: MotorComms_UartGetChar
    ********************************************************************************
    *
    * Summary:
    *  Retrieves the next data element from the receive buffer.
    *  This function is designed for ASCII characters and returns a char
    *  where 1 to 255 are valid characters and 0 indicates an error occurred or
    *  no data present.
    *  - The RX software buffer is disabled: returns the data element
    *    retrieved from the RX FIFO.
    *    Undefined data will be returned if the RX FIFO is empty.
    *  - The RX software buffer is enabled: returns the data element from
    *    the software receive buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  The next data element from the receive buffer.
    *  ASCII character values from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Side Effects:
    *  The errors bits may not correspond with reading characters due to RX FIFO
    *  and software buffer usage.
    *  RX software buffer is enabled: The internal software buffer overflow
    *  does not treat as an error condition.
    *  Check SCB_rxBufferOverflow to capture that error condition.
    *
    *******************************************************************************/
    uint32 MotorComms_UartGetChar(void)
    {
        uint32 rxData = 0u;

        /* Reads data only if there is data to read */
        if(0u != MotorComms_SpiUartGetRxBufferSize())
        {
            rxData = MotorComms_SpiUartReadRxData();
        }

        if(MotorComms_CHECK_INTR_RX(MotorComms_INTR_RX_ERR))
        {
            rxData = 0u; /* Error occurred: returns zero */
            MotorComms_ClearRxInterruptSource(MotorComms_INTR_RX_ERR);
        }

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: MotorComms_UartGetByte
    ********************************************************************************
    *
    * Summary:
    *  Retrieves the next data element from the receive buffer, returns the
    *  received byte and error condition.
    *   - The RX software buffer is disabled: returns the data element retrieved
    *     from the RX FIFO. Undefined data will be returned if the RX FIFO is
    *     empty.
    *   - The RX software buffer is enabled: returns data element from the
    *     software receive buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Bits 7-0 contain the next data element from the receive buffer and
    *  other bits contain the error condition.
    *
    * Side Effects:
    *  The errors bits may not correspond with reading characters due to RX FIFO
    *  and software buffer usage.
    *  RX software buffer is disabled: The internal software buffer overflow
    *  is not returned as status by this function.
    *  Check SCB_rxBufferOverflow to capture that error condition.
    *
    *******************************************************************************/
    uint32 MotorComms_UartGetByte(void)
    {
        uint32 rxData;
        uint32 tmpStatus;
        uint32 intSourceMask;

        intSourceMask = MotorComms_SpiUartDisableIntRx();

        if(0u != MotorComms_SpiUartGetRxBufferSize())
        {
            /* Enables interrupt to receive more bytes: at least one byte is in
            * buffer.
            */
            MotorComms_SpiUartEnableIntRx(intSourceMask);

            /* Get received byte */
            rxData = MotorComms_SpiUartReadRxData();
        }
        else
        {
            /* Reads a byte directly from RX FIFO: underflow is raised in the case
            * of empty. Otherwise the first received byte will be read.
            */
            rxData = MotorComms_RX_FIFO_RD_REG;

            /* Enables interrupt to receive more bytes.
            * The RX_NOT_EMPTY interrupt is cleared by the interrupt routine
            * in case the byte was received and read by code above.
            */
            MotorComms_SpiUartEnableIntRx(intSourceMask);
        }

        /* Get and clear RX error mask */
        tmpStatus = (MotorComms_GetRxInterruptSource() & MotorComms_INTR_RX_ERR);
        MotorComms_ClearRxInterruptSource(MotorComms_INTR_RX_ERR);

        /* Puts together data and error status:
        * MP mode and accept address: 9th bit is set to notify mark.
        */
        rxData |= ((uint32) (tmpStatus << 8u));

        return(rxData);
    }


    #if !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: MotorComms_UartSetRtsPolarity
        ********************************************************************************
        *
        * Summary:
        *  Sets active polarity of RTS output signal.
        *
        * Parameters:
        *  polarity: Active polarity of RTS output signal.
        *   MotorComms_UART_RTS_ACTIVE_LOW  - RTS signal is active low.
        *   MotorComms_UART_RTS_ACTIVE_HIGH - RTS signal is active high.
        *
        * Return:
        *  None
        *
        *******************************************************************************/
        void MotorComms_UartSetRtsPolarity(uint32 polarity)
        {
            if(0u != polarity)
            {
                MotorComms_UART_FLOW_CTRL_REG |= (uint32)  MotorComms_UART_FLOW_CTRL_RTS_POLARITY;
            }
            else
            {
                MotorComms_UART_FLOW_CTRL_REG &= (uint32) ~MotorComms_UART_FLOW_CTRL_RTS_POLARITY;
            }
        }


        /*******************************************************************************
        * Function Name: MotorComms_UartSetRtsFifoLevel
        ********************************************************************************
        *
        * Summary:
        *  Sets level in the RX FIFO for RTS signal activation.
        *  While the RX FIFO has fewer entries than the RX FIFO level the RTS signal
        *  remains active, otherwise the RTS signal becomes inactive.
        *
        * Parameters:
        *  level: Level in the RX FIFO for RTS signal activation.
        *         The range of valid level values is between 0 and RX FIFO depth - 1.
        *         Setting level value to 0 disables RTS signal activation.
        *
        * Return:
        *  None
        *
        *******************************************************************************/
        void MotorComms_UartSetRtsFifoLevel(uint32 level)
        {
            uint32 uartFlowCtrl;

            uartFlowCtrl = MotorComms_UART_FLOW_CTRL_REG;

            uartFlowCtrl &= ((uint32) ~MotorComms_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
            uartFlowCtrl |= ((uint32) (MotorComms_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK & level));

            MotorComms_UART_FLOW_CTRL_REG = uartFlowCtrl;
        }
    #endif /* !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */

#endif /* (MotorComms_UART_RX_DIRECTION) */


#if(MotorComms_UART_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: MotorComms_UartPutString
    ********************************************************************************
    *
    * Summary:
    *  Places a NULL terminated string in the transmit buffer to be sent at the
    *  next available bus time.
    *  This function is blocking and waits until there is space available to put
    *  all the requested data into the  transmit buffer.
    *
    * Parameters:
    *  string: pointer to the null terminated string array to be placed in the
    *          transmit buffer.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MotorComms_UartPutString(const char8 string[])
    {
        uint32 bufIndex;

        bufIndex = 0u;

        /* Blocks the control flow until all data has been sent */
        while(string[bufIndex] != ((char8) 0))
        {
            MotorComms_UartPutChar((uint32) string[bufIndex]);
            bufIndex++;
        }
    }


    /*******************************************************************************
    * Function Name: MotorComms_UartPutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Places a byte of data followed by a carriage return (0x0D) and
    *  line feed (0x0A) into the transmit buffer.
    *  This function is blocking and waits until there is space available to put
    *  all the requested data into the  transmit buffer.
    *
    * Parameters:
    *  txDataByte : the data to be transmitted.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MotorComms_UartPutCRLF(uint32 txDataByte)
    {
        MotorComms_UartPutChar(txDataByte);  /* Blocks control flow until all data has been sent */
        MotorComms_UartPutChar(0x0Du);       /* Blocks control flow until all data has been sent */
        MotorComms_UartPutChar(0x0Au);       /* Blocks control flow until all data has been sent */
    }


    #if !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: MotorCommsSCB_UartEnableCts
        ********************************************************************************
        *
        * Summary:
        *  Enables usage of CTS input signal by the UART transmitter.
        *
        * Parameters:
        *  None
        *
        * Return:
        *  None
        *
        *******************************************************************************/
        void MotorComms_UartEnableCts(void)
        {
            MotorComms_UART_FLOW_CTRL_REG |= (uint32)  MotorComms_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: MotorComms_UartDisableCts
        ********************************************************************************
        *
        * Summary:
        *  Disables usage of CTS input signal by the UART transmitter.
        *
        * Parameters:
        *  None
        *
        * Return:
        *  None
        *
        *******************************************************************************/
        void MotorComms_UartDisableCts(void)
        {
            MotorComms_UART_FLOW_CTRL_REG &= (uint32) ~MotorComms_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: MotorComms_UartSetCtsPolarity
        ********************************************************************************
        *
        * Summary:
        *  Sets active polarity of CTS input signal.
        *
        * Parameters:
        *  polarity: Active polarity of CTS output signal.
        *   MotorComms_UART_CTS_ACTIVE_LOW  - CTS signal is active low.
        *   MotorComms_UART_CTS_ACTIVE_HIGH - CTS signal is active high.
        *
        * Return:
        *  None
        *
        *******************************************************************************/
        void MotorComms_UartSetCtsPolarity(uint32 polarity)
        {
            if (0u != polarity)
            {
                MotorComms_UART_FLOW_CTRL_REG |= (uint32)  MotorComms_UART_FLOW_CTRL_CTS_POLARITY;
            }
            else
            {
                MotorComms_UART_FLOW_CTRL_REG &= (uint32) ~MotorComms_UART_FLOW_CTRL_CTS_POLARITY;
            }
        }
    #endif /* !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */

#endif /* (MotorComms_UART_TX_DIRECTION) */


#if(MotorComms_UART_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: MotorComms_UartSaveConfig
    ********************************************************************************
    *
    * Summary:
    *  Clears and enables interrupt on a falling edge of the Rx input. The GPIO
    *  event wakes up the device and SKIP_START feature allows the UART continue
    *  receiving data bytes properly. The GPIO interrupt does not track in the
    *  active mode therefore requires to be cleared by this API.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MotorComms_UartSaveConfig(void)
    {
        /* Clear interrupt activity:
        *  - set skip start and disable RX. At GPIO wakeup RX will be enabled.
        *  - clear rx_wake interrupt source as it triggers during normal operation.
        *  - clear wake interrupt pending state as it becomes pending in active mode.
        */

        MotorComms_UART_RX_CTRL_REG |= MotorComms_UART_RX_CTRL_SKIP_START;

    #if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
        #if(MotorComms_MOSI_SCL_RX_WAKE_PIN)
            (void) MotorComms_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
        #endif /* (MotorComms_MOSI_SCL_RX_WAKE_PIN) */
    #else
        #if(MotorComms_UART_RX_WAKE_PIN)
            (void) MotorComms_rx_wake_ClearInterrupt();
        #endif /* (MotorComms_UART_RX_WAKE_PIN) */
    #endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */

    #if(MotorComms_UART_RX_WAKEUP_IRQ)
        MotorComms_RxWakeClearPendingInt();
        MotorComms_RxWakeEnableInt();
    #endif /* (MotorComms_UART_RX_WAKEUP_IRQ) */
    }


    /*******************************************************************************
    * Function Name: MotorComms_UartRestoreConfig
    ********************************************************************************
    *
    * Summary:
    *  Disables the RX GPIO interrupt. Until this function is called the interrupt
    *  remains active and triggers on every falling edge of the UART RX line.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MotorComms_UartRestoreConfig(void)
    {
    /* Disable RX GPIO interrupt: no more triggers in active mode */
    #if(MotorComms_UART_RX_WAKEUP_IRQ)
        MotorComms_RxWakeDisableInt();
    #endif /* (MotorComms_UART_RX_WAKEUP_IRQ) */
    }
#endif /* (MotorComms_UART_WAKE_ENABLE_CONST) */


#if(MotorComms_UART_RX_WAKEUP_IRQ)
    /*******************************************************************************
    * Function Name: MotorComms_UART_WAKEUP_ISR
    ********************************************************************************
    *
    * Summary:
    *  Handles the Interrupt Service Routine for the SCB UART mode GPIO wakeup
    *  event. This event is configured to trigger on a falling edge of the RX line.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    CY_ISR(MotorComms_UART_WAKEUP_ISR)
    {
        /* Clear interrupt source: the event becomes multi triggered and is
        * only disabled by MotorComms_UartRestoreConfig() call.
        */
    #if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
        #if(MotorComms_MOSI_SCL_RX_WAKE_PIN)
            (void) MotorComms_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
        #endif /* (MotorComms_MOSI_SCL_RX_WAKE_PIN) */
    #else
        #if(MotorComms_UART_RX_WAKE_PIN)
            (void) MotorComms_rx_wake_ClearInterrupt();
        #endif /* (MotorComms_UART_RX_WAKE_PIN) */
    #endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */
    }
#endif /* (MotorComms_UART_RX_WAKEUP_IRQ) */


/* [] END OF FILE */
