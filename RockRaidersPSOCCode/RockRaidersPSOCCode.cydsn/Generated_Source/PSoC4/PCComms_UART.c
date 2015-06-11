/*******************************************************************************
* File Name: PCComms_UART.c
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

#include "PCComms_PVT.h"
#include "PCComms_SPI_UART_PVT.h"


#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Configuration Structure Initialization
    ***************************************/

    const PCComms_UART_INIT_STRUCT PCComms_configUart =
    {
        PCComms_UART_SUB_MODE,
        PCComms_UART_DIRECTION,
        PCComms_UART_DATA_BITS_NUM,
        PCComms_UART_PARITY_TYPE,
        PCComms_UART_STOP_BITS_NUM,
        PCComms_UART_OVS_FACTOR,
        PCComms_UART_IRDA_LOW_POWER,
        PCComms_UART_MEDIAN_FILTER_ENABLE,
        PCComms_UART_RETRY_ON_NACK,
        PCComms_UART_IRDA_POLARITY,
        PCComms_UART_DROP_ON_PARITY_ERR,
        PCComms_UART_DROP_ON_FRAME_ERR,
        PCComms_UART_WAKE_ENABLE,
        0u,
        NULL,
        0u,
        NULL,
        PCComms_UART_MP_MODE_ENABLE,
        PCComms_UART_MP_ACCEPT_ADDRESS,
        PCComms_UART_MP_RX_ADDRESS,
        PCComms_UART_MP_RX_ADDRESS_MASK,
        (uint32) PCComms_SCB_IRQ_INTERNAL,
        PCComms_UART_INTR_RX_MASK,
        PCComms_UART_RX_TRIGGER_LEVEL,
        PCComms_UART_INTR_TX_MASK,
        PCComms_UART_TX_TRIGGER_LEVEL,
        (uint8) PCComms_UART_BYTE_MODE_ENABLE,
        (uint8) PCComms_UART_CTS_ENABLE,
        (uint8) PCComms_UART_CTS_POLARITY,
        (uint8) PCComms_UART_RTS_POLARITY,
        (uint8) PCComms_UART_RTS_FIFO_LEVEL
    };


    /*******************************************************************************
    * Function Name: PCComms_UartInit
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
    void PCComms_UartInit(const PCComms_UART_INIT_STRUCT *config)
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

        #if !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
            /* Add RTS and CTS pins to configure */
            pinsConfig |= (0u != config->rtsRxFifoLevel) ? (PCComms_UART_RTS_PIN_ENABLE) : (0u);
            pinsConfig |= (0u != config->enableCts)         ? (PCComms_UART_CTS_PIN_ENABLE) : (0u);
        #endif /* !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */

            /* Configure pins */
            PCComms_SetPins(PCComms_SCB_MODE_UART, config->mode, pinsConfig);

            /* Store internal configuration */
            PCComms_scbMode       = (uint8) PCComms_SCB_MODE_UART;
            PCComms_scbEnableWake = (uint8) config->enableWake;
            PCComms_scbEnableIntr = (uint8) config->enableInterrupt;

            /* Set RX direction internal variables */
            PCComms_rxBuffer      =         config->rxBuffer;
            PCComms_rxDataBits    = (uint8) config->dataBits;
            PCComms_rxBufferSize  = (uint8) config->rxBufferSize;

            /* Set TX direction internal variables */
            PCComms_txBuffer      =         config->txBuffer;
            PCComms_txDataBits    = (uint8) config->dataBits;
            PCComms_txBufferSize  = (uint8) config->txBufferSize;

            /* Configure UART interface */
            if(PCComms_UART_MODE_IRDA == config->mode)
            {
                /* OVS settings: IrDA */
                PCComms_CTRL_REG  = ((0u != config->enableIrdaLowPower) ?
                                                (PCComms_UART_GET_CTRL_OVS_IRDA_LP(config->oversample)) :
                                                (PCComms_CTRL_OVS_IRDA_OVS16));
            }
            else
            {
                /* OVS settings: UART and SmartCard */
                PCComms_CTRL_REG  = PCComms_GET_CTRL_OVS(config->oversample);
            }

            PCComms_CTRL_REG     |= PCComms_GET_CTRL_BYTE_MODE  (config->enableByteMode)      |
                                             PCComms_GET_CTRL_ADDR_ACCEPT(config->multiprocAcceptAddr) |
                                             PCComms_CTRL_UART;

            /* Configure sub-mode: UART, SmartCard or IrDA */
            PCComms_UART_CTRL_REG = PCComms_GET_UART_CTRL_MODE(config->mode);

            /* Configure RX direction */
            PCComms_UART_RX_CTRL_REG = PCComms_GET_UART_RX_CTRL_MODE(config->stopBits)              |
                                        PCComms_GET_UART_RX_CTRL_POLARITY(config->enableInvertedRx)          |
                                        PCComms_GET_UART_RX_CTRL_MP_MODE(config->enableMultiproc)            |
                                        PCComms_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(config->dropOnParityErr) |
                                        PCComms_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(config->dropOnFrameErr);

            if(PCComms_UART_PARITY_NONE != config->parity)
            {
               PCComms_UART_RX_CTRL_REG |= PCComms_GET_UART_RX_CTRL_PARITY(config->parity) |
                                                    PCComms_UART_RX_CTRL_PARITY_ENABLED;
            }

            PCComms_RX_CTRL_REG      = PCComms_GET_RX_CTRL_DATA_WIDTH(config->dataBits)       |
                                                PCComms_GET_RX_CTRL_MEDIAN(config->enableMedianFilter) |
                                                PCComms_GET_UART_RX_CTRL_ENABLED(config->direction);

            PCComms_RX_FIFO_CTRL_REG = PCComms_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(config->rxTriggerLevel);

            /* Configure MP address */
            PCComms_RX_MATCH_REG     = PCComms_GET_RX_MATCH_ADDR(config->multiprocAddr) |
                                                PCComms_GET_RX_MATCH_MASK(config->multiprocAddrMask);

            /* Configure RX direction */
            PCComms_UART_TX_CTRL_REG = PCComms_GET_UART_TX_CTRL_MODE(config->stopBits) |
                                                PCComms_GET_UART_TX_CTRL_RETRY_NACK(config->enableRetryNack);

            if(PCComms_UART_PARITY_NONE != config->parity)
            {
               PCComms_UART_TX_CTRL_REG |= PCComms_GET_UART_TX_CTRL_PARITY(config->parity) |
                                                    PCComms_UART_TX_CTRL_PARITY_ENABLED;
            }

            PCComms_TX_CTRL_REG      = PCComms_GET_TX_CTRL_DATA_WIDTH(config->dataBits)    |
                                                PCComms_GET_UART_TX_CTRL_ENABLED(config->direction);

            PCComms_TX_FIFO_CTRL_REG = PCComms_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(config->txTriggerLevel);

        #if !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
            PCComms_UART_FLOW_CTRL_REG = PCComms_GET_UART_FLOW_CTRL_CTS_ENABLE(config->enableCts) | \
                                            PCComms_GET_UART_FLOW_CTRL_CTS_POLARITY (config->ctsPolarity)  | \
                                            PCComms_GET_UART_FLOW_CTRL_RTS_POLARITY(config->rtsPolarity)   | \
                                            PCComms_GET_UART_FLOW_CTRL_TRIGGER_LEVEL(config->rtsRxFifoLevel);
        #endif /* !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */

            /* Configure interrupt with UART handler but do not enable it */
            CyIntDisable    (PCComms_ISR_NUMBER);
            CyIntSetPriority(PCComms_ISR_NUMBER, PCComms_ISR_PRIORITY);
            (void) CyIntSetVector(PCComms_ISR_NUMBER, &PCComms_SPI_UART_ISR);

            /* Configure WAKE interrupt */
        #if(PCComms_UART_RX_WAKEUP_IRQ)
            CyIntDisable    (PCComms_RX_WAKE_ISR_NUMBER);
            CyIntSetPriority(PCComms_RX_WAKE_ISR_NUMBER, PCComms_RX_WAKE_ISR_PRIORITY);
            (void) CyIntSetVector(PCComms_RX_WAKE_ISR_NUMBER, &PCComms_UART_WAKEUP_ISR);
        #endif /* (PCComms_UART_RX_WAKEUP_IRQ) */

            /* Configure interrupt sources */
            PCComms_INTR_I2C_EC_MASK_REG = PCComms_NO_INTR_SOURCES;
            PCComms_INTR_SPI_EC_MASK_REG = PCComms_NO_INTR_SOURCES;
            PCComms_INTR_SLAVE_MASK_REG  = PCComms_NO_INTR_SOURCES;
            PCComms_INTR_MASTER_MASK_REG = PCComms_NO_INTR_SOURCES;
            PCComms_INTR_RX_MASK_REG     = config->rxInterruptMask;
            PCComms_INTR_TX_MASK_REG     = config->txInterruptMask;

            /* Clear RX buffer indexes */
            PCComms_rxBufferHead     = 0u;
            PCComms_rxBufferTail     = 0u;
            PCComms_rxBufferOverflow = 0u;

            /* Clear TX buffer indexes */
            PCComms_txBufferHead = 0u;
            PCComms_txBufferTail = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: PCComms_UartInit
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
    void PCComms_UartInit(void)
    {
        /* Configure UART interface */
        PCComms_CTRL_REG = PCComms_UART_DEFAULT_CTRL;

        /* Configure sub-mode: UART, SmartCard or IrDA */
        PCComms_UART_CTRL_REG = PCComms_UART_DEFAULT_UART_CTRL;

        /* Configure RX direction */
        PCComms_UART_RX_CTRL_REG = PCComms_UART_DEFAULT_UART_RX_CTRL;
        PCComms_RX_CTRL_REG      = PCComms_UART_DEFAULT_RX_CTRL;
        PCComms_RX_FIFO_CTRL_REG = PCComms_UART_DEFAULT_RX_FIFO_CTRL;
        PCComms_RX_MATCH_REG     = PCComms_UART_DEFAULT_RX_MATCH_REG;

        /* Configure TX direction */
        PCComms_UART_TX_CTRL_REG = PCComms_UART_DEFAULT_UART_TX_CTRL;
        PCComms_TX_CTRL_REG      = PCComms_UART_DEFAULT_TX_CTRL;
        PCComms_TX_FIFO_CTRL_REG = PCComms_UART_DEFAULT_TX_FIFO_CTRL;

    #if !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
        PCComms_UART_FLOW_CTRL_REG = PCComms_UART_DEFAULT_FLOW_CTRL;
    #endif /* !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */

        /* Configure interrupt with UART handler but do not enable it */
    #if(PCComms_SCB_IRQ_INTERNAL)
        CyIntDisable    (PCComms_ISR_NUMBER);
        CyIntSetPriority(PCComms_ISR_NUMBER, PCComms_ISR_PRIORITY);
        (void) CyIntSetVector(PCComms_ISR_NUMBER, &PCComms_SPI_UART_ISR);
    #endif /* (PCComms_SCB_IRQ_INTERNAL) */

        /* Configure WAKE interrupt */
    #if(PCComms_UART_RX_WAKEUP_IRQ)
        CyIntDisable    (PCComms_RX_WAKE_ISR_NUMBER);
        CyIntSetPriority(PCComms_RX_WAKE_ISR_NUMBER, PCComms_RX_WAKE_ISR_PRIORITY);
        (void) CyIntSetVector(PCComms_RX_WAKE_ISR_NUMBER, &PCComms_UART_WAKEUP_ISR);
    #endif /* (PCComms_UART_RX_WAKEUP_IRQ) */

        /* Configure interrupt sources */
        PCComms_INTR_I2C_EC_MASK_REG = PCComms_UART_DEFAULT_INTR_I2C_EC_MASK;
        PCComms_INTR_SPI_EC_MASK_REG = PCComms_UART_DEFAULT_INTR_SPI_EC_MASK;
        PCComms_INTR_SLAVE_MASK_REG  = PCComms_UART_DEFAULT_INTR_SLAVE_MASK;
        PCComms_INTR_MASTER_MASK_REG = PCComms_UART_DEFAULT_INTR_MASTER_MASK;
        PCComms_INTR_RX_MASK_REG     = PCComms_UART_DEFAULT_INTR_RX_MASK;
        PCComms_INTR_TX_MASK_REG     = PCComms_UART_DEFAULT_INTR_TX_MASK;

    #if(PCComms_INTERNAL_RX_SW_BUFFER_CONST)
        PCComms_rxBufferHead     = 0u;
        PCComms_rxBufferTail     = 0u;
        PCComms_rxBufferOverflow = 0u;
    #endif /* (PCComms_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(PCComms_INTERNAL_TX_SW_BUFFER_CONST)
        PCComms_txBufferHead = 0u;
        PCComms_txBufferTail = 0u;
    #endif /* (PCComms_INTERNAL_TX_SW_BUFFER_CONST) */
    }
#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: PCComms_UartSetRxAddress
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
void PCComms_UartSetRxAddress(uint32 address)
{
     uint32 matchReg;

    matchReg = PCComms_RX_MATCH_REG;

    matchReg &= ((uint32) ~PCComms_RX_MATCH_ADDR_MASK); /* Clear address bits */
    matchReg |= ((uint32)  (address & PCComms_RX_MATCH_ADDR_MASK)); /* Set address  */

    PCComms_RX_MATCH_REG = matchReg;
}


/*******************************************************************************
* Function Name: PCComms_UartSetRxAddressMask
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
void PCComms_UartSetRxAddressMask(uint32 addressMask)
{
    uint32 matchReg;

    matchReg = PCComms_RX_MATCH_REG;

    matchReg &= ((uint32) ~PCComms_RX_MATCH_MASK_MASK); /* Clear address mask bits */
    matchReg |= ((uint32) (addressMask << PCComms_RX_MATCH_MASK_POS));

    PCComms_RX_MATCH_REG = matchReg;
}


#if(PCComms_UART_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: PCComms_UartGetChar
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
    uint32 PCComms_UartGetChar(void)
    {
        uint32 rxData = 0u;

        /* Reads data only if there is data to read */
        if(0u != PCComms_SpiUartGetRxBufferSize())
        {
            rxData = PCComms_SpiUartReadRxData();
        }

        if(PCComms_CHECK_INTR_RX(PCComms_INTR_RX_ERR))
        {
            rxData = 0u; /* Error occurred: returns zero */
            PCComms_ClearRxInterruptSource(PCComms_INTR_RX_ERR);
        }

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: PCComms_UartGetByte
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
    uint32 PCComms_UartGetByte(void)
    {
        uint32 rxData;
        uint32 tmpStatus;
        uint32 intSourceMask;

        intSourceMask = PCComms_SpiUartDisableIntRx();

        if(0u != PCComms_SpiUartGetRxBufferSize())
        {
            /* Enables interrupt to receive more bytes: at least one byte is in
            * buffer.
            */
            PCComms_SpiUartEnableIntRx(intSourceMask);

            /* Get received byte */
            rxData = PCComms_SpiUartReadRxData();
        }
        else
        {
            /* Reads a byte directly from RX FIFO: underflow is raised in the case
            * of empty. Otherwise the first received byte will be read.
            */
            rxData = PCComms_RX_FIFO_RD_REG;

            /* Enables interrupt to receive more bytes.
            * The RX_NOT_EMPTY interrupt is cleared by the interrupt routine
            * in case the byte was received and read by code above.
            */
            PCComms_SpiUartEnableIntRx(intSourceMask);
        }

        /* Get and clear RX error mask */
        tmpStatus = (PCComms_GetRxInterruptSource() & PCComms_INTR_RX_ERR);
        PCComms_ClearRxInterruptSource(PCComms_INTR_RX_ERR);

        /* Puts together data and error status:
        * MP mode and accept address: 9th bit is set to notify mark.
        */
        rxData |= ((uint32) (tmpStatus << 8u));

        return(rxData);
    }


    #if !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: PCComms_UartSetRtsPolarity
        ********************************************************************************
        *
        * Summary:
        *  Sets active polarity of RTS output signal.
        *
        * Parameters:
        *  polarity: Active polarity of RTS output signal.
        *   PCComms_UART_RTS_ACTIVE_LOW  - RTS signal is active low.
        *   PCComms_UART_RTS_ACTIVE_HIGH - RTS signal is active high.
        *
        * Return:
        *  None
        *
        *******************************************************************************/
        void PCComms_UartSetRtsPolarity(uint32 polarity)
        {
            if(0u != polarity)
            {
                PCComms_UART_FLOW_CTRL_REG |= (uint32)  PCComms_UART_FLOW_CTRL_RTS_POLARITY;
            }
            else
            {
                PCComms_UART_FLOW_CTRL_REG &= (uint32) ~PCComms_UART_FLOW_CTRL_RTS_POLARITY;
            }
        }


        /*******************************************************************************
        * Function Name: PCComms_UartSetRtsFifoLevel
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
        void PCComms_UartSetRtsFifoLevel(uint32 level)
        {
            uint32 uartFlowCtrl;

            uartFlowCtrl = PCComms_UART_FLOW_CTRL_REG;

            uartFlowCtrl &= ((uint32) ~PCComms_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
            uartFlowCtrl |= ((uint32) (PCComms_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK & level));

            PCComms_UART_FLOW_CTRL_REG = uartFlowCtrl;
        }
    #endif /* !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */

#endif /* (PCComms_UART_RX_DIRECTION) */


#if(PCComms_UART_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: PCComms_UartPutString
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
    void PCComms_UartPutString(const char8 string[])
    {
        uint32 bufIndex;

        bufIndex = 0u;

        /* Blocks the control flow until all data has been sent */
        while(string[bufIndex] != ((char8) 0))
        {
            PCComms_UartPutChar((uint32) string[bufIndex]);
            bufIndex++;
        }
    }


    /*******************************************************************************
    * Function Name: PCComms_UartPutCRLF
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
    void PCComms_UartPutCRLF(uint32 txDataByte)
    {
        PCComms_UartPutChar(txDataByte);  /* Blocks control flow until all data has been sent */
        PCComms_UartPutChar(0x0Du);       /* Blocks control flow until all data has been sent */
        PCComms_UartPutChar(0x0Au);       /* Blocks control flow until all data has been sent */
    }


    #if !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: PCCommsSCB_UartEnableCts
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
        void PCComms_UartEnableCts(void)
        {
            PCComms_UART_FLOW_CTRL_REG |= (uint32)  PCComms_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: PCComms_UartDisableCts
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
        void PCComms_UartDisableCts(void)
        {
            PCComms_UART_FLOW_CTRL_REG &= (uint32) ~PCComms_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: PCComms_UartSetCtsPolarity
        ********************************************************************************
        *
        * Summary:
        *  Sets active polarity of CTS input signal.
        *
        * Parameters:
        *  polarity: Active polarity of CTS output signal.
        *   PCComms_UART_CTS_ACTIVE_LOW  - CTS signal is active low.
        *   PCComms_UART_CTS_ACTIVE_HIGH - CTS signal is active high.
        *
        * Return:
        *  None
        *
        *******************************************************************************/
        void PCComms_UartSetCtsPolarity(uint32 polarity)
        {
            if (0u != polarity)
            {
                PCComms_UART_FLOW_CTRL_REG |= (uint32)  PCComms_UART_FLOW_CTRL_CTS_POLARITY;
            }
            else
            {
                PCComms_UART_FLOW_CTRL_REG &= (uint32) ~PCComms_UART_FLOW_CTRL_CTS_POLARITY;
            }
        }
    #endif /* !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */

#endif /* (PCComms_UART_TX_DIRECTION) */


#if(PCComms_UART_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: PCComms_UartSaveConfig
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
    void PCComms_UartSaveConfig(void)
    {
        /* Clear interrupt activity:
        *  - set skip start and disable RX. At GPIO wakeup RX will be enabled.
        *  - clear rx_wake interrupt source as it triggers during normal operation.
        *  - clear wake interrupt pending state as it becomes pending in active mode.
        */

        PCComms_UART_RX_CTRL_REG |= PCComms_UART_RX_CTRL_SKIP_START;

    #if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
        #if(PCComms_MOSI_SCL_RX_WAKE_PIN)
            (void) PCComms_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
        #endif /* (PCComms_MOSI_SCL_RX_WAKE_PIN) */
    #else
        #if(PCComms_UART_RX_WAKE_PIN)
            (void) PCComms_rx_wake_ClearInterrupt();
        #endif /* (PCComms_UART_RX_WAKE_PIN) */
    #endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */

    #if(PCComms_UART_RX_WAKEUP_IRQ)
        PCComms_RxWakeClearPendingInt();
        PCComms_RxWakeEnableInt();
    #endif /* (PCComms_UART_RX_WAKEUP_IRQ) */
    }


    /*******************************************************************************
    * Function Name: PCComms_UartRestoreConfig
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
    void PCComms_UartRestoreConfig(void)
    {
    /* Disable RX GPIO interrupt: no more triggers in active mode */
    #if(PCComms_UART_RX_WAKEUP_IRQ)
        PCComms_RxWakeDisableInt();
    #endif /* (PCComms_UART_RX_WAKEUP_IRQ) */
    }
#endif /* (PCComms_UART_WAKE_ENABLE_CONST) */


#if(PCComms_UART_RX_WAKEUP_IRQ)
    /*******************************************************************************
    * Function Name: PCComms_UART_WAKEUP_ISR
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
    CY_ISR(PCComms_UART_WAKEUP_ISR)
    {
        /* Clear interrupt source: the event becomes multi triggered and is
        * only disabled by PCComms_UartRestoreConfig() call.
        */
    #if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
        #if(PCComms_MOSI_SCL_RX_WAKE_PIN)
            (void) PCComms_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
        #endif /* (PCComms_MOSI_SCL_RX_WAKE_PIN) */
    #else
        #if(PCComms_UART_RX_WAKE_PIN)
            (void) PCComms_rx_wake_ClearInterrupt();
        #endif /* (PCComms_UART_RX_WAKE_PIN) */
    #endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */
    }
#endif /* (PCComms_UART_RX_WAKEUP_IRQ) */


/* [] END OF FILE */
