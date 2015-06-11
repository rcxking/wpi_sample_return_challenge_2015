/*******************************************************************************
* File Name: PCComms_SPI_UART.c
* Version 2.0
*
* Description:
*  This file provides the source code to the API for the SCB Component in
*  SPI and UART modes.
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

/***************************************
*        SPI/UART Private Vars
***************************************/

#if(PCComms_INTERNAL_RX_SW_BUFFER_CONST)
    volatile uint32 PCComms_rxBufferHead;
    volatile uint32 PCComms_rxBufferTail;
    volatile uint8  PCComms_rxBufferOverflow;
#endif /* (PCComms_INTERNAL_RX_SW_BUFFER_CONST) */

#if(PCComms_INTERNAL_TX_SW_BUFFER_CONST)
    volatile uint32 PCComms_txBufferHead;
    volatile uint32 PCComms_txBufferTail;
#endif /* (PCComms_INTERNAL_TX_SW_BUFFER_CONST) */

#if(PCComms_INTERNAL_RX_SW_BUFFER)
    /* Add one element to the buffer to receive full packet. One byte in receive buffer is always empty */
    volatile uint8 PCComms_rxBufferInternal[PCComms_RX_BUFFER_SIZE];
#endif /* (PCComms_INTERNAL_RX_SW_BUFFER) */

#if(PCComms_INTERNAL_TX_SW_BUFFER)
    volatile uint8 PCComms_txBufferInternal[PCComms_TX_BUFFER_SIZE];
#endif /* (PCComms_INTERNAL_TX_SW_BUFFER) */


#if(PCComms_RX_DIRECTION)

    /*******************************************************************************
    * Function Name: PCComms_SpiUartReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Retrieves the next data element from the receive buffer.
    *   - RX software buffer is disabled: Returns data element retrieved from
    *     RX FIFO. Undefined data will be returned if the RX FIFO is empty.
    *   - RX software buffer is enabled: Returns data element from the software
    *     receive buffer. Zero value is returned if the software receive buffer
    *     is empty.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Next data element from the receive buffer.
    *
    * Global Variables:
    *  Look into PCComms_SpiInit for description.
    *
    *******************************************************************************/
    uint32 PCComms_SpiUartReadRxData(void)
    {
        uint32 rxData = 0u;

        #if(PCComms_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (PCComms_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(PCComms_CHECK_RX_SW_BUFFER)
        {
            if(PCComms_rxBufferHead != PCComms_rxBufferTail)
            {
                /* There is data in RX software buffer */

                /* Calculate index to read from */
                locTail = (PCComms_rxBufferTail + 1u);

                if(PCComms_RX_BUFFER_SIZE == locTail)
                {
                    locTail = 0u;
                }

                /* Get data from RX software buffer */
                rxData = PCComms_GetWordFromRxBuffer(locTail);

                /* Change index in the buffer */
                PCComms_rxBufferTail = locTail;
            }
        }
        #else
        {
            rxData = PCComms_RX_FIFO_RD_REG; /* Read data from RX FIFO */
        }
        #endif

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: PCComms_SpiUartGetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of received data elements in the receive buffer.
    *   - RX software buffer disabled: returns the number of used entries in
    *     RX FIFO.
    *   - RX software buffer enabled: returns the number of elements which were
    *     placed in the receive buffer. This does not include the hardware RX FIFO.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Number of received data elements
    *
    *******************************************************************************/
    uint32 PCComms_SpiUartGetRxBufferSize(void)
    {
        uint32 size;
        #if(PCComms_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locHead;
        #endif /* (PCComms_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(PCComms_CHECK_RX_SW_BUFFER)
        {
            locHead = PCComms_rxBufferHead;

            if(locHead >= PCComms_rxBufferTail)
            {
                size = (locHead - PCComms_rxBufferTail);
            }
            else
            {
                size = (locHead + (PCComms_RX_BUFFER_SIZE - PCComms_rxBufferTail));
            }
        }
        #else
        {
            size = PCComms_GET_RX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: PCComms_SpiUartClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the receive buffer and RX FIFO.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PCComms_SpiUartClearRxBuffer(void)
    {
        #if(PCComms_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (PCComms_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(PCComms_CHECK_RX_SW_BUFFER)
        {
            intSourceMask = PCComms_SpiUartDisableIntRx();

            PCComms_CLEAR_RX_FIFO;

            /* Flush RX software buffer */
            PCComms_rxBufferHead     = PCComms_rxBufferTail;
            PCComms_rxBufferOverflow = 0u;

            /* End RX transfer */
            PCComms_ClearRxInterruptSource(PCComms_INTR_RX_ALL);

            PCComms_SpiUartEnableIntRx(intSourceMask);
        }
        #else
        {
            PCComms_CLEAR_RX_FIFO;
        }
        #endif
    }

#endif /* (PCComms_RX_DIRECTION) */


#if(PCComms_TX_DIRECTION)

    /*******************************************************************************
    * Function Name: PCComms_SpiUartWriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Places a data entry into the transmit buffer to be sent at the next available
    *  bus time.
    *  This function is blocking and waits until there is space available to put the
    *  requested data in the transmit buffer.
    *
    * Parameters:
    *  txDataByte: the data to be transmitted.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PCComms_SpiUartWriteTxData(uint32 txData)
    {
        #if(PCComms_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locHead;
            uint32 intSourceMask;
        #endif /* (PCComms_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(PCComms_CHECK_TX_SW_BUFFER)
        {
            /* Head index to put data */
            locHead = (PCComms_txBufferHead + 1u);

            /* Adjust TX software buffer index */
            if(PCComms_TX_BUFFER_SIZE == locHead)
            {
                locHead = 0u;
            }

            while(locHead == PCComms_txBufferTail)
            {
                /* Wait for space in TX software buffer */
            }

            /* TX software buffer has at least one room */

            if((PCComms_txBufferHead == PCComms_txBufferTail) &&
               (PCComms_SPI_UART_FIFO_SIZE != PCComms_GET_TX_FIFO_ENTRIES))
            {
                /* TX software buffer is empty: put data directly in TX FIFO */
                PCComms_TX_FIFO_WR_REG = txData;
            }
            /* Put data in TX software buffer */
            else
            {
                /* Clear old status of INTR_TX_NOT_FULL. It sets at the end of transfer when TX FIFO is empty. */
                PCComms_ClearTxInterruptSource(PCComms_INTR_TX_NOT_FULL);

                PCComms_PutWordInTxBuffer(locHead, txData);

                PCComms_txBufferHead = locHead;

                /* Enable interrupt to transmit */
                intSourceMask  = PCComms_INTR_TX_NOT_FULL;
                intSourceMask |= PCComms_GetTxInterruptMode();
                PCComms_SpiUartEnableIntTx(intSourceMask);
            }
        }
        #else
        {
            while(PCComms_SPI_UART_FIFO_SIZE == PCComms_GET_TX_FIFO_ENTRIES)
            {
                /* Block while TX FIFO is FULL */
            }

            PCComms_TX_FIFO_WR_REG = txData;
        }
        #endif
    }


    /*******************************************************************************
    * Function Name: PCComms_SpiUartPutArray
    ********************************************************************************
    *
    * Summary:
    *  Places an array of data into the transmit buffer to be sent.
    *  This function is blocking and waits until there is a space available to put
    *  all the requested data in the transmit buffer. The array size can be greater
    *  than transmit buffer size.
    *
    * Parameters:
    *  wrBuf:  pointer to an array with data to be placed in transmit buffer.
    *  count:  number of data elements to be placed in the transmit buffer.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PCComms_SpiUartPutArray(const uint8 wrBuf[], uint32 count)
    {
        uint32 i;

        for(i=0u; i < count; i++)
        {
            PCComms_SpiUartWriteTxData((uint32) wrBuf[i]);
        }
    }


    /*******************************************************************************
    * Function Name: PCComms_SpiUartGetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    * Returns the number of elements currently in the transmit buffer.
    *  - TX software buffer is disabled: returns the number of used entries in
    *    TX FIFO.
    *  - TX software buffer is enabled: returns the number of elements currently
    *    used in the transmit buffer. This number does not include used entries in
    *    the TX FIFO. The transmit buffer size is zero until the TX FIFO is
    *    not full.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Number of data elements ready to transmit.
    *
    *******************************************************************************/
    uint32 PCComms_SpiUartGetTxBufferSize(void)
    {
        uint32 size;
        #if(PCComms_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (PCComms_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(PCComms_CHECK_TX_SW_BUFFER)
        {
            /* Get current Tail index */
            locTail = PCComms_txBufferTail;

            if(PCComms_txBufferHead >= locTail)
            {
                size = (PCComms_txBufferHead - locTail);
            }
            else
            {
                size = (PCComms_txBufferHead + (PCComms_TX_BUFFER_SIZE - locTail));
            }
        }
        #else
        {
            size = PCComms_GET_TX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: PCComms_SpiUartClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the transmit buffer and TX FIFO.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PCComms_SpiUartClearTxBuffer(void)
    {
        #if(PCComms_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (PCComms_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(PCComms_CHECK_TX_SW_BUFFER)
        {
            intSourceMask = PCComms_SpiUartDisableIntTx();

            PCComms_CLEAR_TX_FIFO;

            /* Flush TX software buffer */
            PCComms_txBufferHead = PCComms_txBufferTail;

            /* End TX transfer if it is in progress */
            intSourceMask &= (uint32) ~PCComms_INTR_TX_NOT_FULL;

            PCComms_SpiUartEnableIntTx(intSourceMask);
        }
        #else
        {
            PCComms_CLEAR_TX_FIFO;
        }
        #endif
    }

#endif /* (PCComms_TX_DIRECTION) */


/*******************************************************************************
* Function Name: PCComms_SpiUartDisableIntRx
********************************************************************************
*
* Summary:
*  Disables the RX interrupt sources.
*
* Parameters:
*  None
*
* Return:
*  Returns the RX interrupt sources enabled before the function call.
*
*******************************************************************************/
uint32 PCComms_SpiUartDisableIntRx(void)
{
    uint32 intSource;

    intSource = PCComms_GetRxInterruptMode();

    PCComms_SetRxInterruptMode(PCComms_NO_INTR_SOURCES);

    return(intSource);
}


/*******************************************************************************
* Function Name: PCComms_SpiUartDisableIntTx
********************************************************************************
*
* Summary:
*  Disables TX interrupt sources.
*
* Parameters:
*  None
*
* Return:
*  Returns TX interrupt sources enabled before function call.
*
*******************************************************************************/
uint32 PCComms_SpiUartDisableIntTx(void)
{
    uint32 intSourceMask;

    intSourceMask = PCComms_GetTxInterruptMode();

    PCComms_SetTxInterruptMode(PCComms_NO_INTR_SOURCES);

    return(intSourceMask);
}


#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: PCComms_PutWordInRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Stores a byte/word into the RX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    * Parameters:
    *  index:      index to store data byte/word in the RX buffer.
    *  rxDataByte: byte/word to store.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PCComms_PutWordInRxBuffer(uint32 idx, uint32 rxDataByte)
    {
        /* Put data in buffer */
        if(PCComms_ONE_BYTE_WIDTH == PCComms_rxDataBits)
        {
            PCComms_rxBuffer[idx] = ((uint8) rxDataByte);
        }
        else
        {
            PCComms_rxBuffer[(uint32)(idx << 1u)]      = LO8(LO16(rxDataByte));
            PCComms_rxBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(rxDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: PCComms_GetWordFromRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Reads byte/word from RX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Returns byte/word read from RX buffer.
    *
    *******************************************************************************/
    uint32 PCComms_GetWordFromRxBuffer(uint32 idx)
    {
        uint32 value;

        if(PCComms_ONE_BYTE_WIDTH == PCComms_rxDataBits)
        {
            value = PCComms_rxBuffer[idx];
        }
        else
        {
            value  = (uint32) PCComms_rxBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32)PCComms_rxBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }


    /*******************************************************************************
    * Function Name: PCComms_PutWordInTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Stores byte/word into the TX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    * Parameters:
    *  idx:        index to store data byte/word in the TX buffer.
    *  txDataByte: byte/word to store.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PCComms_PutWordInTxBuffer(uint32 idx, uint32 txDataByte)
    {
        /* Put data in buffer */
        if(PCComms_ONE_BYTE_WIDTH == PCComms_txDataBits)
        {
            PCComms_txBuffer[idx] = ((uint8) txDataByte);
        }
        else
        {
            PCComms_txBuffer[(uint32)(idx << 1u)]      = LO8(LO16(txDataByte));
            PCComms_txBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(txDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: PCComms_GetWordFromTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Reads byte/word from the TX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    * Parameters:
    *  idx: index to get data byte/word from the TX buffer.
    *
    * Return:
    *  Returns byte/word read from the TX buffer.
    *
    *******************************************************************************/
    uint32 PCComms_GetWordFromTxBuffer(uint32 idx)
    {
        uint32 value;

        if(PCComms_ONE_BYTE_WIDTH == PCComms_txDataBits)
        {
            value = (uint32) PCComms_txBuffer[idx];
        }
        else
        {
            value  = (uint32) PCComms_txBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32) PCComms_txBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }

#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
