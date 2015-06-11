/*******************************************************************************
* File Name: MotorComms_SPI_UART.c
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

#include "MotorComms_PVT.h"
#include "MotorComms_SPI_UART_PVT.h"

/***************************************
*        SPI/UART Private Vars
***************************************/

#if(MotorComms_INTERNAL_RX_SW_BUFFER_CONST)
    volatile uint32 MotorComms_rxBufferHead;
    volatile uint32 MotorComms_rxBufferTail;
    volatile uint8  MotorComms_rxBufferOverflow;
#endif /* (MotorComms_INTERNAL_RX_SW_BUFFER_CONST) */

#if(MotorComms_INTERNAL_TX_SW_BUFFER_CONST)
    volatile uint32 MotorComms_txBufferHead;
    volatile uint32 MotorComms_txBufferTail;
#endif /* (MotorComms_INTERNAL_TX_SW_BUFFER_CONST) */

#if(MotorComms_INTERNAL_RX_SW_BUFFER)
    /* Add one element to the buffer to receive full packet. One byte in receive buffer is always empty */
    volatile uint8 MotorComms_rxBufferInternal[MotorComms_RX_BUFFER_SIZE];
#endif /* (MotorComms_INTERNAL_RX_SW_BUFFER) */

#if(MotorComms_INTERNAL_TX_SW_BUFFER)
    volatile uint8 MotorComms_txBufferInternal[MotorComms_TX_BUFFER_SIZE];
#endif /* (MotorComms_INTERNAL_TX_SW_BUFFER) */


#if(MotorComms_RX_DIRECTION)

    /*******************************************************************************
    * Function Name: MotorComms_SpiUartReadRxData
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
    *  Look into MotorComms_SpiInit for description.
    *
    *******************************************************************************/
    uint32 MotorComms_SpiUartReadRxData(void)
    {
        uint32 rxData = 0u;

        #if(MotorComms_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (MotorComms_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(MotorComms_CHECK_RX_SW_BUFFER)
        {
            if(MotorComms_rxBufferHead != MotorComms_rxBufferTail)
            {
                /* There is data in RX software buffer */

                /* Calculate index to read from */
                locTail = (MotorComms_rxBufferTail + 1u);

                if(MotorComms_RX_BUFFER_SIZE == locTail)
                {
                    locTail = 0u;
                }

                /* Get data from RX software buffer */
                rxData = MotorComms_GetWordFromRxBuffer(locTail);

                /* Change index in the buffer */
                MotorComms_rxBufferTail = locTail;
            }
        }
        #else
        {
            rxData = MotorComms_RX_FIFO_RD_REG; /* Read data from RX FIFO */
        }
        #endif

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: MotorComms_SpiUartGetRxBufferSize
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
    uint32 MotorComms_SpiUartGetRxBufferSize(void)
    {
        uint32 size;
        #if(MotorComms_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locHead;
        #endif /* (MotorComms_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(MotorComms_CHECK_RX_SW_BUFFER)
        {
            locHead = MotorComms_rxBufferHead;

            if(locHead >= MotorComms_rxBufferTail)
            {
                size = (locHead - MotorComms_rxBufferTail);
            }
            else
            {
                size = (locHead + (MotorComms_RX_BUFFER_SIZE - MotorComms_rxBufferTail));
            }
        }
        #else
        {
            size = MotorComms_GET_RX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: MotorComms_SpiUartClearRxBuffer
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
    void MotorComms_SpiUartClearRxBuffer(void)
    {
        #if(MotorComms_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (MotorComms_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(MotorComms_CHECK_RX_SW_BUFFER)
        {
            intSourceMask = MotorComms_SpiUartDisableIntRx();

            MotorComms_CLEAR_RX_FIFO;

            /* Flush RX software buffer */
            MotorComms_rxBufferHead     = MotorComms_rxBufferTail;
            MotorComms_rxBufferOverflow = 0u;

            /* End RX transfer */
            MotorComms_ClearRxInterruptSource(MotorComms_INTR_RX_ALL);

            MotorComms_SpiUartEnableIntRx(intSourceMask);
        }
        #else
        {
            MotorComms_CLEAR_RX_FIFO;
        }
        #endif
    }

#endif /* (MotorComms_RX_DIRECTION) */


#if(MotorComms_TX_DIRECTION)

    /*******************************************************************************
    * Function Name: MotorComms_SpiUartWriteTxData
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
    void MotorComms_SpiUartWriteTxData(uint32 txData)
    {
        #if(MotorComms_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locHead;
            uint32 intSourceMask;
        #endif /* (MotorComms_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(MotorComms_CHECK_TX_SW_BUFFER)
        {
            /* Head index to put data */
            locHead = (MotorComms_txBufferHead + 1u);

            /* Adjust TX software buffer index */
            if(MotorComms_TX_BUFFER_SIZE == locHead)
            {
                locHead = 0u;
            }

            while(locHead == MotorComms_txBufferTail)
            {
                /* Wait for space in TX software buffer */
            }

            /* TX software buffer has at least one room */

            if((MotorComms_txBufferHead == MotorComms_txBufferTail) &&
               (MotorComms_SPI_UART_FIFO_SIZE != MotorComms_GET_TX_FIFO_ENTRIES))
            {
                /* TX software buffer is empty: put data directly in TX FIFO */
                MotorComms_TX_FIFO_WR_REG = txData;
            }
            /* Put data in TX software buffer */
            else
            {
                /* Clear old status of INTR_TX_NOT_FULL. It sets at the end of transfer when TX FIFO is empty. */
                MotorComms_ClearTxInterruptSource(MotorComms_INTR_TX_NOT_FULL);

                MotorComms_PutWordInTxBuffer(locHead, txData);

                MotorComms_txBufferHead = locHead;

                /* Enable interrupt to transmit */
                intSourceMask  = MotorComms_INTR_TX_NOT_FULL;
                intSourceMask |= MotorComms_GetTxInterruptMode();
                MotorComms_SpiUartEnableIntTx(intSourceMask);
            }
        }
        #else
        {
            while(MotorComms_SPI_UART_FIFO_SIZE == MotorComms_GET_TX_FIFO_ENTRIES)
            {
                /* Block while TX FIFO is FULL */
            }

            MotorComms_TX_FIFO_WR_REG = txData;
        }
        #endif
    }


    /*******************************************************************************
    * Function Name: MotorComms_SpiUartPutArray
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
    void MotorComms_SpiUartPutArray(const uint8 wrBuf[], uint32 count)
    {
        uint32 i;

        for(i=0u; i < count; i++)
        {
            MotorComms_SpiUartWriteTxData((uint32) wrBuf[i]);
        }
    }


    /*******************************************************************************
    * Function Name: MotorComms_SpiUartGetTxBufferSize
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
    uint32 MotorComms_SpiUartGetTxBufferSize(void)
    {
        uint32 size;
        #if(MotorComms_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (MotorComms_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(MotorComms_CHECK_TX_SW_BUFFER)
        {
            /* Get current Tail index */
            locTail = MotorComms_txBufferTail;

            if(MotorComms_txBufferHead >= locTail)
            {
                size = (MotorComms_txBufferHead - locTail);
            }
            else
            {
                size = (MotorComms_txBufferHead + (MotorComms_TX_BUFFER_SIZE - locTail));
            }
        }
        #else
        {
            size = MotorComms_GET_TX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: MotorComms_SpiUartClearTxBuffer
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
    void MotorComms_SpiUartClearTxBuffer(void)
    {
        #if(MotorComms_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (MotorComms_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(MotorComms_CHECK_TX_SW_BUFFER)
        {
            intSourceMask = MotorComms_SpiUartDisableIntTx();

            MotorComms_CLEAR_TX_FIFO;

            /* Flush TX software buffer */
            MotorComms_txBufferHead = MotorComms_txBufferTail;

            /* End TX transfer if it is in progress */
            intSourceMask &= (uint32) ~MotorComms_INTR_TX_NOT_FULL;

            MotorComms_SpiUartEnableIntTx(intSourceMask);
        }
        #else
        {
            MotorComms_CLEAR_TX_FIFO;
        }
        #endif
    }

#endif /* (MotorComms_TX_DIRECTION) */


/*******************************************************************************
* Function Name: MotorComms_SpiUartDisableIntRx
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
uint32 MotorComms_SpiUartDisableIntRx(void)
{
    uint32 intSource;

    intSource = MotorComms_GetRxInterruptMode();

    MotorComms_SetRxInterruptMode(MotorComms_NO_INTR_SOURCES);

    return(intSource);
}


/*******************************************************************************
* Function Name: MotorComms_SpiUartDisableIntTx
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
uint32 MotorComms_SpiUartDisableIntTx(void)
{
    uint32 intSourceMask;

    intSourceMask = MotorComms_GetTxInterruptMode();

    MotorComms_SetTxInterruptMode(MotorComms_NO_INTR_SOURCES);

    return(intSourceMask);
}


#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: MotorComms_PutWordInRxBuffer
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
    void MotorComms_PutWordInRxBuffer(uint32 idx, uint32 rxDataByte)
    {
        /* Put data in buffer */
        if(MotorComms_ONE_BYTE_WIDTH == MotorComms_rxDataBits)
        {
            MotorComms_rxBuffer[idx] = ((uint8) rxDataByte);
        }
        else
        {
            MotorComms_rxBuffer[(uint32)(idx << 1u)]      = LO8(LO16(rxDataByte));
            MotorComms_rxBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(rxDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: MotorComms_GetWordFromRxBuffer
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
    uint32 MotorComms_GetWordFromRxBuffer(uint32 idx)
    {
        uint32 value;

        if(MotorComms_ONE_BYTE_WIDTH == MotorComms_rxDataBits)
        {
            value = MotorComms_rxBuffer[idx];
        }
        else
        {
            value  = (uint32) MotorComms_rxBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32)MotorComms_rxBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }


    /*******************************************************************************
    * Function Name: MotorComms_PutWordInTxBuffer
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
    void MotorComms_PutWordInTxBuffer(uint32 idx, uint32 txDataByte)
    {
        /* Put data in buffer */
        if(MotorComms_ONE_BYTE_WIDTH == MotorComms_txDataBits)
        {
            MotorComms_txBuffer[idx] = ((uint8) txDataByte);
        }
        else
        {
            MotorComms_txBuffer[(uint32)(idx << 1u)]      = LO8(LO16(txDataByte));
            MotorComms_txBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(txDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: MotorComms_GetWordFromTxBuffer
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
    uint32 MotorComms_GetWordFromTxBuffer(uint32 idx)
    {
        uint32 value;

        if(MotorComms_ONE_BYTE_WIDTH == MotorComms_txDataBits)
        {
            value = (uint32) MotorComms_txBuffer[idx];
        }
        else
        {
            value  = (uint32) MotorComms_txBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32) MotorComms_txBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }

#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
