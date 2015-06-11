/*******************************************************************************
* File Name: PCComms_SPI_UART_INT.c
* Version 2.0
*
* Description:
*  This file provides the source code to the Interrupt Service Routine for
*  the SCB Component in SPI and UART modes.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "PCComms_PVT.h"
#include "PCComms_SPI_UART_PVT.h"


/*******************************************************************************
* Function Name: PCComms_SPI_UART_ISR
********************************************************************************
*
* Summary:
*  Handles the Interrupt Service Routine for the SCB SPI or UART modes.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
CY_ISR(PCComms_SPI_UART_ISR)
{
#if(PCComms_INTERNAL_RX_SW_BUFFER_CONST)
    uint32 locHead;
    uint32 dataRx;
#endif /* (PCComms_INTERNAL_RX_SW_BUFFER_CONST) */

#if(PCComms_INTERNAL_TX_SW_BUFFER_CONST)
    uint32 locTail;
#endif /* (PCComms_INTERNAL_TX_SW_BUFFER_CONST) */

    if(NULL != PCComms_customIntrHandler)
    {
        PCComms_customIntrHandler();
    }

    #if(PCComms_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        PCComms_ClearSpiExtClkInterruptSource(PCComms_INTR_SPI_EC_WAKE_UP);
    }
    #endif

    #if(PCComms_CHECK_RX_SW_BUFFER)
    {
        if(PCComms_CHECK_INTR_RX_MASKED(PCComms_INTR_RX_NOT_EMPTY))
        {
            while(0u != PCComms_GET_RX_FIFO_ENTRIES)
            {
                /* Get data from RX FIFO */
                dataRx = PCComms_RX_FIFO_RD_REG;

                /* Move local head index */
                locHead = (PCComms_rxBufferHead + 1u);

                /* Adjust local head index */
                if(PCComms_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if(locHead == PCComms_rxBufferTail)
                {
                    /* Overflow: through away new data */
                    PCComms_rxBufferOverflow = (uint8) PCComms_INTR_RX_OVERFLOW;
                }
                else
                {
                    /* Store received data */
                    PCComms_PutWordInRxBuffer(locHead, dataRx);

                    /* Move head index */
                    PCComms_rxBufferHead = locHead;
                }
            }

            PCComms_ClearRxInterruptSource(PCComms_INTR_RX_NOT_EMPTY);
        }
    }
    #endif


    #if(PCComms_CHECK_TX_SW_BUFFER)
    {
        if(PCComms_CHECK_INTR_TX_MASKED(PCComms_INTR_TX_NOT_FULL))
        {
            /* Put data into TX FIFO */
            while(PCComms_SPI_UART_FIFO_SIZE != PCComms_GET_TX_FIFO_ENTRIES)
            {
                /* Check for room in TX software buffer */
                if(PCComms_txBufferHead != PCComms_txBufferTail)
                {
                    /* Move local tail index */
                    locTail = (PCComms_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if(PCComms_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    PCComms_TX_FIFO_WR_REG = PCComms_GetWordFromTxBuffer(locTail);

                    /* Move tail index */
                    PCComms_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is empty: complete transfer */
                    PCComms_DISABLE_INTR_TX(PCComms_INTR_TX_NOT_FULL);
                    break;
                }
            }

            PCComms_ClearTxInterruptSource(PCComms_INTR_TX_NOT_FULL);
        }
    }
    #endif
}


/* [] END OF FILE */
