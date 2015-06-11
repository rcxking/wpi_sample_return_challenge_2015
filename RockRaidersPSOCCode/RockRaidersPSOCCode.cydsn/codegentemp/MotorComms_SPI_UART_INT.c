/*******************************************************************************
* File Name: MotorComms_SPI_UART_INT.c
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

#include "MotorComms_PVT.h"
#include "MotorComms_SPI_UART_PVT.h"


/*******************************************************************************
* Function Name: MotorComms_SPI_UART_ISR
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
CY_ISR(MotorComms_SPI_UART_ISR)
{
#if(MotorComms_INTERNAL_RX_SW_BUFFER_CONST)
    uint32 locHead;
    uint32 dataRx;
#endif /* (MotorComms_INTERNAL_RX_SW_BUFFER_CONST) */

#if(MotorComms_INTERNAL_TX_SW_BUFFER_CONST)
    uint32 locTail;
#endif /* (MotorComms_INTERNAL_TX_SW_BUFFER_CONST) */

    if(NULL != MotorComms_customIntrHandler)
    {
        MotorComms_customIntrHandler();
    }

    #if(MotorComms_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        MotorComms_ClearSpiExtClkInterruptSource(MotorComms_INTR_SPI_EC_WAKE_UP);
    }
    #endif

    #if(MotorComms_CHECK_RX_SW_BUFFER)
    {
        if(MotorComms_CHECK_INTR_RX_MASKED(MotorComms_INTR_RX_NOT_EMPTY))
        {
            while(0u != MotorComms_GET_RX_FIFO_ENTRIES)
            {
                /* Get data from RX FIFO */
                dataRx = MotorComms_RX_FIFO_RD_REG;

                /* Move local head index */
                locHead = (MotorComms_rxBufferHead + 1u);

                /* Adjust local head index */
                if(MotorComms_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if(locHead == MotorComms_rxBufferTail)
                {
                    /* Overflow: through away new data */
                    MotorComms_rxBufferOverflow = (uint8) MotorComms_INTR_RX_OVERFLOW;
                }
                else
                {
                    /* Store received data */
                    MotorComms_PutWordInRxBuffer(locHead, dataRx);

                    /* Move head index */
                    MotorComms_rxBufferHead = locHead;
                }
            }

            MotorComms_ClearRxInterruptSource(MotorComms_INTR_RX_NOT_EMPTY);
        }
    }
    #endif


    #if(MotorComms_CHECK_TX_SW_BUFFER)
    {
        if(MotorComms_CHECK_INTR_TX_MASKED(MotorComms_INTR_TX_NOT_FULL))
        {
            /* Put data into TX FIFO */
            while(MotorComms_SPI_UART_FIFO_SIZE != MotorComms_GET_TX_FIFO_ENTRIES)
            {
                /* Check for room in TX software buffer */
                if(MotorComms_txBufferHead != MotorComms_txBufferTail)
                {
                    /* Move local tail index */
                    locTail = (MotorComms_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if(MotorComms_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    MotorComms_TX_FIFO_WR_REG = MotorComms_GetWordFromTxBuffer(locTail);

                    /* Move tail index */
                    MotorComms_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is empty: complete transfer */
                    MotorComms_DISABLE_INTR_TX(MotorComms_INTR_TX_NOT_FULL);
                    break;
                }
            }

            MotorComms_ClearTxInterruptSource(MotorComms_INTR_TX_NOT_FULL);
        }
    }
    #endif
}


/* [] END OF FILE */
