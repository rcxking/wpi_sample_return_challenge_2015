/*******************************************************************************
* File Name: PCComms_BOOT.c
* Version 2.0
*
* Description:
*  This file provides the source code to the API for the bootloader
*  communication support in the SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "PCComms_BOOT.h"


#if defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_BTLDR_COMM_MODE_ENABLED)

/*******************************************************************************
* Function Name: PCComms_CyBtldrCommStart
********************************************************************************
*
* Summary:
*  Calls the CyBtldrCommStart function of the bootloader communication
*  component for the selected mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PCComms_CyBtldrCommStart(void)
{
    #if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
        if(PCComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            PCComms_I2CCyBtldrCommStart();
        }
        else if(PCComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            PCComms_SpiCyBtldrCommStart();
        }
        else if(PCComms_SCB_MODE_UART_RUNTM_CFG)
        {
            PCComms_UartCyBtldrCommStart();
        }
        else if(PCComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
             PCComms_EzI2CCyBtldrCommStart();
        }
        else
        {
            /* Unknown mode */
        }
    #elif(PCComms_SCB_MODE_I2C_CONST_CFG)
        PCComms_I2CCyBtldrCommStart();

    #elif(PCComms_SCB_MODE_SPI_CONST_CFG)
        PCComms_SpiCyBtldrCommStart();

    #elif(PCComms_SCB_MODE_UART_CONST_CFG)
        PCComms_UartCyBtldrCommStart();

    #elif(PCComms_SCB_MODE_EZI2C_CONST_CFG)
        PCComms_EzI2CCyBtldrCommStart();

    #else
        /* Unknown mode */

    #endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PCComms_CyBtldrCommStop
********************************************************************************
*
* Summary:
*  Calls the CyBtldrCommStop function of the bootloader communication
*  component for the selected mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PCComms_CyBtldrCommStop(void)
{
    #if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
        if(PCComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            PCComms_I2CCyBtldrCommStop();
        }
        else if(PCComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            PCComms_SpiCyBtldrCommStop();
        }
        else if(PCComms_SCB_MODE_UART_RUNTM_CFG)
        {
            PCComms_UartCyBtldrCommStop();
        }
        else if(PCComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            PCComms_EzI2CCyBtldrCommStop();
        }
        else
        {
            /* Unknown mode */
        }
    #elif(PCComms_SCB_MODE_I2C_CONST_CFG)
        PCComms_I2CCyBtldrCommStop();

    #elif(PCComms_SCB_MODE_SPI_CONST_CFG)
        PCComms_SpiCyBtldrCommStop();

    #elif(PCComms_SCB_MODE_UART_CONST_CFG)
        PCComms_UartCyBtldrCommStop();

    #elif(PCComms_SCB_MODE_EZI2C_CONST_CFG)
        PCComms_EzI2CCyBtldrCommStop();

    #else
        /* Unknown mode */

    #endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PCComms_CyBtldrCommReset
********************************************************************************
*
* Summary:
*  Calls the CyBtldrCommReset function of the bootloader communication
*  component for the selected mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PCComms_CyBtldrCommReset(void)
{
    #if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
        if(PCComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            PCComms_I2CCyBtldrCommReset();
        }
        else if(PCComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            PCComms_SpiCyBtldrCommReset();
        }
        else if(PCComms_SCB_MODE_UART_RUNTM_CFG)
        {
            PCComms_UartCyBtldrCommReset();
        }
        else if(PCComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            PCComms_EzI2CCyBtldrCommReset();
        }
        else
        {
            /* Unknown mode */
        }
    #elif(PCComms_SCB_MODE_I2C_CONST_CFG)
        PCComms_I2CCyBtldrCommReset();

    #elif(PCComms_SCB_MODE_SPI_CONST_CFG)
        PCComms_SpiCyBtldrCommReset();

    #elif(PCComms_SCB_MODE_UART_CONST_CFG)
        PCComms_UartCyBtldrCommReset();

    #elif(PCComms_SCB_MODE_EZI2C_CONST_CFG)
        PCComms_EzI2CCyBtldrCommReset();

    #else
        /* Unknown mode */

    #endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PCComms_CyBtldrCommRead
********************************************************************************
*
* Summary:
*  Calls the CyBtldrCommRead function of the bootloader communication
*  component for the selected mode.
*
* Parameters:
*  pData:    Pointer to storage for the block of data to be read from the
*            bootloader host
*  size:     Number of bytes to be read.
*  count:    Pointer to the variable to write the number of bytes actually
*            read.
*  timeOut:  Number of units in 10 ms to wait before returning because of a
*            timeout.
*
* Return:
*  Returns CYRET_SUCCESS if no problem was encountered or returns the value
*  that best describes the problem.
*
*******************************************************************************/
cystatus PCComms_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;

    #if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
        if(PCComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            status = PCComms_I2CCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(PCComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            status = PCComms_SpiCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(PCComms_SCB_MODE_UART_RUNTM_CFG)
        {
            status = PCComms_UartCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(PCComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            status = PCComms_EzI2CCyBtldrCommRead(pData, size, count, timeOut);
        }
        else
        {
            status = CYRET_INVALID_STATE; /* Unknown mode: return status */
        }

    #elif(PCComms_SCB_MODE_I2C_CONST_CFG)
        status = PCComms_I2CCyBtldrCommRead(pData, size, count, timeOut);

    #elif(PCComms_SCB_MODE_SPI_CONST_CFG)
        status = PCComms_SpiCyBtldrCommRead(pData, size, count, timeOut);

    #elif(PCComms_SCB_MODE_UART_CONST_CFG)
        status = PCComms_UartCyBtldrCommRead(pData, size, count, timeOut);

    #elif(PCComms_SCB_MODE_EZI2C_CONST_CFG)
        status = PCComms_EzI2CCyBtldrCommRead(pData, size, count, timeOut);

    #else
        status = CYRET_INVALID_STATE; /* Unknown mode: return status */

    #endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */

    return(status);
}


/*******************************************************************************
* Function Name: PCComms_CyBtldrCommWrite
********************************************************************************
*
* Summary:
*  Calls the CyBtldrCommWrite  function of the bootloader communication
*  component for the selected mode.
*
* Parameters:
*  pData:    Pointer to the block of data to be written to the bootloader host.
*  size:     Number of bytes to be written.
*  count:    Pointer to the variable to write the number of bytes actually
*            written.
*  timeOut:  Number of units in 10 ms to wait before returning because of a
*            timeout.
*
* Return:
*  Returns CYRET_SUCCESS if no problem was encountered or returns the value
*  that best describes the problem.
*
*******************************************************************************/
cystatus PCComms_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;

    #if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
        if(PCComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            status = PCComms_I2CCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(PCComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            status = PCComms_SpiCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(PCComms_SCB_MODE_UART_RUNTM_CFG)
        {
            status = PCComms_UartCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(PCComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            status = PCComms_EzI2CCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else
        {
            status = CYRET_INVALID_STATE; /* Unknown mode */
        }
    #elif(PCComms_SCB_MODE_I2C_CONST_CFG)
        status = PCComms_I2CCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(PCComms_SCB_MODE_SPI_CONST_CFG)
        status = PCComms_SpiCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(PCComms_SCB_MODE_UART_CONST_CFG)
        status = PCComms_UartCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(PCComms_SCB_MODE_EZI2C_CONST_CFG)
        status = PCComms_EzI2CCyBtldrCommWrite(pData, size, count, timeOut);

    #else
        status = CYRET_INVALID_STATE; /* Unknown mode */

    #endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */

    return(status);
}

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (PCComms_BTLDR_COMM_MODE_ENABLED) */


/* [] END OF FILE */
