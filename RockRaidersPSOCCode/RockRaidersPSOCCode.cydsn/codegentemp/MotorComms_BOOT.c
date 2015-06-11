/*******************************************************************************
* File Name: MotorComms_BOOT.c
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

#include "MotorComms_BOOT.h"


#if defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_BTLDR_COMM_MODE_ENABLED)

/*******************************************************************************
* Function Name: MotorComms_CyBtldrCommStart
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
void MotorComms_CyBtldrCommStart(void)
{
    #if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
        if(MotorComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            MotorComms_I2CCyBtldrCommStart();
        }
        else if(MotorComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            MotorComms_SpiCyBtldrCommStart();
        }
        else if(MotorComms_SCB_MODE_UART_RUNTM_CFG)
        {
            MotorComms_UartCyBtldrCommStart();
        }
        else if(MotorComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
             MotorComms_EzI2CCyBtldrCommStart();
        }
        else
        {
            /* Unknown mode */
        }
    #elif(MotorComms_SCB_MODE_I2C_CONST_CFG)
        MotorComms_I2CCyBtldrCommStart();

    #elif(MotorComms_SCB_MODE_SPI_CONST_CFG)
        MotorComms_SpiCyBtldrCommStart();

    #elif(MotorComms_SCB_MODE_UART_CONST_CFG)
        MotorComms_UartCyBtldrCommStart();

    #elif(MotorComms_SCB_MODE_EZI2C_CONST_CFG)
        MotorComms_EzI2CCyBtldrCommStart();

    #else
        /* Unknown mode */

    #endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: MotorComms_CyBtldrCommStop
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
void MotorComms_CyBtldrCommStop(void)
{
    #if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
        if(MotorComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            MotorComms_I2CCyBtldrCommStop();
        }
        else if(MotorComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            MotorComms_SpiCyBtldrCommStop();
        }
        else if(MotorComms_SCB_MODE_UART_RUNTM_CFG)
        {
            MotorComms_UartCyBtldrCommStop();
        }
        else if(MotorComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            MotorComms_EzI2CCyBtldrCommStop();
        }
        else
        {
            /* Unknown mode */
        }
    #elif(MotorComms_SCB_MODE_I2C_CONST_CFG)
        MotorComms_I2CCyBtldrCommStop();

    #elif(MotorComms_SCB_MODE_SPI_CONST_CFG)
        MotorComms_SpiCyBtldrCommStop();

    #elif(MotorComms_SCB_MODE_UART_CONST_CFG)
        MotorComms_UartCyBtldrCommStop();

    #elif(MotorComms_SCB_MODE_EZI2C_CONST_CFG)
        MotorComms_EzI2CCyBtldrCommStop();

    #else
        /* Unknown mode */

    #endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: MotorComms_CyBtldrCommReset
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
void MotorComms_CyBtldrCommReset(void)
{
    #if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
        if(MotorComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            MotorComms_I2CCyBtldrCommReset();
        }
        else if(MotorComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            MotorComms_SpiCyBtldrCommReset();
        }
        else if(MotorComms_SCB_MODE_UART_RUNTM_CFG)
        {
            MotorComms_UartCyBtldrCommReset();
        }
        else if(MotorComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            MotorComms_EzI2CCyBtldrCommReset();
        }
        else
        {
            /* Unknown mode */
        }
    #elif(MotorComms_SCB_MODE_I2C_CONST_CFG)
        MotorComms_I2CCyBtldrCommReset();

    #elif(MotorComms_SCB_MODE_SPI_CONST_CFG)
        MotorComms_SpiCyBtldrCommReset();

    #elif(MotorComms_SCB_MODE_UART_CONST_CFG)
        MotorComms_UartCyBtldrCommReset();

    #elif(MotorComms_SCB_MODE_EZI2C_CONST_CFG)
        MotorComms_EzI2CCyBtldrCommReset();

    #else
        /* Unknown mode */

    #endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: MotorComms_CyBtldrCommRead
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
cystatus MotorComms_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;

    #if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
        if(MotorComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            status = MotorComms_I2CCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(MotorComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            status = MotorComms_SpiCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(MotorComms_SCB_MODE_UART_RUNTM_CFG)
        {
            status = MotorComms_UartCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(MotorComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            status = MotorComms_EzI2CCyBtldrCommRead(pData, size, count, timeOut);
        }
        else
        {
            status = CYRET_INVALID_STATE; /* Unknown mode: return status */
        }

    #elif(MotorComms_SCB_MODE_I2C_CONST_CFG)
        status = MotorComms_I2CCyBtldrCommRead(pData, size, count, timeOut);

    #elif(MotorComms_SCB_MODE_SPI_CONST_CFG)
        status = MotorComms_SpiCyBtldrCommRead(pData, size, count, timeOut);

    #elif(MotorComms_SCB_MODE_UART_CONST_CFG)
        status = MotorComms_UartCyBtldrCommRead(pData, size, count, timeOut);

    #elif(MotorComms_SCB_MODE_EZI2C_CONST_CFG)
        status = MotorComms_EzI2CCyBtldrCommRead(pData, size, count, timeOut);

    #else
        status = CYRET_INVALID_STATE; /* Unknown mode: return status */

    #endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */

    return(status);
}


/*******************************************************************************
* Function Name: MotorComms_CyBtldrCommWrite
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
cystatus MotorComms_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;

    #if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
        if(MotorComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            status = MotorComms_I2CCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(MotorComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            status = MotorComms_SpiCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(MotorComms_SCB_MODE_UART_RUNTM_CFG)
        {
            status = MotorComms_UartCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(MotorComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            status = MotorComms_EzI2CCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else
        {
            status = CYRET_INVALID_STATE; /* Unknown mode */
        }
    #elif(MotorComms_SCB_MODE_I2C_CONST_CFG)
        status = MotorComms_I2CCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(MotorComms_SCB_MODE_SPI_CONST_CFG)
        status = MotorComms_SpiCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(MotorComms_SCB_MODE_UART_CONST_CFG)
        status = MotorComms_UartCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(MotorComms_SCB_MODE_EZI2C_CONST_CFG)
        status = MotorComms_EzI2CCyBtldrCommWrite(pData, size, count, timeOut);

    #else
        status = CYRET_INVALID_STATE; /* Unknown mode */

    #endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */

    return(status);
}

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MotorComms_BTLDR_COMM_MODE_ENABLED) */


/* [] END OF FILE */
