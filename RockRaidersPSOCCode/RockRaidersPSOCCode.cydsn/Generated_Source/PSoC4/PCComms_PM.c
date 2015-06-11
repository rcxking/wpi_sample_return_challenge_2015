/*******************************************************************************
* File Name: PCComms_PM.c
* Version 2.0
*
* Description:
*  This file provides the source code to the Power Management support for
*  the SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "PCComms.h"
#include "PCComms_PVT.h"

#if(PCComms_SCB_MODE_I2C_INC)
    #include "PCComms_I2C_PVT.h"
#endif /* (PCComms_SCB_MODE_I2C_INC) */

#if(PCComms_SCB_MODE_EZI2C_INC)
    #include "PCComms_EZI2C_PVT.h"
#endif /* (PCComms_SCB_MODE_EZI2C_INC) */

#if(PCComms_SCB_MODE_SPI_INC || PCComms_SCB_MODE_UART_INC)
    #include "PCComms_SPI_UART_PVT.h"
#endif /* (PCComms_SCB_MODE_SPI_INC || PCComms_SCB_MODE_UART_INC) */


/***************************************
*   Backup Structure declaration
***************************************/

#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG || \
   (PCComms_SCB_MODE_I2C_CONST_CFG   && (!PCComms_I2C_WAKE_ENABLE_CONST))   || \
   (PCComms_SCB_MODE_EZI2C_CONST_CFG && (!PCComms_EZI2C_WAKE_ENABLE_CONST)) || \
   (PCComms_SCB_MODE_SPI_CONST_CFG   && (!PCComms_SPI_WAKE_ENABLE_CONST))   || \
   (PCComms_SCB_MODE_UART_CONST_CFG  && (!PCComms_UART_WAKE_ENABLE_CONST)))

    PCComms_BACKUP_STRUCT PCComms_backup =
    {
        0u, /* enableState */
    };
#endif


/*******************************************************************************
* Function Name: PCComms_Sleep
********************************************************************************
*
* Summary:
*  Prepares the component to enter Deep Sleep.
*  The "Enable wakeup from Sleep Mode" selection has an influence on
*  this function implementation.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PCComms_Sleep(void)
{
#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)

    if(PCComms_SCB_WAKE_ENABLE_CHECK)
    {
        if(PCComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            PCComms_I2CSaveConfig();
        }
        else if(PCComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            PCComms_EzI2CSaveConfig();
        }
    #if(!PCComms_CY_SCBIP_V1)
        else if(PCComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            PCComms_SpiSaveConfig();
        }
        else if(PCComms_SCB_MODE_UART_RUNTM_CFG)
        {
            PCComms_UartSaveConfig();
        }
    #endif /* (!PCComms_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        PCComms_backup.enableState = (uint8) PCComms_GET_CTRL_ENABLED;

        if(0u != PCComms_backup.enableState)
        {
            PCComms_Stop();
        }
    }

#else

    #if (PCComms_SCB_MODE_I2C_CONST_CFG && PCComms_I2C_WAKE_ENABLE_CONST)
        PCComms_I2CSaveConfig();

    #elif (PCComms_SCB_MODE_EZI2C_CONST_CFG && PCComms_EZI2C_WAKE_ENABLE_CONST)
        PCComms_EzI2CSaveConfig();

    #elif (PCComms_SCB_MODE_SPI_CONST_CFG && PCComms_SPI_WAKE_ENABLE_CONST)
        PCComms_SpiSaveConfig();

    #elif (PCComms_SCB_MODE_UART_CONST_CFG && PCComms_UART_WAKE_ENABLE_CONST)
        PCComms_UartSaveConfig();

    #else

        PCComms_backup.enableState = (uint8) PCComms_GET_CTRL_ENABLED;

        if(0u != PCComms_backup.enableState)
        {
            PCComms_Stop();
        }

    #endif /* defined (PCComms_SCB_MODE_I2C_CONST_CFG) && (PCComms_I2C_WAKE_ENABLE_CONST) */

#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PCComms_Wakeup
********************************************************************************
*
* Summary:
*  Prepares the component for the Active mode operation after exiting
*  Deep Sleep. The "Enable wakeup from Sleep Mode" option has an influence
*  on this function implementation.
*  This function should not be called after exiting Sleep.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PCComms_Wakeup(void)
{
#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)

    if(PCComms_SCB_WAKE_ENABLE_CHECK)
    {
        if(PCComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            PCComms_I2CRestoreConfig();
        }
        else if(PCComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            PCComms_EzI2CRestoreConfig();
        }
    #if(!PCComms_CY_SCBIP_V1)
        else if(PCComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            PCComms_SpiRestoreConfig();
        }
        else if(PCComms_SCB_MODE_UART_RUNTM_CFG)
        {
            PCComms_UartRestoreConfig();
        }
    #endif /* (!PCComms_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        if(0u != PCComms_backup.enableState)
        {
            PCComms_Enable();
        }
    }

#else

    #if (PCComms_SCB_MODE_I2C_CONST_CFG  && PCComms_I2C_WAKE_ENABLE_CONST)
        PCComms_I2CRestoreConfig();

    #elif (PCComms_SCB_MODE_EZI2C_CONST_CFG && PCComms_EZI2C_WAKE_ENABLE_CONST)
        PCComms_EzI2CRestoreConfig();

    #elif (PCComms_SCB_MODE_SPI_CONST_CFG && PCComms_SPI_WAKE_ENABLE_CONST)
        PCComms_SpiRestoreConfig();

    #elif (PCComms_SCB_MODE_UART_CONST_CFG && PCComms_UART_WAKE_ENABLE_CONST)
        PCComms_UartRestoreConfig();

    #else

        if(0u != PCComms_backup.enableState)
        {
            PCComms_Enable();
        }

    #endif /* (PCComms_I2C_WAKE_ENABLE_CONST) */

#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
