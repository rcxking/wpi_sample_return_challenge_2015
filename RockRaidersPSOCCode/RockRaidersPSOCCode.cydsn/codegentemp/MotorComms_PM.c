/*******************************************************************************
* File Name: MotorComms_PM.c
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

#include "MotorComms.h"
#include "MotorComms_PVT.h"

#if(MotorComms_SCB_MODE_I2C_INC)
    #include "MotorComms_I2C_PVT.h"
#endif /* (MotorComms_SCB_MODE_I2C_INC) */

#if(MotorComms_SCB_MODE_EZI2C_INC)
    #include "MotorComms_EZI2C_PVT.h"
#endif /* (MotorComms_SCB_MODE_EZI2C_INC) */

#if(MotorComms_SCB_MODE_SPI_INC || MotorComms_SCB_MODE_UART_INC)
    #include "MotorComms_SPI_UART_PVT.h"
#endif /* (MotorComms_SCB_MODE_SPI_INC || MotorComms_SCB_MODE_UART_INC) */


/***************************************
*   Backup Structure declaration
***************************************/

#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG || \
   (MotorComms_SCB_MODE_I2C_CONST_CFG   && (!MotorComms_I2C_WAKE_ENABLE_CONST))   || \
   (MotorComms_SCB_MODE_EZI2C_CONST_CFG && (!MotorComms_EZI2C_WAKE_ENABLE_CONST)) || \
   (MotorComms_SCB_MODE_SPI_CONST_CFG   && (!MotorComms_SPI_WAKE_ENABLE_CONST))   || \
   (MotorComms_SCB_MODE_UART_CONST_CFG  && (!MotorComms_UART_WAKE_ENABLE_CONST)))

    MotorComms_BACKUP_STRUCT MotorComms_backup =
    {
        0u, /* enableState */
    };
#endif


/*******************************************************************************
* Function Name: MotorComms_Sleep
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
void MotorComms_Sleep(void)
{
#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)

    if(MotorComms_SCB_WAKE_ENABLE_CHECK)
    {
        if(MotorComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            MotorComms_I2CSaveConfig();
        }
        else if(MotorComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            MotorComms_EzI2CSaveConfig();
        }
    #if(!MotorComms_CY_SCBIP_V1)
        else if(MotorComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            MotorComms_SpiSaveConfig();
        }
        else if(MotorComms_SCB_MODE_UART_RUNTM_CFG)
        {
            MotorComms_UartSaveConfig();
        }
    #endif /* (!MotorComms_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        MotorComms_backup.enableState = (uint8) MotorComms_GET_CTRL_ENABLED;

        if(0u != MotorComms_backup.enableState)
        {
            MotorComms_Stop();
        }
    }

#else

    #if (MotorComms_SCB_MODE_I2C_CONST_CFG && MotorComms_I2C_WAKE_ENABLE_CONST)
        MotorComms_I2CSaveConfig();

    #elif (MotorComms_SCB_MODE_EZI2C_CONST_CFG && MotorComms_EZI2C_WAKE_ENABLE_CONST)
        MotorComms_EzI2CSaveConfig();

    #elif (MotorComms_SCB_MODE_SPI_CONST_CFG && MotorComms_SPI_WAKE_ENABLE_CONST)
        MotorComms_SpiSaveConfig();

    #elif (MotorComms_SCB_MODE_UART_CONST_CFG && MotorComms_UART_WAKE_ENABLE_CONST)
        MotorComms_UartSaveConfig();

    #else

        MotorComms_backup.enableState = (uint8) MotorComms_GET_CTRL_ENABLED;

        if(0u != MotorComms_backup.enableState)
        {
            MotorComms_Stop();
        }

    #endif /* defined (MotorComms_SCB_MODE_I2C_CONST_CFG) && (MotorComms_I2C_WAKE_ENABLE_CONST) */

#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: MotorComms_Wakeup
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
void MotorComms_Wakeup(void)
{
#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)

    if(MotorComms_SCB_WAKE_ENABLE_CHECK)
    {
        if(MotorComms_SCB_MODE_I2C_RUNTM_CFG)
        {
            MotorComms_I2CRestoreConfig();
        }
        else if(MotorComms_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            MotorComms_EzI2CRestoreConfig();
        }
    #if(!MotorComms_CY_SCBIP_V1)
        else if(MotorComms_SCB_MODE_SPI_RUNTM_CFG)
        {
            MotorComms_SpiRestoreConfig();
        }
        else if(MotorComms_SCB_MODE_UART_RUNTM_CFG)
        {
            MotorComms_UartRestoreConfig();
        }
    #endif /* (!MotorComms_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        if(0u != MotorComms_backup.enableState)
        {
            MotorComms_Enable();
        }
    }

#else

    #if (MotorComms_SCB_MODE_I2C_CONST_CFG  && MotorComms_I2C_WAKE_ENABLE_CONST)
        MotorComms_I2CRestoreConfig();

    #elif (MotorComms_SCB_MODE_EZI2C_CONST_CFG && MotorComms_EZI2C_WAKE_ENABLE_CONST)
        MotorComms_EzI2CRestoreConfig();

    #elif (MotorComms_SCB_MODE_SPI_CONST_CFG && MotorComms_SPI_WAKE_ENABLE_CONST)
        MotorComms_SpiRestoreConfig();

    #elif (MotorComms_SCB_MODE_UART_CONST_CFG && MotorComms_UART_WAKE_ENABLE_CONST)
        MotorComms_UartRestoreConfig();

    #else

        if(0u != MotorComms_backup.enableState)
        {
            MotorComms_Enable();
        }

    #endif /* (MotorComms_I2C_WAKE_ENABLE_CONST) */

#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
