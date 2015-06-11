/*******************************************************************************
* File Name: MotorComms.c
* Version 2.0
*
* Description:
*  This file provides the source code to the API for the SCB Component.
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
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for Unconfigured mode */
#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    uint8 MotorComms_scbMode = MotorComms_SCB_MODE_UNCONFIG;
    uint8 MotorComms_scbEnableWake;
    uint8 MotorComms_scbEnableIntr;

    /* I2C configuration variables */
    uint8 MotorComms_mode;
    uint8 MotorComms_acceptAddr;

    /* SPI/UART configuration variables */
    volatile uint8 * MotorComms_rxBuffer;
    uint8  MotorComms_rxDataBits;
    uint32 MotorComms_rxBufferSize;

    volatile uint8 * MotorComms_txBuffer;
    uint8  MotorComms_txDataBits;
    uint32 MotorComms_txBufferSize;

    /* EZI2C configuration variables */
    uint8 MotorComms_numberOfAddr;
    uint8 MotorComms_subAddrSize;
#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/

uint8 MotorComms_initVar = 0u;

#if !defined (CY_REMOVE_MotorComms_CUSTOM_INTR_HANDLER)
    cyisraddress MotorComms_customIntrHandler = NULL;
#endif /* !defined (CY_REMOVE_MotorComms_CUSTOM_INTR_HANDLER) */


/***************************************
*    Private Function Prototypes
***************************************/

static void MotorComms_ScbEnableIntr(void);
static void MotorComms_ScbModeStop(void);


/*******************************************************************************
* Function Name: MotorComms_Init
********************************************************************************
*
* Summary:
*  Initializes the SCB component to operate in one of the selected
*  configurations: I2C, SPI, UART or EZI2C.
*  When the configuration is set to "Unconfigured SCB", this function does
*  not do any initialization. Use mode-specific initialization APIs instead:
*  SCB_I2CInit, SCB_SpiInit, SCB_UartInit or SCB_EzI2CInit.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MotorComms_Init(void)
{
#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    if(MotorComms_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        MotorComms_initVar = 0u;
    }
    else
    {
        /* Initialization was done before this function call */
    }

#elif(MotorComms_SCB_MODE_I2C_CONST_CFG)
    MotorComms_I2CInit();

#elif(MotorComms_SCB_MODE_SPI_CONST_CFG)
    MotorComms_SpiInit();

#elif(MotorComms_SCB_MODE_UART_CONST_CFG)
    MotorComms_UartInit();

#elif(MotorComms_SCB_MODE_EZI2C_CONST_CFG)
    MotorComms_EzI2CInit();

#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: MotorComms_Enable
********************************************************************************
*
* Summary:
*  Enables the SCB component operation.
*  The SCB configuration should be not changed when the component is enabled.
*  Any configuration changes should be made after disabling the component.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MotorComms_Enable(void)
{
#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if(!MotorComms_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        MotorComms_CTRL_REG |= MotorComms_CTRL_ENABLED;

        MotorComms_ScbEnableIntr();
    }
#else
    MotorComms_CTRL_REG |= MotorComms_CTRL_ENABLED;

    MotorComms_ScbEnableIntr();
#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: MotorComms_Start
********************************************************************************
*
* Summary:
*  Invokes SCB_Init() and SCB_Enable().
*  After this function call, the component is enabled and ready for operation.
*  When configuration is set to "Unconfigured SCB", the component must first be
*  initialized to operate in one of the following configurations: I2C, SPI, UART
*  or EZ I2C. Otherwise this function does not enable the component.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  MotorComms_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void MotorComms_Start(void)
{
    if(0u == MotorComms_initVar)
    {
        MotorComms_Init();
        MotorComms_initVar = 1u; /* Component was initialized */
    }

    MotorComms_Enable();
}


/*******************************************************************************
* Function Name: MotorComms_Stop
********************************************************************************
*
* Summary:
*  Disables the SCB component and its interrupt.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MotorComms_Stop(void)
{
#if(MotorComms_SCB_IRQ_INTERNAL)
    MotorComms_DisableInt();
#endif /* (MotorComms_SCB_IRQ_INTERNAL) */

    MotorComms_CTRL_REG &= (uint32) ~MotorComms_CTRL_ENABLED;  /* Disable scb IP */

#if(MotorComms_SCB_IRQ_INTERNAL)
    MotorComms_ClearPendingInt();
#endif /* (MotorComms_SCB_IRQ_INTERNAL) */

    MotorComms_ScbModeStop(); /* Calls scbMode specific Stop function */
}


/*******************************************************************************
* Function Name: MotorComms_SetRxFifoLevel
********************************************************************************
*
* Summary:
*  Sets level in the RX FIFO to generate a RX level interrupt.
*  When the RX FIFO has more entries than the RX FIFO level an RX level
*  interrupt request is generated.
*
* Parameters:
*  level: Level in the RX FIFO to generate RX level interrupt.
*         The range of valid level values is between 0 and RX FIFO depth - 1.
*
* Return:
*  None
*
*******************************************************************************/
void MotorComms_SetRxFifoLevel(uint32 level)
{
    uint32 rxFifoCtrl;

    rxFifoCtrl = MotorComms_RX_FIFO_CTRL_REG;

    rxFifoCtrl &= ((uint32) ~MotorComms_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    rxFifoCtrl |= ((uint32) (MotorComms_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    MotorComms_RX_FIFO_CTRL_REG = rxFifoCtrl;
}


/*******************************************************************************
* Function Name: MotorComms_SetTxFifoLevel
********************************************************************************
*
* Summary:
*  Sets level in the TX FIFO to generate a TX level interrupt.
*  When the TX FIFO has more entries than the TX FIFO level an TX level
*  interrupt request is generated.
*
* Parameters:
*  level: Level in the TX FIFO to generate TX level interrupt.
*         The range of valid level values is between 0 and TX FIFO depth - 1.
*
* Return:
*  None
*
*******************************************************************************/
void MotorComms_SetTxFifoLevel(uint32 level)
{
    uint32 txFifoCtrl;

    txFifoCtrl = MotorComms_TX_FIFO_CTRL_REG;

    txFifoCtrl &= ((uint32) ~MotorComms_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    txFifoCtrl |= ((uint32) (MotorComms_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    MotorComms_TX_FIFO_CTRL_REG = txFifoCtrl;
}


/*******************************************************************************
* Function Name: MotorComms_SetCustomInterruptHandler
********************************************************************************
*
* Summary:
*  Registers a function to be called by the internal interrupt handler.
*  First the function that is registered is called, then the internal interrupt
*  handler performs any operation such as software buffer management functions
*  before the interrupt returns.  It is the user's responsibility not to break
*  the software buffer operations. Only one custom handler is supported, which
*  is the function provided by the most recent call.
*  At the initialization time no custom handler is registered.
*
* Parameters:
*  func: Pointer to the function to register.
*        The value NULL indicates to remove the current custom interrupt
*        handler.
*
* Return:
*  None
*
*******************************************************************************/
void MotorComms_SetCustomInterruptHandler(cyisraddress func)
{
#if !defined (CY_REMOVE_MotorComms_CUSTOM_INTR_HANDLER)
    MotorComms_customIntrHandler = func; /* Register interrupt handler */
#else
    if(NULL != func)
    {
        /* Suppress compiler warning */
    }
#endif /* !defined (CY_REMOVE_MotorComms_CUSTOM_INTR_HANDLER) */
}


/*******************************************************************************
* Function Name: MotorComms_ScbModeEnableIntr
********************************************************************************
*
* Summary:
*  Enables an interrupt for a specific mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void MotorComms_ScbEnableIntr(void)
{
#if(MotorComms_SCB_IRQ_INTERNAL)
    #if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Enable interrupt in NVIC */
        if(0u != MotorComms_scbEnableIntr)
        {
            MotorComms_EnableInt();
        }
    #else
        MotorComms_EnableInt();

    #endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (MotorComms_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: MotorComms_ScbModeStop
********************************************************************************
*
* Summary:
*  Calls the Stop function for a specific operation mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void MotorComms_ScbModeStop(void)
{
#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    if(MotorComms_SCB_MODE_I2C_RUNTM_CFG)
    {
        MotorComms_I2CStop();
    }
    else if (MotorComms_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        MotorComms_EzI2CStop();
    }
#if (!MotorComms_CY_SCBIP_V1)
    else if (MotorComms_SCB_MODE_UART_RUNTM_CFG)
    {
        MotorComms_UartStop();
    }
#endif /* (!MotorComms_CY_SCBIP_V1) */
    else
    {
        /* Do nothing for other modes */
    }
#elif(MotorComms_SCB_MODE_I2C_CONST_CFG)
    MotorComms_I2CStop();

#elif(MotorComms_SCB_MODE_EZI2C_CONST_CFG)
    MotorComms_EzI2CStop();

#elif(MotorComms_SCB_MODE_UART_CONST_CFG)
    MotorComms_UartStop();

#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if(MotorComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: MotorComms_SetPins
    ********************************************************************************
    *
    * Summary:
    *  Sets the pins settings accordingly to the selected operation mode.
    *  Only available in the Unconfigured operation mode. The mode specific
    *  initialization function calls it.
    *  Pins configuration is set by PSoC Creator when a specific mode of operation
    *  is selected in design time.
    *
    * Parameters:
    *  mode:      Mode of SCB operation.
    *  subMode:   Sub-mode of SCB operation. It is only required for SPI and UART
    *             modes.
    *  uartEnableMask: enables TX or RX direction and RTS and CTS signals.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MotorComms_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask)
    {
        uint32 hsiomSel [MotorComms_SCB_PINS_NUMBER];
        uint32 pinsDm   [MotorComms_SCB_PINS_NUMBER];

    #if (!MotorComms_CY_SCBIP_V1)
        uint32 pinsInBuf = 0u;
    #endif /* (!MotorComms_CY_SCBIP_V1) */

        uint32 i;

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for(i = 0u; i < MotorComms_SCB_PINS_NUMBER; i++)
        {
            hsiomSel[i]  = MotorComms_HSIOM_DEF_SEL;
            pinsDm[i]    = MotorComms_PIN_DM_ALG_HIZ;
        }

        if((MotorComms_SCB_MODE_I2C   == mode) ||
           (MotorComms_SCB_MODE_EZI2C == mode))
        {
            hsiomSel[MotorComms_MOSI_SCL_RX_PIN_INDEX] = MotorComms_HSIOM_I2C_SEL;
            hsiomSel[MotorComms_MISO_SDA_TX_PIN_INDEX] = MotorComms_HSIOM_I2C_SEL;

            pinsDm[MotorComms_MOSI_SCL_RX_PIN_INDEX] = MotorComms_PIN_DM_OD_LO;
            pinsDm[MotorComms_MISO_SDA_TX_PIN_INDEX] = MotorComms_PIN_DM_OD_LO;
        }
    #if (!MotorComms_CY_SCBIP_V1)
        else if(MotorComms_SCB_MODE_SPI == mode)
        {
            hsiomSel[MotorComms_MOSI_SCL_RX_PIN_INDEX] = MotorComms_HSIOM_SPI_SEL;
            hsiomSel[MotorComms_MISO_SDA_TX_PIN_INDEX] = MotorComms_HSIOM_SPI_SEL;
            hsiomSel[MotorComms_SCLK_PIN_INDEX]        = MotorComms_HSIOM_SPI_SEL;

            if(MotorComms_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[MotorComms_MOSI_SCL_RX_PIN_INDEX] = MotorComms_PIN_DM_DIG_HIZ;
                pinsDm[MotorComms_MISO_SDA_TX_PIN_INDEX] = MotorComms_PIN_DM_STRONG;
                pinsDm[MotorComms_SCLK_PIN_INDEX]        = MotorComms_PIN_DM_DIG_HIZ;

            #if(MotorComms_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[MotorComms_SS0_PIN_INDEX] = MotorComms_HSIOM_SPI_SEL;
                pinsDm  [MotorComms_SS0_PIN_INDEX] = MotorComms_PIN_DM_DIG_HIZ;
            #endif /* (MotorComms_SS1_PIN) */

            #if(MotorComms_MISO_SDA_TX_PIN)
                /* Disable input buffer */
                 pinsInBuf |= MotorComms_MISO_SDA_TX_PIN_MASK;
            #endif /* (MotorComms_MISO_SDA_TX_PIN_PIN) */
            }
            else /* (Master) */
            {
                pinsDm[MotorComms_MOSI_SCL_RX_PIN_INDEX] = MotorComms_PIN_DM_STRONG;
                pinsDm[MotorComms_MISO_SDA_TX_PIN_INDEX] = MotorComms_PIN_DM_DIG_HIZ;
                pinsDm[MotorComms_SCLK_PIN_INDEX]        = MotorComms_PIN_DM_STRONG;

            #if(MotorComms_SS0_PIN)
                hsiomSel [MotorComms_SS0_PIN_INDEX] = MotorComms_HSIOM_SPI_SEL;
                pinsDm   [MotorComms_SS0_PIN_INDEX] = MotorComms_PIN_DM_STRONG;
                pinsInBuf                                |= MotorComms_SS0_PIN_MASK;
            #endif /* (MotorComms_SS0_PIN) */

            #if(MotorComms_SS1_PIN)
                hsiomSel [MotorComms_SS1_PIN_INDEX] = MotorComms_HSIOM_SPI_SEL;
                pinsDm   [MotorComms_SS1_PIN_INDEX] = MotorComms_PIN_DM_STRONG;
                pinsInBuf                                |= MotorComms_SS1_PIN_MASK;
            #endif /* (MotorComms_SS1_PIN) */

            #if(MotorComms_SS2_PIN)
                hsiomSel [MotorComms_SS2_PIN_INDEX] = MotorComms_HSIOM_SPI_SEL;
                pinsDm   [MotorComms_SS2_PIN_INDEX] = MotorComms_PIN_DM_STRONG;
                pinsInBuf                                |= MotorComms_SS2_PIN_MASK;
            #endif /* (MotorComms_SS2_PIN) */

            #if(MotorComms_SS3_PIN)
                hsiomSel [MotorComms_SS3_PIN_INDEX] = MotorComms_HSIOM_SPI_SEL;
                pinsDm   [MotorComms_SS3_PIN_INDEX] = MotorComms_PIN_DM_STRONG;
                pinsInBuf                                |= MotorComms_SS3_PIN_MASK;
            #endif /* (MotorComms_SS2_PIN) */

                /* Disable input buffers */
            #if(MotorComms_MOSI_SCL_RX_PIN)
                pinsInBuf |= MotorComms_MOSI_SCL_RX_PIN_MASK;
            #endif /* (MotorComms_MOSI_SCL_RX_PIN) */

             #if(MotorComms_MOSI_SCL_RX_WAKE_PIN)
                pinsInBuf |= MotorComms_MOSI_SCL_RX_WAKE_PIN_MASK;
            #endif /* (MotorComms_MOSI_SCL_RX_WAKE_PIN) */

            #if(MotorComms_SCLK_PIN)
                pinsInBuf |= MotorComms_SCLK_PIN_MASK;
            #endif /* (MotorComms_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if(MotorComms_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
                hsiomSel[MotorComms_MISO_SDA_TX_PIN_INDEX] = MotorComms_HSIOM_UART_SEL;
                pinsDm  [MotorComms_MISO_SDA_TX_PIN_INDEX] = MotorComms_PIN_DM_OD_LO;
            }
            else /* Standard or IrDA */
            {
                if(0u != (MotorComms_UART_RX_PIN_ENABLE & uartEnableMask))
                {
                    hsiomSel[MotorComms_MOSI_SCL_RX_PIN_INDEX] = MotorComms_HSIOM_UART_SEL;
                    pinsDm  [MotorComms_MOSI_SCL_RX_PIN_INDEX] = MotorComms_PIN_DM_DIG_HIZ;
                }

                if(0u != (MotorComms_UART_TX_PIN_ENABLE & uartEnableMask))
                {
                    hsiomSel[MotorComms_MISO_SDA_TX_PIN_INDEX] = MotorComms_HSIOM_UART_SEL;
                    pinsDm  [MotorComms_MISO_SDA_TX_PIN_INDEX] = MotorComms_PIN_DM_STRONG;

                #if(MotorComms_MISO_SDA_TX_PIN)
                     pinsInBuf |= MotorComms_MISO_SDA_TX_PIN_MASK;
                #endif /* (MotorComms_MISO_SDA_TX_PIN_PIN) */
                }

            #if !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
                if(MotorComms_UART_MODE_STD == subMode)
                {
                    if(0u != (MotorComms_UART_CTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* CTS input is multiplexed with SCLK */
                        hsiomSel[MotorComms_SCLK_PIN_INDEX] = MotorComms_HSIOM_UART_SEL;
                        pinsDm  [MotorComms_SCLK_PIN_INDEX] = MotorComms_PIN_DM_DIG_HIZ;
                    }

                    if(0u != (MotorComms_UART_RTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* RTS output is multiplexed with SS0 */
                        hsiomSel[MotorComms_SS0_PIN_INDEX] = MotorComms_HSIOM_UART_SEL;
                        pinsDm  [MotorComms_SS0_PIN_INDEX] = MotorComms_PIN_DM_STRONG;

                    #if(MotorComms_SS0_PIN)
                        /* Disable input buffer */
                        pinsInBuf |= MotorComms_SS0_PIN_MASK;
                    #endif /* (MotorComms_SS0_PIN) */
                    }
                }
            #endif /* !(MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */
            }
        }
    #endif /* (!MotorComms_CY_SCBIP_V1) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settings do not effect the pin output if HSIOM is other than GPIO */

    #if(MotorComms_MOSI_SCL_RX_PIN)
        MotorComms_SET_HSIOM_SEL(MotorComms_MOSI_SCL_RX_HSIOM_REG,
                                       MotorComms_MOSI_SCL_RX_HSIOM_MASK,
                                       MotorComms_MOSI_SCL_RX_HSIOM_POS,
                                       hsiomSel[MotorComms_MOSI_SCL_RX_PIN_INDEX]);

        MotorComms_spi_mosi_i2c_scl_uart_rx_SetDriveMode((uint8) pinsDm[MotorComms_MOSI_SCL_RX_PIN_INDEX]);

    #if (!MotorComms_CY_SCBIP_V1)
        MotorComms_SET_INP_DIS(MotorComms_spi_mosi_i2c_scl_uart_rx_INP_DIS,
                                     MotorComms_spi_mosi_i2c_scl_uart_rx_MASK,
                                     (0u != (pinsInBuf & MotorComms_MOSI_SCL_RX_PIN_MASK)));
    #endif /* (!MotorComms_CY_SCBIP_V1) */
    #endif /* (MotorComms_MOSI_SCL_RX_PIN) */

    #if(MotorComms_MOSI_SCL_RX_WAKE_PIN)
        MotorComms_SET_HSIOM_SEL(MotorComms_MOSI_SCL_RX_WAKE_HSIOM_REG,
                                       MotorComms_MOSI_SCL_RX_WAKE_HSIOM_MASK,
                                       MotorComms_MOSI_SCL_RX_WAKE_HSIOM_POS,
                                       hsiomSel[MotorComms_MOSI_SCL_RX_WAKE_PIN_INDEX]);

        MotorComms_spi_mosi_i2c_scl_uart_rx_wake_SetDriveMode((uint8)
                                                               pinsDm[MotorComms_MOSI_SCL_RX_WAKE_PIN_INDEX]);

        MotorComms_SET_INP_DIS(MotorComms_spi_mosi_i2c_scl_uart_rx_wake_INP_DIS,
                                     MotorComms_spi_mosi_i2c_scl_uart_rx_wake_MASK,
                                     (0u != (pinsInBuf & MotorComms_MOSI_SCL_RX_WAKE_PIN_MASK)));

         /* Set interrupt on falling edge */
        MotorComms_SET_INCFG_TYPE(MotorComms_MOSI_SCL_RX_WAKE_INTCFG_REG,
                                        MotorComms_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK,
                                        MotorComms_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS,
                                        MotorComms_INTCFG_TYPE_FALLING_EDGE);
    #endif /* (MotorComms_MOSI_SCL_RX_WAKE_PIN) */

    #if(MotorComms_MISO_SDA_TX_PIN)
        MotorComms_SET_HSIOM_SEL(MotorComms_MISO_SDA_TX_HSIOM_REG,
                                       MotorComms_MISO_SDA_TX_HSIOM_MASK,
                                       MotorComms_MISO_SDA_TX_HSIOM_POS,
                                       hsiomSel[MotorComms_MISO_SDA_TX_PIN_INDEX]);

        MotorComms_spi_miso_i2c_sda_uart_tx_SetDriveMode((uint8) pinsDm[MotorComms_MISO_SDA_TX_PIN_INDEX]);

    #if (!MotorComms_CY_SCBIP_V1)
        MotorComms_SET_INP_DIS(MotorComms_spi_miso_i2c_sda_uart_tx_INP_DIS,
                                     MotorComms_spi_miso_i2c_sda_uart_tx_MASK,
                                    (0u != (pinsInBuf & MotorComms_MISO_SDA_TX_PIN_MASK)));
    #endif /* (!MotorComms_CY_SCBIP_V1) */
    #endif /* (MotorComms_MOSI_SCL_RX_PIN) */

    #if(MotorComms_SCLK_PIN)
        MotorComms_SET_HSIOM_SEL(MotorComms_SCLK_HSIOM_REG, MotorComms_SCLK_HSIOM_MASK,
                                       MotorComms_SCLK_HSIOM_POS, hsiomSel[MotorComms_SCLK_PIN_INDEX]);

        MotorComms_spi_sclk_SetDriveMode((uint8) pinsDm[MotorComms_SCLK_PIN_INDEX]);

        MotorComms_SET_INP_DIS(MotorComms_spi_sclk_INP_DIS,
                                     MotorComms_spi_sclk_MASK,
                                     (0u != (pinsInBuf & MotorComms_SCLK_PIN_MASK)));
    #endif /* (MotorComms_SCLK_PIN) */

    #if(MotorComms_SS0_PIN)
        MotorComms_SET_HSIOM_SEL(MotorComms_SS0_HSIOM_REG, MotorComms_SS0_HSIOM_MASK,
                                       MotorComms_SS0_HSIOM_POS, hsiomSel[MotorComms_SS0_PIN_INDEX]);

        MotorComms_spi_ss0_SetDriveMode((uint8) pinsDm[MotorComms_SS0_PIN_INDEX]);

        MotorComms_SET_INP_DIS(MotorComms_spi_ss0_INP_DIS,
                                     MotorComms_spi_ss0_MASK,
                                     (0u != (pinsInBuf & MotorComms_SS0_PIN_MASK)));
    #endif /* (MotorComms_SS1_PIN) */

    #if(MotorComms_SS1_PIN)
        MotorComms_SET_HSIOM_SEL(MotorComms_SS1_HSIOM_REG, MotorComms_SS1_HSIOM_MASK,
                                       MotorComms_SS1_HSIOM_POS, hsiomSel[MotorComms_SS1_PIN_INDEX]);

        MotorComms_spi_ss1_SetDriveMode((uint8) pinsDm[MotorComms_SS1_PIN_INDEX]);

        MotorComms_SET_INP_DIS(MotorComms_spi_ss1_INP_DIS,
                                     MotorComms_spi_ss1_MASK,
                                     (0u != (pinsInBuf & MotorComms_SS1_PIN_MASK)));
    #endif /* (MotorComms_SS1_PIN) */

    #if(MotorComms_SS2_PIN)
        MotorComms_SET_HSIOM_SEL(MotorComms_SS2_HSIOM_REG, MotorComms_SS2_HSIOM_MASK,
                                       MotorComms_SS2_HSIOM_POS, hsiomSel[MotorComms_SS2_PIN_INDEX]);

        MotorComms_spi_ss2_SetDriveMode((uint8) pinsDm[MotorComms_SS2_PIN_INDEX]);

        MotorComms_SET_INP_DIS(MotorComms_spi_ss2_INP_DIS,
                                     MotorComms_spi_ss2_MASK,
                                     (0u != (pinsInBuf & MotorComms_SS2_PIN_MASK)));
    #endif /* (MotorComms_SS2_PIN) */

    #if(MotorComms_SS3_PIN)
        MotorComms_SET_HSIOM_SEL(MotorComms_SS3_HSIOM_REG,  MotorComms_SS3_HSIOM_MASK,
                                       MotorComms_SS3_HSIOM_POS, hsiomSel[MotorComms_SS3_PIN_INDEX]);

        MotorComms_spi_ss3_SetDriveMode((uint8) pinsDm[MotorComms_SS3_PIN_INDEX]);

        MotorComms_SET_INP_DIS(MotorComms_spi_ss3_INP_DIS,
                                     MotorComms_spi_ss3_MASK,
                                     (0u != (pinsInBuf & MotorComms_SS3_PIN_MASK)));
    #endif /* (MotorComms_SS3_PIN) */
    }

#endif /* (MotorComms_SCB_MODE_UNCONFIG_CONST_CFG) */


#if (MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1)
    /*******************************************************************************
    * Function Name: MotorComms_I2CSlaveNackGeneration
    ********************************************************************************
    *
    * Summary:
    *  Sets command to generate NACK to the address or data.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MotorComms_I2CSlaveNackGeneration(void)
    {
        /* Check for EC_AM toggle condition: EC_AM and clock stretching for address are enabled */
        if ((0u != (MotorComms_CTRL_REG & MotorComms_CTRL_EC_AM_MODE)) &&
            (0u == (MotorComms_I2C_CTRL_REG & MotorComms_I2C_CTRL_S_NOT_READY_ADDR_NACK)))
        {
            /* Toggle EC_AM before NACK generation */
            MotorComms_CTRL_REG &= ~MotorComms_CTRL_EC_AM_MODE;
            MotorComms_CTRL_REG |=  MotorComms_CTRL_EC_AM_MODE;
        }

        MotorComms_I2C_SLAVE_CMD_REG = MotorComms_I2C_SLAVE_CMD_S_NACK;
    }
#endif /* (MotorComms_CY_SCBIP_V0 || MotorComms_CY_SCBIP_V1) */


/* [] END OF FILE */
