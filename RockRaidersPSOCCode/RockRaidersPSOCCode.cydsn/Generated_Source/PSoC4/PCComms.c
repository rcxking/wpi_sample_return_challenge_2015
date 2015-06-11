/*******************************************************************************
* File Name: PCComms.c
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
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for Unconfigured mode */
#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    uint8 PCComms_scbMode = PCComms_SCB_MODE_UNCONFIG;
    uint8 PCComms_scbEnableWake;
    uint8 PCComms_scbEnableIntr;

    /* I2C configuration variables */
    uint8 PCComms_mode;
    uint8 PCComms_acceptAddr;

    /* SPI/UART configuration variables */
    volatile uint8 * PCComms_rxBuffer;
    uint8  PCComms_rxDataBits;
    uint32 PCComms_rxBufferSize;

    volatile uint8 * PCComms_txBuffer;
    uint8  PCComms_txDataBits;
    uint32 PCComms_txBufferSize;

    /* EZI2C configuration variables */
    uint8 PCComms_numberOfAddr;
    uint8 PCComms_subAddrSize;
#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/

uint8 PCComms_initVar = 0u;

#if !defined (CY_REMOVE_PCComms_CUSTOM_INTR_HANDLER)
    cyisraddress PCComms_customIntrHandler = NULL;
#endif /* !defined (CY_REMOVE_PCComms_CUSTOM_INTR_HANDLER) */


/***************************************
*    Private Function Prototypes
***************************************/

static void PCComms_ScbEnableIntr(void);
static void PCComms_ScbModeStop(void);


/*******************************************************************************
* Function Name: PCComms_Init
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
void PCComms_Init(void)
{
#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    if(PCComms_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        PCComms_initVar = 0u;
    }
    else
    {
        /* Initialization was done before this function call */
    }

#elif(PCComms_SCB_MODE_I2C_CONST_CFG)
    PCComms_I2CInit();

#elif(PCComms_SCB_MODE_SPI_CONST_CFG)
    PCComms_SpiInit();

#elif(PCComms_SCB_MODE_UART_CONST_CFG)
    PCComms_UartInit();

#elif(PCComms_SCB_MODE_EZI2C_CONST_CFG)
    PCComms_EzI2CInit();

#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PCComms_Enable
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
void PCComms_Enable(void)
{
#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if(!PCComms_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        PCComms_CTRL_REG |= PCComms_CTRL_ENABLED;

        PCComms_ScbEnableIntr();
    }
#else
    PCComms_CTRL_REG |= PCComms_CTRL_ENABLED;

    PCComms_ScbEnableIntr();
#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PCComms_Start
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
*  PCComms_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void PCComms_Start(void)
{
    if(0u == PCComms_initVar)
    {
        PCComms_Init();
        PCComms_initVar = 1u; /* Component was initialized */
    }

    PCComms_Enable();
}


/*******************************************************************************
* Function Name: PCComms_Stop
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
void PCComms_Stop(void)
{
#if(PCComms_SCB_IRQ_INTERNAL)
    PCComms_DisableInt();
#endif /* (PCComms_SCB_IRQ_INTERNAL) */

    PCComms_CTRL_REG &= (uint32) ~PCComms_CTRL_ENABLED;  /* Disable scb IP */

#if(PCComms_SCB_IRQ_INTERNAL)
    PCComms_ClearPendingInt();
#endif /* (PCComms_SCB_IRQ_INTERNAL) */

    PCComms_ScbModeStop(); /* Calls scbMode specific Stop function */
}


/*******************************************************************************
* Function Name: PCComms_SetRxFifoLevel
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
void PCComms_SetRxFifoLevel(uint32 level)
{
    uint32 rxFifoCtrl;

    rxFifoCtrl = PCComms_RX_FIFO_CTRL_REG;

    rxFifoCtrl &= ((uint32) ~PCComms_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    rxFifoCtrl |= ((uint32) (PCComms_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    PCComms_RX_FIFO_CTRL_REG = rxFifoCtrl;
}


/*******************************************************************************
* Function Name: PCComms_SetTxFifoLevel
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
void PCComms_SetTxFifoLevel(uint32 level)
{
    uint32 txFifoCtrl;

    txFifoCtrl = PCComms_TX_FIFO_CTRL_REG;

    txFifoCtrl &= ((uint32) ~PCComms_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    txFifoCtrl |= ((uint32) (PCComms_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    PCComms_TX_FIFO_CTRL_REG = txFifoCtrl;
}


/*******************************************************************************
* Function Name: PCComms_SetCustomInterruptHandler
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
void PCComms_SetCustomInterruptHandler(cyisraddress func)
{
#if !defined (CY_REMOVE_PCComms_CUSTOM_INTR_HANDLER)
    PCComms_customIntrHandler = func; /* Register interrupt handler */
#else
    if(NULL != func)
    {
        /* Suppress compiler warning */
    }
#endif /* !defined (CY_REMOVE_PCComms_CUSTOM_INTR_HANDLER) */
}


/*******************************************************************************
* Function Name: PCComms_ScbModeEnableIntr
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
static void PCComms_ScbEnableIntr(void)
{
#if(PCComms_SCB_IRQ_INTERNAL)
    #if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Enable interrupt in NVIC */
        if(0u != PCComms_scbEnableIntr)
        {
            PCComms_EnableInt();
        }
    #else
        PCComms_EnableInt();

    #endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (PCComms_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: PCComms_ScbModeStop
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
static void PCComms_ScbModeStop(void)
{
#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    if(PCComms_SCB_MODE_I2C_RUNTM_CFG)
    {
        PCComms_I2CStop();
    }
    else if (PCComms_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        PCComms_EzI2CStop();
    }
#if (!PCComms_CY_SCBIP_V1)
    else if (PCComms_SCB_MODE_UART_RUNTM_CFG)
    {
        PCComms_UartStop();
    }
#endif /* (!PCComms_CY_SCBIP_V1) */
    else
    {
        /* Do nothing for other modes */
    }
#elif(PCComms_SCB_MODE_I2C_CONST_CFG)
    PCComms_I2CStop();

#elif(PCComms_SCB_MODE_EZI2C_CONST_CFG)
    PCComms_EzI2CStop();

#elif(PCComms_SCB_MODE_UART_CONST_CFG)
    PCComms_UartStop();

#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if(PCComms_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: PCComms_SetPins
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
    void PCComms_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask)
    {
        uint32 hsiomSel [PCComms_SCB_PINS_NUMBER];
        uint32 pinsDm   [PCComms_SCB_PINS_NUMBER];

    #if (!PCComms_CY_SCBIP_V1)
        uint32 pinsInBuf = 0u;
    #endif /* (!PCComms_CY_SCBIP_V1) */

        uint32 i;

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for(i = 0u; i < PCComms_SCB_PINS_NUMBER; i++)
        {
            hsiomSel[i]  = PCComms_HSIOM_DEF_SEL;
            pinsDm[i]    = PCComms_PIN_DM_ALG_HIZ;
        }

        if((PCComms_SCB_MODE_I2C   == mode) ||
           (PCComms_SCB_MODE_EZI2C == mode))
        {
            hsiomSel[PCComms_MOSI_SCL_RX_PIN_INDEX] = PCComms_HSIOM_I2C_SEL;
            hsiomSel[PCComms_MISO_SDA_TX_PIN_INDEX] = PCComms_HSIOM_I2C_SEL;

            pinsDm[PCComms_MOSI_SCL_RX_PIN_INDEX] = PCComms_PIN_DM_OD_LO;
            pinsDm[PCComms_MISO_SDA_TX_PIN_INDEX] = PCComms_PIN_DM_OD_LO;
        }
    #if (!PCComms_CY_SCBIP_V1)
        else if(PCComms_SCB_MODE_SPI == mode)
        {
            hsiomSel[PCComms_MOSI_SCL_RX_PIN_INDEX] = PCComms_HSIOM_SPI_SEL;
            hsiomSel[PCComms_MISO_SDA_TX_PIN_INDEX] = PCComms_HSIOM_SPI_SEL;
            hsiomSel[PCComms_SCLK_PIN_INDEX]        = PCComms_HSIOM_SPI_SEL;

            if(PCComms_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[PCComms_MOSI_SCL_RX_PIN_INDEX] = PCComms_PIN_DM_DIG_HIZ;
                pinsDm[PCComms_MISO_SDA_TX_PIN_INDEX] = PCComms_PIN_DM_STRONG;
                pinsDm[PCComms_SCLK_PIN_INDEX]        = PCComms_PIN_DM_DIG_HIZ;

            #if(PCComms_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[PCComms_SS0_PIN_INDEX] = PCComms_HSIOM_SPI_SEL;
                pinsDm  [PCComms_SS0_PIN_INDEX] = PCComms_PIN_DM_DIG_HIZ;
            #endif /* (PCComms_SS1_PIN) */

            #if(PCComms_MISO_SDA_TX_PIN)
                /* Disable input buffer */
                 pinsInBuf |= PCComms_MISO_SDA_TX_PIN_MASK;
            #endif /* (PCComms_MISO_SDA_TX_PIN_PIN) */
            }
            else /* (Master) */
            {
                pinsDm[PCComms_MOSI_SCL_RX_PIN_INDEX] = PCComms_PIN_DM_STRONG;
                pinsDm[PCComms_MISO_SDA_TX_PIN_INDEX] = PCComms_PIN_DM_DIG_HIZ;
                pinsDm[PCComms_SCLK_PIN_INDEX]        = PCComms_PIN_DM_STRONG;

            #if(PCComms_SS0_PIN)
                hsiomSel [PCComms_SS0_PIN_INDEX] = PCComms_HSIOM_SPI_SEL;
                pinsDm   [PCComms_SS0_PIN_INDEX] = PCComms_PIN_DM_STRONG;
                pinsInBuf                                |= PCComms_SS0_PIN_MASK;
            #endif /* (PCComms_SS0_PIN) */

            #if(PCComms_SS1_PIN)
                hsiomSel [PCComms_SS1_PIN_INDEX] = PCComms_HSIOM_SPI_SEL;
                pinsDm   [PCComms_SS1_PIN_INDEX] = PCComms_PIN_DM_STRONG;
                pinsInBuf                                |= PCComms_SS1_PIN_MASK;
            #endif /* (PCComms_SS1_PIN) */

            #if(PCComms_SS2_PIN)
                hsiomSel [PCComms_SS2_PIN_INDEX] = PCComms_HSIOM_SPI_SEL;
                pinsDm   [PCComms_SS2_PIN_INDEX] = PCComms_PIN_DM_STRONG;
                pinsInBuf                                |= PCComms_SS2_PIN_MASK;
            #endif /* (PCComms_SS2_PIN) */

            #if(PCComms_SS3_PIN)
                hsiomSel [PCComms_SS3_PIN_INDEX] = PCComms_HSIOM_SPI_SEL;
                pinsDm   [PCComms_SS3_PIN_INDEX] = PCComms_PIN_DM_STRONG;
                pinsInBuf                                |= PCComms_SS3_PIN_MASK;
            #endif /* (PCComms_SS2_PIN) */

                /* Disable input buffers */
            #if(PCComms_MOSI_SCL_RX_PIN)
                pinsInBuf |= PCComms_MOSI_SCL_RX_PIN_MASK;
            #endif /* (PCComms_MOSI_SCL_RX_PIN) */

             #if(PCComms_MOSI_SCL_RX_WAKE_PIN)
                pinsInBuf |= PCComms_MOSI_SCL_RX_WAKE_PIN_MASK;
            #endif /* (PCComms_MOSI_SCL_RX_WAKE_PIN) */

            #if(PCComms_SCLK_PIN)
                pinsInBuf |= PCComms_SCLK_PIN_MASK;
            #endif /* (PCComms_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if(PCComms_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
                hsiomSel[PCComms_MISO_SDA_TX_PIN_INDEX] = PCComms_HSIOM_UART_SEL;
                pinsDm  [PCComms_MISO_SDA_TX_PIN_INDEX] = PCComms_PIN_DM_OD_LO;
            }
            else /* Standard or IrDA */
            {
                if(0u != (PCComms_UART_RX_PIN_ENABLE & uartEnableMask))
                {
                    hsiomSel[PCComms_MOSI_SCL_RX_PIN_INDEX] = PCComms_HSIOM_UART_SEL;
                    pinsDm  [PCComms_MOSI_SCL_RX_PIN_INDEX] = PCComms_PIN_DM_DIG_HIZ;
                }

                if(0u != (PCComms_UART_TX_PIN_ENABLE & uartEnableMask))
                {
                    hsiomSel[PCComms_MISO_SDA_TX_PIN_INDEX] = PCComms_HSIOM_UART_SEL;
                    pinsDm  [PCComms_MISO_SDA_TX_PIN_INDEX] = PCComms_PIN_DM_STRONG;

                #if(PCComms_MISO_SDA_TX_PIN)
                     pinsInBuf |= PCComms_MISO_SDA_TX_PIN_MASK;
                #endif /* (PCComms_MISO_SDA_TX_PIN_PIN) */
                }

            #if !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
                if(PCComms_UART_MODE_STD == subMode)
                {
                    if(0u != (PCComms_UART_CTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* CTS input is multiplexed with SCLK */
                        hsiomSel[PCComms_SCLK_PIN_INDEX] = PCComms_HSIOM_UART_SEL;
                        pinsDm  [PCComms_SCLK_PIN_INDEX] = PCComms_PIN_DM_DIG_HIZ;
                    }

                    if(0u != (PCComms_UART_RTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* RTS output is multiplexed with SS0 */
                        hsiomSel[PCComms_SS0_PIN_INDEX] = PCComms_HSIOM_UART_SEL;
                        pinsDm  [PCComms_SS0_PIN_INDEX] = PCComms_PIN_DM_STRONG;

                    #if(PCComms_SS0_PIN)
                        /* Disable input buffer */
                        pinsInBuf |= PCComms_SS0_PIN_MASK;
                    #endif /* (PCComms_SS0_PIN) */
                    }
                }
            #endif /* !(PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */
            }
        }
    #endif /* (!PCComms_CY_SCBIP_V1) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settings do not effect the pin output if HSIOM is other than GPIO */

    #if(PCComms_MOSI_SCL_RX_PIN)
        PCComms_SET_HSIOM_SEL(PCComms_MOSI_SCL_RX_HSIOM_REG,
                                       PCComms_MOSI_SCL_RX_HSIOM_MASK,
                                       PCComms_MOSI_SCL_RX_HSIOM_POS,
                                       hsiomSel[PCComms_MOSI_SCL_RX_PIN_INDEX]);

        PCComms_spi_mosi_i2c_scl_uart_rx_SetDriveMode((uint8) pinsDm[PCComms_MOSI_SCL_RX_PIN_INDEX]);

    #if (!PCComms_CY_SCBIP_V1)
        PCComms_SET_INP_DIS(PCComms_spi_mosi_i2c_scl_uart_rx_INP_DIS,
                                     PCComms_spi_mosi_i2c_scl_uart_rx_MASK,
                                     (0u != (pinsInBuf & PCComms_MOSI_SCL_RX_PIN_MASK)));
    #endif /* (!PCComms_CY_SCBIP_V1) */
    #endif /* (PCComms_MOSI_SCL_RX_PIN) */

    #if(PCComms_MOSI_SCL_RX_WAKE_PIN)
        PCComms_SET_HSIOM_SEL(PCComms_MOSI_SCL_RX_WAKE_HSIOM_REG,
                                       PCComms_MOSI_SCL_RX_WAKE_HSIOM_MASK,
                                       PCComms_MOSI_SCL_RX_WAKE_HSIOM_POS,
                                       hsiomSel[PCComms_MOSI_SCL_RX_WAKE_PIN_INDEX]);

        PCComms_spi_mosi_i2c_scl_uart_rx_wake_SetDriveMode((uint8)
                                                               pinsDm[PCComms_MOSI_SCL_RX_WAKE_PIN_INDEX]);

        PCComms_SET_INP_DIS(PCComms_spi_mosi_i2c_scl_uart_rx_wake_INP_DIS,
                                     PCComms_spi_mosi_i2c_scl_uart_rx_wake_MASK,
                                     (0u != (pinsInBuf & PCComms_MOSI_SCL_RX_WAKE_PIN_MASK)));

         /* Set interrupt on falling edge */
        PCComms_SET_INCFG_TYPE(PCComms_MOSI_SCL_RX_WAKE_INTCFG_REG,
                                        PCComms_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK,
                                        PCComms_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS,
                                        PCComms_INTCFG_TYPE_FALLING_EDGE);
    #endif /* (PCComms_MOSI_SCL_RX_WAKE_PIN) */

    #if(PCComms_MISO_SDA_TX_PIN)
        PCComms_SET_HSIOM_SEL(PCComms_MISO_SDA_TX_HSIOM_REG,
                                       PCComms_MISO_SDA_TX_HSIOM_MASK,
                                       PCComms_MISO_SDA_TX_HSIOM_POS,
                                       hsiomSel[PCComms_MISO_SDA_TX_PIN_INDEX]);

        PCComms_spi_miso_i2c_sda_uart_tx_SetDriveMode((uint8) pinsDm[PCComms_MISO_SDA_TX_PIN_INDEX]);

    #if (!PCComms_CY_SCBIP_V1)
        PCComms_SET_INP_DIS(PCComms_spi_miso_i2c_sda_uart_tx_INP_DIS,
                                     PCComms_spi_miso_i2c_sda_uart_tx_MASK,
                                    (0u != (pinsInBuf & PCComms_MISO_SDA_TX_PIN_MASK)));
    #endif /* (!PCComms_CY_SCBIP_V1) */
    #endif /* (PCComms_MOSI_SCL_RX_PIN) */

    #if(PCComms_SCLK_PIN)
        PCComms_SET_HSIOM_SEL(PCComms_SCLK_HSIOM_REG, PCComms_SCLK_HSIOM_MASK,
                                       PCComms_SCLK_HSIOM_POS, hsiomSel[PCComms_SCLK_PIN_INDEX]);

        PCComms_spi_sclk_SetDriveMode((uint8) pinsDm[PCComms_SCLK_PIN_INDEX]);

        PCComms_SET_INP_DIS(PCComms_spi_sclk_INP_DIS,
                                     PCComms_spi_sclk_MASK,
                                     (0u != (pinsInBuf & PCComms_SCLK_PIN_MASK)));
    #endif /* (PCComms_SCLK_PIN) */

    #if(PCComms_SS0_PIN)
        PCComms_SET_HSIOM_SEL(PCComms_SS0_HSIOM_REG, PCComms_SS0_HSIOM_MASK,
                                       PCComms_SS0_HSIOM_POS, hsiomSel[PCComms_SS0_PIN_INDEX]);

        PCComms_spi_ss0_SetDriveMode((uint8) pinsDm[PCComms_SS0_PIN_INDEX]);

        PCComms_SET_INP_DIS(PCComms_spi_ss0_INP_DIS,
                                     PCComms_spi_ss0_MASK,
                                     (0u != (pinsInBuf & PCComms_SS0_PIN_MASK)));
    #endif /* (PCComms_SS1_PIN) */

    #if(PCComms_SS1_PIN)
        PCComms_SET_HSIOM_SEL(PCComms_SS1_HSIOM_REG, PCComms_SS1_HSIOM_MASK,
                                       PCComms_SS1_HSIOM_POS, hsiomSel[PCComms_SS1_PIN_INDEX]);

        PCComms_spi_ss1_SetDriveMode((uint8) pinsDm[PCComms_SS1_PIN_INDEX]);

        PCComms_SET_INP_DIS(PCComms_spi_ss1_INP_DIS,
                                     PCComms_spi_ss1_MASK,
                                     (0u != (pinsInBuf & PCComms_SS1_PIN_MASK)));
    #endif /* (PCComms_SS1_PIN) */

    #if(PCComms_SS2_PIN)
        PCComms_SET_HSIOM_SEL(PCComms_SS2_HSIOM_REG, PCComms_SS2_HSIOM_MASK,
                                       PCComms_SS2_HSIOM_POS, hsiomSel[PCComms_SS2_PIN_INDEX]);

        PCComms_spi_ss2_SetDriveMode((uint8) pinsDm[PCComms_SS2_PIN_INDEX]);

        PCComms_SET_INP_DIS(PCComms_spi_ss2_INP_DIS,
                                     PCComms_spi_ss2_MASK,
                                     (0u != (pinsInBuf & PCComms_SS2_PIN_MASK)));
    #endif /* (PCComms_SS2_PIN) */

    #if(PCComms_SS3_PIN)
        PCComms_SET_HSIOM_SEL(PCComms_SS3_HSIOM_REG,  PCComms_SS3_HSIOM_MASK,
                                       PCComms_SS3_HSIOM_POS, hsiomSel[PCComms_SS3_PIN_INDEX]);

        PCComms_spi_ss3_SetDriveMode((uint8) pinsDm[PCComms_SS3_PIN_INDEX]);

        PCComms_SET_INP_DIS(PCComms_spi_ss3_INP_DIS,
                                     PCComms_spi_ss3_MASK,
                                     (0u != (pinsInBuf & PCComms_SS3_PIN_MASK)));
    #endif /* (PCComms_SS3_PIN) */
    }

#endif /* (PCComms_SCB_MODE_UNCONFIG_CONST_CFG) */


#if (PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1)
    /*******************************************************************************
    * Function Name: PCComms_I2CSlaveNackGeneration
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
    void PCComms_I2CSlaveNackGeneration(void)
    {
        /* Check for EC_AM toggle condition: EC_AM and clock stretching for address are enabled */
        if ((0u != (PCComms_CTRL_REG & PCComms_CTRL_EC_AM_MODE)) &&
            (0u == (PCComms_I2C_CTRL_REG & PCComms_I2C_CTRL_S_NOT_READY_ADDR_NACK)))
        {
            /* Toggle EC_AM before NACK generation */
            PCComms_CTRL_REG &= ~PCComms_CTRL_EC_AM_MODE;
            PCComms_CTRL_REG |=  PCComms_CTRL_EC_AM_MODE;
        }

        PCComms_I2C_SLAVE_CMD_REG = PCComms_I2C_SLAVE_CMD_S_NACK;
    }
#endif /* (PCComms_CY_SCBIP_V0 || PCComms_CY_SCBIP_V1) */


/* [] END OF FILE */
