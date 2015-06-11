/*******************************************************************************
* File Name: PCComms_PINS.h
* Version 2.0
*
* Description:
*  This file provides constants and parameter values for the pin components
*  buried into SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PINS_PCComms_H)
#define CY_SCB_PINS_PCComms_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define PCComms_REMOVE_MOSI_SCL_RX_WAKE_PIN    (1u)
#define PCComms_REMOVE_MOSI_SCL_RX_PIN         (1u)
#define PCComms_REMOVE_MISO_SDA_TX_PIN         (1u)
#define PCComms_REMOVE_SCLK_PIN                (1u)
#define PCComms_REMOVE_SS0_PIN                 (1u)
#define PCComms_REMOVE_SS1_PIN                 (1u)
#define PCComms_REMOVE_SS2_PIN                 (1u)
#define PCComms_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define PCComms_REMOVE_I2C_PINS                (1u)
#define PCComms_REMOVE_SPI_MASTER_PINS         (1u)
#define PCComms_REMOVE_SPI_SLAVE_PINS          (1u)
#define PCComms_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define PCComms_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define PCComms_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define PCComms_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define PCComms_REMOVE_UART_TX_PIN             (0u)
#define PCComms_REMOVE_UART_RX_TX_PIN          (1u)
#define PCComms_REMOVE_UART_RX_PIN             (0u)
#define PCComms_REMOVE_UART_RX_WAKE_PIN        (1u)
#define PCComms_REMOVE_UART_RTS_PIN            (1u)
#define PCComms_REMOVE_UART_CTS_PIN            (1u)

/* Unconfigured pins */
#define PCComms_MOSI_SCL_RX_WAKE_PIN   (0u == PCComms_REMOVE_MOSI_SCL_RX_WAKE_PIN)
#define PCComms_MOSI_SCL_RX_PIN        (0u == PCComms_REMOVE_MOSI_SCL_RX_PIN)
#define PCComms_MISO_SDA_TX_PIN        (0u == PCComms_REMOVE_MISO_SDA_TX_PIN)
#define PCComms_SCLK_PIN               (0u == PCComms_REMOVE_SCLK_PIN)
#define PCComms_SS0_PIN                (0u == PCComms_REMOVE_SS0_PIN)
#define PCComms_SS1_PIN                (0u == PCComms_REMOVE_SS1_PIN)
#define PCComms_SS2_PIN                (0u == PCComms_REMOVE_SS2_PIN)
#define PCComms_SS3_PIN                (0u == PCComms_REMOVE_SS3_PIN)

/* Mode defined pins */
#define PCComms_I2C_PINS               (0u == PCComms_REMOVE_I2C_PINS)
#define PCComms_SPI_MASTER_PINS        (0u == PCComms_REMOVE_SPI_MASTER_PINS)
#define PCComms_SPI_SLAVE_PINS         (0u == PCComms_REMOVE_SPI_SLAVE_PINS)
#define PCComms_SPI_MASTER_SS0_PIN     (0u == PCComms_REMOVE_SPI_MASTER_SS0_PIN)
#define PCComms_SPI_MASTER_SS1_PIN     (0u == PCComms_REMOVE_SPI_MASTER_SS1_PIN)
#define PCComms_SPI_MASTER_SS2_PIN     (0u == PCComms_REMOVE_SPI_MASTER_SS2_PIN)
#define PCComms_SPI_MASTER_SS3_PIN     (0u == PCComms_REMOVE_SPI_MASTER_SS3_PIN)
#define PCComms_UART_TX_PIN            (0u == PCComms_REMOVE_UART_TX_PIN)
#define PCComms_UART_RX_TX_PIN         (0u == PCComms_REMOVE_UART_RX_TX_PIN)
#define PCComms_UART_RX_PIN            (0u == PCComms_REMOVE_UART_RX_PIN)
#define PCComms_UART_RX_WAKE_PIN       (0u == PCComms_REMOVE_UART_RX_WAKE_PIN)
#define PCComms_UART_RTS_PIN           (0u == PCComms_REMOVE_UART_RTS_PIN)
#define PCComms_UART_CTS_PIN           (0u == PCComms_REMOVE_UART_CTS_PIN)


/***************************************
*             Includes
****************************************/

#if(PCComms_MOSI_SCL_RX_WAKE_PIN)
    #include "PCComms_spi_mosi_i2c_scl_uart_rx_wake.h"
#endif /* (PCComms_MOSI_SCL_RX_WAKE_PIN) */

#if(PCComms_MOSI_SCL_RX_PIN)
    #include "PCComms_spi_mosi_i2c_scl_uart_rx.h"
#endif /* (PCComms_MOSI_SCL_RX_PIN) */

#if(PCComms_MISO_SDA_TX_PIN)
    #include "PCComms_spi_miso_i2c_sda_uart_tx.h"
#endif /* (PCComms_MISO_SDA_TX_PIN_PIN) */

#if(PCComms_SCLK_PIN)
    #include "PCComms_spi_sclk.h"
#endif /* (PCComms_SCLK_PIN) */

#if(PCComms_SS0_PIN)
    #include "PCComms_spi_ss0.h"
#endif /* (PCComms_SS1_PIN) */

#if(PCComms_SS1_PIN)
    #include "PCComms_spi_ss1.h"
#endif /* (PCComms_SS1_PIN) */

#if(PCComms_SS2_PIN)
    #include "PCComms_spi_ss2.h"
#endif /* (PCComms_SS2_PIN) */

#if(PCComms_SS3_PIN)
    #include "PCComms_spi_ss3.h"
#endif /* (PCComms_SS3_PIN) */

#if(PCComms_I2C_PINS)
    #include "PCComms_scl.h"
    #include "PCComms_sda.h"
#endif /* (PCComms_I2C_PINS) */

#if(PCComms_SPI_MASTER_PINS)
    #include "PCComms_sclk_m.h"
    #include "PCComms_mosi_m.h"
    #include "PCComms_miso_m.h"
#endif /* (PCComms_SPI_MASTER_PINS) */

#if(PCComms_SPI_SLAVE_PINS)
    #include "PCComms_sclk_s.h"
    #include "PCComms_mosi_s.h"
    #include "PCComms_miso_s.h"
    #include "PCComms_ss_s.h"
#endif /* (PCComms_SPI_SLAVE_PINS) */

#if(PCComms_SPI_MASTER_SS0_PIN)
    #include "PCComms_ss0_m.h"
#endif /* (PCComms_SPI_MASTER_SS0_PIN) */

#if(PCComms_SPI_MASTER_SS1_PIN)
    #include "PCComms_ss1_m.h"
#endif /* (PCComms_SPI_MASTER_SS1_PIN) */

#if(PCComms_SPI_MASTER_SS2_PIN)
    #include "PCComms_ss2_m.h"
#endif /* (PCComms_SPI_MASTER_SS2_PIN) */

#if(PCComms_SPI_MASTER_SS3_PIN)
    #include "PCComms_ss3_m.h"
#endif /* (PCComms_SPI_MASTER_SS3_PIN) */

#if(PCComms_UART_TX_PIN)
    #include "PCComms_tx.h"
#endif /* (PCComms_UART_TX_PIN) */

#if(PCComms_UART_RX_TX_PIN)
    #include "PCComms_rx_tx.h"
#endif /* (PCComms_UART_RX_TX_PIN) */

#if(PCComms_UART_RX_PIN)
    #include "PCComms_rx.h"
#endif /* (PCComms_UART_RX_PIN) */

#if(PCComms_UART_RX_WAKE_PIN)
    #include "PCComms_rx_wake.h"
#endif /* (PCComms_UART_RX_WAKE_PIN) */

#if(PCComms_UART_RTS_PIN)
    #include "PCComms_rts.h"
#endif /* (PCComms_UART_RTS_PIN) */

#if(PCComms_UART_CTS_PIN)
    #include "PCComms_cts.h"
#endif /* (PCComms_UART_CTS_PIN) */


/***************************************
*              Registers
***************************************/

#if(PCComms_MOSI_SCL_RX_WAKE_PIN)
    #define PCComms_MOSI_SCL_RX_WAKE_HSIOM_REG  \
                                                (*(reg32 *) PCComms_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define PCComms_MOSI_SCL_RX_WAKE_HSIOM_PTR  \
                                                ( (reg32 *) PCComms_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define PCComms_MOSI_SCL_RX_WAKE_HSIOM_MASK \
                                                (PCComms_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_MASK)
    #define PCComms_MOSI_SCL_RX_WAKE_HSIOM_POS  \
                                                (PCComms_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_SHIFT)

    #define PCComms_MOSI_SCL_RX_WAKE_INTCFG_REG    (*(reg32 *) \
                                                              PCComms_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)
    #define PCComms_MOSI_SCL_RX_WAKE_INTCFG_PTR    ( (reg32 *) \
                                                              PCComms_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)

    #define PCComms_INTCFG_TYPE_MASK                  (0x03u)
    #define PCComms_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS  (PCComms_spi_mosi_i2c_scl_uart_rx_wake__SHIFT)
    #define PCComms_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK ((uint32)                                           \
                                                                    ((uint32) PCComms_INTCFG_TYPE_MASK << \
                                                                    PCComms_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS))
#endif /* (PCComms_MOSI_SCL_RX_WAKE_PIN) */

#if(PCComms_MOSI_SCL_RX_PIN)
    #define PCComms_MOSI_SCL_RX_HSIOM_REG      (*(reg32 *) PCComms_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define PCComms_MOSI_SCL_RX_HSIOM_PTR      ( (reg32 *) PCComms_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define PCComms_MOSI_SCL_RX_HSIOM_MASK     (PCComms_spi_mosi_i2c_scl_uart_rx__0__HSIOM_MASK)
    #define PCComms_MOSI_SCL_RX_HSIOM_POS      (PCComms_spi_mosi_i2c_scl_uart_rx__0__HSIOM_SHIFT)
#endif /* (PCComms_MOSI_SCL_RX_PIN) */

#if(PCComms_MISO_SDA_TX_PIN)
    #define PCComms_MISO_SDA_TX_HSIOM_REG      (*(reg32 *) PCComms_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define PCComms_MISO_SDA_TX_HSIOM_PTR      ( (reg32 *) PCComms_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define PCComms_MISO_SDA_TX_HSIOM_MASK     (PCComms_spi_miso_i2c_sda_uart_tx__0__HSIOM_MASK)
    #define PCComms_MISO_SDA_TX_HSIOM_POS      (PCComms_spi_miso_i2c_sda_uart_tx__0__HSIOM_SHIFT)
#endif /* (PCComms_MISO_SDA_TX_PIN_PIN) */

#if(PCComms_SCLK_PIN)
    #define PCComms_SCLK_HSIOM_REG     (*(reg32 *) PCComms_spi_sclk__0__HSIOM)
    #define PCComms_SCLK_HSIOM_PTR     ( (reg32 *) PCComms_spi_sclk__0__HSIOM)
    #define PCComms_SCLK_HSIOM_MASK    (PCComms_spi_sclk__0__HSIOM_MASK)
    #define PCComms_SCLK_HSIOM_POS     (PCComms_spi_sclk__0__HSIOM_SHIFT)
#endif /* (PCComms_SCLK_PIN) */

#if(PCComms_SS0_PIN)
    #define PCComms_SS0_HSIOM_REG      (*(reg32 *) PCComms_spi_ss0__0__HSIOM)
    #define PCComms_SS0_HSIOM_PTR      ( (reg32 *) PCComms_spi_ss0__0__HSIOM)
    #define PCComms_SS0_HSIOM_MASK     (PCComms_spi_ss0__0__HSIOM_MASK)
    #define PCComms_SS0_HSIOM_POS      (PCComms_spi_ss0__0__HSIOM_SHIFT)
#endif /* (PCComms_SS1_PIN) */

#if(PCComms_SS1_PIN)
    #define PCComms_SS1_HSIOM_REG      (*(reg32 *) PCComms_spi_ss1__0__HSIOM)
    #define PCComms_SS1_HSIOM_PTR      ( (reg32 *) PCComms_spi_ss1__0__HSIOM)
    #define PCComms_SS1_HSIOM_MASK     (PCComms_spi_ss1__0__HSIOM_MASK)
    #define PCComms_SS1_HSIOM_POS      (PCComms_spi_ss1__0__HSIOM_SHIFT)
#endif /* (PCComms_SS1_PIN) */

#if(PCComms_SS2_PIN)
    #define PCComms_SS2_HSIOM_REG     (*(reg32 *) PCComms_spi_ss2__0__HSIOM)
    #define PCComms_SS2_HSIOM_PTR     ( (reg32 *) PCComms_spi_ss2__0__HSIOM)
    #define PCComms_SS2_HSIOM_MASK    (PCComms_spi_ss2__0__HSIOM_MASK)
    #define PCComms_SS2_HSIOM_POS     (PCComms_spi_ss2__0__HSIOM_SHIFT)
#endif /* (PCComms_SS2_PIN) */

#if(PCComms_SS3_PIN)
    #define PCComms_SS3_HSIOM_REG     (*(reg32 *) PCComms_spi_ss3__0__HSIOM)
    #define PCComms_SS3_HSIOM_PTR     ( (reg32 *) PCComms_spi_ss3__0__HSIOM)
    #define PCComms_SS3_HSIOM_MASK    (PCComms_spi_ss3__0__HSIOM_MASK)
    #define PCComms_SS3_HSIOM_POS     (PCComms_spi_ss3__0__HSIOM_SHIFT)
#endif /* (PCComms_SS3_PIN) */

#if(PCComms_I2C_PINS)
    #define PCComms_SCL_HSIOM_REG     (*(reg32 *) PCComms_scl__0__HSIOM)
    #define PCComms_SCL_HSIOM_PTR     ( (reg32 *) PCComms_scl__0__HSIOM)
    #define PCComms_SCL_HSIOM_MASK    (PCComms_scl__0__HSIOM_MASK)
    #define PCComms_SCL_HSIOM_POS     (PCComms_scl__0__HSIOM_SHIFT)

    #define PCComms_SDA_HSIOM_REG     (*(reg32 *) PCComms_sda__0__HSIOM)
    #define PCComms_SDA_HSIOM_PTR     ( (reg32 *) PCComms_sda__0__HSIOM)
    #define PCComms_SDA_HSIOM_MASK    (PCComms_sda__0__HSIOM_MASK)
    #define PCComms_SDA_HSIOM_POS     (PCComms_sda__0__HSIOM_SHIFT)
#endif /* (PCComms_I2C_PINS) */


/***************************************
*        Registers Constants
***************************************/

/* Pins constants */
#define PCComms_HSIOM_DEF_SEL      (0x00u)
#define PCComms_HSIOM_GPIO_SEL     (0x00u)
#define PCComms_HSIOM_UART_SEL     (0x09u)
#define PCComms_HSIOM_I2C_SEL      (0x0Eu)
#define PCComms_HSIOM_SPI_SEL      (0x0Fu)

#define PCComms_MOSI_SCL_RX_PIN_INDEX      (0u) /* RX pins without interrupt */
#define PCComms_MOSI_SCL_RX_WAKE_PIN_INDEX (0u) /* RX pin with interrupt     */
#define PCComms_MISO_SDA_TX_PIN_INDEX      (1u)
#define PCComms_SCLK_PIN_INDEX             (2u)
#define PCComms_SS0_PIN_INDEX              (3u)
#define PCComms_SS1_PIN_INDEX              (4u)
#define PCComms_SS2_PIN_INDEX              (5u)
#define PCComms_SS3_PIN_INDEX              (6u)

#define PCComms_MOSI_SCL_RX_PIN_MASK      ((uint32) 0x01u << PCComms_MOSI_SCL_RX_PIN_INDEX)
#define PCComms_MOSI_SCL_RX_WAKE_PIN_MASK ((uint32) 0x01u << PCComms_MOSI_SCL_RX_WAKE_PIN_INDEX)
#define PCComms_MISO_SDA_TX_PIN_MASK      ((uint32) 0x01u << PCComms_MISO_SDA_TX_PIN_INDEX)
#define PCComms_SCLK_PIN_MASK             ((uint32) 0x01u << PCComms_SCLK_PIN_INDEX)
#define PCComms_SS0_PIN_MASK              ((uint32) 0x01u << PCComms_SS0_PIN_INDEX)
#define PCComms_SS1_PIN_MASK              ((uint32) 0x01u << PCComms_SS1_PIN_INDEX)
#define PCComms_SS2_PIN_MASK              ((uint32) 0x01u << PCComms_SS2_PIN_INDEX)
#define PCComms_SS3_PIN_MASK              ((uint32) 0x01u << PCComms_SS3_PIN_INDEX)

#define PCComms_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin DM defines */
#define PCComms_PIN_DM_ALG_HIZ  (0u)
#define PCComms_PIN_DM_DIG_HIZ  (1u)
#define PCComms_PIN_DM_OD_LO    (4u)
#define PCComms_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

/* Set bits-mask in register */
#define PCComms_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

/* Set bit in the register */
#define PCComms_SET_REGISTER_BIT(reg, mask, val) \
                    ((val) ? ((reg) |= (mask)) : ((reg) &= ((uint32) ~((uint32) (mask)))))

#define PCComms_SET_HSIOM_SEL(reg, mask, pos, sel) PCComms_SET_REGISTER_BITS(reg, mask, pos, sel)
#define PCComms_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        PCComms_SET_REGISTER_BITS(reg, mask, pos, intType)
#define PCComms_SET_INP_DIS(reg, mask, val) PCComms_SET_REGISTER_BIT(reg, mask, val)

/* PCComms_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  PCComms_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* SCB I2C: scl signal */
#if(PCComms_I2C_PINS)
    #define PCComms_SET_I2C_SCL_DR(val) PCComms_scl_Write(val)

    #define PCComms_SET_I2C_SCL_HSIOM_SEL(sel) \
                          PCComms_SET_HSIOM_SEL(PCComms_SCL_HSIOM_REG,  \
                                                         PCComms_SCL_HSIOM_MASK, \
                                                         PCComms_SCL_HSIOM_POS,  \
                                                         (sel))
    #define PCComms_WAIT_SCL_SET_HIGH  (0u == PCComms_scl_Read())

/* Unconfigured SCB: scl signal */
#elif(PCComms_MOSI_SCL_RX_WAKE_PIN)
    #define PCComms_SET_I2C_SCL_DR(val) \
                            PCComms_spi_mosi_i2c_scl_uart_rx_wake_Write(val)

    #define PCComms_SET_I2C_SCL_HSIOM_SEL(sel) \
                    PCComms_SET_HSIOM_SEL(PCComms_MOSI_SCL_RX_WAKE_HSIOM_REG,  \
                                                   PCComms_MOSI_SCL_RX_WAKE_HSIOM_MASK, \
                                                   PCComms_MOSI_SCL_RX_WAKE_HSIOM_POS,  \
                                                   (sel))

    #define PCComms_WAIT_SCL_SET_HIGH  (0u == PCComms_spi_mosi_i2c_scl_uart_rx_wake_Read())

#elif(PCComms_MOSI_SCL_RX_PIN)
    #define PCComms_SET_I2C_SCL_DR(val) \
                            PCComms_spi_mosi_i2c_scl_uart_rx_Write(val)


    #define PCComms_SET_I2C_SCL_HSIOM_SEL(sel) \
                            PCComms_SET_HSIOM_SEL(PCComms_MOSI_SCL_RX_HSIOM_REG,  \
                                                           PCComms_MOSI_SCL_RX_HSIOM_MASK, \
                                                           PCComms_MOSI_SCL_RX_HSIOM_POS,  \
                                                           (sel))

    #define PCComms_WAIT_SCL_SET_HIGH  (0u == PCComms_spi_mosi_i2c_scl_uart_rx_Read())

#else
    #define PCComms_SET_I2C_SCL_DR(val) \
                                                    do{ /* Does nothing */ }while(0)
    #define PCComms_SET_I2C_SCL_HSIOM_SEL(sel) \
                                                    do{ /* Does nothing */ }while(0)

    #define PCComms_WAIT_SCL_SET_HIGH  (0u)
#endif /* (PCComms_MOSI_SCL_RX_PIN) */

/* SCB I2C: sda signal */
#if(PCComms_I2C_PINS)
    #define PCComms_WAIT_SDA_SET_HIGH  (0u == PCComms_sda_Read())

/* Unconfigured SCB: sda signal */
#elif(PCComms_MISO_SDA_TX_PIN)
    #define PCComms_WAIT_SDA_SET_HIGH  (0u == PCComms_spi_miso_i2c_sda_uart_tx_Read())

#else
    #define PCComms_WAIT_SDA_SET_HIGH  (0u)
#endif /* (PCComms_MOSI_SCL_RX_PIN) */

#endif /* (CY_SCB_PINS_PCComms_H) */


/* [] END OF FILE */
