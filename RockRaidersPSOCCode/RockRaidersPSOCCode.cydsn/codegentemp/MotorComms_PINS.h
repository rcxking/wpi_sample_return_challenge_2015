/*******************************************************************************
* File Name: MotorComms_PINS.h
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

#if !defined(CY_SCB_PINS_MotorComms_H)
#define CY_SCB_PINS_MotorComms_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define MotorComms_REMOVE_MOSI_SCL_RX_WAKE_PIN    (1u)
#define MotorComms_REMOVE_MOSI_SCL_RX_PIN         (1u)
#define MotorComms_REMOVE_MISO_SDA_TX_PIN         (1u)
#define MotorComms_REMOVE_SCLK_PIN                (1u)
#define MotorComms_REMOVE_SS0_PIN                 (1u)
#define MotorComms_REMOVE_SS1_PIN                 (1u)
#define MotorComms_REMOVE_SS2_PIN                 (1u)
#define MotorComms_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define MotorComms_REMOVE_I2C_PINS                (1u)
#define MotorComms_REMOVE_SPI_MASTER_PINS         (1u)
#define MotorComms_REMOVE_SPI_SLAVE_PINS          (1u)
#define MotorComms_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define MotorComms_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define MotorComms_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define MotorComms_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define MotorComms_REMOVE_UART_TX_PIN             (0u)
#define MotorComms_REMOVE_UART_RX_TX_PIN          (1u)
#define MotorComms_REMOVE_UART_RX_PIN             (0u)
#define MotorComms_REMOVE_UART_RX_WAKE_PIN        (1u)
#define MotorComms_REMOVE_UART_RTS_PIN            (1u)
#define MotorComms_REMOVE_UART_CTS_PIN            (1u)

/* Unconfigured pins */
#define MotorComms_MOSI_SCL_RX_WAKE_PIN   (0u == MotorComms_REMOVE_MOSI_SCL_RX_WAKE_PIN)
#define MotorComms_MOSI_SCL_RX_PIN        (0u == MotorComms_REMOVE_MOSI_SCL_RX_PIN)
#define MotorComms_MISO_SDA_TX_PIN        (0u == MotorComms_REMOVE_MISO_SDA_TX_PIN)
#define MotorComms_SCLK_PIN               (0u == MotorComms_REMOVE_SCLK_PIN)
#define MotorComms_SS0_PIN                (0u == MotorComms_REMOVE_SS0_PIN)
#define MotorComms_SS1_PIN                (0u == MotorComms_REMOVE_SS1_PIN)
#define MotorComms_SS2_PIN                (0u == MotorComms_REMOVE_SS2_PIN)
#define MotorComms_SS3_PIN                (0u == MotorComms_REMOVE_SS3_PIN)

/* Mode defined pins */
#define MotorComms_I2C_PINS               (0u == MotorComms_REMOVE_I2C_PINS)
#define MotorComms_SPI_MASTER_PINS        (0u == MotorComms_REMOVE_SPI_MASTER_PINS)
#define MotorComms_SPI_SLAVE_PINS         (0u == MotorComms_REMOVE_SPI_SLAVE_PINS)
#define MotorComms_SPI_MASTER_SS0_PIN     (0u == MotorComms_REMOVE_SPI_MASTER_SS0_PIN)
#define MotorComms_SPI_MASTER_SS1_PIN     (0u == MotorComms_REMOVE_SPI_MASTER_SS1_PIN)
#define MotorComms_SPI_MASTER_SS2_PIN     (0u == MotorComms_REMOVE_SPI_MASTER_SS2_PIN)
#define MotorComms_SPI_MASTER_SS3_PIN     (0u == MotorComms_REMOVE_SPI_MASTER_SS3_PIN)
#define MotorComms_UART_TX_PIN            (0u == MotorComms_REMOVE_UART_TX_PIN)
#define MotorComms_UART_RX_TX_PIN         (0u == MotorComms_REMOVE_UART_RX_TX_PIN)
#define MotorComms_UART_RX_PIN            (0u == MotorComms_REMOVE_UART_RX_PIN)
#define MotorComms_UART_RX_WAKE_PIN       (0u == MotorComms_REMOVE_UART_RX_WAKE_PIN)
#define MotorComms_UART_RTS_PIN           (0u == MotorComms_REMOVE_UART_RTS_PIN)
#define MotorComms_UART_CTS_PIN           (0u == MotorComms_REMOVE_UART_CTS_PIN)


/***************************************
*             Includes
****************************************/

#if(MotorComms_MOSI_SCL_RX_WAKE_PIN)
    #include "MotorComms_spi_mosi_i2c_scl_uart_rx_wake.h"
#endif /* (MotorComms_MOSI_SCL_RX_WAKE_PIN) */

#if(MotorComms_MOSI_SCL_RX_PIN)
    #include "MotorComms_spi_mosi_i2c_scl_uart_rx.h"
#endif /* (MotorComms_MOSI_SCL_RX_PIN) */

#if(MotorComms_MISO_SDA_TX_PIN)
    #include "MotorComms_spi_miso_i2c_sda_uart_tx.h"
#endif /* (MotorComms_MISO_SDA_TX_PIN_PIN) */

#if(MotorComms_SCLK_PIN)
    #include "MotorComms_spi_sclk.h"
#endif /* (MotorComms_SCLK_PIN) */

#if(MotorComms_SS0_PIN)
    #include "MotorComms_spi_ss0.h"
#endif /* (MotorComms_SS1_PIN) */

#if(MotorComms_SS1_PIN)
    #include "MotorComms_spi_ss1.h"
#endif /* (MotorComms_SS1_PIN) */

#if(MotorComms_SS2_PIN)
    #include "MotorComms_spi_ss2.h"
#endif /* (MotorComms_SS2_PIN) */

#if(MotorComms_SS3_PIN)
    #include "MotorComms_spi_ss3.h"
#endif /* (MotorComms_SS3_PIN) */

#if(MotorComms_I2C_PINS)
    #include "MotorComms_scl.h"
    #include "MotorComms_sda.h"
#endif /* (MotorComms_I2C_PINS) */

#if(MotorComms_SPI_MASTER_PINS)
    #include "MotorComms_sclk_m.h"
    #include "MotorComms_mosi_m.h"
    #include "MotorComms_miso_m.h"
#endif /* (MotorComms_SPI_MASTER_PINS) */

#if(MotorComms_SPI_SLAVE_PINS)
    #include "MotorComms_sclk_s.h"
    #include "MotorComms_mosi_s.h"
    #include "MotorComms_miso_s.h"
    #include "MotorComms_ss_s.h"
#endif /* (MotorComms_SPI_SLAVE_PINS) */

#if(MotorComms_SPI_MASTER_SS0_PIN)
    #include "MotorComms_ss0_m.h"
#endif /* (MotorComms_SPI_MASTER_SS0_PIN) */

#if(MotorComms_SPI_MASTER_SS1_PIN)
    #include "MotorComms_ss1_m.h"
#endif /* (MotorComms_SPI_MASTER_SS1_PIN) */

#if(MotorComms_SPI_MASTER_SS2_PIN)
    #include "MotorComms_ss2_m.h"
#endif /* (MotorComms_SPI_MASTER_SS2_PIN) */

#if(MotorComms_SPI_MASTER_SS3_PIN)
    #include "MotorComms_ss3_m.h"
#endif /* (MotorComms_SPI_MASTER_SS3_PIN) */

#if(MotorComms_UART_TX_PIN)
    #include "MotorComms_tx.h"
#endif /* (MotorComms_UART_TX_PIN) */

#if(MotorComms_UART_RX_TX_PIN)
    #include "MotorComms_rx_tx.h"
#endif /* (MotorComms_UART_RX_TX_PIN) */

#if(MotorComms_UART_RX_PIN)
    #include "MotorComms_rx.h"
#endif /* (MotorComms_UART_RX_PIN) */

#if(MotorComms_UART_RX_WAKE_PIN)
    #include "MotorComms_rx_wake.h"
#endif /* (MotorComms_UART_RX_WAKE_PIN) */

#if(MotorComms_UART_RTS_PIN)
    #include "MotorComms_rts.h"
#endif /* (MotorComms_UART_RTS_PIN) */

#if(MotorComms_UART_CTS_PIN)
    #include "MotorComms_cts.h"
#endif /* (MotorComms_UART_CTS_PIN) */


/***************************************
*              Registers
***************************************/

#if(MotorComms_MOSI_SCL_RX_WAKE_PIN)
    #define MotorComms_MOSI_SCL_RX_WAKE_HSIOM_REG  \
                                                (*(reg32 *) MotorComms_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define MotorComms_MOSI_SCL_RX_WAKE_HSIOM_PTR  \
                                                ( (reg32 *) MotorComms_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define MotorComms_MOSI_SCL_RX_WAKE_HSIOM_MASK \
                                                (MotorComms_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_MASK)
    #define MotorComms_MOSI_SCL_RX_WAKE_HSIOM_POS  \
                                                (MotorComms_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_SHIFT)

    #define MotorComms_MOSI_SCL_RX_WAKE_INTCFG_REG    (*(reg32 *) \
                                                              MotorComms_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)
    #define MotorComms_MOSI_SCL_RX_WAKE_INTCFG_PTR    ( (reg32 *) \
                                                              MotorComms_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)

    #define MotorComms_INTCFG_TYPE_MASK                  (0x03u)
    #define MotorComms_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS  (MotorComms_spi_mosi_i2c_scl_uart_rx_wake__SHIFT)
    #define MotorComms_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK ((uint32)                                           \
                                                                    ((uint32) MotorComms_INTCFG_TYPE_MASK << \
                                                                    MotorComms_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS))
#endif /* (MotorComms_MOSI_SCL_RX_WAKE_PIN) */

#if(MotorComms_MOSI_SCL_RX_PIN)
    #define MotorComms_MOSI_SCL_RX_HSIOM_REG      (*(reg32 *) MotorComms_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define MotorComms_MOSI_SCL_RX_HSIOM_PTR      ( (reg32 *) MotorComms_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define MotorComms_MOSI_SCL_RX_HSIOM_MASK     (MotorComms_spi_mosi_i2c_scl_uart_rx__0__HSIOM_MASK)
    #define MotorComms_MOSI_SCL_RX_HSIOM_POS      (MotorComms_spi_mosi_i2c_scl_uart_rx__0__HSIOM_SHIFT)
#endif /* (MotorComms_MOSI_SCL_RX_PIN) */

#if(MotorComms_MISO_SDA_TX_PIN)
    #define MotorComms_MISO_SDA_TX_HSIOM_REG      (*(reg32 *) MotorComms_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define MotorComms_MISO_SDA_TX_HSIOM_PTR      ( (reg32 *) MotorComms_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define MotorComms_MISO_SDA_TX_HSIOM_MASK     (MotorComms_spi_miso_i2c_sda_uart_tx__0__HSIOM_MASK)
    #define MotorComms_MISO_SDA_TX_HSIOM_POS      (MotorComms_spi_miso_i2c_sda_uart_tx__0__HSIOM_SHIFT)
#endif /* (MotorComms_MISO_SDA_TX_PIN_PIN) */

#if(MotorComms_SCLK_PIN)
    #define MotorComms_SCLK_HSIOM_REG     (*(reg32 *) MotorComms_spi_sclk__0__HSIOM)
    #define MotorComms_SCLK_HSIOM_PTR     ( (reg32 *) MotorComms_spi_sclk__0__HSIOM)
    #define MotorComms_SCLK_HSIOM_MASK    (MotorComms_spi_sclk__0__HSIOM_MASK)
    #define MotorComms_SCLK_HSIOM_POS     (MotorComms_spi_sclk__0__HSIOM_SHIFT)
#endif /* (MotorComms_SCLK_PIN) */

#if(MotorComms_SS0_PIN)
    #define MotorComms_SS0_HSIOM_REG      (*(reg32 *) MotorComms_spi_ss0__0__HSIOM)
    #define MotorComms_SS0_HSIOM_PTR      ( (reg32 *) MotorComms_spi_ss0__0__HSIOM)
    #define MotorComms_SS0_HSIOM_MASK     (MotorComms_spi_ss0__0__HSIOM_MASK)
    #define MotorComms_SS0_HSIOM_POS      (MotorComms_spi_ss0__0__HSIOM_SHIFT)
#endif /* (MotorComms_SS1_PIN) */

#if(MotorComms_SS1_PIN)
    #define MotorComms_SS1_HSIOM_REG      (*(reg32 *) MotorComms_spi_ss1__0__HSIOM)
    #define MotorComms_SS1_HSIOM_PTR      ( (reg32 *) MotorComms_spi_ss1__0__HSIOM)
    #define MotorComms_SS1_HSIOM_MASK     (MotorComms_spi_ss1__0__HSIOM_MASK)
    #define MotorComms_SS1_HSIOM_POS      (MotorComms_spi_ss1__0__HSIOM_SHIFT)
#endif /* (MotorComms_SS1_PIN) */

#if(MotorComms_SS2_PIN)
    #define MotorComms_SS2_HSIOM_REG     (*(reg32 *) MotorComms_spi_ss2__0__HSIOM)
    #define MotorComms_SS2_HSIOM_PTR     ( (reg32 *) MotorComms_spi_ss2__0__HSIOM)
    #define MotorComms_SS2_HSIOM_MASK    (MotorComms_spi_ss2__0__HSIOM_MASK)
    #define MotorComms_SS2_HSIOM_POS     (MotorComms_spi_ss2__0__HSIOM_SHIFT)
#endif /* (MotorComms_SS2_PIN) */

#if(MotorComms_SS3_PIN)
    #define MotorComms_SS3_HSIOM_REG     (*(reg32 *) MotorComms_spi_ss3__0__HSIOM)
    #define MotorComms_SS3_HSIOM_PTR     ( (reg32 *) MotorComms_spi_ss3__0__HSIOM)
    #define MotorComms_SS3_HSIOM_MASK    (MotorComms_spi_ss3__0__HSIOM_MASK)
    #define MotorComms_SS3_HSIOM_POS     (MotorComms_spi_ss3__0__HSIOM_SHIFT)
#endif /* (MotorComms_SS3_PIN) */

#if(MotorComms_I2C_PINS)
    #define MotorComms_SCL_HSIOM_REG     (*(reg32 *) MotorComms_scl__0__HSIOM)
    #define MotorComms_SCL_HSIOM_PTR     ( (reg32 *) MotorComms_scl__0__HSIOM)
    #define MotorComms_SCL_HSIOM_MASK    (MotorComms_scl__0__HSIOM_MASK)
    #define MotorComms_SCL_HSIOM_POS     (MotorComms_scl__0__HSIOM_SHIFT)

    #define MotorComms_SDA_HSIOM_REG     (*(reg32 *) MotorComms_sda__0__HSIOM)
    #define MotorComms_SDA_HSIOM_PTR     ( (reg32 *) MotorComms_sda__0__HSIOM)
    #define MotorComms_SDA_HSIOM_MASK    (MotorComms_sda__0__HSIOM_MASK)
    #define MotorComms_SDA_HSIOM_POS     (MotorComms_sda__0__HSIOM_SHIFT)
#endif /* (MotorComms_I2C_PINS) */


/***************************************
*        Registers Constants
***************************************/

/* Pins constants */
#define MotorComms_HSIOM_DEF_SEL      (0x00u)
#define MotorComms_HSIOM_GPIO_SEL     (0x00u)
#define MotorComms_HSIOM_UART_SEL     (0x09u)
#define MotorComms_HSIOM_I2C_SEL      (0x0Eu)
#define MotorComms_HSIOM_SPI_SEL      (0x0Fu)

#define MotorComms_MOSI_SCL_RX_PIN_INDEX      (0u) /* RX pins without interrupt */
#define MotorComms_MOSI_SCL_RX_WAKE_PIN_INDEX (0u) /* RX pin with interrupt     */
#define MotorComms_MISO_SDA_TX_PIN_INDEX      (1u)
#define MotorComms_SCLK_PIN_INDEX             (2u)
#define MotorComms_SS0_PIN_INDEX              (3u)
#define MotorComms_SS1_PIN_INDEX              (4u)
#define MotorComms_SS2_PIN_INDEX              (5u)
#define MotorComms_SS3_PIN_INDEX              (6u)

#define MotorComms_MOSI_SCL_RX_PIN_MASK      ((uint32) 0x01u << MotorComms_MOSI_SCL_RX_PIN_INDEX)
#define MotorComms_MOSI_SCL_RX_WAKE_PIN_MASK ((uint32) 0x01u << MotorComms_MOSI_SCL_RX_WAKE_PIN_INDEX)
#define MotorComms_MISO_SDA_TX_PIN_MASK      ((uint32) 0x01u << MotorComms_MISO_SDA_TX_PIN_INDEX)
#define MotorComms_SCLK_PIN_MASK             ((uint32) 0x01u << MotorComms_SCLK_PIN_INDEX)
#define MotorComms_SS0_PIN_MASK              ((uint32) 0x01u << MotorComms_SS0_PIN_INDEX)
#define MotorComms_SS1_PIN_MASK              ((uint32) 0x01u << MotorComms_SS1_PIN_INDEX)
#define MotorComms_SS2_PIN_MASK              ((uint32) 0x01u << MotorComms_SS2_PIN_INDEX)
#define MotorComms_SS3_PIN_MASK              ((uint32) 0x01u << MotorComms_SS3_PIN_INDEX)

#define MotorComms_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin DM defines */
#define MotorComms_PIN_DM_ALG_HIZ  (0u)
#define MotorComms_PIN_DM_DIG_HIZ  (1u)
#define MotorComms_PIN_DM_OD_LO    (4u)
#define MotorComms_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

/* Set bits-mask in register */
#define MotorComms_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

/* Set bit in the register */
#define MotorComms_SET_REGISTER_BIT(reg, mask, val) \
                    ((val) ? ((reg) |= (mask)) : ((reg) &= ((uint32) ~((uint32) (mask)))))

#define MotorComms_SET_HSIOM_SEL(reg, mask, pos, sel) MotorComms_SET_REGISTER_BITS(reg, mask, pos, sel)
#define MotorComms_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        MotorComms_SET_REGISTER_BITS(reg, mask, pos, intType)
#define MotorComms_SET_INP_DIS(reg, mask, val) MotorComms_SET_REGISTER_BIT(reg, mask, val)

/* MotorComms_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  MotorComms_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* SCB I2C: scl signal */
#if(MotorComms_I2C_PINS)
    #define MotorComms_SET_I2C_SCL_DR(val) MotorComms_scl_Write(val)

    #define MotorComms_SET_I2C_SCL_HSIOM_SEL(sel) \
                          MotorComms_SET_HSIOM_SEL(MotorComms_SCL_HSIOM_REG,  \
                                                         MotorComms_SCL_HSIOM_MASK, \
                                                         MotorComms_SCL_HSIOM_POS,  \
                                                         (sel))
    #define MotorComms_WAIT_SCL_SET_HIGH  (0u == MotorComms_scl_Read())

/* Unconfigured SCB: scl signal */
#elif(MotorComms_MOSI_SCL_RX_WAKE_PIN)
    #define MotorComms_SET_I2C_SCL_DR(val) \
                            MotorComms_spi_mosi_i2c_scl_uart_rx_wake_Write(val)

    #define MotorComms_SET_I2C_SCL_HSIOM_SEL(sel) \
                    MotorComms_SET_HSIOM_SEL(MotorComms_MOSI_SCL_RX_WAKE_HSIOM_REG,  \
                                                   MotorComms_MOSI_SCL_RX_WAKE_HSIOM_MASK, \
                                                   MotorComms_MOSI_SCL_RX_WAKE_HSIOM_POS,  \
                                                   (sel))

    #define MotorComms_WAIT_SCL_SET_HIGH  (0u == MotorComms_spi_mosi_i2c_scl_uart_rx_wake_Read())

#elif(MotorComms_MOSI_SCL_RX_PIN)
    #define MotorComms_SET_I2C_SCL_DR(val) \
                            MotorComms_spi_mosi_i2c_scl_uart_rx_Write(val)


    #define MotorComms_SET_I2C_SCL_HSIOM_SEL(sel) \
                            MotorComms_SET_HSIOM_SEL(MotorComms_MOSI_SCL_RX_HSIOM_REG,  \
                                                           MotorComms_MOSI_SCL_RX_HSIOM_MASK, \
                                                           MotorComms_MOSI_SCL_RX_HSIOM_POS,  \
                                                           (sel))

    #define MotorComms_WAIT_SCL_SET_HIGH  (0u == MotorComms_spi_mosi_i2c_scl_uart_rx_Read())

#else
    #define MotorComms_SET_I2C_SCL_DR(val) \
                                                    do{ /* Does nothing */ }while(0)
    #define MotorComms_SET_I2C_SCL_HSIOM_SEL(sel) \
                                                    do{ /* Does nothing */ }while(0)

    #define MotorComms_WAIT_SCL_SET_HIGH  (0u)
#endif /* (MotorComms_MOSI_SCL_RX_PIN) */

/* SCB I2C: sda signal */
#if(MotorComms_I2C_PINS)
    #define MotorComms_WAIT_SDA_SET_HIGH  (0u == MotorComms_sda_Read())

/* Unconfigured SCB: sda signal */
#elif(MotorComms_MISO_SDA_TX_PIN)
    #define MotorComms_WAIT_SDA_SET_HIGH  (0u == MotorComms_spi_miso_i2c_sda_uart_tx_Read())

#else
    #define MotorComms_WAIT_SDA_SET_HIGH  (0u)
#endif /* (MotorComms_MOSI_SCL_RX_PIN) */

#endif /* (CY_SCB_PINS_MotorComms_H) */


/* [] END OF FILE */
