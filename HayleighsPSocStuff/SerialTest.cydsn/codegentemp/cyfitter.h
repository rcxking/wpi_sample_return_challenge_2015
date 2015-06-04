#ifndef INCLUDED_CYFITTER_H
#define INCLUDED_CYFITTER_H
#include <cydevice_trm.h>

/* UART_rx */
#define UART_rx__0__DM__MASK 0x7000u
#define UART_rx__0__DM__SHIFT 12
#define UART_rx__0__DR CYREG_PRT0_DR
#define UART_rx__0__HSIOM CYREG_HSIOM_PORT_SEL0
#define UART_rx__0__HSIOM_MASK 0x000F0000u
#define UART_rx__0__HSIOM_SHIFT 16u
#define UART_rx__0__INTCFG CYREG_PRT0_INTCFG
#define UART_rx__0__INTSTAT CYREG_PRT0_INTSTAT
#define UART_rx__0__MASK 0x10u
#define UART_rx__0__PA__CFG0 CYREG_UDB_PA0_CFG0
#define UART_rx__0__PA__CFG1 CYREG_UDB_PA0_CFG1
#define UART_rx__0__PA__CFG10 CYREG_UDB_PA0_CFG10
#define UART_rx__0__PA__CFG11 CYREG_UDB_PA0_CFG11
#define UART_rx__0__PA__CFG12 CYREG_UDB_PA0_CFG12
#define UART_rx__0__PA__CFG13 CYREG_UDB_PA0_CFG13
#define UART_rx__0__PA__CFG14 CYREG_UDB_PA0_CFG14
#define UART_rx__0__PA__CFG2 CYREG_UDB_PA0_CFG2
#define UART_rx__0__PA__CFG3 CYREG_UDB_PA0_CFG3
#define UART_rx__0__PA__CFG4 CYREG_UDB_PA0_CFG4
#define UART_rx__0__PA__CFG5 CYREG_UDB_PA0_CFG5
#define UART_rx__0__PA__CFG6 CYREG_UDB_PA0_CFG6
#define UART_rx__0__PA__CFG7 CYREG_UDB_PA0_CFG7
#define UART_rx__0__PA__CFG8 CYREG_UDB_PA0_CFG8
#define UART_rx__0__PA__CFG9 CYREG_UDB_PA0_CFG9
#define UART_rx__0__PC CYREG_PRT0_PC
#define UART_rx__0__PC2 CYREG_PRT0_PC2
#define UART_rx__0__PORT 0u
#define UART_rx__0__PS CYREG_PRT0_PS
#define UART_rx__0__SHIFT 4
#define UART_rx__DR CYREG_PRT0_DR
#define UART_rx__INTCFG CYREG_PRT0_INTCFG
#define UART_rx__INTSTAT CYREG_PRT0_INTSTAT
#define UART_rx__MASK 0x10u
#define UART_rx__PA__CFG0 CYREG_UDB_PA0_CFG0
#define UART_rx__PA__CFG1 CYREG_UDB_PA0_CFG1
#define UART_rx__PA__CFG10 CYREG_UDB_PA0_CFG10
#define UART_rx__PA__CFG11 CYREG_UDB_PA0_CFG11
#define UART_rx__PA__CFG12 CYREG_UDB_PA0_CFG12
#define UART_rx__PA__CFG13 CYREG_UDB_PA0_CFG13
#define UART_rx__PA__CFG14 CYREG_UDB_PA0_CFG14
#define UART_rx__PA__CFG2 CYREG_UDB_PA0_CFG2
#define UART_rx__PA__CFG3 CYREG_UDB_PA0_CFG3
#define UART_rx__PA__CFG4 CYREG_UDB_PA0_CFG4
#define UART_rx__PA__CFG5 CYREG_UDB_PA0_CFG5
#define UART_rx__PA__CFG6 CYREG_UDB_PA0_CFG6
#define UART_rx__PA__CFG7 CYREG_UDB_PA0_CFG7
#define UART_rx__PA__CFG8 CYREG_UDB_PA0_CFG8
#define UART_rx__PA__CFG9 CYREG_UDB_PA0_CFG9
#define UART_rx__PC CYREG_PRT0_PC
#define UART_rx__PC2 CYREG_PRT0_PC2
#define UART_rx__PORT 0u
#define UART_rx__PS CYREG_PRT0_PS
#define UART_rx__SHIFT 4

/* UART_SCB */
#define UART_SCB__BIST_CONTROL CYREG_SCB1_BIST_CONTROL
#define UART_SCB__BIST_DATA CYREG_SCB1_BIST_DATA
#define UART_SCB__CTRL CYREG_SCB1_CTRL
#define UART_SCB__EZ_DATA00 CYREG_SCB1_EZ_DATA00
#define UART_SCB__EZ_DATA01 CYREG_SCB1_EZ_DATA01
#define UART_SCB__EZ_DATA02 CYREG_SCB1_EZ_DATA02
#define UART_SCB__EZ_DATA03 CYREG_SCB1_EZ_DATA03
#define UART_SCB__EZ_DATA04 CYREG_SCB1_EZ_DATA04
#define UART_SCB__EZ_DATA05 CYREG_SCB1_EZ_DATA05
#define UART_SCB__EZ_DATA06 CYREG_SCB1_EZ_DATA06
#define UART_SCB__EZ_DATA07 CYREG_SCB1_EZ_DATA07
#define UART_SCB__EZ_DATA08 CYREG_SCB1_EZ_DATA08
#define UART_SCB__EZ_DATA09 CYREG_SCB1_EZ_DATA09
#define UART_SCB__EZ_DATA10 CYREG_SCB1_EZ_DATA10
#define UART_SCB__EZ_DATA11 CYREG_SCB1_EZ_DATA11
#define UART_SCB__EZ_DATA12 CYREG_SCB1_EZ_DATA12
#define UART_SCB__EZ_DATA13 CYREG_SCB1_EZ_DATA13
#define UART_SCB__EZ_DATA14 CYREG_SCB1_EZ_DATA14
#define UART_SCB__EZ_DATA15 CYREG_SCB1_EZ_DATA15
#define UART_SCB__EZ_DATA16 CYREG_SCB1_EZ_DATA16
#define UART_SCB__EZ_DATA17 CYREG_SCB1_EZ_DATA17
#define UART_SCB__EZ_DATA18 CYREG_SCB1_EZ_DATA18
#define UART_SCB__EZ_DATA19 CYREG_SCB1_EZ_DATA19
#define UART_SCB__EZ_DATA20 CYREG_SCB1_EZ_DATA20
#define UART_SCB__EZ_DATA21 CYREG_SCB1_EZ_DATA21
#define UART_SCB__EZ_DATA22 CYREG_SCB1_EZ_DATA22
#define UART_SCB__EZ_DATA23 CYREG_SCB1_EZ_DATA23
#define UART_SCB__EZ_DATA24 CYREG_SCB1_EZ_DATA24
#define UART_SCB__EZ_DATA25 CYREG_SCB1_EZ_DATA25
#define UART_SCB__EZ_DATA26 CYREG_SCB1_EZ_DATA26
#define UART_SCB__EZ_DATA27 CYREG_SCB1_EZ_DATA27
#define UART_SCB__EZ_DATA28 CYREG_SCB1_EZ_DATA28
#define UART_SCB__EZ_DATA29 CYREG_SCB1_EZ_DATA29
#define UART_SCB__EZ_DATA30 CYREG_SCB1_EZ_DATA30
#define UART_SCB__EZ_DATA31 CYREG_SCB1_EZ_DATA31
#define UART_SCB__I2C_CFG CYREG_SCB1_I2C_CFG
#define UART_SCB__I2C_CTRL CYREG_SCB1_I2C_CTRL
#define UART_SCB__I2C_M_CMD CYREG_SCB1_I2C_M_CMD
#define UART_SCB__I2C_S_CMD CYREG_SCB1_I2C_S_CMD
#define UART_SCB__I2C_STATUS CYREG_SCB1_I2C_STATUS
#define UART_SCB__INTR_CAUSE CYREG_SCB1_INTR_CAUSE
#define UART_SCB__INTR_I2C_EC CYREG_SCB1_INTR_I2C_EC
#define UART_SCB__INTR_I2C_EC_MASK CYREG_SCB1_INTR_I2C_EC_MASK
#define UART_SCB__INTR_I2C_EC_MASKED CYREG_SCB1_INTR_I2C_EC_MASKED
#define UART_SCB__INTR_M CYREG_SCB1_INTR_M
#define UART_SCB__INTR_M_MASK CYREG_SCB1_INTR_M_MASK
#define UART_SCB__INTR_M_MASKED CYREG_SCB1_INTR_M_MASKED
#define UART_SCB__INTR_M_SET CYREG_SCB1_INTR_M_SET
#define UART_SCB__INTR_RX CYREG_SCB1_INTR_RX
#define UART_SCB__INTR_RX_MASK CYREG_SCB1_INTR_RX_MASK
#define UART_SCB__INTR_RX_MASKED CYREG_SCB1_INTR_RX_MASKED
#define UART_SCB__INTR_RX_SET CYREG_SCB1_INTR_RX_SET
#define UART_SCB__INTR_S CYREG_SCB1_INTR_S
#define UART_SCB__INTR_S_MASK CYREG_SCB1_INTR_S_MASK
#define UART_SCB__INTR_S_MASKED CYREG_SCB1_INTR_S_MASKED
#define UART_SCB__INTR_S_SET CYREG_SCB1_INTR_S_SET
#define UART_SCB__INTR_SPI_EC CYREG_SCB1_INTR_SPI_EC
#define UART_SCB__INTR_SPI_EC_MASK CYREG_SCB1_INTR_SPI_EC_MASK
#define UART_SCB__INTR_SPI_EC_MASKED CYREG_SCB1_INTR_SPI_EC_MASKED
#define UART_SCB__INTR_TX CYREG_SCB1_INTR_TX
#define UART_SCB__INTR_TX_MASK CYREG_SCB1_INTR_TX_MASK
#define UART_SCB__INTR_TX_MASKED CYREG_SCB1_INTR_TX_MASKED
#define UART_SCB__INTR_TX_SET CYREG_SCB1_INTR_TX_SET
#define UART_SCB__RX_CTRL CYREG_SCB1_RX_CTRL
#define UART_SCB__RX_FIFO_CTRL CYREG_SCB1_RX_FIFO_CTRL
#define UART_SCB__RX_FIFO_RD CYREG_SCB1_RX_FIFO_RD
#define UART_SCB__RX_FIFO_RD_SILENT CYREG_SCB1_RX_FIFO_RD_SILENT
#define UART_SCB__RX_FIFO_STATUS CYREG_SCB1_RX_FIFO_STATUS
#define UART_SCB__RX_MATCH CYREG_SCB1_RX_MATCH
#define UART_SCB__SPI_CTRL CYREG_SCB1_SPI_CTRL
#define UART_SCB__SPI_STATUS CYREG_SCB1_SPI_STATUS
#define UART_SCB__SS0_POSISTION 0u
#define UART_SCB__SS1_POSISTION 1u
#define UART_SCB__SS2_POSISTION 2u
#define UART_SCB__SS3_POSISTION 3u
#define UART_SCB__STATUS CYREG_SCB1_STATUS
#define UART_SCB__TX_CTRL CYREG_SCB1_TX_CTRL
#define UART_SCB__TX_FIFO_CTRL CYREG_SCB1_TX_FIFO_CTRL
#define UART_SCB__TX_FIFO_STATUS CYREG_SCB1_TX_FIFO_STATUS
#define UART_SCB__TX_FIFO_WR CYREG_SCB1_TX_FIFO_WR
#define UART_SCB__UART_CTRL CYREG_SCB1_UART_CTRL
#define UART_SCB__UART_RX_CTRL CYREG_SCB1_UART_RX_CTRL
#define UART_SCB__UART_RX_STATUS CYREG_SCB1_UART_RX_STATUS
#define UART_SCB__UART_TX_CTRL CYREG_SCB1_UART_TX_CTRL

/* UART_SCBCLK */
#define UART_SCBCLK__DIVIDER_MASK 0x0000FFFFu
#define UART_SCBCLK__ENABLE CYREG_CLK_DIVIDER_A00
#define UART_SCBCLK__ENABLE_MASK 0x80000000u
#define UART_SCBCLK__MASK 0x80000000u
#define UART_SCBCLK__REGISTER CYREG_CLK_DIVIDER_A00

/* UART_tx */
#define UART_tx__0__DM__MASK 0x38000u
#define UART_tx__0__DM__SHIFT 15
#define UART_tx__0__DR CYREG_PRT0_DR
#define UART_tx__0__HSIOM CYREG_HSIOM_PORT_SEL0
#define UART_tx__0__HSIOM_MASK 0x00F00000u
#define UART_tx__0__HSIOM_SHIFT 20u
#define UART_tx__0__INTCFG CYREG_PRT0_INTCFG
#define UART_tx__0__INTSTAT CYREG_PRT0_INTSTAT
#define UART_tx__0__MASK 0x20u
#define UART_tx__0__OUT_SEL CYREG_UDB_PA0_CFG10
#define UART_tx__0__OUT_SEL_SHIFT 10u
#define UART_tx__0__OUT_SEL_VAL -1u
#define UART_tx__0__PA__CFG0 CYREG_UDB_PA0_CFG0
#define UART_tx__0__PA__CFG1 CYREG_UDB_PA0_CFG1
#define UART_tx__0__PA__CFG10 CYREG_UDB_PA0_CFG10
#define UART_tx__0__PA__CFG11 CYREG_UDB_PA0_CFG11
#define UART_tx__0__PA__CFG12 CYREG_UDB_PA0_CFG12
#define UART_tx__0__PA__CFG13 CYREG_UDB_PA0_CFG13
#define UART_tx__0__PA__CFG14 CYREG_UDB_PA0_CFG14
#define UART_tx__0__PA__CFG2 CYREG_UDB_PA0_CFG2
#define UART_tx__0__PA__CFG3 CYREG_UDB_PA0_CFG3
#define UART_tx__0__PA__CFG4 CYREG_UDB_PA0_CFG4
#define UART_tx__0__PA__CFG5 CYREG_UDB_PA0_CFG5
#define UART_tx__0__PA__CFG6 CYREG_UDB_PA0_CFG6
#define UART_tx__0__PA__CFG7 CYREG_UDB_PA0_CFG7
#define UART_tx__0__PA__CFG8 CYREG_UDB_PA0_CFG8
#define UART_tx__0__PA__CFG9 CYREG_UDB_PA0_CFG9
#define UART_tx__0__PC CYREG_PRT0_PC
#define UART_tx__0__PC2 CYREG_PRT0_PC2
#define UART_tx__0__PORT 0u
#define UART_tx__0__PS CYREG_PRT0_PS
#define UART_tx__0__SHIFT 5
#define UART_tx__DR CYREG_PRT0_DR
#define UART_tx__INTCFG CYREG_PRT0_INTCFG
#define UART_tx__INTSTAT CYREG_PRT0_INTSTAT
#define UART_tx__MASK 0x20u
#define UART_tx__PA__CFG0 CYREG_UDB_PA0_CFG0
#define UART_tx__PA__CFG1 CYREG_UDB_PA0_CFG1
#define UART_tx__PA__CFG10 CYREG_UDB_PA0_CFG10
#define UART_tx__PA__CFG11 CYREG_UDB_PA0_CFG11
#define UART_tx__PA__CFG12 CYREG_UDB_PA0_CFG12
#define UART_tx__PA__CFG13 CYREG_UDB_PA0_CFG13
#define UART_tx__PA__CFG14 CYREG_UDB_PA0_CFG14
#define UART_tx__PA__CFG2 CYREG_UDB_PA0_CFG2
#define UART_tx__PA__CFG3 CYREG_UDB_PA0_CFG3
#define UART_tx__PA__CFG4 CYREG_UDB_PA0_CFG4
#define UART_tx__PA__CFG5 CYREG_UDB_PA0_CFG5
#define UART_tx__PA__CFG6 CYREG_UDB_PA0_CFG6
#define UART_tx__PA__CFG7 CYREG_UDB_PA0_CFG7
#define UART_tx__PA__CFG8 CYREG_UDB_PA0_CFG8
#define UART_tx__PA__CFG9 CYREG_UDB_PA0_CFG9
#define UART_tx__PC CYREG_PRT0_PC
#define UART_tx__PC2 CYREG_PRT0_PC2
#define UART_tx__PORT 0u
#define UART_tx__PS CYREG_PRT0_PS
#define UART_tx__SHIFT 5

/* Miscellaneous */
#define CY_VERSION "PSoC Creator  3.1 SP1"
#define CYDEV_BCLK__HFCLK__HZ 24000000U
#define CYDEV_BCLK__HFCLK__KHZ 24000U
#define CYDEV_BCLK__HFCLK__MHZ 24U
#define CYDEV_BCLK__SYSCLK__HZ 24000000U
#define CYDEV_BCLK__SYSCLK__KHZ 24000U
#define CYDEV_BCLK__SYSCLK__MHZ 24U
#define CYDEV_CHIP_DIE_LEOPARD 1u
#define CYDEV_CHIP_DIE_PANTHER 6u
#define CYDEV_CHIP_DIE_PSOC4A 3u
#define CYDEV_CHIP_DIE_PSOC5LP 5u
#define CYDEV_CHIP_DIE_UNKNOWN 0u
#define CYDEV_CHIP_FAMILY_PSOC3 1u
#define CYDEV_CHIP_FAMILY_PSOC4 2u
#define CYDEV_CHIP_FAMILY_PSOC5 3u
#define CYDEV_CHIP_FAMILY_UNKNOWN 0u
#define CYDEV_CHIP_FAMILY_USED CYDEV_CHIP_FAMILY_PSOC4
#define CYDEV_CHIP_JTAG_ID 0x04C81193u
#define CYDEV_CHIP_MEMBER_3A 1u
#define CYDEV_CHIP_MEMBER_4A 3u
#define CYDEV_CHIP_MEMBER_4D 2u
#define CYDEV_CHIP_MEMBER_4F 4u
#define CYDEV_CHIP_MEMBER_5A 6u
#define CYDEV_CHIP_MEMBER_5B 5u
#define CYDEV_CHIP_MEMBER_UNKNOWN 0u
#define CYDEV_CHIP_MEMBER_USED CYDEV_CHIP_MEMBER_4A
#define CYDEV_CHIP_DIE_EXPECT CYDEV_CHIP_MEMBER_USED
#define CYDEV_CHIP_DIE_ACTUAL CYDEV_CHIP_DIE_EXPECT
#define CYDEV_CHIP_REV_LEOPARD_ES1 0u
#define CYDEV_CHIP_REV_LEOPARD_ES2 1u
#define CYDEV_CHIP_REV_LEOPARD_ES3 3u
#define CYDEV_CHIP_REV_LEOPARD_PRODUCTION 3u
#define CYDEV_CHIP_REV_PANTHER_ES0 0u
#define CYDEV_CHIP_REV_PANTHER_ES1 1u
#define CYDEV_CHIP_REV_PANTHER_PRODUCTION 1u
#define CYDEV_CHIP_REV_PSOC4A_ES0 17u
#define CYDEV_CHIP_REV_PSOC4A_PRODUCTION 17u
#define CYDEV_CHIP_REV_PSOC5LP_ES0 0u
#define CYDEV_CHIP_REV_PSOC5LP_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_3A_ES1 0u
#define CYDEV_CHIP_REVISION_3A_ES2 1u
#define CYDEV_CHIP_REVISION_3A_ES3 3u
#define CYDEV_CHIP_REVISION_3A_PRODUCTION 3u
#define CYDEV_CHIP_REVISION_4A_ES0 17u
#define CYDEV_CHIP_REVISION_4A_PRODUCTION 17u
#define CYDEV_CHIP_REVISION_4D_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4F_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_5A_ES0 0u
#define CYDEV_CHIP_REVISION_5A_ES1 1u
#define CYDEV_CHIP_REVISION_5A_PRODUCTION 1u
#define CYDEV_CHIP_REVISION_5B_ES0 0u
#define CYDEV_CHIP_REVISION_5B_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_USED CYDEV_CHIP_REVISION_4A_PRODUCTION
#define CYDEV_CHIP_REV_EXPECT CYDEV_CHIP_REVISION_USED
#define CYDEV_CONFIG_READ_ACCELERATOR 1
#define CYDEV_CONFIG_UNUSED_IO_AllowButWarn 0
#define CYDEV_CONFIG_UNUSED_IO CYDEV_CONFIG_UNUSED_IO_AllowButWarn
#define CYDEV_CONFIG_UNUSED_IO_AllowWithInfo 1
#define CYDEV_CONFIG_UNUSED_IO_Disallowed 2
#define CYDEV_CONFIGURATION_COMPRESSED 1
#define CYDEV_CONFIGURATION_MODE_COMPRESSED 0
#define CYDEV_CONFIGURATION_MODE CYDEV_CONFIGURATION_MODE_COMPRESSED
#define CYDEV_CONFIGURATION_MODE_DMA 2
#define CYDEV_CONFIGURATION_MODE_UNCOMPRESSED 1
#define CYDEV_DEBUG_PROTECT_KILL 4
#define CYDEV_DEBUG_PROTECT_OPEN 1
#define CYDEV_DEBUG_PROTECT CYDEV_DEBUG_PROTECT_OPEN
#define CYDEV_DEBUG_PROTECT_PROTECTED 2
#define CYDEV_DEBUGGING_DPS_Disable 3
#define CYDEV_DEBUGGING_DPS_SWD 2
#define CYDEV_DEBUGGING_DPS CYDEV_DEBUGGING_DPS_SWD
#define CYDEV_DEBUGGING_ENABLE 1
#define CYDEV_HEAP_SIZE 0x80
#define CYDEV_PROJ_TYPE 2
#define CYDEV_PROJ_TYPE_BOOTLOADER 1
#define CYDEV_PROJ_TYPE_LOADABLE 2
#define CYDEV_PROJ_TYPE_MULTIAPPBOOTLOADER 3
#define CYDEV_PROJ_TYPE_STANDARD 0
#define CYDEV_STACK_SIZE 0x0400
#define CYDEV_USE_BUNDLED_CMSIS 1
#define CYDEV_VARIABLE_VDDA 1
#define CYDEV_VDDA 3.3
#define CYDEV_VDDA_MV 3300
#define CYDEV_VDDD 3.3
#define CYDEV_VDDD_MV 3300
#define CYIPBLOCK_M0S8_CTBM_VERSION 0
#define CYIPBLOCK_m0s8cpuss_VERSION 0
#define CYIPBLOCK_m0s8csd_VERSION 0
#define CYIPBLOCK_m0s8gpio2_VERSION 0
#define CYIPBLOCK_m0s8hsiom4a_VERSION 0
#define CYIPBLOCK_m0s8lcd_VERSION 0
#define CYIPBLOCK_m0s8lpcomp_VERSION 0
#define CYIPBLOCK_m0s8pclk_VERSION 0
#define CYIPBLOCK_m0s8sar_VERSION 0
#define CYIPBLOCK_m0s8scb_VERSION 0
#define CYIPBLOCK_m0s8srssv2_VERSION 1
#define CYIPBLOCK_m0s8tcpwm_VERSION 0
#define CYIPBLOCK_m0s8udbif_VERSION 0
#define CYIPBLOCK_S8_GPIO_VERSION 2
#define CYDEV_BOOTLOADER_ENABLE 0

#endif /* INCLUDED_CYFITTER_H */
