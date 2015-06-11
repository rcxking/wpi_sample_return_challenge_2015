/*******************************************************************************
* File Name: MotorComms_SCBCLK.h
* Version 2.20
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CLOCK_MotorComms_SCBCLK_H)
#define CY_CLOCK_MotorComms_SCBCLK_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/
#if defined CYREG_PERI_DIV_CMD

void MotorComms_SCBCLK_StartEx(uint32 alignClkDiv);
#define MotorComms_SCBCLK_Start() \
    MotorComms_SCBCLK_StartEx(MotorComms_SCBCLK__PA_DIV_ID)

#else

void MotorComms_SCBCLK_Start(void);

#endif/* CYREG_PERI_DIV_CMD */

void MotorComms_SCBCLK_Stop(void);

void MotorComms_SCBCLK_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 MotorComms_SCBCLK_GetDividerRegister(void);
uint8  MotorComms_SCBCLK_GetFractionalDividerRegister(void);

#define MotorComms_SCBCLK_Enable()                         MotorComms_SCBCLK_Start()
#define MotorComms_SCBCLK_Disable()                        MotorComms_SCBCLK_Stop()
#define MotorComms_SCBCLK_SetDividerRegister(clkDivider, reset)  \
    MotorComms_SCBCLK_SetFractionalDividerRegister((clkDivider), 0u)
#define MotorComms_SCBCLK_SetDivider(clkDivider)           MotorComms_SCBCLK_SetDividerRegister((clkDivider), 1u)
#define MotorComms_SCBCLK_SetDividerValue(clkDivider)      MotorComms_SCBCLK_SetDividerRegister((clkDivider) - 1u, 1u)


/***************************************
*             Registers
***************************************/
#if defined CYREG_PERI_DIV_CMD

#define MotorComms_SCBCLK_DIV_ID     MotorComms_SCBCLK__DIV_ID

#define MotorComms_SCBCLK_CMD_REG    (*(reg32 *)CYREG_PERI_DIV_CMD)
#define MotorComms_SCBCLK_CTRL_REG   (*(reg32 *)MotorComms_SCBCLK__CTRL_REGISTER)
#define MotorComms_SCBCLK_DIV_REG    (*(reg32 *)MotorComms_SCBCLK__DIV_REGISTER)

#define MotorComms_SCBCLK_CMD_DIV_SHIFT          (0u)
#define MotorComms_SCBCLK_CMD_PA_DIV_SHIFT       (8u)
#define MotorComms_SCBCLK_CMD_DISABLE_SHIFT      (30u)
#define MotorComms_SCBCLK_CMD_ENABLE_SHIFT       (31u)

#define MotorComms_SCBCLK_CMD_DISABLE_MASK       ((uint32)((uint32)1u << MotorComms_SCBCLK_CMD_DISABLE_SHIFT))
#define MotorComms_SCBCLK_CMD_ENABLE_MASK        ((uint32)((uint32)1u << MotorComms_SCBCLK_CMD_ENABLE_SHIFT))

#define MotorComms_SCBCLK_DIV_FRAC_MASK  (0x000000F8u)
#define MotorComms_SCBCLK_DIV_FRAC_SHIFT (3u)
#define MotorComms_SCBCLK_DIV_INT_MASK   (0xFFFFFF00u)
#define MotorComms_SCBCLK_DIV_INT_SHIFT  (8u)

#else 

#define MotorComms_SCBCLK_DIV_REG        (*(reg32 *)MotorComms_SCBCLK__REGISTER)
#define MotorComms_SCBCLK_ENABLE_REG     MotorComms_SCBCLK_DIV_REG
#define MotorComms_SCBCLK_DIV_FRAC_MASK  MotorComms_SCBCLK__FRAC_MASK
#define MotorComms_SCBCLK_DIV_FRAC_SHIFT (16u)
#define MotorComms_SCBCLK_DIV_INT_MASK   MotorComms_SCBCLK__DIVIDER_MASK
#define MotorComms_SCBCLK_DIV_INT_SHIFT  (0u)

#endif/* CYREG_PERI_DIV_CMD */

#endif /* !defined(CY_CLOCK_MotorComms_SCBCLK_H) */

/* [] END OF FILE */
