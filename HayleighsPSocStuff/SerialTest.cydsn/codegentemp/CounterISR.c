/*******************************************************************************
* File Name: CounterISR.c  
* Version 1.70
*
*  Description:
*   API for controlling the state of an interrupt.
*
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/


#include <cydevice_trm.h>
#include <CyLib.h>
#include <CounterISR.h>

#if !defined(CounterISR__REMOVED) /* Check for removal by optimization */

/*******************************************************************************
*  Place your includes, defines and code here 
********************************************************************************/
/* `#START CounterISR_intc` */

/* `#END` */

extern cyisraddress CyRamVectors[CYINT_IRQ_BASE + CY_NUM_INTERRUPTS];

/* Declared in startup, used to set unused interrupts to. */
CY_ISR_PROTO(IntDefaultHandler);


/*******************************************************************************
* Function Name: CounterISR_Start
********************************************************************************
*
* Summary:
*  Set up the interrupt and enable it.
*
* Parameters:  
*   None
*
* Return:
*   None
*
*******************************************************************************/
void CounterISR_Start(void)
{
    /* For all we know the interrupt is active. */
    CounterISR_Disable();

    /* Set the ISR to point to the CounterISR Interrupt. */
    CounterISR_SetVector(&CounterISR_Interrupt);

    /* Set the priority. */
    CounterISR_SetPriority((uint8)CounterISR_INTC_PRIOR_NUMBER);

    /* Enable it. */
    CounterISR_Enable();
}


/*******************************************************************************
* Function Name: CounterISR_StartEx
********************************************************************************
*
* Summary:
*  Set up the interrupt and enable it.
*
* Parameters:  
*   address: Address of the ISR to set in the interrupt vector table.
*
* Return:
*   None
*
*******************************************************************************/
void CounterISR_StartEx(cyisraddress address)
{
    /* For all we know the interrupt is active. */
    CounterISR_Disable();

    /* Set the ISR to point to the CounterISR Interrupt. */
    CounterISR_SetVector(address);

    /* Set the priority. */
    CounterISR_SetPriority((uint8)CounterISR_INTC_PRIOR_NUMBER);

    /* Enable it. */
    CounterISR_Enable();
}


/*******************************************************************************
* Function Name: CounterISR_Stop
********************************************************************************
*
* Summary:
*   Disables and removes the interrupt.
*
* Parameters:  
*
* Return:
*   None
*
*******************************************************************************/
void CounterISR_Stop(void)
{
    /* Disable this interrupt. */
    CounterISR_Disable();

    /* Set the ISR to point to the passive one. */
    CounterISR_SetVector(&IntDefaultHandler);
}


/*******************************************************************************
* Function Name: CounterISR_Interrupt
********************************************************************************
*
* Summary:
*   The default Interrupt Service Routine for CounterISR.
*
*   Add custom code between the coments to keep the next version of this file
*   from over writting your code.
*
* Parameters:  
*   None
*
* Return:
*   None
*
*******************************************************************************/
CY_ISR(CounterISR_Interrupt)
{
    /*  Place your Interrupt code here. */
    /* `#START CounterISR_Interrupt` */

    /* `#END` */
}


/*******************************************************************************
* Function Name: CounterISR_SetVector
********************************************************************************
*
* Summary:
*   Change the ISR vector for the Interrupt. Note calling CounterISR_Start
*   will override any effect this method would have had. To set the vector 
*   before the component has been started use CounterISR_StartEx instead.
*
* Parameters:
*   address: Address of the ISR to set in the interrupt vector table.
*
* Return:
*   None
*
*******************************************************************************/
void CounterISR_SetVector(cyisraddress address)
{
    CyRamVectors[CYINT_IRQ_BASE + CounterISR__INTC_NUMBER] = address;
}


/*******************************************************************************
* Function Name: CounterISR_GetVector
********************************************************************************
*
* Summary:
*   Gets the "address" of the current ISR vector for the Interrupt.
*
* Parameters:
*   None
*
* Return:
*   Address of the ISR in the interrupt vector table.
*
*******************************************************************************/
cyisraddress CounterISR_GetVector(void)
{
    return CyRamVectors[CYINT_IRQ_BASE + CounterISR__INTC_NUMBER];
}


/*******************************************************************************
* Function Name: CounterISR_SetPriority
********************************************************************************
*
* Summary:
*   Sets the Priority of the Interrupt. Note calling CounterISR_Start
*   or CounterISR_StartEx will override any effect this method would 
*   have had. This method should only be called after CounterISR_Start or 
*   CounterISR_StartEx has been called. To set the initial
*   priority for the component use the cydwr file in the tool.
*
* Parameters:
*   priority: Priority of the interrupt. 0 - 3, 0 being the highest.
*
* Return:
*   None
*
*******************************************************************************/
void CounterISR_SetPriority(uint8 priority)
{
	uint8 interruptState;
    uint32 priorityOffset = ((CounterISR__INTC_NUMBER % 4u) * 8u) + 6u;
    
	interruptState = CyEnterCriticalSection();
    *CounterISR_INTC_PRIOR = (*CounterISR_INTC_PRIOR & (uint32)(~CounterISR__INTC_PRIOR_MASK)) |
                                    ((uint32)priority << priorityOffset);
	CyExitCriticalSection(interruptState);
}


/*******************************************************************************
* Function Name: CounterISR_GetPriority
********************************************************************************
*
* Summary:
*   Gets the Priority of the Interrupt.
*
* Parameters:
*   None
*
* Return:
*   Priority of the interrupt. 0 - 3, 0 being the highest.
*
*******************************************************************************/
uint8 CounterISR_GetPriority(void)
{
    uint32 priority;
	uint32 priorityOffset = ((CounterISR__INTC_NUMBER % 4u) * 8u) + 6u;

    priority = (*CounterISR_INTC_PRIOR & CounterISR__INTC_PRIOR_MASK) >> priorityOffset;

    return (uint8)priority;
}


/*******************************************************************************
* Function Name: CounterISR_Enable
********************************************************************************
*
* Summary:
*   Enables the interrupt.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void CounterISR_Enable(void)
{
    /* Enable the general interrupt. */
    *CounterISR_INTC_SET_EN = CounterISR__INTC_MASK;
}


/*******************************************************************************
* Function Name: CounterISR_GetState
********************************************************************************
*
* Summary:
*   Gets the state (enabled, disabled) of the Interrupt.
*
* Parameters:
*   None
*
* Return:
*   1 if enabled, 0 if disabled.
*
*******************************************************************************/
uint8 CounterISR_GetState(void)
{
    /* Get the state of the general interrupt. */
    return ((*CounterISR_INTC_SET_EN & (uint32)CounterISR__INTC_MASK) != 0u) ? 1u:0u;
}


/*******************************************************************************
* Function Name: CounterISR_Disable
********************************************************************************
*
* Summary:
*   Disables the Interrupt.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void CounterISR_Disable(void)
{
    /* Disable the general interrupt. */
    *CounterISR_INTC_CLR_EN = CounterISR__INTC_MASK;
}


/*******************************************************************************
* Function Name: CounterISR_SetPending
********************************************************************************
*
* Summary:
*   Causes the Interrupt to enter the pending state, a software method of
*   generating the interrupt.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void CounterISR_SetPending(void)
{
    *CounterISR_INTC_SET_PD = CounterISR__INTC_MASK;
}


/*******************************************************************************
* Function Name: CounterISR_ClearPending
********************************************************************************
*
* Summary:
*   Clears a pending interrupt.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void CounterISR_ClearPending(void)
{
    *CounterISR_INTC_CLR_PD = CounterISR__INTC_MASK;
}

#endif /* End check for removal by optimization */


/* [] END OF FILE */
