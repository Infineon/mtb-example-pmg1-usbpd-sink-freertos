/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the USBPD Sink Example using FreeRTOS
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"

#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "cy_app_instrumentation.h"
#include "cy_app_fault_handlers.h"
#include "cy_app.h"
#include "cy_app_pdo.h"
#include "cy_app_sink.h"
#include "cy_app_swap.h"
#include "cy_app_vdm.h"
#include "cy_app_battery_charging.h"
#include "mtbcfg_ezpd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cyabs_rtos.h"

/*******************************************************************************
* Macro definitions
*******************************************************************************/
#define LED_DELAY_MS                     500
#define DPM_TASK_SEMA_TIMOUT_TICK        10
#define APP_TASK_SEMA_TIMOUT_TICK        20
#define INST_TASK_DELAY_MS               100

#define INST_TASK_PRIORITY              (tskIDLE_PRIORITY + 1)
#define DPM_TASK_PRIORITY               (tskIDLE_PRIORITY + 3)
#define APP_TASK_PRIORITY               (tskIDLE_PRIORITY + 2)

#define INST_TASK_STACK_SIZE            (configMINIMAL_STACK_SIZE*2)
#define DPM_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE*4)
#define APP_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE*2)

#define UART_TIMEOUT_COUNT              (3000)

/* Structure to hold the user LED status. */
typedef struct
{
    GPIO_PRT_Type* gpioPort;     /* User LED port base address */
    uint32_t gpioPin;            /* User LED pin GPIO number */
    uint16_t blinkRate;        /* User LED blink rate in millisecond */
}cy_stc_user_led_status;

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Variable used for logging the task execution count. */
uint32_t gl_DPMTaskCount = 0;
uint32_t gl_APPTaskCount = 0;
uint32_t gl_preAppTaskCount = 0;
uint32_t gl_preDpmTaskCount = 0;

/* Instrumentation task handler. */
TaskHandle_t gl_InstTaskHandle = NULL;

/* Semaphore handle used by the instrumentation task. */
SemaphoreHandle_t gl_InstSemaHandle = NULL;

/* Task handler used for port-0 tasks */
TaskHandle_t gl_DpmPort0TaskHandle = NULL;
TaskHandle_t gl_AppPort0TaskHandle = NULL;

/* Semaphore handle used for port-0 DPM sync */
SemaphoreHandle_t gl_DpmPort0SemaHandle = NULL;

#if PMG1_PD_DUALPORT_ENABLE
/* Task handler used for port-1 app tasks variables */
TaskHandle_t gl_DpmPort1TaskHandle = NULL;
TaskHandle_t gl_AppPort1TaskHandle = NULL;

/* Semaphore handle used for port-1 DPM sync */
SemaphoreHandle_t gl_DpmPort1SemaHandle = NULL;
#endif

/* Variable to store the user LED status */
static cy_stc_user_led_status gl_userLedStatus[NO_OF_TYPEC_PORTS] =
{
        {CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, LED_TIMER_PERIOD_DETACHED},
#if PMG1_PD_DUALPORT_ENABLE
        {CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_PIN, LED_TIMER_PERIOD_DETACHED},
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

/* Software timer context variable. */
cy_stc_pdutils_sw_timer_t gl_TimerCtx;

/* USB-PD and PdStack context variable for port-0. */
cy_stc_usbpd_context_t gl_UsbPdPort0Ctx;
cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

#if PMG1_PD_DUALPORT_ENABLE
/* USB-PD and PdStack context variable for port-1. */
cy_stc_usbpd_context_t gl_UsbPdPort1Ctx;
cy_stc_pdstack_context_t gl_PdStackPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */

const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
    .dpmSnkWaitCapPeriod = 400,
    .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
    .dpmDefCableCap = 300,
    .muxEnableDelayPeriod = 0,
    .typeCSnkWaitCapPeriod = 0,
    .defCur = 90
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_pdstack_dpm_params_t pdstack_port1_dpm_params =
{
    .dpmSnkWaitCapPeriod = 400,
    .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
    .dpmDefCableCap = 300,
    .muxEnableDelayPeriod = 0,
    .typeCSnkWaitCapPeriod = 0,
    .defCur = 90
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

uint32_t gl_discIdRespPort0[7] = {0xFF00A841, 0x184004B4, 0x00000000, 0xF5000000};

const cy_stc_app_params_t port0_app_params =
{
    .appVbusPollAdcId = APP_VBUS_POLL_ADC_ID,
    .appVbusPollAdcInput = APP_VBUS_POLL_ADC_INPUT,
    .discIdResp = (cy_pd_pd_do_t *)&gl_discIdRespPort0[0],
    .discIdLen = 0x14,
    .swapResponse = 0x3F
};

#if PMG1_PD_DUALPORT_ENABLE
uint32_t gl_discIdRespPort1[7] = {0xFF00A841, 0x184004B4, 0x00000000, 0xF5000000};

const cy_stc_app_params_t port1_app_params =
{
    .appVbusPollAdcId = APP_VBUS_POLL_ADC_ID,
    .appVbusPollAdcInput = APP_VBUS_POLL_ADC_INPUT,
    .discIdResp = (cy_pd_pd_do_t *)&gl_discIdRespPort1[0],
    .discIdLen = 0x14,
    .swapResponse = 0x3F
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

cy_stc_pdstack_context_t * gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
    &gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
    &gl_PdStackPort1Ctx
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

/*******************************************************************************
* Function Definition
*******************************************************************************/
bool mux_ctrl_init(uint8_t port)
{
    /* No MUXes to be controlled on the PMG1 proto kits. */
    CY_UNUSED_PARAMETER(port);
    return true;
}

const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0U,
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_sysint_t usbpd_port1_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port1_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_DS_IRQ,
    .intrPriority = 0U,
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*******************************************************************************
* Function Name: get_pdstack_context
********************************************************************************
* Summary:
*   Returns the respective port PD Stack Context
*
* Parameters:
*  portIdx - Port Index
*
* Return:
*  cy_stc_pdstack_context_t
*
*******************************************************************************/
cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

/*******************************************************************************
* Function Name: sln_pd_event_handler
********************************************************************************
* Summary:
*   Solution PD Event Handler
*   Handles the Extended message event
*
* Parameters:
*  ctx - PD Stack Context
*  evt - App Event
*  data - Data
*
* Return:
*  None
*
*******************************************************************************/
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    (void)ctx;
    (void)evt;
    (void)data;
}

/*******************************************************************************
* Function Name: instrumentation_cb
********************************************************************************
* Summary:
*  Callback function for handling instrumentation faults
*
* Parameters:
*  port - Port
*  evt - Event
*
* Return:
*  None
*
*******************************************************************************/
void instrumentation_cb(uint8_t port, uint8_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

/*******************************************************************************
* Function Name: wdt_interrupt_handler
********************************************************************************
* Summary:
*  Interrupt Handler for Watch Dog Timer
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    Cy_WDT_ClearInterrupt();

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    Cy_WDT_SetMatch((Cy_WDT_GetCount() + gl_TimerCtx.multiplier));
#endif /* (TIMER_TICKLESS_ENABLE == 0) */

    /* Invoke the timer handler. */
    Cy_PdUtils_SwTimer_InterruptHandler (&(gl_TimerCtx));
}

/*******************************************************************************
* Function Name: cy_usbpd0_intr0_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD0 Interrupt 0
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd0_intr0_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);

    /* Trigger an OS event */
    xSemaphoreGiveFromISR( gl_DpmPort0SemaHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
* Function Name: cy_usbpd0_intr1_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD0 Interrupt 1
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd0_intr1_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);

    /* Trigger an OS event */
    xSemaphoreGiveFromISR( gl_DpmPort0SemaHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#if PMG1_PD_DUALPORT_ENABLE
/*******************************************************************************
* Function Name: cy_usbpd1_intr0_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD1 Interrupt 0
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd1_intr0_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    Cy_USBPD_Intr0Handler(&gl_UsbPdPort1Ctx);

    /* Trigger an OS event */
    xSemaphoreGiveFromISR( gl_DpmPort1SemaHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
* Function Name: cy_usbpd1_intr1_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD1 Interrupt 1
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd1_intr1_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    Cy_USBPD_Intr1Handler(&gl_UsbPdPort1Ctx);

    /* Trigger an OS event */
    xSemaphoreGiveFromISR( gl_DpmPort1SemaHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if APP_FW_LED_ENABLE
/*******************************************************************************
* Function Name: led_timer_cb
********************************************************************************
* Summary:
*  Sets the desired LED blink rate based on the Type-C connection
*
* Parameters:
*  id - Timer ID
*  callbackContext - Context
*
* Return:
*  None
*
*******************************************************************************/
void led_timer_cb (
        cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    cy_stc_pdstack_context_t *stack_ctx = (cy_stc_pdstack_context_t *)callbackContext;
    cy_stc_user_led_status *user_led = &gl_userLedStatus[stack_ctx->port];
#if BATTERY_CHARGING_ENABLE
    const cy_stc_bc_status_t    *bc_stat;
#endif /* #if BATTERY_CHARGING_ENABLE */

    /* Toggle the User LED and re-start timer to schedule the next toggle event. */
    Cy_GPIO_Inv(user_led->gpioPort, user_led->gpioPin);

    /* Calculate the desired LED blink rate based on the correct Type-C connection state. */
    if (stack_ctx->dpmConfig.attach)
    {
        if (stack_ctx->dpmConfig.contractExist)
        {
            user_led->blinkRate = LED_TIMER_PERIOD_PD_SRC;
        }
        else
        {
#if BATTERY_CHARGING_ENABLE
            bc_stat = Cy_App_Bc_GetStatus(stack_ctx->ptrUsbPdContext);
            if (bc_stat->bc_fsm_state == BC_FSM_SINK_DCP_CONNECTED)
            {
                user_led->blinkRate = LED_TIMER_PERIOD_DCP_SRC;
            }
            else if (bc_stat->bc_fsm_state == BC_FSM_SINK_CDP_CONNECTED)
            {
                user_led->blinkRate = LED_TIMER_PERIOD_CDP_SRC;
            }
            else if (bc_stat->bc_fsm_state == BC_FSM_SINK_APPLE_BRICK_ID_DETECT)
            {
                user_led->blinkRate = LED_TIMER_PERIOD_APPLE_SRC;
            }
            else if (bc_stat->bc_fsm_state == BC_FSM_SINK_APPLE_BRICK_ID_DETECT)
            {
                user_led->blinkRate = LED_TIMER_PERIOD_APPLE_SRC;
            }
            else
#endif /* BATTERY_CHARGING_ENABLE */
            {
                user_led->blinkRate = LED_TIMER_PERIOD_TYPEC_SRC;
            }
        }
    }
    else
    {
        user_led->blinkRate = LED_TIMER_PERIOD_DETACHED;
    }

    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, callbackContext, id, user_led->blinkRate, led_timer_cb);
}
#endif /* APP_FW_LED_ENABLE */

/*******************************************************************************
* Function Name: get_dpm_connect_stat
********************************************************************************
* Summary:
*  Gets the DPM configuration for Port 0
*
* Parameters:
*  None
*
* Return:
*  cy_stc_pd_dpm_config_t
*
*******************************************************************************/
cy_stc_pd_dpm_config_t* get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

#if PMG1_PD_DUALPORT_ENABLE
/*******************************************************************************
* Function Name: get_dpm_port1_connect_stat
********************************************************************************
* Summary:
*  Gets the DPM configuration for Port 1
*
* Parameters:
*  None
*
* Return:
*  cy_stc_pd_dpm_config_t
*
*******************************************************************************/
cy_stc_pd_dpm_config_t* get_dpm_port1_connect_stat()
{
    return &(gl_PdStackPort1Ctx.dpmConfig);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    .app_event_handler = Cy_App_EventHandler,
    .vconn_enable = Cy_App_VconnEnable,
    .vconn_disable = Cy_App_VconnDisable,
    .vconn_is_present = Cy_App_VconnIsPresent,
    .vbus_is_present = Cy_App_VbusIsPresent,
    .vbus_discharge_on = Cy_App_VbusDischargeOn,
    .vbus_discharge_off = Cy_App_VbusDischargeOff,
    .psnk_set_voltage = Cy_App_Sink_SetVoltage,
    .psnk_set_current = Cy_App_Sink_SetCurrent,
    .psnk_enable = Cy_App_Sink_Enable,
    .psnk_disable = Cy_App_Sink_Disable,
    .eval_src_cap = Cy_App_Pdo_EvalSrcCap,
    .eval_dr_swap = Cy_App_Swap_EvalDrSwap,
    .eval_pr_swap = Cy_App_Swap_EvalPrSwap,
    .eval_vconn_swap = Cy_App_Swap_EvalVconnSwap,
    .eval_vdm = Cy_App_Vdm_EvalVdmMsg,
    .vbus_get_value = Cy_App_VbusGetValue
};

/*******************************************************************************
* Function Name: app_get_callback_ptr
********************************************************************************
* Summary:
*  Returns pointer to the structure holding the application callback functions
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  cy_stc_pdstack_app_cbk_t
*
*******************************************************************************/
cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

/*******************************************************************************
* Function Name: dpm_port0_rtos_sema_give
********************************************************************************
* Summary:
*  Function releases the DPM Port-0 semaphore and returns the RTOS status code. 
*  It is used by the PdStack middleware library.
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  RTOS status code
*
*******************************************************************************/
int32_t dpm_port0_rtos_sema_give(cy_stc_pdstack_context_t * context)
{
    (void)context;

    return xSemaphoreGive( gl_DpmPort0SemaHandle);
}

/*******************************************************************************
* Function Name: dpm_port0_rtos_sema_give
********************************************************************************
* Summary:
*  Function acquires the DPM Port-0 semamphore and returns the RTOS status code.
*  It is used by the PdStack middleware library.
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  RTOS status code
*
*******************************************************************************/
int32_t dpm_port0_rtos_sema_take(cy_stc_pdstack_context_t * context, uint32_t waitTick)
{
    (void)context;

    return xSemaphoreTake(gl_DpmPort0SemaHandle, waitTick);
}

/* RTOS callback structure for the PdStack middleware library. */
cy_stc_pdstack_rtos_context_t gl_PdStackRtos0Ctx =
{
    dpm_port0_rtos_sema_give,
    dpm_port0_rtos_sema_take
};

#if PMG1_PD_DUALPORT_ENABLE
/*******************************************************************************
* Function Name: dpm_port1_rtos_sema_give
********************************************************************************
* Summary:
*  Function releases the DPM Port-1 semaphore and returns the RTOS status code. 
*  It is used by the PdStack middleware library.
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  RTOS status code
*
*******************************************************************************/
int32_t dpm_port1_rtos_sema_give(cy_stc_pdstack_context_t * context)
{
    (void)context;

    return xSemaphoreGive( gl_DpmPort1SemaHandle);
}

/*******************************************************************************
* Function Name: dpm_port1_rtos_sema_give
********************************************************************************
* Summary:
*  Function acquires the DPM Port-1 semamphore and returns the RTOS status code.
*  It is used by the PdStack middleware library.
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  RTOS status code
*
*******************************************************************************/
int32_t dpm_port1_rtos_sema_take(cy_stc_pdstack_context_t * context, uint32_t waitTick)
{
    (void)context;

    return xSemaphoreTake(gl_DpmPort1SemaHandle, waitTick);
}

/* RTOS callback structure for the PdStack middleware library. */
cy_stc_pdstack_rtos_context_t gl_PdStackRtos1Ctx =
{
    dpm_port1_rtos_sema_give,
    dpm_port1_rtos_sema_take
};
#endif /* PMG1_PD_DUALPORT_ENABLE*/

/*******************************************************************************
* Function Name: PdStack_Dpm_Port0_Task
********************************************************************************
* Summary:
*  This task manages Port-0 Type-C and Power Delivery (PD) events and messages.
*
* Parameters:
*  param - Pointer to the RTOS task context.
*
* Return:
*  None.
*
*******************************************************************************/
void PdStack_Dpm_Port0_Task (void *param)
{
    (void) param;
    bool dpm_slept = false;
    TickType_t semaWaitTime = portMAX_DELAY;
    for(;;)
    {
        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort0Ctx);

        /* Do one DPM sleep capability check  */
        Cy_PdStack_Dpm_IsIdle (&gl_PdStackPort0Ctx, &dpm_slept);
        if(false == dpm_slept)
        {
            semaWaitTime = DPM_TASK_SEMA_TIMOUT_TICK;
        }
        else
        {
            semaWaitTime = portMAX_DELAY;
        }

        gl_DPMTaskCount++;

        /* Wait for semaphore for any event */
        xSemaphoreTake(gl_DpmPort0SemaHandle, semaWaitTime);
    }
}

#if PMG1_PD_DUALPORT_ENABLE
/*******************************************************************************
* Function Name: PdStack_Dpm_Port1_Task
********************************************************************************
* Summary:
*  This task manages Port-0 Type-C and Power Delivery (PD) events and messages.
*
* Parameters:
*  param - Pointer to the RTOS task context.
*
* Return:
*  None.
*
*******************************************************************************/
void PdStack_Dpm_Port1_Task (void *param)
{
    (void) param;
    bool dpm_slept = false;
    TickType_t semaWaitTime = portMAX_DELAY;
    for(;;)
    {
        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort1Ctx);

        /* Do one DPM sleep capability check  */
        Cy_PdStack_Dpm_IsIdle (&gl_PdStackPort1Ctx, &dpm_slept);
        if(false == dpm_slept)
        {
            semaWaitTime = DPM_TASK_SEMA_TIMOUT_TICK;
        }
        else
        {
            semaWaitTime = portMAX_DELAY;
        }

        gl_DPMTaskCount++;

        /* Wait for semaphore for any event */
        xSemaphoreTake(gl_DpmPort1SemaHandle, semaWaitTime);
    }
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*******************************************************************************
* Function Name: App_Port0_Task
********************************************************************************
* Summary:
*  Task performs any application specific task eg. Legacy charging on Port-0.
*
* Parameters:
*  param - Pointer to the RTOS task context.
*
* Return:
*  None.
*
*******************************************************************************/
void App_Port0_Task(void *param)
{
    (void) param;

    for(;;)
    {
        /* Wait for charger detect events*/
        if(true == Cy_App_GetRtosEvent(&gl_PdStackPort0Ctx, CY_RTOS_NEVER_TIMEOUT))
        {
            /* Perform any application level tasks. */
            Cy_App_Task(&gl_PdStackPort0Ctx);

            gl_APPTaskCount++;
        }
    }
}

#if PMG1_PD_DUALPORT_ENABLE
/*******************************************************************************
* Function Name: App_Port1_Task
********************************************************************************
* Summary:
*  Task performs any application specific task eg. Legacy charging on Port-0.
*
* Parameters:
*  param - Pointer to the RTOS task context.
*
* Return:
*  None.
*
*******************************************************************************/
void App_Port1_Task(void *param)
{
    (void) param;
    for(;;)
    {
        /* Wait for charger detect events*/
        if(true == Cy_App_GetRtosEvent(&gl_PdStackPort1Ctx, CY_RTOS_NEVER_TIMEOUT))
        {
            /* Perform any application level tasks. */
            Cy_App_Task(&gl_PdStackPort1Ctx);

            gl_APPTaskCount++;
        }
    }
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*******************************************************************************
* Function Name: Instrumentation_Task
********************************************************************************
* Summary:
*  Task performs any system instrumentation and system task eg. Stack Monitoring.
*
* Parameters:
*  param - Pointer to the RTOS task context.
*
* Return:
*  None.
*
*******************************************************************************/
void Instrumentation_Task(void *param)
{
    (void) param;
    char string[32];
    size_t stringSize = 0;
    int32_t timeOut = UART_TIMEOUT_COUNT;

    for(;;)
    {
        /* Wait for semaphore to get timeout or get an event. */
        xSemaphoreTake(gl_InstSemaHandle, pdMS_TO_TICKS(INST_TASK_DELAY_MS));

        /* Perform tasks associated with instrumentation. */
        Cy_App_Instrumentation_Task();

        if((gl_preAppTaskCount != gl_APPTaskCount) || (gl_preDpmTaskCount != gl_DPMTaskCount))
        {
            stringSize = snprintf(string, 32, "App:%u Dpm:%u\r\n",
                                  (unsigned int)gl_APPTaskCount,
                                  (unsigned int)gl_DPMTaskCount);

            Cy_SCB_UART_PutArrayBlocking(CYBSP_UART_HW, string, stringSize);

            timeOut = UART_TIMEOUT_COUNT;
            while((!Cy_SCB_UART_IsTxComplete(CYBSP_UART_HW)) && (--timeOut > 0));

            /* Store the previous count. */
            gl_preAppTaskCount = gl_APPTaskCount;
            gl_preDpmTaskCount = gl_DPMTaskCount;
        }
    }
}

/*******************************************************************************
* Function Name: vApplicationIdleHook
********************************************************************************
* Summary:
*  Function will execute as an idle task and performs the system sleep operations.
*
* Parameters:
*  None
*
* Return:
*  None.
*
*******************************************************************************/
void vApplicationIdleHook()
{
#if SYS_DEEPSLEEP_ENABLE
    /* If possible, enter deep sleep mode for power saving. */
    Cy_App_SystemSleep(&gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
                &gl_PdStackPort1Ctx
#else
                NULL
#endif /* PMG1_PD_DUALPORT_ENABLE */
                );
    /* Since the SysTick will be disabled during the deep sleep, send a event to
     * the instrumentation task to exit from the block state to avoid the watchdog reset.
     */
    xSemaphoreGive(gl_InstSemaHandle);
#endif /* SYS_DEEPSLEEP_ENABLE */
}

/*******************************************************************************
* Function Name: soln_sink_fet_off
********************************************************************************
* Summary:
*  Turns off the consumer FET
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  None
*
*******************************************************************************/
void soln_sink_fet_off(cy_stc_pdstack_context_t * context)
{
#if CY_APP_SINK_FET_CTRL_GPIO_EN
    if (context->port == 0u)
    {
        Cy_GPIO_Clr (PFET_SNK_CTRL_P0_PORT, PFET_SNK_CTRL_P0_PIN);
    }
#if PMG1_PD_DUALPORT_ENABLE
    else
    {
        Cy_GPIO_Clr (PFET_SNK_CTRL_P1_PORT, PFET_SNK_CTRL_P1_PIN);
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_SINK_FET_CTRL_GPIO_EN */
}

/*******************************************************************************
* Function Name: soln_sink_fet_on
********************************************************************************
* Summary:
*  Turns on the consumer FET
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  None
*
*******************************************************************************/
void soln_sink_fet_on(cy_stc_pdstack_context_t * context)
{
#if CY_APP_SINK_FET_CTRL_GPIO_EN
    if (context->port == 0u)
    {
        Cy_GPIO_Set (PFET_SNK_CTRL_P0_PORT, PFET_SNK_CTRL_P0_PIN);
    }
#if PMG1_PD_DUALPORT_ENABLE
    else
    {
        Cy_GPIO_Set (PFET_SNK_CTRL_P1_PORT, PFET_SNK_CTRL_P1_PIN);
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_SINK_FET_CTRL_GPIO_EN */
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - Initial setup of device
*  - Enables Watchdog timer, USB PD Port 0 and Port 1 interrupt
*  - Initializes USBPD block and PDStack
*  - Starts the Scheduler
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_stc_pdutils_timer_config_t timerConfig;
    cy_stc_scb_uart_context_t CYBSP_UART_context;

    const char string[] = "Scheduler Started\r\n";

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);

    timerConfig.sys_clk_freq = Cy_SysClk_ClkSysGetFrequency();
    timerConfig.hw_timer_ctx = NULL;

    /* Initialize the soft timer module. */
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timerConfig);

    /* Create instrumentation task. */
    xTaskCreate(Instrumentation_Task, "Instrumentation Task", INST_TASK_STACK_SIZE , NULL, INST_TASK_PRIORITY , &gl_InstTaskHandle) ;

    /* Create DPM task for the port-0. */
    xTaskCreate(PdStack_Dpm_Port0_Task, "DPM Port0 Task", DPM_TASK_STACK_SIZE , NULL, DPM_TASK_PRIORITY , &gl_DpmPort0TaskHandle) ;

    /* Create application task for the port-0. */
    xTaskCreate(App_Port0_Task, "APP Port0 Task", APP_TASK_STACK_SIZE , NULL, APP_TASK_PRIORITY , &gl_AppPort0TaskHandle) ;


#if PMG1_PD_DUALPORT_ENABLE
    /* Create DPM task for the port-1. */
    xTaskCreate(PdStack_Dpm_Port1_Task, "DPM Port1 Task", DPM_TASK_STACK_SIZE , NULL, DPM_TASK_PRIORITY , &gl_DpmPort1TaskHandle) ;

    /* Create application task for the port-1. */
    xTaskCreate(App_Port1_Task, "APP Port1 Task", APP_TASK_STACK_SIZE , NULL, APP_TASK_PRIORITY , &gl_AppPort1TaskHandle) ;
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Create a counting semaphore for the instrumentation task.
     * This is used for running the instrumentation task periodically.*/
    gl_InstSemaHandle = xSemaphoreCreateCounting(32, 0);
    if(NULL == gl_InstSemaHandle)
    {
        CY_ASSERT(0);
    }

    /* Create a counting semaphore for the USB-PD and Type-C event notification.
     * This is used for sending an event to the Port-0 DPM task.*/
    gl_DpmPort0SemaHandle = xSemaphoreCreateCounting(32, 0);
    if(NULL == gl_DpmPort0SemaHandle)
    {
        CY_ASSERT(0);
    }

#if PMG1_PD_DUALPORT_ENABLE
    /* Create a counting semaphore for the USB-PD and Type-C event notification.
     * This is used for sending an event to the Port-1 DPM task.*/
    gl_DpmPort1SemaHandle = xSemaphoreCreateCounting(32, 0);
    if(NULL == gl_DpmPort1SemaHandle)
    {
        CY_ASSERT(0);
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the instrumentation related data structures. */
    Cy_App_Instrumentation_Init(&gl_TimerCtx);

    /* Register callback function to be executed when instrumentation fault occurs. */
    Cy_App_Instrumentation_RegisterCb((cy_app_instrumentation_cb_t)instrumentation_cb);

    /* Configure and enable the USBPD interrupts for Port-0*/
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

#if PMG1_PD_DUALPORT_ENABLE
    /* Configure and enable the USBPD interrupts for Port-1. */
    Cy_SysInt_Init(&usbpd_port1_intr0_config, &cy_usbpd1_intr0_handler);
    NVIC_EnableIRQ(usbpd_port1_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port1_intr1_config, &cy_usbpd1_intr1_handler);
    NVIC_EnableIRQ(usbpd_port1_intr1_config.intrSrc);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the USBPD driver */
#if defined(CY_DEVICE_CCG3)
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, NULL,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#else
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_USBPD_Init(&gl_UsbPdPort1Ctx, 1, mtb_usbpd_port1_HW, mtb_usbpd_port1_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port1_config, get_dpm_port1_connect_stat);
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* defined(CY_DEVICE_CCG3) */

    /* Initialize the Device Policy Manager for Port-0. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
                       &gl_UsbPdPort0Ctx,
                       &mtb_usbpd_port0_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort0Ctx),
                       &pdstack_port0_dpm_params,
                       &gl_TimerCtx);

    Cy_PdStack_Dpm_Rtos_Init(&gl_PdStackPort0Ctx,
                        &gl_PdStackRtos0Ctx);

#if PMG1_PD_DUALPORT_ENABLE
    /* Initialize the Device Policy Manager for Port-1. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort1Ctx,
                       &gl_UsbPdPort1Ctx,
                       &mtb_usbpd_port1_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort1Ctx),
                       &pdstack_port1_dpm_params,
                       &gl_TimerCtx);

    Cy_PdStack_Dpm_Rtos_Init(&gl_PdStackPort1Ctx,
                        &gl_PdStackRtos1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Perform application level initialization. */
    Cy_App_Init(&gl_PdStackPort0Ctx, &port0_app_params);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_App_Init(&gl_PdStackPort1Ctx, &port1_app_params);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the fault configuration values */
    Cy_App_Fault_InitVars(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_App_Fault_InitVars(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Start any timers or tasks associated with application instrumentation. */
    Cy_App_Instrumentation_Start();

    /* Start the device policy manager operation. This will initialize the
     * USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Start(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if APP_FW_LED_ENABLE
    /* Start a timer that will blink the FW ACTIVE LED. */
    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)LED1_TIMER_ID,
            LED_TIMER_PERIOD_DETACHED, led_timer_cb);
#if PMG1_PD_DUALPORT_ENABLE
    /* Start a timer that will blink the FW ACTIVE LED. */
    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, (void *)&gl_PdStackPort1Ctx, (cy_timer_id_t)LED2_TIMER_ID,
            LED_TIMER_PERIOD_DETACHED, led_timer_cb);
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* APP_FW_LED_ENABLE */

    /*
     * After the initialization is complete, keep processing the USB-PD device policy manager task in a loop.
     * Since this application does not have any other function, the PMG1 device can be placed in "deep sleep"
     * mode for power saving whenever the PD stack and drivers are idle.
     */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, string);

    vTaskStartScheduler();

    for (;;)
    {
        /* Program will never reach here */
    }
}

/* [] END OF FILE */
