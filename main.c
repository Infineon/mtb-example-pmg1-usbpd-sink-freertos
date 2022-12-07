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
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "instrumentation.h"
#include "app.h"
#include "pdo.h"
#include "psink.h"
#include "swap.h"
#include "vdm.h"
#include "charger_detect.h"
#include "mtbcfg_ezpd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define LED_DELAY_MS                     500
#define DPM_TASK_SEMA_TIMOUT_TICK        10
#define APP_TASK_SEMA_TIMOUT_TICK        20
#define INST_TASK_DELAY_MS               100

/* Task handle used for app tasks variables */
TaskHandle_t gl_InstTaskHandle = NULL;
TaskHandle_t gl_DpmPort0TaskHandle = NULL;
TaskHandle_t gl_AppPort0TaskHandle = NULL;
TaskHandle_t gl_FaultPort0TaskHandle = NULL;

/* Semaphore handle used for DPM sync */
SemaphoreHandle_t     gl_DpmPort0SemaHandle = NULL;

uint32_t gl_DPMTaskCount = 0;
uint32_t gl_APPTaskCount = 0;
uint32_t gl_FaultTaskCount = 0;

#if PMG1_PD_DUALPORT_ENABLE
/* Task handle used for app tasks variables */
TaskHandle_t gl_DpmPort1TaskHandle = NULL;
TaskHandle_t gl_AppPort1TaskHandle = NULL;
TaskHandle_t gl_FaultPort1TaskHandle = NULL;

/* Semaphore handle used for DPM sync */
SemaphoreHandle_t gl_DpmPort1SemaHandle = NULL;
#endif

/* LED blink rate in milliseconds */
static uint16_t gl_LedBlinkRate = LED_TIMER_PERIOD_DETACHED;

cy_stc_pdutils_sw_timer_t        gl_TimerCtx;
cy_stc_usbpd_context_t   gl_UsbPdPort0Ctx;
cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

#if PMG1_PD_DUALPORT_ENABLE
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
cy_stc_pdstack_context_t * gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
    &gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
    &gl_PdStackPort1Ctx
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

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

cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

/* Solution PD event handler */
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    (void)ctx;

    if(evt == APP_EVT_HANDLE_EXTENDED_MSG)
    {
        cy_stc_pd_packet_extd_t * ext_mes = (cy_stc_pd_packet_extd_t * )data;
        if ((ext_mes->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (ext_mes->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            Cy_PdStack_Dpm_SendPdCommand(ctx, CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
    }
}

void instrumentation_cb(uint8_t port, inst_evt_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

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

static void cy_usbpd0_intr0_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);

    /* Trigger an OS event */
    xSemaphoreGiveFromISR( gl_DpmPort0SemaHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void cy_usbpd0_intr1_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);

    /* Trigger an OS event */
    xSemaphoreGiveFromISR( gl_DpmPort0SemaHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#if PMG1_PD_DUALPORT_ENABLE
static void cy_usbpd1_intr0_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    Cy_USBPD_Intr0Handler(&gl_UsbPdPort1Ctx);

    /* Trigger an OS event */
    xSemaphoreGiveFromISR( gl_DpmPort1SemaHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

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
void led_timer_cb (
        cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    cy_stc_pdstack_context_t *stack_ctx = (cy_stc_pdstack_context_t *)callbackContext;
#if BATTERY_CHARGING_ENABLE
    const chgdet_status_t    *chgdet_stat;
#endif /* #if BATTERY_CHARGING_ENABLE */

    /* Toggle the User LED and re-start timer to schedule the next toggle event. */
    Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

    /* Calculate the desired LED blink rate based on the correct Type-C connection state. */
    if (stack_ctx->dpmConfig.attach)
    {
        if (stack_ctx->dpmConfig.contractExist)
        {
            gl_LedBlinkRate = LED_TIMER_PERIOD_PD_SRC;
        }
        else
        {
#if BATTERY_CHARGING_ENABLE
            chgdet_stat = chgdet_get_status(stack_ctx);
            if (chgdet_stat->chgdet_fsm_state == CHGDET_FSM_SINK_DCP_CONNECTED)
            {
                gl_LedBlinkRate = LED_TIMER_PERIOD_DCP_SRC;
            }
            else if (chgdet_stat->chgdet_fsm_state == CHGDET_FSM_SINK_CDP_CONNECTED)
            {
                gl_LedBlinkRate = LED_TIMER_PERIOD_CDP_SRC;
            }
            else
#endif /* BATTERY_CHARGING_ENABLE */
            {
                gl_LedBlinkRate = LED_TIMER_PERIOD_TYPEC_SRC;
            }
        }
    }
    else
    {
        gl_LedBlinkRate = LED_TIMER_PERIOD_DETACHED;
    }

    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, callbackContext, id, gl_LedBlinkRate, led_timer_cb);
}
#endif /* APP_FW_LED_ENABLE */

cy_stc_pd_dpm_config_t* get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

#if PMG1_PD_DUALPORT_ENABLE
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
    app_event_handler,
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm,
    vbus_get_value,
};

cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

int32_t dpm_port0_rtos_sema_give(cy_stc_pdstack_context_t * context)
{
    (void)context;

    return xSemaphoreGive( gl_DpmPort0SemaHandle);
}

int32_t dpm_port0_rtos_sema_take(cy_stc_pdstack_context_t * context, uint32_t waitTick)
{
    (void)context;

    return xSemaphoreTake(gl_DpmPort0SemaHandle, waitTick);
}

cy_stc_pdstack_rtos_context_t gl_PdStackRtos0Ctx =
{
    dpm_port0_rtos_sema_give,
    dpm_port0_rtos_sema_take
};

#if PMG1_PD_DUALPORT_ENABLE
int32_t dpm_port1_rtos_sema_give(cy_stc_pdstack_context_t * context)
{
    (void)context;

    return xSemaphoreGive( gl_DpmPort1SemaHandle);
}

int32_t dpm_port1_rtos_sema_take(cy_stc_pdstack_context_t * context, uint32_t waitTick)
{
    (void)context;

    return xSemaphoreTake(gl_DpmPort1SemaHandle, waitTick);
}

cy_stc_pdstack_rtos_context_t gl_PdStackRtos1Ctx =
{
    dpm_port1_rtos_sema_give,
    dpm_port1_rtos_sema_take
};
#endif


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

        /* Wait for semaphore for any event */
        xSemaphoreTake(gl_DpmPort1SemaHandle, semaWaitTime);
    }
}
#endif

void App_Port0_Task(void *param)
{
    (void) param;

    for(;;)
    {
        /* Wait for charger detect events*/
        if(true == chgdet_rtos_event_wait(&gl_PdStackPort0Ctx))
        {
            /* Perform any application level tasks. */
            app_task(&gl_PdStackPort0Ctx);

            gl_APPTaskCount++;
        }
    }
}

#if PMG1_PD_DUALPORT_ENABLE
void App_Port1_Task(void *param)
{
    (void) param;
    for(;;)
    {
        /* Wait for charger detect events*/
        if(true == chgdet_rtos_event_wait(&gl_PdStackPort1Ctx))
        {
            /* Perform any application level tasks. */
            app_task(&gl_PdStackPort1Ctx);
        }
    }
}
#endif

void FaultHandler_Port0_Task(void *param)
{
    (void) param;

    for(;;)
    {
        /* Wait for fault events */
        if(true == fault_rtos_event_wait(&gl_PdStackPort0Ctx))
        {
            /* Perform tasks associated with instrumentation. */
            app_fault_handler_task(&gl_PdStackPort0Ctx);

            gl_FaultTaskCount++;
        }
    }
}

#if PMG1_PD_DUALPORT_ENABLE
void FaultHandler_Port1_Task(void *param)
{
    (void) param;
    for(;;)
    {
        /* Wait for fault events */
        if(true == fault_rtos_event_wait(&gl_PdStackPort1Ctx))
        {
            /* Perform tasks associated with instrumentation. */
            app_fault_handler_task(&gl_PdStackPort1Ctx);
        }
    }
}
#endif

void Instrumentation_Task(void *param)
{
    (void) param;
    char string[32];
    size_t stringSize = 0;
    uint32_t preAppTaskCount = 0;
    uint32_t preDpmTaskCount = 0;
    uint32_t preFaultTaskCount = 0;
    for(;;)
    {
        /* Perform tasks associated with instrumentation. */
        instrumentation_task();

        if((preAppTaskCount != gl_APPTaskCount) || (preDpmTaskCount != gl_DPMTaskCount)
                || (preFaultTaskCount != gl_FaultTaskCount))
        {
            stringSize = snprintf(string, 32, "A:%u D:%u F:%u", (unsigned int)gl_APPTaskCount,
                                            (unsigned int)gl_DPMTaskCount, (unsigned int)gl_FaultTaskCount);
            (void)stringSize;
            
            Cy_SCB_UART_PutString(CYBSP_UART_HW, string);
            
            Cy_SCB_UART_PutString(CYBSP_UART_HW, (void *)"\r\n");

            preAppTaskCount = gl_APPTaskCount;
            preDpmTaskCount = gl_DPMTaskCount;
            preFaultTaskCount = gl_FaultTaskCount;
        }

        /* Given 100 ms delay for instrumentation task*/
        vTaskDelay(pdMS_TO_TICKS(INST_TASK_DELAY_MS));
    }
}


void vApplicationIdleHook()
{
#if SYS_DEEPSLEEP_ENABLE
    /* If possible, enter deep sleep mode for power saving. */
    system_sleep(&gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
                &gl_PdStackPort1Ctx
#else
                NULL
#endif /* PMG1_PD_DUALPORT_ENABLE */
                );
#endif /* SYS_DEEPSLEEP_ENABLE */
}

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

    /*
     * Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);

    timerConfig.sys_clk_freq = Cy_SysClk_ClkSysGetFrequency();
    timerConfig.hw_timer_ctx = NULL;

    /* Initialize the soft timer module. */
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timerConfig);

    xTaskCreate(Instrumentation_Task, "Instrumentation Task", (configMINIMAL_STACK_SIZE*2) , NULL, 1 , &gl_InstTaskHandle) ;
    xTaskCreate(PdStack_Dpm_Port0_Task, "DPM Port0 Task", configMINIMAL_STACK_SIZE , NULL, 1 , &gl_DpmPort0TaskHandle) ;
    xTaskCreate(App_Port0_Task, "APP Port0 Task", configMINIMAL_STACK_SIZE , NULL, 1 , &gl_AppPort0TaskHandle) ;

#if (VBUS_OVP_ENABLE || VBUS_UVP_ENABLE)
    xTaskCreate(FaultHandler_Port0_Task, "Fault Port0 Task", configMINIMAL_STACK_SIZE , NULL, 1 , &gl_FaultPort0TaskHandle) ;
#endif

#if PMG1_PD_DUALPORT_ENABLE
    xTaskCreate(PdStack_Dpm_Port1_Task, "DPM Port1 Task", configMINIMAL_STACK_SIZE , NULL, 1 , &gl_DPMPort1TaskHandle) ;
    xTaskCreate(App_Port1_Task, "APP Port1 Task", configMINIMAL_STACK_SIZE , NULL, 1 , &gl_AppPort1TaskHandle) ;
    xTaskCreate(FaultHandler_Port1_Task, "Fault Port1 Task", configMINIMAL_STACK_SIZE , NULL, 1 , &gl_FaultPort1TaskHandle) ;
#endif //PMG1_PD_DUALPORT_ENABLE

    gl_DpmPort0SemaHandle = xSemaphoreCreateCounting(32, 0);
    if(NULL == gl_DpmPort0SemaHandle)
    {
        CY_ASSERT(0);
    }

#if PMG1_PD_DUALPORT_ENABLE
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
    instrumentation_init();

    /* Register callback function to be executed when instrumentation fault occurs. */
    instrumentation_register_cb((instrumentation_cb_t)instrumentation_cb);

    /* Configure and enable the USBPD interrupts */
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

#if PMG1_PD_DUALPORT_ENABLE
    /* Configure and enable the USBPD interrupts for Port #1. */
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
#endif

    /* Initialize the Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
                       &gl_UsbPdPort0Ctx,
                       &mtb_usbpd_port0_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort0Ctx),
                       &pdstack_port0_dpm_params,
                       &gl_TimerCtx
                       );

    Cy_PdStack_Dpm_Rtos_Init(&gl_PdStackPort0Ctx,
                        &gl_PdStackRtos0Ctx
                                 );

#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Init(&gl_PdStackPort1Ctx,
                       &gl_UsbPdPort1Ctx,
                       &mtb_usbpd_port1_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort1Ctx),
                       &pdstack_port1_dpm_params,
                       &gl_TimerCtx);

    Cy_PdStack_Dpm_Rtos_Init(&gl_PdStackPort1Ctx,
                        &gl_PdStackRtos1Ctx
                                 );
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Perform application level initialization. */
    app_init(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    app_init(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the fault configuration values */
    fault_handler_init_vars(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    fault_handler_init_vars(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Start any timers or tasks associated with application instrumentation. */
    instrumentation_start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Start(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if APP_FW_LED_ENABLE
    /* Start a timer that will blink the FW ACTIVE LED. */
    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)LED_TIMER_ID,
            gl_LedBlinkRate, led_timer_cb);
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
