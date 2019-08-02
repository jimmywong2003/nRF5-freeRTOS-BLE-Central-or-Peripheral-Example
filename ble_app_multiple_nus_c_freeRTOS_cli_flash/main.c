/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
// Board/nrf6310/ble/ble_app_hrs_rtx/main.c
/**
 *
 * @brief Heart Rate Service Sample Application with RTX main file.
 *
 * This file contains the source code for a sample application using RTX and the
 * Heart Rate service (and also Battery and Device Information services).
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"

#include "app_uart.h"
#include "app_util_platform.h"

#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"

#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"

#include "ble_conn_state.h"
#include "nrf_ble_scan.h"
#include "ble_db_discovery.h"
#include "nrf_ble_gatt.h"

#include "ble_nus_c.h"

#include "nrf_drv_clock.h"
#include "nrf_drv_saadc.h"
#include "nrf_stack_guard.h"

#include "nrf_fstorage_sd.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_backend_flash.h"

#include "nrf_log_default_backends.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_cli.h"
#include "nrf_cli_rtt.h"

#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0                     /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_1_LED         BSP_BOARD_LED_2                     /**< Connected LED will be on when the device is connected. */
#define CENTRAL_CONNECTED_2_LED         BSP_BOARD_LED_3                     /**< Connected LED will be on when the device is connected. */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                 /**< Battery level measurement interval (ticks). This value corresponds to 120 seconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds).  */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)       /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  270                                     /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                                    /**< Maximum digital value for 10-bit ADC conversion. */

#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

#define NRF_BLE_CLI_TASK_STACK 1024

typedef void (*nrf_sdh_cli_task_hook_t)(void * p_context);


/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
        ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

static nrf_saadc_value_t adc_buf[2];

NRF_BLE_SCAN_DEF(m_scan);                                       /**< Scanning module instance. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */

BLE_NUS_C_ARRAY_DEF(m_ble_nus_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;              /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static char const m_target_periph_name[] = "JIMMY_UART";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/

static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static TimerHandle_t m_battery_timer;                               /**< Definition of battery timer. */

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif


static TaskHandle_t m_cli_task;
static nrf_sdh_cli_task_hook_t m_task_hook;

static bool m_cli_task_is_running = false;
static bool m_log_task_is_running = false;

static uint32_t m_cli_task_count = 0;
static uint32_t m_log_task_count = 0;

static uint32_t m_total_count = 0;

NRF_CLI_RTT_DEF(cli_rtt);
NRF_CLI_DEF(m_cli, "rtt_cli:~$ ", &cli_rtt.transport,'\r', 4);

NRF_LOG_BACKEND_FLASHLOG_DEF(m_flash_log_backend);
NRF_LOG_BACKEND_CRASHLOG_DEF(m_crash_log_backend);

// Extra test CLI command example

static void led_on_cmd(nrf_cli_t const *p_cli, size_t argc, char **argv)
{
        bsp_board_led_on(3);
        nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "LED turned on.\r\n");
}

static void led_off_cmd(nrf_cli_t const *p_cli, size_t argc, char **argv)
{
        bsp_board_led_off(3);
        nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "LED turned off.\r\n");
}

static void default_cmd(nrf_cli_t const *p_cli, size_t argc, char **argv)
{
        if ((argc == 1) || nrf_cli_help_requested(p_cli))
        {
                nrf_cli_help_print(p_cli, NULL, 0);
                return;
        }

        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s:%s%s\r\n", argv[0], " unknown parameter: ", argv[1]);
}


// Register "mpu" command and it's subcommands in CLI.
NRF_CLI_CREATE_STATIC_SUBCMD_SET(led_commands)
{
        NRF_CLI_CMD(on, NULL, "Turn LED on.", led_on_cmd),
        NRF_CLI_CMD(off, NULL, "Turn LED off.", led_off_cmd),
        NRF_CLI_SUBCMD_SET_END
};

NRF_CLI_CMD_REGISTER(led, &led_commands, "Commands for LED control", default_cmd);


static void cmd_writeflash(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);

    if (nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

//     NRF_LOG_WARNING("0123456789ABCED");
//     NRF_LOG_INFO("INFO TESTING");

   uint32_t err_code = NRF_ERROR_NOT_FOUND;
   APP_ERROR_CHECK(err_code);
}

NRF_CLI_CMD_REGISTER(writeflash, NULL, "Write flash test.", cmd_writeflash);


// NRF_CLI_CMD_REGISTER(freertos, &freertos_commands, "FreeRTOS statistic", default_cmd);

static void cli_task(void * p_context)
{
        UNUSED_PARAMETER(p_context);

        nrf_cli_t * p_cli = (nrf_cli_t *)p_context;

        m_cli_task_is_running = true;

        ret_code_t ret = nrf_cli_start(p_cli);
        APP_ERROR_CHECK(ret);

        while (1)
        {
                NRF_LOG_FLUSH();
                nrf_cli_process(p_cli);
                vTaskSuspend(NULL); // Suspend myself
        }

        m_cli_task_is_running = false;
}


void nrf_cli_freertos_init(nrf_sdh_freertos_task_hook_t hook_fn, void * p_context)
{
        NRF_LOG_DEBUG("Creating a CLI task.");

        m_task_hook = hook_fn;

        BaseType_t xReturned = xTaskCreate(cli_task,
                                           "CLI",
                                           NRF_BLE_CLI_TASK_STACK,
                                           p_context,
                                           2,
                                           &m_cli_task);
        if (xReturned != pdPASS)
        {
                NRF_LOG_ERROR("CLI task not created.");
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
}



void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
        NRF_LOG_INFO("Task %s: Stack Overflow", pcTaskName);

        NRF_LOG_ERROR("%s", pcTaskName); // --> sd_fstorage_sd --> write flash

        for(;; );
}


void vApplicationTickHook(void)
{
        m_total_count++;

        if (m_cli_task_is_running)
                m_cli_task_count++;

        if (m_log_task_is_running)
                m_log_task_count++;
}

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
        vTaskResume(m_logger_thread);
#endif
        vTaskResume(m_cli_task);
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void flashlog_init(void)
{
        ret_code_t ret;
        int32_t backend_id;

        ret = nrf_log_backend_flash_init(&nrf_fstorage_sd);
        APP_ERROR_CHECK(ret);

        backend_id = nrf_log_backend_add(&m_flash_log_backend, NRF_LOG_SEVERITY_WARNING);
        APP_ERROR_CHECK_BOOL(backend_id >= 0);

        backend_id = nrf_log_backend_add(&m_crash_log_backend, NRF_LOG_SEVERITY_ERROR);
        APP_ERROR_CHECK_BOOL(backend_id >= 0);

        nrf_log_backend_enable(&m_flash_log_backend);
        nrf_log_backend_enable(&m_crash_log_backend);
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_scan_start(&m_scan);
        APP_ERROR_CHECK(err_code);

        err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
        ret_code_t err_code;

        switch(p_scan_evt->scan_evt_id)
        {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
                err_code = p_scan_evt->params.connecting_err.err_code;
                APP_ERROR_CHECK(err_code);
                break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
                ble_gap_evt_connected_t const * p_connected =
                        p_scan_evt->params.connected.p_connected;
                // Scan is automatically stopped by the connection.
                NRF_LOG_INFO("Connecting to target 0x%02x%02x%02x%02x%02x%02x",
                             p_connected->peer_addr.addr[0],
                             p_connected->peer_addr.addr[1],
                             p_connected->peer_addr.addr[2],
                             p_connected->peer_addr.addr[3],
                             p_connected->peer_addr.addr[4],
                             p_connected->peer_addr.addr[5]
                             );
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
        } break;
        default:
                break;
        }
}

static void scan_init(void)
{
        ret_code_t err_code;
        nrf_ble_scan_init_t init_scan;

        memset(&init_scan, 0, sizeof(init_scan));

        init_scan.connect_if_match = true;
        init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

        err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
        APP_ERROR_CHECK(err_code);

        // Setting filters for scanning.
        err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
        ret_code_t ret_val;

        NRF_LOG_DEBUG("Receiving data.");
        NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

        for (uint32_t i = 0; i < data_len; i++)
        {
                do
                {
                        ret_val = app_uart_put(p_data[i]);
                        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
                        {
                                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                                APP_ERROR_CHECK(ret_val);
                        }
                } while (ret_val == NRF_ERROR_BUSY);
        }
        if (p_data[data_len-1] == '\r')
        {
                while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint16_t index = 0;
        uint32_t ret_val;

        switch (p_event->evt_type)
        {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
                UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                index++;

                if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
                {
                        NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                        NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                        do
                        {
                                for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
                                {
                                        ret_val = ble_nus_c_string_send(&m_ble_nus_c[i], data_array, index);
//                                        if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
//                                        {
//                                                APP_ERROR_CHECK(ret_val);
//                                        }
                                }
                        } while (ret_val == NRF_ERROR_RESOURCES);

                        index = 0;
                }
                break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
                NRF_LOG_ERROR("Communication error occurred while handling UART.");
                APP_ERROR_HANDLER(p_event->data.error_communication);
                break;

        case APP_UART_FIFO_ERROR:
                NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
                APP_ERROR_HANDLER(p_event->data.error_code);
                break;

        default:
                break;
        }
}


/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_c_evt)
{
        ret_code_t err_code;

        switch (p_ble_nus_c_evt->evt_type)
        {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
                NRF_LOG_INFO("NUS Service discovered on conn_handle 0x%x",
                             p_ble_nus_c_evt->conn_handle);

                err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_c_evt->conn_handle, &p_ble_nus_c_evt->handles);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_INFO("Before enable the tx notification");
                NRF_LOG_HEXDUMP_DEBUG(p_ble_nus_c, sizeof(ble_nus_c_t));
                err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_INFO("Connected to device with Nordic UART Service.\n\n");
                break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
                ble_nus_chars_received_uart_print(p_ble_nus_c_evt->p_data, p_ble_nus_c_evt->data_len);
                break;

        case BLE_NUS_C_EVT_DISCONNECTED:
                NRF_LOG_INFO("Conn_handle %d is disconnected", p_ble_nus_c_evt->conn_handle);
                NRF_LOG_INFO("Disconnected.");
                scan_start();
                break;
        }
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
        if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
        {
                nrf_saadc_value_t adc_result;
                uint16_t batt_lvl_in_milli_volts;
                uint8_t percentage_batt_lvl;
                uint32_t err_code;

                adc_result = p_event->data.done.p_buffer[0];

                err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
                APP_ERROR_CHECK(err_code);

                batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                          DIODE_FWD_VOLT_DROP_MILLIVOLTS;
                percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);

                NRF_LOG_DEBUG("Battery in percentage %03d%%", percentage_batt_lvl);
//                err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl, BLE_CONN_HANDLE_ALL);
//                if ((err_code != NRF_SUCCESS) &&
//                    (err_code != NRF_ERROR_INVALID_STATE) &&
//                    (err_code != NRF_ERROR_RESOURCES) &&
//                    (err_code != NRF_ERROR_BUSY) &&
//                    (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//                    )
//                {
//                        APP_ERROR_HANDLER(err_code);
//                }
                nrf_drv_saadc_uninit();
        }
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
        ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
        APP_ERROR_CHECK(err_code);

        nrf_saadc_channel_config_t config =
                NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
        err_code = nrf_drv_saadc_channel_init(0, &config);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
        APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the Battery measurement timer time-out.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void battery_level_meas_timeout_handler(TimerHandle_t xTimer)
{
        UNUSED_PARAMETER(xTimer);

        ret_code_t err_code;
        adc_configure();
        err_code = nrf_drv_saadc_sample();
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
        // Initialize timer module.
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);

        // Create timers.
        m_battery_timer = xTimerCreate("BATT",
                                       BATTERY_LEVEL_MEAS_INTERVAL,
                                       pdTRUE,
                                       NULL,
                                       battery_level_meas_timeout_handler);
        /* Error checking */
        if ( (NULL == m_battery_timer))
        {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
        uint32_t data_length;
        if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
        {
                m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
        }
        else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
                NRF_LOG_INFO("gatt_event: Data len is set to 0x%X (%d)", data_length, data_length);
                m_ble_nus_max_data_len = data_length;
        }
        NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                      p_gatt->att_mtu_desired_central,
                      p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
        ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(err_code);
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
        uint32_t err_code;
        app_uart_comm_params_t const comm_params =
        {
                .rx_pin_no    = RX_PIN_NUMBER,
                .tx_pin_no    = TX_PIN_NUMBER,
                .rts_pin_no   = RTS_PIN_NUMBER,
                .cts_pin_no   = CTS_PIN_NUMBER,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity   = false,
#if defined (UART_PRESENT)
                .baud_rate    = NRF_UART_BAUDRATE_115200
#else
                .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
        };

        APP_UART_FIFO_INIT(&comm_params,
                           UART_RX_BUF_SIZE,
                           UART_TX_BUF_SIZE,
                           uart_event_handle,
                           APP_IRQ_PRIORITY_LOWEST,
                           err_code);
        APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief   Function for starting application timers.
 * @details Timers are run after the scheduler has started.
 */
static void application_timers_start(void)
{
        // Start application timers.
        if (pdPASS != xTimerStart(m_battery_timer, OSTIMER_WAIT_FOR_QUEUE))
        {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
        ret_code_t err_code;

        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);

        // Prepare wakeup buttons.
        err_code = bsp_btn_ble_sleep_mode_prepare();
        APP_ERROR_CHECK(err_code);

        // Go to system-off mode (this function will not return; wakeup will cause a reset).
        err_code = sd_power_system_off();
        APP_ERROR_CHECK(err_code);
}




/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
        uint32_t err_code;

        // For readability.
        ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:
                NRF_LOG_INFO("\n\n Connection 0x%x established, starting DB discovery.\n\n",
                             p_gap_evt->conn_handle);

                APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

                err_code = ble_nus_c_handles_assign(&m_ble_nus_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
                APP_ERROR_CHECK(err_code);

                // Update LEDs status and check whether it is needed to look for more
                // peripherals to connect to.
                switch (p_gap_evt->conn_handle)
                {
                case 0:
                        bsp_board_led_on(CENTRAL_CONNECTED_1_LED);
                        break;
                case 1:
                        bsp_board_led_on(CENTRAL_CONNECTED_2_LED);
                        break;
                }

                if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
                {
                        bsp_board_led_off(CENTRAL_SCANNING_LED);
                        //bsp_board_led_off(CENTRAL_CONNECTED_LED);

                        //NRF_LOG_HEXDUMP_INFO(&m_traceBuffer, sizeof(m_traceBuffer));
                }
                else
                {
                        // Resume scanning.
                        err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
                        APP_ERROR_CHECK(err_code);
                        scan_start();
                }
                break;

        case BLE_GAP_EVT_DISCONNECTED:
        {
                NRF_LOG_INFO("Central link 0x%x disconnected (reason: 0x%x)",
                             p_gap_evt->conn_handle,
                             p_gap_evt->params.disconnected.reason);

                switch (p_gap_evt->conn_handle)
                {
                case 0:
                        bsp_board_led_off(CENTRAL_CONNECTED_1_LED);
                        break;
                case 1:
                        bsp_board_led_off(CENTRAL_CONNECTED_2_LED);
                        break;
                }

                if (ble_conn_state_central_conn_count() == 0)
                {
                        // Turn off the LED that indicates the connection.
                        //bsp_board_led_off(CENTRAL_CONNECTED_LED);
                        err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
                        APP_ERROR_CHECK(err_code);
                }
                scan_start();
        }
        break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
                // Accept parameters requested by peer.
                err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                        &p_gap_evt->params.conn_param_update_request.conn_params);
                APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
                NRF_LOG_DEBUG("PHY update request.");
                ble_gap_phys_t const phys =
                {
                        .rx_phys = BLE_GAP_PHY_AUTO,
                        .tx_phys = BLE_GAP_PHY_AUTO,
                };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
                // We have not specified a timeout for scanning, so only connection attemps can timeout.
                if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
                {
                        NRF_LOG_DEBUG("Connection request timed out.");
                }
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                NRF_LOG_DEBUG("GATT Client Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                NRF_LOG_DEBUG("GATT Server Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        default:
                // No implementation needed.
                break;
        }
}


/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
        ret_code_t err_code;
        ble_nus_c_init_t init;

        init.evt_handler = ble_nus_c_evt_handler;

        for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
        {
                err_code = ble_nus_c_init(&m_ble_nus_c[i], &init);
                APP_ERROR_CHECK(err_code);
        }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
        ret_code_t err_code;

        err_code = nrf_sdh_enable_request();
        APP_ERROR_CHECK(err_code);

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        uint32_t ram_start = 0;
        err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
        APP_ERROR_CHECK(err_code);

        ble_cfg_t ble_cfg;
        // Configure the GATTS attribute table.
        memset(&ble_cfg, 0x00, sizeof(ble_cfg));
        ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = NRF_SDH_BLE_PERIPHERAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.central_role_count = NRF_SDH_BLE_CENTRAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.qos_channel_survey_role_available = false; /* Enable channel survey role */

        err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, &ram_start);
        if (err_code != NRF_SUCCESS)
        {
                NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_GAP_CFG_ROLE_COUNT.",
                              nrf_strerror_get(err_code));
        }

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);


        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
        NRF_LOG_DEBUG("call to ble_nus_c_on_db_disc_evt for instance %d and link 0x%x!",
                      p_evt->conn_handle,
                      p_evt->conn_handle);
        ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);

}

/**@brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
        ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
        ret_code_t err_code;

        switch (event)
        {
        case BSP_EVENT_SLEEP:
                sleep_mode_enter();
                break;

        case BSP_EVENT_DISCONNECT:
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                        APP_ERROR_CHECK(err_code);
                }
                break;

        case BSP_EVENT_WHITELIST_OFF:
//                if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
//                {
//                        err_code = ble_advertising_restart_without_whitelist(&m_advertising);
//                        if (err_code != NRF_ERROR_INVALID_STATE)
//                        {
//                                APP_ERROR_CHECK(err_code);
//                        }
//                }
                break;

        default:
                break;
        }
}

/* From https://devzone.nordicsemi.com/question/70989/fpu-divide-by-0-and-high-current-consumption/
 * */
#define FPU_EXCEPTION_MASK 0x0000009F

void FPU_IRQHandler(void)
{
        uint32_t *fpscr = (uint32_t *)(FPU->FPCAR+0x40);
        (void)__get_FPSCR();

        *fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
        ret_code_t err_code = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
        ret_code_t err_code;
        bsp_event_t startup_event;

        err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
        //err_code = bsp_init(BSP_INIT_NONE, bsp_event_handler);
        APP_ERROR_CHECK(err_code);

        err_code = bsp_btn_ble_init(NULL, &startup_event);
        APP_ERROR_CHECK(err_code);

        *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
        UNUSED_PARAMETER(arg);
        m_log_task_is_running = true;
        while (1)
        {
                NRF_LOG_FLUSH();

                vTaskSuspend(NULL); // Suspend myself
        }
        m_log_task_is_running = false;
}
#endif //NRF_LOG_ENABLED


/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
        ret_code_t err_code = nrf_drv_clock_init();
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
        bool erase_bonds;

        // Initialize modules.
        uart_init();
        log_init();
        clock_init();

        nrf_drv_clock_lfclk_request(NULL);
        APP_ERROR_CHECK(nrf_stack_guard_init());

        // Do not start any interrupt that uses system functions before system initialisation.
        // The best solution is to start the OS before any other initalisation.
#if NRF_LOG_ENABLED
        // Start execution.
        if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
        {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
#endif
        if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
        {
                APP_ERROR_CHECK(nrf_cli_init(&m_cli, NULL, true, true, NRF_LOG_SEVERITY_INFO));
                nrf_cli_freertos_init(nrf_cli_freertos_init, &m_cli);
        }

        // Activate deep sleep mode.
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

        // Configure and initialize the BLE stack.
        ble_stack_init();

        // Initialize modules.
        timers_init();
        buttons_leds_init(&erase_bonds);

        scan_init();
        gatt_init();
        db_discovery_init();

        nus_c_init();
        ble_conn_state_init();

        application_timers_start();

        // Create a FreeRTOS task for the BLE stack.
        // The task will run advertising_start() before entering its loop.
        nrf_sdh_freertos_init(scan_start, NULL);

        flashlog_init();
        NRF_LOG_INFO("BLE Central NUS x 2 with FreeRTOS.");



        // Start FreeRTOS scheduler.
        vTaskStartScheduler();

        for (;;)
        {
                APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
        }
}
