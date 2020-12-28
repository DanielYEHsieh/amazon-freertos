/*
 * FreeRTOS V1.4.7
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

#include "iot_config.h"

/* FreeRTOS includes. */

#include "FreeRTOS.h"
#include "task.h"

/* Demo includes */
#include "aws_demo.h"
#include "aws_dev_mode_key_provisioning.h"

/* AWS System includes. */
#include "bt_hal_manager.h"
#include "iot_system_init.h"
#include "iot_logging_task.h"

#include "nvs_flash.h"
#if !AFR_ESP_LWIP
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#endif

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_interface.h"
#include "esp_log.h"
#include "esp_bt.h"
#include <esp_task_wdt.h>
#include <esp_sntp.h>
#if CONFIG_NIMBLE_ENABLED == 1
    #include "esp_nimble_hci.h"
#else
    #include "esp_gap_ble_api.h"
    #include "esp_bt_main.h"
#endif

#include "driver/uart.h"
#include "aws_application_version.h"
#include "tcpip_adapter.h"

#include "iot_network_manager_private.h"

#include "iot_uart.h"

#if BLE_ENABLED
    #include "bt_hal_manager_adapter_ble.h"
    #include "bt_hal_manager.h"
    #include "bt_hal_gatt_server.h"

    #include "iot_ble.h"
    #include "iot_ble_config.h"
    #include "iot_ble_wifi_provisioning.h"
    #include "iot_ble_numericComparison.h"
#endif

#include "open62541.h"
#include "DHT22.h"
#include "model.h"

#define SEVER_MODE 0

/* Logging Task Defines. */
#define mainLOGGING_MESSAGE_QUEUE_LENGTH    ( 32 )
#define mainLOGGING_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE * 4 )
#define mainDEVICE_NICK_NAME                "Espressif_Demo"

#define TAG "OPCUA_ESP32"
#define SNTP_TAG "SNTP"
#define MEMORY_TAG "MEMORY"
#define ENABLE_MDNS 1

/* Static arrays for FreeRTOS+TCP stack initialization for Ethernet network connections
 * are use are below. If you are using an Ethernet connection on your MCU device it is
 * recommended to use the FreeRTOS+TCP stack. The default values are defined in
 * FreeRTOSConfig.h. */

/**
 * @brief Initializes the board.
 */
static void prvMiscInitialization( void );

#if BLE_ENABLED
/* Initializes bluetooth */
    static esp_err_t prvBLEStackInit( void );
    /** Helper function to teardown BLE stack. **/
    esp_err_t xBLEStackTeardown( void );
#endif

IotUARTHandle_t xConsoleUart;

UA_ServerConfig *config;
static UA_Boolean running = true;
static UA_Boolean isServerCreated = false;
static UA_Boolean isConnectedServer = false;

RTC_DATA_ATTR static int boot_count = 0;
static struct tm timeinfo;
static time_t now = 0;


/**
 * @brief      Retry expression or statement with exponential backoff
 *
 * @param      xCommand            The expression or statement that should be
 *                                 retried
 * @param      xSuccessStatus      The success status where a xCommand need not
 *                                 be retried any more
 * @param      ulStartingPeriodMs  The initial delay period in milliseconds
 * @param      lRetries            The number of times to retry.  xCommand will
 *                                 be tried once, and then retried n times.
 *
 * @code
 *  int a = 0;
 *  RETRY_EXPONENTIAL( printf( "foo\n" ), 4, 150, 8 );
 *  RETRY_EXPONENTIAL( a = printf( "bar\n" ), 0, 250, 8 );
 *  RETRY_EXPONENTIAL( a = printf( "bar\n" ), 0, 250, 8 );
 *  RETRY_EXPONENTIAL( a = connect_to_server(), CONNECTION_SUCCESS, 250, 8 );
 *  RETRY_EXPONENTIAL( a++, 10, 250, 8 );
 * @endcode
 *
 * @return     None
 */
#define RETRY_EXPONENTIAL(                                               \
        xCommand, xSuccessStatus, ulStartingPeriodMs, lRetries )         \
    {                                                                    \
        int32_t lRetried = 0;                                            \
        uint32_t ulPeriodMs = ulStartingPeriodMs;                        \
        int32_t lStatus;                                                 \
        for( ; lRetried <= lRetries; lRetried++ ) {                      \
            if( lRetried ) {                                             \
                configPRINTF( ( "retrying \"%s\", %d of %d, in %d ms\n", \
                                # xCommand, lRetried,                    \
                                lRetries, ulPeriodMs ) );                \
                vTaskDelay( pdMS_TO_TICKS( ulPeriodMs ) );               \
                ulPeriodMs *= 2;                                         \
            }                                                            \
            lStatus = xCommand;                                          \
            if( xSuccessStatus == lStatus ) {                            \
                break;                                                   \
            }                                                            \
            configPRINTF( ( "expected %d, got %d\n",                     \
                            xSuccessStatus, lStatus ) );                 \
        }                                                                \
    }

/**
 * @brief      Returns the file name at the end of a windows path
 *
 * @param      full_path  The full path
 *
 * @return     file name
 */
#define WIN_FILENAME( full_path ) \
    ( strrchr( full_path, '\\' ) ? strrchr( full_path, '\\' ) + 1 : full_path )

/**
 * @brief      Returns the file name at the end of a linux path
 *
 * @param      full_path  The full path
 *
 * @return     file name
 */
#define NIX_FILENAME( full_path ) \
    ( strrchr( full_path, '/' ) ? strrchr( full_path, '/' ) + 1 : full_path )

/**
 * The name of the current file, stripped of the path
 */
#define __FILENAME__    WIN_FILENAME( NIX_FILENAME( __FILE__ ) )




static UA_StatusCode
UA_ServerConfig_setUriName(UA_ServerConfig *uaServerConfig, const char *uri, const char *name)
{
    // delete pre-initialized values
    UA_String_deleteMembers(&uaServerConfig->applicationDescription.applicationUri);
    UA_LocalizedText_deleteMembers(&uaServerConfig->applicationDescription.applicationName);

    uaServerConfig->applicationDescription.applicationUri = UA_String_fromChars(uri);
    uaServerConfig->applicationDescription.applicationName.locale = UA_STRING_NULL;
    uaServerConfig->applicationDescription.applicationName.text = UA_String_fromChars(name);

    for (size_t i = 0; i < uaServerConfig->endpointsSize; i++)
    {
        UA_String_deleteMembers(&uaServerConfig->endpoints[i].server.applicationUri);
        UA_LocalizedText_deleteMembers(
            &uaServerConfig->endpoints[i].server.applicationName);

        UA_String_copy(&uaServerConfig->applicationDescription.applicationUri,
                       &uaServerConfig->endpoints[i].server.applicationUri);

        UA_LocalizedText_copy(&uaServerConfig->applicationDescription.applicationName,
                              &uaServerConfig->endpoints[i].server.applicationName);
    }

    return UA_STATUSCODE_GOOD;
}

static void opcua_client_task( )
{

    UA_Int32 sendBufferSize = 32768;
    UA_Int32 recvBufferSize = 32768;

    //ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    ESP_LOGI(TAG, "Fire up OPC UA Client.");
    UA_Client *client = UA_Client_new();
    UA_ClientConfig_setDefault(UA_Client_getConfig(client));
    UA_StatusCode status = UA_Client_connect(client, "opc.tcp://192.168.10.141:26543");
    if(status != UA_STATUSCODE_GOOD) {
        UA_Client_delete(client);
        return status;
    }
    else
    {
        printf("UA_Client_connect status good\n");
    }

    /* Read the value attribute of the node. UA_Client_readValueAttribute is a
     * wrapper for the raw read service available as UA_Client_Service_read. */
    UA_Variant value; /* Variants can hold scalar values and arrays of any type */
    UA_Variant_init(&value);
    UA_NodeId newNodeId = UA_NODEID_STRING(1, "Pressure");
    while (1)
    {
        printf("UA_Client_readValueAttribute----->\n");
        status = UA_Client_readValueAttribute(client, UA_NODEID_STRING(1, "PumpSpeed"), &value);

        if( status == UA_STATUSCODE_GOOD ) {
            printf("the value is: %f\n", *(UA_Double*)value.data);
        }
        else
        {
            printf("status %x\n", status );
        }
        //status = UA_Client_writeNodeIdAttribute( client, newNodeId,
        //                       &newNodeId);
        //printf("status %x\n", status );
    }
    /* Clean up */
    UA_Variant_clear(&value);
    UA_Client_delete(client); /* Disconnects the client internally */
    return status;
}


static void opcua_server_task(void *arg)
{

    UA_Int32 sendBufferSize = 32768;
    UA_Int32 recvBufferSize = 32768;

    //ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    ESP_LOGI(TAG, "Fire up OPC UA Server.");
    UA_Server *server = UA_Server_new();

    if( server == NULL )
    {
        configPRINTF(("server new failed...\n"));
        while(1)
        {

        }
    }
    else{
        configPRINTF(("server new success...\n"));
    }

    UA_ServerConfig *config = UA_Server_getConfig(server);
    UA_ServerConfig_setMinimalCustomBuffer(config, 4840, 0, sendBufferSize, recvBufferSize);

    const char *appUri = "open62541.esp32.server";
    UA_String hostName = UA_STRING("opcua-esp32");
#ifdef ENABLE_MDNS
    config->discovery.mdnsEnable = true;
    config->discovery.mdns.mdnsServerName = UA_String_fromChars(appUri);
    config->discovery.mdns.serverCapabilitiesSize = 2;
    UA_String *caps = (UA_String *)UA_Array_new(2, &UA_TYPES[UA_TYPES_STRING]);
    caps[0] = UA_String_fromChars("LDS");
    caps[1] = UA_String_fromChars("NA");
    config->discovery.mdns.serverCapabilities = caps;

    // We need to set the default IP address for mDNS since internally it's not able to detect it.
    tcpip_adapter_ip_info_t default_ip;
    esp_err_t ret = tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &default_ip);
    if ((ESP_OK == ret) && (default_ip.ip.addr != INADDR_ANY))
    {
        config->discovery.ipAddressListSize = 1;
        config->discovery.ipAddressList = (uint32_t *)UA_malloc(sizeof(uint32_t) * config->discovery.ipAddressListSize);
        memcpy(config->discovery.ipAddressList, &default_ip.ip.addr, sizeof(uint32_t));
        //ESP_LOGI(TAG, "Get default IP Address %s!", default_ip.ip.addr );

        ESP_LOGI(TAG, "default_ip ip: " IPSTR,
               IP2STR(&default_ip.ip));
    }
    else
    {
        ESP_LOGI(TAG, "Could not get default IP Address!");
    }
#endif
    UA_ServerConfig_setUriName(config, appUri, "OPC_UA_Server_ESP32");
    UA_ServerConfig_setCustomHostname(config, hostName);

    /* Add Information Model Objects Here */
    addLEDMethod(server);
    addCurrentTemperatureDataSourceVariable(server);
    addRelay0ControlNode(server);
    addRelay1ControlNode(server);

    ESP_LOGI(TAG, "Heap Left : %d", xPortGetFreeHeapSize());
    UA_StatusCode retval = UA_Server_run_startup(server);
    if (retval == UA_STATUSCODE_GOOD)
    {
        while (running)
        {
            UA_Server_run_iterate(server, false);
            //ESP_ERROR_CHECK(esp_task_wdt_reset());
            taskYIELD();
        }
        UA_Server_run_shutdown(server);
    }
    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
}


void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(SNTP_TAG, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
    ESP_LOGI(SNTP_TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}

static bool obtain_time(void)
{
    initialize_sntp();
    //ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    memset(&timeinfo, 0, sizeof(struct tm));
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry <= retry_count)
    {
        ESP_LOGI(SNTP_TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        //ESP_ERROR_CHECK(esp_task_wdt_reset());
    }
    time(&now);
    localtime_r(&now, &timeinfo);
    //ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    return timeinfo.tm_year > (2016 - 1900);
}


static void opc_event_handler( )
{
    if (timeinfo.tm_year < (2016 - 1900))
    {
        ESP_LOGI(SNTP_TAG, "Time is not set yet. Settting up network connection and getting time over NTP.");
        if (!obtain_time())
        {
            ESP_LOGE(SNTP_TAG, "Could not get time from NTP. Using default timestamp.");
        }
        time(&now);
    }
    localtime_r(&now, &timeinfo);
    ESP_LOGI(SNTP_TAG, "Current time: %d-%02d-%02d %02d:%02d:%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

#if SEVER_MODE
    if (!isServerCreated)
    {
        xTaskCreatePinnedToCore(opcua_server_task, "opcua_server_task", 24336, NULL, 10, NULL, 1);
        ESP_LOGI(MEMORY_TAG, "Heap size after OPC UA Task : %d", esp_get_free_heap_size());
        isServerCreated = true;
    }
#else
    if (!isConnectedServer)
    {
        xTaskCreatePinnedToCore(opcua_client_task, "opcua_client_task", 24336, NULL, 10, NULL, 1);
        ESP_LOGI(MEMORY_TAG, "Heap size after OPC UA Task : %d", esp_get_free_heap_size());
        isConnectedServer = true;
    }
#endif
}

static void disconnect_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{

}

static void iot_uart_init( void )
{
    IotUARTConfig_t xUartConfig;
    int32_t status = IOT_UART_SUCCESS;
    
    xConsoleUart = iot_uart_open( UART_NUM_0 );
    configASSERT( xConsoleUart );
    
    status = iot_uart_ioctl( xConsoleUart, eUartGetConfig, &xUartConfig );
    configASSERT( status == IOT_UART_SUCCESS );
    
    xUartConfig.ulBaudrate = 115200;
    xUartConfig.xParity = eUartParityNone;
    xUartConfig.xStopbits = eUartStopBitsOne;
    xUartConfig.ucFlowControl = true;

    status = iot_uart_ioctl( xConsoleUart, eUartSetConfig, &xUartConfig );
    configASSERT( status == IOT_UART_SUCCESS );
}

void prvWifiConnect( void )
{
    WIFINetworkParams_t xJoinAPParams;
    WIFIReturnCode_t eWiFiStatus;
    uint32_t ulInitialRetryPeriodMs = 500;
    BaseType_t xMaxRetries = 6;
    size_t xSSIDLength = 0, xPasswordLength = 0;
    const char *pcSSID = ( const char * ) clientcredentialWIFI_SSID;
    const char *pcPassword = ( const char * ) clientcredentialWIFI_PASSWORD;

    eWiFiStatus = WIFI_On();

    if( eWiFiStatus == eWiFiSuccess )
    {
        configPRINTF( ( "WiFi module initialized. Connecting to AP %s\r\n", clientcredentialWIFI_SSID ) );
    }
    else
    {
        configPRINTF( ( "WiFi module failed to initialize.\r\n" ) );

        while( 1 )
        {
        }
    }


    /* Setup parameters. */
    if( ( pcSSID == NULL ) || ( strcmp( pcSSID, "") == 0 ) )
    {
        configPRINTF(( "[Error] WiFi SSID is not configured (either null or empty).\r\n" ));
        while( 1 )
        {
        }
    }
    else
    {
        xSSIDLength = strlen( pcSSID );

        if( xSSIDLength > sizeof( xJoinAPParams.ucSSID ) )
        {
            configPRINTF(( "[Error] WiFi SSID length exceeeds allowable size of %u bytes.", sizeof( xJoinAPParams.ucSSID ) ));
            while( 1 )
            {
            }
        } 
    }

    memcpy( xJoinAPParams.ucSSID, pcSSID, xSSIDLength );
    xJoinAPParams.ucSSIDLength = xSSIDLength;
    xJoinAPParams.xSecurity = clientcredentialWIFI_SECURITY;

    if ( ( xJoinAPParams.xSecurity == eWiFiSecurityWPA2 ) ||
         ( xJoinAPParams.xSecurity == eWiFiSecurityWPA ) )
    {
        if( pcPassword != NULL )
        {
            xPasswordLength = strlen( pcPassword );
            if( xPasswordLength > sizeof( xJoinAPParams.xPassword.xWPA.cPassphrase ) )
            {
                configPRINTF(( "[Error] WiFi password exceeds allowable size of %u bytes.\r\n", sizeof( xJoinAPParams.xPassword.xWPA.cPassphrase ) ));
                while( 1 )
                {
                }
            }
            memcpy( xJoinAPParams.xPassword.xWPA.cPassphrase, pcPassword, xPasswordLength );
            xJoinAPParams.xPassword.xWPA.ucLength = xPasswordLength;
        }
        else
        {
            configPRINTF(( "[Error] WiFi security is configured as WPA2 but password is not provided.\r\n" ));
            while( 1 )
            {
            }
        }
        
    }

    RETRY_EXPONENTIAL( eWiFiStatus = WIFI_ConnectAP( &( xJoinAPParams ) ),
                       eWiFiSuccess, ulInitialRetryPeriodMs, xMaxRetries );

    if( eWiFiStatus == eWiFiSuccess )
    {
        configPRINTF( ( "WiFi Connected to AP. Creating tasks which use network...\r\n" ) );
    }
    else
    {
        configPRINTF( ( "WiFi failed to connect to AP %s.\r\n", clientcredentialWIFI_SSID ) );

        while( 1 )
        {
        }
    }
}

/*-----------------------------------------------------------*/

/**
 * @brief Application runtime entry point.
 */
int app_main( void )
{
    /* Perform any hardware initialization that does not require the RTOS to be
     * running.  */

    prvMiscInitialization();

    if( SYSTEM_Init() == pdPASS )
    {
        /* A simple example to demonstrate key and certificate provisioning in
         * microcontroller flash using PKCS#11 interface. This should be replaced
         * by production ready key provisioning mechanism. */
        vDevModeKeyProvisioning();

        #if BLE_ENABLED
            /* Initialize BLE. */
            ESP_ERROR_CHECK( esp_bt_controller_mem_release( ESP_BT_MODE_CLASSIC_BT ) );

            if( prvBLEStackInit() != ESP_OK )
            {
                configPRINTF( ( "Failed to initialize the bluetooth stack\n " ) );

                while( 1 )
                {
                }
            }
        #else
            ESP_ERROR_CHECK( esp_bt_controller_mem_release( ESP_BT_MODE_CLASSIC_BT ) );
            ESP_ERROR_CHECK( esp_bt_controller_mem_release( ESP_BT_MODE_BLE ) );
        #endif /* if BLE_ENABLED */
        /* Run all demos. */
        prvWifiConnect();
        //opc_event_handler();
        opcua_client_task();
//        while( 1 )
//        {
//            taskYIELD();
//        }
        DEMO_RUNNER_RunDemos();
    }

    /* Start the scheduler.  Initialization that requires the OS to be running,
     * including the WiFi initialization, is performed in the RTOS daemon task
     * startup hook. */
    /* Following is taken care by initialization code in ESP IDF */
    /* vTaskStartScheduler(); */
    return 0;
}

/*-----------------------------------------------------------*/
extern void vApplicationIPInit( void );
static void prvMiscInitialization( void )
{
    int32_t uartRet;
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();

    if( ( ret == ESP_ERR_NVS_NO_FREE_PAGES ) || ( ret == ESP_ERR_NVS_NEW_VERSION_FOUND ) )
    {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );

    iot_uart_init();

    #if BLE_ENABLED
        NumericComparisonInit();
    #endif

    /* Create tasks that are not dependent on the WiFi being initialized. */
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
                            tskIDLE_PRIORITY + 5,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );

#if AFR_ESP_LWIP
    configPRINTF( ("Initializing lwIP TCP stack\r\n") );
    tcpip_adapter_init();
#else
    configPRINTF( ("Initializing FreeRTOS TCP stack\r\n") );
    vApplicationIPInit();
#endif
}

/*-----------------------------------------------------------*/

#if BLE_ENABLED

    #if CONFIG_NIMBLE_ENABLED == 1
        esp_err_t prvBLEStackInit( void )
        {
            return ESP_OK;
        }


        esp_err_t xBLEStackTeardown( void )
        {
            esp_err_t xRet;

            xRet = esp_bt_controller_mem_release( ESP_BT_MODE_BLE );

            return xRet;
        }

    #else /* if CONFIG_NIMBLE_ENABLED == 1 */

        static esp_err_t prvBLEStackInit( void )
        {
            return ESP_OK;
        }

        esp_err_t xBLEStackTeardown( void )
        {
            esp_err_t xRet = ESP_OK;

            if( esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED )
            {
                xRet = esp_bluedroid_disable();
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bluedroid_deinit();
            }

            if( xRet == ESP_OK )
            {
                if( esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED )
                {
                    xRet = esp_bt_controller_disable();
                }
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bt_controller_deinit();
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bt_controller_mem_release( ESP_BT_MODE_BLE );
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bt_controller_mem_release( ESP_BT_MODE_BTDM );
            }

            return xRet;
        }
    #endif /* if CONFIG_NIMBLE_ENABLED == 1 */
#endif /* if BLE_ENABLED */

/*-----------------------------------------------------------*/


#if BLE_ENABLED
/*-----------------------------------------------------------*/

    static void prvUartCallback( IotUARTOperationStatus_t xStatus,
                                      void * pvUserContext )
    {
        SemaphoreHandle_t xUartSem = ( SemaphoreHandle_t ) pvUserContext;
        configASSERT( xUartSem != NULL );
        xSemaphoreGive( xUartSem );
    }

  
    BaseType_t getUserMessage( INPUTMessage_t * pxINPUTmessage,
                               TickType_t xAuthTimeout )
    {
        BaseType_t xReturnMessage = pdFALSE;
        SemaphoreHandle_t xUartSem;
        int32_t status, bytesRead = 0;
        uint8_t *pucResponse;

        xUartSem = xSemaphoreCreateBinary();

        
        /* BLE Numeric comparison response is one character (y/n). */
        pucResponse = ( uint8_t * ) pvPortMalloc( sizeof( uint8_t ) );

        if( ( xUartSem != NULL ) && ( pucResponse != NULL ) )
        {
            iot_uart_set_callback( xConsoleUart, prvUartCallback, xUartSem );

            status = iot_uart_read_async( xConsoleUart, pucResponse, 1 );

            /* Wait for  auth timeout to get the input character. */
            xSemaphoreTake( xUartSem, xAuthTimeout );

            /* Cancel the uart operation if the character is received or timeout occured. */
            iot_uart_cancel( xConsoleUart );

            /* Reset the callback. */
            iot_uart_set_callback( xConsoleUart, NULL, NULL );

            iot_uart_ioctl( xConsoleUart, eGetRxNoOfbytes, &bytesRead );

            if( bytesRead == 1 )
            {
                pxINPUTmessage->pcData = pucResponse;
                pxINPUTmessage->xDataSize = 1;
                xReturnMessage = pdTRUE;
            }

            vSemaphoreDelete( xUartSem );
        }

        return xReturnMessage;
    }
#endif /* if BLE_ENABLED */

/*-----------------------------------------------------------*/

extern void esp_vApplicationTickHook();
void IRAM_ATTR vApplicationTickHook()
{
    esp_vApplicationTickHook();
}

/*-----------------------------------------------------------*/
extern void esp_vApplicationIdleHook();
void vApplicationIdleHook()
{
    esp_vApplicationIdleHook();
}

/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
}

#if !AFR_ESP_LWIP
/*-----------------------------------------------------------*/
void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
    uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
    system_event_t evt;

    if( eNetworkEvent == eNetworkUp )
    {
        /* Print out the network configuration, which may have come from a DHCP
         * server. */
        FreeRTOS_GetAddressConfiguration(
            &ulIPAddress,
            &ulNetMask,
            &ulGatewayAddress,
            &ulDNSServerAddress );

        evt.event_id = SYSTEM_EVENT_STA_GOT_IP;
        evt.event_info.got_ip.ip_changed = true;
        evt.event_info.got_ip.ip_info.ip.addr = ulIPAddress;
        evt.event_info.got_ip.ip_info.netmask.addr = ulNetMask;
        evt.event_info.got_ip.ip_info.gw.addr = ulGatewayAddress;
        esp_event_send( &evt );
    }
}
#endif
