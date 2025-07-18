/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "utilities.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "timer.h"
#include "board-config.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "board.h"
#include <pthread.h> // pthread_mutex_t, pthread_mutexattr_t,
                     // pthread_mutexattr_init, pthread_mutexattr_setpshared,
                     // pthread_mutex_init, pthread_mutex_destroy
#include <stdlib.h>
#include <string.h>
#include "labscim_platform_socket.h"
extern uint8_t mac_addr[];

/*!
 * Unique Devices IDs register set ( STM32L0xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )

/*!
 * LED GPIO pins objects
 */
Gpio_t Led1;
Gpio_t Led2;
Gpio_t Led3;
Gpio_t Led4;

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

/*!
 * System Clock Configuration
 */
static void SystemClockConfig( void );

/*!
 * Used to measure and calibrate the system wake-up time from STOP mode
 */
static void CalibrateSystemWakeupTime( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );

/*!
 * Timer used at first boot to calibrate the SystemWakeupTime
 */
static TimerEvent_t CalibrateSystemWakeupTimeTimer;

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/*!
 * Flag used to indicate if board is powered from the USB
 */
static bool UsbIsConnected = false;

/*!
 * UART2 FIFO buffers size
 */
#define UART2_FIFO_TX_SIZE                                256
#define UART2_FIFO_RX_SIZE                                1024

uint8_t Uart2TxBuffer[UART2_FIFO_TX_SIZE];
//uint8_t Uart2RxBuffer[UART2_FIFO_RX_SIZE];

/*!
 * Flag to indicate if the SystemWakeupTime is Calibrated
 */
static volatile bool SystemWakeupTimeCalibrated = false;

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
static void OnCalibrateSystemWakeupTimeTimerEvent( void* context )
{
    // RtcSetMcuWakeUpTime( );
    SystemWakeupTimeCalibrated = true;
}

void BoardCriticalSectionBegin( uint32_t *mask )
{
    // *mask = __get_PRIMASK( );
    // __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
    // __set_PRIMASK( *mask );
}

void BoardInitPeriph( void )
{

}

void BoardInitMcu( void )
{
    if (McuInitialized == false)
    {
        uint8_t *inbuffername, *outbuffername;
        inbuffername = (uint8_t *)malloc(sizeof(uint8_t) * strlen(gNodeName) + 4);
        if (inbuffername == NULL)
        {
            perror("\nMalloc\n");
            return;
        }
        memcpy(inbuffername + 1, gNodeName, strlen(gNodeName));
        memcpy(inbuffername + strlen(gNodeName) + 1, "in", 2);
        inbuffername[0] = '/';
        inbuffername[strlen(gNodeName) + 3] = 0;

        outbuffername = (uint8_t *)malloc(sizeof(uint8_t) * strlen(gNodeName) + 5);
        if (outbuffername == NULL)
        {
            perror("\nMalloc\n");
            return;
        }
        memcpy(outbuffername + 1, gNodeName, strlen(gNodeName));
        memcpy(outbuffername + strlen(gNodeName) + 1, "out", 3);
        outbuffername[0] = '/';
        outbuffername[strlen(gNodeName) + 4] = 0;

        gNodeOutputBuffer = (buffer_circ_t *)malloc(sizeof(buffer_circ_t));
        if (gNodeOutputBuffer == NULL)
        {
            perror("\nMalloc\n");
            return;
        }
        labscim_buffer_init(gNodeOutputBuffer, outbuffername, gBufferSize, 0);
        labscim_ll_init_list(&gCommands);
        gNodeInputBuffer = (buffer_circ_t *)malloc(sizeof(buffer_circ_t));
        if (gNodeInputBuffer == NULL)
        {
            perror("\nMalloc\n");
            return;
        }
        labscim_buffer_init(gNodeInputBuffer, inbuffername, gBufferSize, 1);
        free(inbuffername);
        free(outbuffername);
        node_is_ready(gNodeOutputBuffer);
        while (!gBootReceived)
        {
            //shared memory communication
            pthread_mutex_lock(gNodeInputBuffer->mutex.mutex);

            labscim_socket_handle_input(gNodeInputBuffer, &gCommands);

            while (gCommands.count == 0)
            {
                pthread_cond_wait(gNodeInputBuffer->mutex.more, gNodeInputBuffer->mutex.mutex);
                labscim_socket_handle_input(gNodeInputBuffer, &gCommands);
            }
            pthread_cond_signal(gNodeInputBuffer->mutex.less);
            pthread_mutex_unlock(gNodeInputBuffer->mutex.mutex);
            socket_process_all_commands();
        }      

        // HAL_Init( );

        // // LEDs
        // GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        // GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        // GpioInit( &Led3, LED_3, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        // GpioInit( &Led4, LED_4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

        // SystemClockConfig( );

        UsbIsConnected = true;

        // FifoInit( &Uart2.FifoTx, Uart2TxBuffer, UART2_FIFO_TX_SIZE );
        // FifoInit( &Uart2.FifoRx, Uart2RxBuffer, UART2_FIFO_RX_SIZE );
        // // Configure your terminal for 8 Bits data (7 data bit + 1 parity bit), no parity and no flow ctrl
        // UartInit( &Uart2, UART_2, UART_TX, UART_RX );
        // UartConfig( &Uart2, RX_TX, 921600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );

        // RtcInit( );

        // GpioWrite( &Led1, 0 );
        // GpioWrite( &Led2, 0 );
        // GpioWrite( &Led3, 0 );
        // GpioWrite( &Led4, 0 );

        // BoardUnusedIoInit( );
        // if( GetBoardPowerSource( ) == BATTERY_POWER )
        // {
        //     // Disables OFF mode - Enables lowest power mode (STOP)
        //     LpmSetOffMode( LPM_APPLI_ID, LPM_DISABLE );
        // }
    }
    // else
    // {
    //      SystemClockReConfig( );
    // }

    // SpiInit( &SX1276.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    // SX1276IoInit( );

    if( McuInitialized == false )
    {
        McuInitialized = true;
        // SX1276IoDbgInit( );
        // SX1276IoTcxoInit( );
        // if( GetBoardPowerSource( ) == BATTERY_POWER )
        // {
        //     CalibrateSystemWakeupTime( );
        // }
    }
}

void BoardResetMcu( void )
{
    CRITICAL_SECTION_BEGIN( );
    while(1);

    // //Restart system
    // NVIC_SystemReset( );
}

void BoardDeInitMcu( void )
{
    //SpiDeInit( &SX1276.Spi );
    //SX1276IoDeInit( );
}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    uint64_t* id64 = (uint64_t*)id;
    *id64 = *((uint64_t*)mac_addr);    
}

uint16_t BoardBatteryMeasureVoltage( void )
{
    return 0;
}

uint32_t BoardGetBatteryVoltage( void )
{
    return 0;
}

uint8_t BoardGetBatteryLevel( void )
{
    return 0;
}

static void BoardUnusedIoInit( void )
{
    //HAL_DBGMCU_EnableDBGSleepMode( );
    //HAL_DBGMCU_EnableDBGStopMode( );
    //HAL_DBGMCU_EnableDBGStandbyMode( );
}

void SystemClockConfig( void )
{
//     RCC_OscInitTypeDef RCC_OscInitStruct;
//     RCC_ClkInitTypeDef RCC_ClkInitStruct;
//     RCC_PeriphCLKInitTypeDef PeriphClkInit;

//     __HAL_RCC_PWR_CLK_ENABLE( );

//     __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

//     RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
//     RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
//     RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//     RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//     RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//     RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
//     RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
//     RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_6;
//     RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLLDIV_3;
//     if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
//     {
//         assert_param( FAIL );
//     }

//     RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//     RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//     RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//     RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//     RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//     if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
//     {
//         assert_param( FAIL );
//     }

//     PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
//     PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
//     if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
//     {
//         assert_param( FAIL );
//     }

//     HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );

//     HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

//     // SysTick_IRQn interrupt configuration
//     HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

void CalibrateSystemWakeupTime( void )
{
    // if( SystemWakeupTimeCalibrated == false )
    // {
    //     TimerInit( &CalibrateSystemWakeupTimeTimer, OnCalibrateSystemWakeupTimeTimerEvent );
    //     TimerSetValue( &CalibrateSystemWakeupTimeTimer, 1000 );
    //     TimerStart( &CalibrateSystemWakeupTimeTimer );
    //     while( SystemWakeupTimeCalibrated == false )
    //     {

    //     }
//    }
}

void SystemClockReConfig( void )
{
    // __HAL_RCC_PWR_CLK_ENABLE( );
    // __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    // // Enable HSI
    // __HAL_RCC_HSI_CONFIG( RCC_HSI_ON );

    // // Wait till HSI is ready
    // while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    // {
    // }

    // // Enable PLL
    // __HAL_RCC_PLL_ENABLE( );

    // // Wait till PLL is ready
    // while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    // {
    // }

    // // Select PLL as system clock source
    // __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );

    // // Wait till PLL is used as system clock source
    // while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
    // {
    // }
}

void SysTick_Handler( void )
{
    // HAL_IncTick( );
    // HAL_SYSTICK_IRQHandler( );
}

uint8_t GetBoardPowerSource( void )
{
    // if( UsbIsConnected == false )
    // {
    //     return BATTERY_POWER;
    // }
    // else
    // {
        return USB_POWER;
    // }
}

/**
  * \brief Enters Low Power Stop Mode
  *
  * \note ARM exists the function when waking up
  */
void LpmEnterStopMode( void)
{
    CRITICAL_SECTION_BEGIN( );

    // BoardDeInitMcu( );

    // // Disable the Power Voltage Detector
    // HAL_PWR_DisablePVD( );

    // // Clear wake up flag
    // SET_BIT( PWR->CR, PWR_CR_CWUF );

    // // Enable Ultra low power mode
    // HAL_PWREx_EnableUltraLowPower( );

    // // Enable the fast wake up from Ultra low power mode
    // HAL_PWREx_EnableFastWakeUp( );

    CRITICAL_SECTION_END( );

    // // Enter Stop Mode
    // HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
}

/*!
 * \brief Exists Low Power Stop Mode
 */
void LpmExitStopMode( void )
{
    // Disable IRQ while the MCU is not running on HSI
    CRITICAL_SECTION_BEGIN( );

    // Initilizes the peripherals
    // BoardInitMcu( );

    CRITICAL_SECTION_END( );
}

/*!
 * \brief Enters Low Power Sleep Mode
 *
 * \note ARM exits the function when waking up
 */
void LpmEnterSleepMode( void)
{
    // HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void BoardLowPowerHandler( void )
{
    //labscim lowpower consists in waiting for a simulator command
    //shared memory communication
    protocol_yield(gNodeOutputBuffer);

    pthread_mutex_lock(gNodeInputBuffer->mutex.mutex);

    labscim_socket_handle_input(gNodeInputBuffer, &gCommands);

    while (gCommands.count == 0)
    {
        pthread_cond_wait(gNodeInputBuffer->mutex.more, gNodeInputBuffer->mutex.mutex);
        labscim_socket_handle_input(gNodeInputBuffer, &gCommands);
    }
    pthread_cond_signal(gNodeInputBuffer->mutex.less);
    pthread_mutex_unlock(gNodeInputBuffer->mutex.mutex);
    socket_process_all_commands();
}

/*
 * Function to be used by stdout for printf etc
 */
int _write( int fd, const void *buf, size_t count )
{
    if(count>UART2_FIFO_TX_SIZE-1)
    {
        count = UART2_FIFO_TX_SIZE-1;    
    }
    memcpy(Uart2TxBuffer,buf,count);
    Uart2TxBuffer[count] = 0;
    printf("%s",Uart2TxBuffer);    
    return count;
}

/*
 * Function to be used by stdin for scanf etc
 */
int _read( int fd, const void *buf, size_t count )
{
    size_t bytesRead = 0;
    // while( UartGetBuffer( &Uart2, ( uint8_t* )buf, count, ( uint16_t* )&bytesRead ) != 0 ){ };
    // // Echo back the character
    // while( UartPutBuffer( &Uart2, ( uint8_t* )buf, ( uint16_t )bytesRead ) != 0 ){ };
    return bytesRead;
}



#ifdef USE_FULL_ASSERT

#include <stdio.h>

/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %lu\n", file, line) */

    printf( "Wrong parameters value: file %s on line %lu\n", ( const char* )file, line );
    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif
