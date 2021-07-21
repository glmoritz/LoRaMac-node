/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer and low power modes management
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
 *              (C)2013-2017 Semtech - STMicroelectronics
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    MCD Application Team (C)( STMicroelectronics International )
 */
#include <math.h>
#include <time.h>
#include "utilities.h"
#include "delay.h"
#include "board.h"
#include "timer.h"
#include "systime.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "labscim_platform_socket.h"
#include "labscim_protocol.h"
#include "labscim_socket.h"

// MCU Wake Up Time
#define MIN_ALARM_DELAY                             1 // in ticks


#define LABSCIM_RTC_ALARM (0x2424)
uint64_t gLabscimCurrentAlarm=0;

/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static bool RtcInitialized = false;

/*!
 * \brief Indicates if the RTC Wake Up Time is calibrated or not
 */
static bool McuWakeUpTimeInitialized = false;

extern uint64_t gTimeReference;
uint64_t gCurrentTime=0;
uint32_t gCurrentTimerContext=0;
uint32_t gRTCBackup[2] = {0,0};



void labscim_set_time(uint64_t time)
{
	gCurrentTime = time ;	
}

void labscim_time_event(struct labscim_time_event* msg)
{
	//time to update the timer
	gCurrentTime = msg->current_time_us;	
	if(msg->time_event_id == LABSCIM_RTC_ALARM)
	{
        RTC_IRQHandler();		
	}
	free(msg);	
	return;
}

/*!
 * \brief Get the current time from calendar in ticks
 *
 * \param [IN] date           Pointer to RTC_DateStruct
 * \param [IN] time           Pointer to RTC_TimeStruct
 * \retval calendarValue Time in ticks
 */
//static uint64_t RtcGetCalendarValue( RTC_DateTypeDef* date, RTC_TimeTypeDef* time );

void RtcInit(void)
{
    if (RtcInitialized == false)
    {
        gCurrentTime = 0;
        gLabscimCurrentAlarm = -1;
        RtcInitialized = true;
    }
}

/*!
 * \brief Sets the RTC timer reference, sets also the RTC_DateStruct and RTC_TimeStruct
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcSetTimerContext( void )
{    
    gCurrentTimerContext = (gTimeReference+gCurrentTime)/1000;
    return (uint32_t)gCurrentTimerContext;
}

/*!
 * \brief Gets the RTC timer reference
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcGetTimerContext( void )
{
    return (uint32_t)gCurrentTimerContext;
}

/*!
 * \brief returns the wake up time in ticks
 *
 * \retval wake up time in ticks
 */
uint32_t RtcGetMinimumTimeout( void )
{
    return( MIN_ALARM_DELAY );
}

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint32_t RtcMs2Tick( uint32_t milliseconds )
{
    return milliseconds;
}

/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
uint32_t RtcTick2Ms( uint32_t tick )
{    
    return tick;
}

/*!
 * \brief a delay of delay ms by polling RTC
 *
 * \param[IN] delay in ms
 */
void RtcDelayMs( uint32_t delay )
{
    DelayMsMcu(delay);
}

/*!
 * \brief Sets the alarm
 *
 * \note The alarm is set at now (read in this function) + timeout
 *
 * \param timeout Duration of the Timer ticks
 */
void RtcSetAlarm( uint32_t timeout )
{
    uint16_t ctrue = 1;
	uint32_t seq;
    RtcStopAlarm();
    gLabscimCurrentAlarm = set_time_event(gNodeOutputBuffer, LABSCIM_RTC_ALARM, ctrue, (timeout-RtcGetTimerElapsedTime( ))*1000);
}

void RtcStopAlarm( void )
{
   if (gLabscimCurrentAlarm != 0)
    {
        cancel_time_event(gNodeOutputBuffer, gLabscimCurrentAlarm);
    }
}

void RtcStartAlarm( uint32_t timeout )
{
    RtcSetAlarm(timeout);
}

uint32_t RtcGetTimerValue( void )
{
    return (uint32_t)((gTimeReference+gCurrentTime)/1000);
}

uint32_t RtcGetTimerElapsedTime( void )
{
 return( (gTimeReference+gCurrentTime) / 1000) - gCurrentTimerContext;
}

void RtcSetMcuWakeUpTime( void )
{

}

int16_t RtcGetMcuWakeUpTime( void )
{
    return 0;
}

static uint64_t RtcGetCalendarValue( /*RTC_DateTypeDef* date, RTC_TimeTypeDef* time*/ )
{
    return(( gTimeReference+gCurrentTime)/1000 );
}

uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
    //RTC_TimeTypeDef time ;
    //RTC_DateTypeDef date;
    //uint32_t ticks;

    //uint64_t calendarValue = RtcGetCalendarValue( &date, &time );

    //uint32_t seconds = ( uint32_t )( calendarValue >> N_PREDIV_S );

    //ticks =  ( uint32_t )calendarValue & PREDIV_S;

    //*milliseconds = RtcTick2Ms( ticks );

    //return seconds;

    *milliseconds = (uint16_t)(((gTimeReference+gCurrentTime)%1000000ull)/1000ull);
    return (uint32_t)((gTimeReference+gCurrentTime)/1000000);
}

/*!
 * \brief RTC IRQ Handler of the RTC Alarm
 */
void RTC_IRQHandler( void )
{
    HAL_RTC_AlarmAEventCallback(  );   
}

/*!
 * \brief  Alarm A callback.
 *
 * \param [IN] hrtc RTC handle
 */
void HAL_RTC_AlarmAEventCallback( /*RTC_HandleTypeDef *hrtc*/ )
{
    TimerIrqHandler( );
}

void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{
    gRTCBackup[0] = data0;
    gRTCBackup[1] = data1;
 }

void RtcBkupRead( uint32_t *data0, uint32_t *data1 )
{
   *data0 = gRTCBackup[0];
   *data1 = gRTCBackup[1];
}

void RtcProcess( void )
{
    // Not used on this platform.
}

TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
    return ( TimerTime_t ) period;
}
