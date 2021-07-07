/*!
 * \file      radio.c
 *
 * \brief     Radio driver API definition
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
#include <math.h>
#include <string.h>
#include "utilities.h"
#include "timer.h"
#include "delay.h"
#include "radio.h"
#include "sx126x.h"
#include "sx126x-board.h"
#include "board.h"
#include "labscim_protocol.h"
#include "labscim_socket.h"
#include "labscim-lora-radio-protocol.h"
#include "labscim_platform_socket.h"

struct labscim_ll gReceivedCommands;
float gRSSI_dbm=-5000.0;
SX126x_t SX126x;

/*!
 * \brief Initializes the radio
 *
 * \param [IN] events Structure containing the driver callback functions
 */
void RadioInit(RadioEvents_t *events);

/*!
 * Return current radio status
 *
 * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
RadioState_t RadioGetStatus(void);

/*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
void RadioSetModem(RadioModems_t modem);

/*!
 * \brief Sets the channel frequency
 *
 * \param [IN] freq         Channel RF frequency
 */
void RadioSetChannel(uint32_t freq);

/*!
 * \brief Checks if the channel is free for the given time
 *
 * \remark The FSK modem is always used for this task as we can select the Rx bandwidth at will.
 *
 * \param [IN] freq                Channel RF frequency in Hertz
 * \param [IN] rxBandwidth         Rx bandwidth in Hertz
 * \param [IN] rssiThresh          RSSI threshold in dBm
 * \param [IN] maxCarrierSenseTime Max time in milliseconds while the RSSI is measured
 *
 * \retval isFree         [true: Channel is free, false: Channel is not free]
 */
bool RadioIsChannelFree(uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime);

/*!
 * \brief Generates a 32 bits random value based on the RSSI readings
 *
 * \remark This function sets the radio in LoRa modem mode and disables
 *         all interrupts.
 *         After calling this function either Radio.SetRxConfig or
 *         Radio.SetTxConfig functions must be called.
 *
 * \retval randomValue    32 bits random value
 */
uint32_t RadioRandom(void);

/*!
 * \brief Sets the reception parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A ( set to 0 )
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] symbTimeout  Sets the RxSingle timeout value
 *                          FSK : timeout in number of bytes
 *                          LoRa: timeout in symbols
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
void RadioSetRxConfig(RadioModems_t modem, uint32_t bandwidth,
                      uint32_t datarate, uint8_t coderate,
                      uint32_t bandwidthAfc, uint16_t preambleLen,
                      uint16_t symbTimeout, bool fixLen,
                      uint8_t payloadLen,
                      bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
                      bool iqInverted, bool rxContinuous);

/*!
 * \brief Sets the transmission parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] power        Sets the output power [dBm]
 * \param [IN] fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] timeout      Transmission timeout [ms]
 */
void RadioSetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev,
                      uint32_t bandwidth, uint32_t datarate,
                      uint8_t coderate, uint16_t preambleLen,
                      bool fixLen, bool crcOn, bool FreqHopOn,
                      uint8_t HopPeriod, bool iqInverted, uint32_t timeout);

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool RadioCheckRfFrequency(uint32_t frequency);

/*!
 * \brief Computes the packet time on air in ms for the given payload
 *
 * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 *
 * \retval airTime        Computed airTime (ms) for the given packet payload length
 */
uint32_t RadioTimeOnAir(RadioModems_t modem, uint32_t bandwidth,
                        uint32_t datarate, uint8_t coderate,
                        uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                        bool crcOn);

/*!
 * \brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void RadioSend(uint8_t *buffer, uint8_t size);

/*!
 * \brief Sets the radio in sleep mode
 */
void RadioSleep(void);

/*!
 * \brief Sets the radio in standby mode
 */
void RadioStandby(void);

/*!
 * \brief Sets the radio in reception mode for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRx(uint32_t timeout);

/*!
 * \brief Start a Channel Activity Detection
 */
void RadioStartCad(void);

/*!
 * \brief Sets the radio in continuous wave transmission mode
 *
 * \param [IN]: freq       Channel RF frequency
 * \param [IN]: power      Sets the output power [dBm]
 * \param [IN]: time       Transmission mode timeout [s]
 */
void RadioSetTxContinuousWave(uint32_t freq, int8_t power, uint16_t time);

/*!
 * \brief Reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
int16_t RadioRssi(RadioModems_t modem);

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void RadioWrite(uint32_t addr, uint8_t data);

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \retval data Register value
 */
uint8_t RadioRead(uint32_t addr);

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void RadioWriteBuffer(uint32_t addr, uint8_t *buffer, uint8_t size);

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void RadioReadBuffer(uint32_t addr, uint8_t *buffer, uint8_t size);

/*!
 * \brief Sets the maximum payload length.
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] max        Maximum payload length in bytes
 */
void RadioSetMaxPayloadLength(RadioModems_t modem, uint8_t max);

/*!
 * \brief Sets the network to public or private. Updates the sync byte.
 *
 * \remark Applies to LoRa modem only
 *
 * \param [IN] enable if true, it enables a public network
 */
void RadioSetPublicNetwork(bool enable);

/*!
 * \brief Gets the time required for the board plus radio to get out of sleep.[ms]
 *
 * \retval time Radio plus board wakeup time in ms.
 */
uint32_t RadioGetWakeupTime(void);

/*!
 * \brief Process radio irq
 */
void RadioIrqProcess(void);

/*!
 * \brief Sets the radio in reception mode with Max LNA gain for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRxBoosted(uint32_t timeout);

/*!
 * \brief Sets the Rx duty cycle management parameters
 *
 * \param [in]  rxTime        Structure describing reception timeout value
 * \param [in]  sleepTime     Structure describing sleep timeout value
 */
void RadioSetRxDutyCycle(uint32_t rxTime, uint32_t sleepTime);

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
    {
        RadioInit,
        RadioGetStatus,
        RadioSetModem,
        RadioSetChannel,
        RadioIsChannelFree,
        RadioRandom,
        RadioSetRxConfig,
        RadioSetTxConfig,
        RadioCheckRfFrequency,
        RadioTimeOnAir,
        RadioSend,
        RadioSleep,
        RadioStandby,
        RadioRx,
        RadioStartCad,
        RadioSetTxContinuousWave,
        RadioRssi,
        RadioWrite,
        RadioRead,
        RadioWriteBuffer,
        RadioReadBuffer,
        RadioSetMaxPayloadLength,
        RadioSetPublicNetwork,
        RadioGetWakeupTime,
        RadioIrqProcess,
        // Available on SX126x only
        RadioRxBoosted,
        RadioSetRxDutyCycle};

/*
 * Local types definition
 */

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t RegValue;
} FskBandwidth_t;

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
    {
        {4800, 0x1F},
        {5800, 0x17},
        {7300, 0x0F},
        {9700, 0x1E},
        {11700, 0x16},
        {14600, 0x0E},
        {19500, 0x1D},
        {23400, 0x15},
        {29300, 0x0D},
        {39000, 0x1C},
        {46900, 0x14},
        {58600, 0x0C},
        {78200, 0x1B},
        {93800, 0x13},
        {117300, 0x0B},
        {156200, 0x1A},
        {187200, 0x12},
        {234300, 0x0A},
        {312000, 0x19},
        {373600, 0x11},
        {467000, 0x09},
        {500000, 0x00}, // Invalid Bandwidth
};

const RadioLoRaBandwidths_t Bandwidths[] = {LORA_BW_125, LORA_BW_250, LORA_BW_500};

uint8_t MaxPayloadLength = 0xFF;

uint32_t TxTimeout = 0;
uint32_t RxTimeout = 0;

bool RxContinuous = false;

bool IrqFired = false;


struct labscim_radio_response *gLastRxPacket = NULL;

/*
 * SX126x DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void RadioOnDioIrq(void *context);

/*!
 * \brief Tx timeout timer callback
 */
void RadioOnTxTimeoutIrq(void *context);

/*!
 * \brief Rx timeout timer callback
 */
void RadioOnRxTimeoutIrq(void *context);

/*
 * Private global variables
 */

/*!
 * Holds the current network type for the radio
 */
typedef struct
{
    bool Previous;
    bool Current;
} RadioPublicNetwork_t;

static RadioPublicNetwork_t RadioPublicNetwork = {false};

/*!
 * Radio callbacks variable
 */
static RadioEvents_t *RadioEvents;

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
//SX126x_t SX126x;

/*!
 * Tx and Rx timers
 */
TimerEvent_t TxTimeoutTimer;
TimerEvent_t RxTimeoutTimer;

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t RadioGetFskBandwidthRegValue(uint32_t bandwidth)
{
    uint8_t i;

    if (bandwidth == 0)
    {
        return (0x1F);
    }

    for (i = 0; i < (sizeof(FskBandwidths) / sizeof(FskBandwidth_t)) - 1; i++)
    {
        if ((bandwidth >= FskBandwidths[i].bandwidth) && (bandwidth < FskBandwidths[i + 1].bandwidth))
        {
            return FskBandwidths[i + 1].RegValue;
        }
    }
    // ERROR: Value not found
    while (1)
        ;
}

void RadioInit(RadioEvents_t *events)
{
    RadioEvents = events;

    // Initialize driver timeout timers
    TimerInit(&TxTimeoutTimer, RadioOnTxTimeoutIrq);
    TimerInit(&RxTimeoutTimer, RadioOnRxTimeoutIrq);

    IrqFired = false;
}

RadioState_t RadioGetStatus(void)
{
    uint32_t ChannelIsFree;
    RadioState_t ret;
    struct labscim_radio_response *resp;
    struct lora_radio_status radio_status;
    uint32_t sequence_number;
    sequence_number = radio_command(gNodeOutputBuffer, LORA_RADIO_GET_STATE, (void *)&radio_status, sizeof(struct lora_radio_status));
    resp = (struct labscim_radio_response *)socket_wait_for_command(LABSCIM_RADIO_RESPONSE, sequence_number);
    socket_process_all_commands();
    if (resp->radio_response_code == LORA_RADIO_GET_STATE_RESULT)
    {
        switch ((RadioMode)((struct lora_radio_status *)resp->radio_struct)->RadioMode)
        {
        case RADIO_MODE_TRANSMITTER:
        {
            ret = RF_TX_RUNNING;
            break;
        }
        case RADIO_MODE_RECEIVER:
        {
            ret = RF_RX_RUNNING;
            break;
        }
        //case MODE_CAD:
        //ret = RF_CAD;
        case RADIO_MODE_OFF:
        case RADIO_MODE_SLEEP:
        {
            ret = RF_IDLE;
            break;
        }
        default:
            //something very wrong
            while (1);
        }
        free(resp);
    }
    else
    {
        //something very wrong happened
        while (1);
    }
    return ret;
}

void RadioSetModem(RadioModems_t modem)
{
    struct lora_set_modem modem_type;
    uint32_t sequence_number;
    modem_type.Modem = (uint32_t)modem;
    sequence_number = radio_command(gNodeOutputBuffer, LORA_RADIO_SET_MODEM, (void *)&modem_type, sizeof(struct lora_set_modem));
    switch (modem)
    {
    default:
    case MODEM_FSK:
        // When switching to GFSK mode the LoRa SyncWord register value is reset
        // Thus, we also reset the RadioPublicNetwork variable
        RadioPublicNetwork.Current = false;
        break;
    case MODEM_LORA:
        // Public/Private network register is reset when switching modems
        if (RadioPublicNetwork.Current != RadioPublicNetwork.Previous)
        {
            RadioPublicNetwork.Current = RadioPublicNetwork.Previous;
            RadioSetPublicNetwork(RadioPublicNetwork.Current);
        }
        break;
    }
}

void RadioSetChannel(uint32_t freq)
{
    struct lora_set_frequency frequency_setup;
    uint32_t sequence_number;
    frequency_setup.Frequency_Hz = freq;
    sequence_number = radio_command(gNodeOutputBuffer, LORA_RADIO_SET_CHANNEL, (void *)&frequency_setup, sizeof(struct lora_set_frequency));
}

bool RadioIsChannelFree(uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime)
{
    uint32_t ChannelIsFree;
    struct labscim_radio_response *resp;
    struct lora_is_channel_free is_free;
    uint32_t sequence_number;

    is_free.Frequency_Hz = freq;
    is_free.RxBandWidth_Hz = rxBandwidth;
    is_free.RSSI_Threshold_dBm = rssiThresh;
    is_free.MaxCarrierSenseTime_us = maxCarrierSenseTime * 1000;

    sequence_number = radio_command(gNodeOutputBuffer, LORA_RADIO_IS_CHANNEL_FREE, (void *)&is_free, sizeof(struct lora_is_channel_free));
    //channel assessment is a blocking call that takes some time. omnet must go on
    protocol_yield(gNodeOutputBuffer);
    resp = (struct labscim_radio_response *)socket_wait_for_command(LABSCIM_RADIO_RESPONSE, sequence_number);
    socket_process_all_commands();
    if (resp->radio_response_code == LORA_RADIO_IS_CHANNEL_FREE_RESULT)
    {
        ChannelIsFree = ((struct lora_is_channel_free_result *)resp->radio_struct)->ChannelIsFree;
    }
    else
    {
        //something very wrong happened
        while (1)
            ;
    }
    free(resp);
    return ChannelIsFree;
}

uint32_t RadioRandom(void)
{
    uint32_t rnd;
    uint32_t sequence_number;
    union random_number param_1, param_2, param_3;
    param_1.int_number = 0;
    param_2.int_number = 0xFFFFFFFF;
    param_3.int_number = 0;
    struct labscim_signal_get_random_response *resp;
    sequence_number = get_random(gNodeOutputBuffer, 14 /*intuniform*/, param_1, param_2, param_3);
    resp = (struct labscim_signal_get_random_response *)socket_wait_for_command(LABSCIM_GET_RANDOM_RESPONSE, sequence_number);
    rnd = resp->result.int_number;
    free(resp);
    return rnd;
}

void RadioSetRxConfig(RadioModems_t modem, uint32_t bandwidth,
                      uint32_t datarate, uint8_t coderate,
                      uint32_t bandwidthAfc, uint16_t preambleLen,
                      uint16_t symbTimeout, bool fixLen,
                      uint8_t payloadLen,
                      bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                      bool iqInverted, bool rxContinuous)
{
    struct lora_set_modulation_params modulation_params;
    struct lora_set_packet_params packet_params;

    RxContinuous = rxContinuous;
    if (rxContinuous == true)
    {
        symbTimeout = 0;
    }
    if (fixLen == true)
    {
        MaxPayloadLength = payloadLen;
    }
    else
    {
        MaxPayloadLength = 0xFF;
    }

    switch (modem)
    {
    case MODEM_FSK:
        SX126x.ModulationParams.PacketType = PACKET_TYPE_GFSK;
        SX126x.ModulationParams.Params.Gfsk.BitRate = datarate;
        SX126x.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
        SX126x.ModulationParams.Params.Gfsk.Bandwidth = RadioGetFskBandwidthRegValue(bandwidth << 1); // SX126x badwidth is double sided
        SX126x.PacketParams.PacketType = PACKET_TYPE_GFSK;
        SX126x.PacketParams.Params.Gfsk.PreambleLength = (preambleLen << 3); // convert byte into bit
        SX126x.PacketParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
        SX126x.PacketParams.Params.Gfsk.SyncWordLength = 3 << 3; // convert byte into bit
        SX126x.PacketParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
        SX126x.PacketParams.Params.Gfsk.HeaderType = (fixLen == true) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;
        SX126x.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength;
        if (crcOn == true)
        {
            SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
        }
        else
        {
            SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
        }
        SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;
        RadioStandby();
        RadioSetModem((SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK) ? MODEM_FSK : MODEM_LORA);

        memcpy(&(modulation_params.ModulationParams), &SX126x.ModulationParams, sizeof(ModulationParams_t));
        memcpy(&(packet_params.PacketParams), &SX126x.PacketParams, sizeof(PacketParams_t));
        radio_command(gNodeOutputBuffer, LORA_RADIO_SET_MODULATION_PARAMS, (void *)&modulation_params, sizeof(struct lora_set_modulation_params));
        radio_command(gNodeOutputBuffer, LORA_RADIO_SET_PACKET_PARAMS, (void *)&packet_params, sizeof(struct lora_set_packet_params));
        //SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
        //SX126xSetWhiteningSeed( 0x01FF );

        RxTimeout = (uint32_t)(symbTimeout * ((1.0 / (double)datarate) * 8.0) * 1000);
        break;
    case MODEM_LORA:
        SX126x.ModulationParams.PacketType = PACKET_TYPE_LORA;
        SX126x.ModulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)datarate;
        SX126x.ModulationParams.Params.LoRa.Bandwidth = Bandwidths[bandwidth];
        SX126x.ModulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)coderate;
        if (((bandwidth == 0) && ((datarate == 11) || (datarate == 12))) ||
            ((bandwidth == 1) && (datarate == 12)))
        {
            SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
        }
        else
        {
            SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
        }
        SX126x.PacketParams.PacketType = PACKET_TYPE_LORA;
        if ((SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5) ||
            (SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6))
        {
            if (preambleLen < 12)
            {
                SX126x.PacketParams.Params.LoRa.PreambleLength = 12;
            }
            else
            {
                SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
            }
        }
        else
        {
            SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
        }
        SX126x.PacketParams.Params.LoRa.HeaderType = (RadioLoRaPacketLengthsMode_t)fixLen;
        SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
        SX126x.PacketParams.Params.LoRa.CrcMode = (RadioLoRaCrcModes_t)crcOn;
        SX126x.PacketParams.Params.LoRa.InvertIQ = (RadioLoRaIQModes_t)iqInverted;
        RadioStandby();
        RadioSetModem((SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK) ? MODEM_FSK : MODEM_LORA);

        memcpy(&(modulation_params.ModulationParams), &SX126x.ModulationParams, sizeof(ModulationParams_t));
        memcpy(&(packet_params.PacketParams), &SX126x.PacketParams, sizeof(PacketParams_t));
        radio_command(gNodeOutputBuffer, LORA_RADIO_SET_MODULATION_PARAMS, (void *)&modulation_params, sizeof(struct lora_set_modulation_params));
        radio_command(gNodeOutputBuffer, LORA_RADIO_SET_PACKET_PARAMS, (void *)&packet_params, sizeof(struct lora_set_packet_params));
        //SX126xSetLoRaSymbNumTimeout( symbTimeout );
        // Timeout Max, Timeout handled directly in SetRx function
        RxTimeout = 0xFFFF;
        break;
    }
}

void RadioSetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev,
                      uint32_t bandwidth, uint32_t datarate,
                      uint8_t coderate, uint16_t preambleLen,
                      bool fixLen, bool crcOn, bool freqHopOn,
                      uint8_t hopPeriod, bool iqInverted, uint32_t timeout)
{
    struct lora_set_modulation_params modulation_params;
    struct lora_set_packet_params packet_params;

    //this emulates the maximum/minimum power of sx1262
    if (power > 22)
    {
        power = 22;
    }
    if(power<-9)
    {
        power = -9;
    }

    modulation_params.TransmitPower_dBm = (float)power;

    switch (modem)
    {
    case MODEM_FSK:
        SX126x.ModulationParams.PacketType = PACKET_TYPE_GFSK;
        SX126x.ModulationParams.Params.Gfsk.BitRate = datarate;

        SX126x.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
        SX126x.ModulationParams.Params.Gfsk.Bandwidth = RadioGetFskBandwidthRegValue(bandwidth << 1); // SX126x badwidth is double sided
        SX126x.ModulationParams.Params.Gfsk.Fdev = fdev;

        SX126x.PacketParams.PacketType = PACKET_TYPE_GFSK;
        SX126x.PacketParams.Params.Gfsk.PreambleLength = (preambleLen << 3); // convert byte into bit
        SX126x.PacketParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
        SX126x.PacketParams.Params.Gfsk.SyncWordLength = 3 << 3; // convert byte into bit
        SX126x.PacketParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
        SX126x.PacketParams.Params.Gfsk.HeaderType = (fixLen == true) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;

        if (crcOn == true)
        {
            SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
        }
        else
        {
            SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
        }
        SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;

        RadioStandby();
        RadioSetModem((SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK) ? MODEM_FSK : MODEM_LORA);
        memcpy(&(modulation_params.ModulationParams), &SX126x.ModulationParams, sizeof(ModulationParams_t));
        memcpy(&(packet_params.PacketParams), &SX126x.PacketParams, sizeof(PacketParams_t));
        radio_command(gNodeOutputBuffer, LORA_RADIO_SET_MODULATION_PARAMS, (void *)&modulation_params, sizeof(struct lora_set_modulation_params));
        radio_command(gNodeOutputBuffer, LORA_RADIO_SET_PACKET_PARAMS, (void *)&packet_params, sizeof(struct lora_set_packet_params));
        //SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
        //SX126xSetWhiteningSeed( 0x01FF );
        break;

    case MODEM_LORA:
        SX126x.ModulationParams.PacketType = PACKET_TYPE_LORA;
        SX126x.ModulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)datarate;
        SX126x.ModulationParams.Params.LoRa.Bandwidth = Bandwidths[bandwidth];
        SX126x.ModulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)coderate;

        if (((bandwidth == 0) && ((datarate == 11) || (datarate == 12))) ||
            ((bandwidth == 1) && (datarate == 12)))
        {
            SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
        }
        else
        {
            SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
        }

        SX126x.PacketParams.PacketType = PACKET_TYPE_LORA;

        if ((SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5) ||
            (SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6))
        {
            if (preambleLen < 12)
            {
                SX126x.PacketParams.Params.LoRa.PreambleLength = 12;
            }
            else
            {
                SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
            }
        }
        else
        {
            SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
        }

        SX126x.PacketParams.Params.LoRa.HeaderType = (RadioLoRaPacketLengthsMode_t)fixLen;
        SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
        SX126x.PacketParams.Params.LoRa.CrcMode = (RadioLoRaCrcModes_t)crcOn;
        SX126x.PacketParams.Params.LoRa.InvertIQ = (RadioLoRaIQModes_t)iqInverted;

        RadioStandby();
        RadioSetModem((SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK) ? MODEM_FSK : MODEM_LORA);
        memcpy(&(modulation_params.ModulationParams), &SX126x.ModulationParams, sizeof(ModulationParams_t));
        memcpy(&(packet_params.PacketParams), &SX126x.PacketParams, sizeof(PacketParams_t));
        radio_command(gNodeOutputBuffer, LORA_RADIO_SET_MODULATION_PARAMS, (void *)&modulation_params, sizeof(struct lora_set_modulation_params));
        radio_command(gNodeOutputBuffer, LORA_RADIO_SET_PACKET_PARAMS, (void *)&packet_params, sizeof(struct lora_set_packet_params));
        break;
    }

    TxTimeout = timeout;
}

bool RadioCheckRfFrequency(uint32_t frequency)
{
    return true;
}

static uint32_t RadioGetLoRaBandwidthInHz(RadioLoRaBandwidths_t bw)
{
    uint32_t bandwidthInHz = 0;

    switch (bw)
    {
    case LORA_BW_007:
        bandwidthInHz = 7812UL;
        break;
    case LORA_BW_010:
        bandwidthInHz = 10417UL;
        break;
    case LORA_BW_015:
        bandwidthInHz = 15625UL;
        break;
    case LORA_BW_020:
        bandwidthInHz = 20833UL;
        break;
    case LORA_BW_031:
        bandwidthInHz = 31250UL;
        break;
    case LORA_BW_041:
        bandwidthInHz = 41667UL;
        break;
    case LORA_BW_062:
        bandwidthInHz = 62500UL;
        break;
    case LORA_BW_125:
        bandwidthInHz = 125000UL;
        break;
    case LORA_BW_250:
        bandwidthInHz = 250000UL;
        break;
    case LORA_BW_500:
        bandwidthInHz = 500000UL;
        break;
    }

    return bandwidthInHz;
}

static uint32_t RadioGetGfskTimeOnAirNumerator(uint32_t datarate, uint8_t coderate,
                                               uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                                               bool crcOn)
{
    const RadioAddressComp_t addrComp = RADIO_ADDRESSCOMP_FILT_OFF;
    const uint8_t syncWordLength = 3;

    return (preambleLen << 3) +
           ((fixLen == false) ? 8 : 0) +
           (syncWordLength << 3) +
           ((payloadLen +
             (addrComp == RADIO_ADDRESSCOMP_FILT_OFF ? 0 : 1) +
             ((crcOn == true) ? 2 : 0))
            << 3);
}

static uint32_t RadioGetLoRaTimeOnAirNumerator(uint32_t bandwidth,
                                               uint32_t datarate, uint8_t coderate,
                                               uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                                               bool crcOn)
{
    int32_t crDenom = coderate + 4;
    bool lowDatareOptimize = false;

    // Ensure that the preamble length is at least 12 symbols when using SF5 or
    // SF6
    if ((datarate == 5) || (datarate == 6))
    {
        if (preambleLen < 12)
        {
            preambleLen = 12;
        }
    }

    if (((bandwidth == 0) && ((datarate == 11) || (datarate == 12))) ||
        ((bandwidth == 1) && (datarate == 12)))
    {
        lowDatareOptimize = true;
    }

    int32_t ceilDenominator;
    int32_t ceilNumerator = (payloadLen << 3) +
                            (crcOn ? 16 : 0) -
                            (4 * datarate) +
                            (fixLen ? 0 : 20);

    if (datarate <= 6)
    {
        ceilDenominator = 4 * datarate;
    }
    else
    {
        ceilNumerator += 8;

        if (lowDatareOptimize == true)
        {
            ceilDenominator = 4 * (datarate - 2);
        }
        else
        {
            ceilDenominator = 4 * datarate;
        }
    }

    if (ceilNumerator < 0)
    {
        ceilNumerator = 0;
    }

    // Perform integral ceil()
    int32_t intermediate =
        ((ceilNumerator + ceilDenominator - 1) / ceilDenominator) * crDenom + preambleLen + 12;

    if (datarate <= 6)
    {
        intermediate += 2;
    }

    return (uint32_t)((4 * intermediate + 1) * (1 << (datarate - 2)));
}

uint32_t RadioTimeOnAir(RadioModems_t modem, uint32_t bandwidth,
                        uint32_t datarate, uint8_t coderate,
                        uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                        bool crcOn)
{
    uint32_t numerator = 0;
    uint32_t denominator = 1;

    switch (modem)
    {
    case MODEM_FSK:
    {
        numerator = 1000U * RadioGetGfskTimeOnAirNumerator(datarate, coderate,
                                                           preambleLen, fixLen,
                                                           payloadLen, crcOn);
        denominator = datarate;
    }
    break;
    case MODEM_LORA:
    {
        numerator = 1000U * RadioGetLoRaTimeOnAirNumerator(bandwidth, datarate,
                                                           coderate, preambleLen,
                                                           fixLen, payloadLen, crcOn);
        denominator = RadioGetLoRaBandwidthInHz(Bandwidths[bandwidth]);
    }
    break;
    }
    // Perform integral ceil()
    return (numerator + denominator - 1) / denominator;
}

void RadioSend(uint8_t *buffer, uint8_t size)
{
    struct lora_radio_payload *msg;
    struct labscim_protocol_header *resp;
    uint32_t sequence_number;

    SX126x.PacketParams.Params.LoRa.PayloadLength = size;
    SX126x.PacketParams.Params.Gfsk.PayloadLength = size;

    msg = (struct lora_radio_payload *)malloc(FIXED_SIZEOF_LORA_RADIO_PAYLOAD + size);
    if (msg == NULL)
    {
        perror("\nMalloc error \n");
        return;
    }
    /* Copy packet data to temporary storage */
    memcpy(msg->Message, buffer, size);
    msg->MessageSize_bytes = size;
    msg->SNR_db = -200;
    msg->RSSI_dbm = -200;
    msg->RX_timestamp_us = 0;
    sequence_number = radio_command(gNodeOutputBuffer, LORA_RADIO_SEND, (uint8_t *)msg, FIXED_SIZEOF_LORA_RADIO_PAYLOAD + size);
    free(msg);
    TimerSetValue(&TxTimeoutTimer, TxTimeout);
    TimerStart(&TxTimeoutTimer);
}

void RadioSleep(void)
{
    SleepParams_t params = {0};
    params.Fields.WarmStart = 1;
    RadioStandby();
    //DelayMs(2);
}

void RadioStandby(void)
{
    struct lora_set_idle si;
    si.unused = 0;
    radio_command(gNodeOutputBuffer, LORA_RADIO_SET_IDLE, (void *)&si, sizeof(struct lora_set_idle));
}

void RadioRx(uint32_t timeout)
{    
    struct lora_set_rx srx;
    srx.Timeout_us = timeout*1000;
    if (timeout != 0)
    {
        TimerSetValue(&RxTimeoutTimer, timeout);
        TimerStart(&RxTimeoutTimer);
    }
    if (RxContinuous == true)
    {
        srx.Timeout_us = ~((uint64_t)0);
    }
    radio_command(gNodeOutputBuffer, LORA_RADIO_SET_RX, (void *)&srx, sizeof(struct lora_set_rx));
    
}

void RadioRxBoosted(uint32_t timeout)
{
    RadioRx(timeout);
}

void RadioSetRxDutyCycle(uint32_t rxTime, uint32_t sleepTime)
{
    struct lora_set_rx_dutycycle srx;
    srx.Rx_window_us = rxTime*1000;
    srx.Sleep_window_us = sleepTime*1000;    
    radio_command(gNodeOutputBuffer, LORA_RADIO_SET_RX_DUTYCYCLE, (void *)&srx, sizeof(struct lora_set_rx_dutycycle));  
}

void RadioStartCad(void)
{
    uint32_t ChannelIsFree;
    struct labscim_radio_response *resp;
    struct lora_is_channel_free is_free;
    uint32_t sequence_number;

    is_free.Frequency_Hz = -1;
    is_free.RxBandWidth_Hz = -1;
    is_free.RSSI_Threshold_dBm = -1;
    is_free.MaxCarrierSenseTime_us = 10 * 1000;
    sequence_number = radio_command(gNodeOutputBuffer, LORA_RADIO_IS_CHANNEL_FREE, (void *)&is_free, sizeof(struct lora_is_channel_free));
}

void RadioSetTxContinuousWave(uint32_t freq, int8_t power, uint16_t time)
{
    return;
    // uint32_t timeout = (uint32_t)time * 1000;
    // TimerSetValue(&TxTimeoutTimer, timeout);
    // TimerStart(&TxTimeoutTimer);
}

int16_t RadioRssi(RadioModems_t modem)
{    
    return (int16_t)gRSSI_dbm; 
}

void RadioWrite(uint32_t addr, uint8_t data)
{
    while(1);
    return;
}

uint8_t RadioRead(uint32_t addr)
{
    while(1);
    return 0;
}

void RadioWriteBuffer(uint32_t addr, uint8_t *buffer, uint8_t size)
{
    while(1);
    return;

    //SX126xWriteRegisters(addr, buffer, size);
}

void RadioReadBuffer(uint32_t addr, uint8_t *buffer, uint8_t size)
{
    while(1);
    return;


    //SX126xReadRegisters(addr, buffer, size);
}

void RadioSetMaxPayloadLength(RadioModems_t modem, uint8_t max)
{
    struct lora_set_packet_params packet_params;
    if (modem == MODEM_LORA)
    {
        SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength = max;
        memcpy(&(packet_params.PacketParams), &SX126x.PacketParams, sizeof(PacketParams_t));
        radio_command(gNodeOutputBuffer, LORA_RADIO_SET_PACKET_PARAMS, (void *)&packet_params, sizeof(struct lora_set_packet_params));
    }
    else
    {
        if (SX126x.PacketParams.Params.Gfsk.HeaderType == RADIO_PACKET_VARIABLE_LENGTH)
        {
            SX126x.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength = max;
            memcpy(&(packet_params.PacketParams), &SX126x.PacketParams, sizeof(PacketParams_t));
            radio_command(gNodeOutputBuffer, LORA_RADIO_SET_PACKET_PARAMS, (void *)&packet_params, sizeof(struct lora_set_packet_params));            
        }
    }
}

void RadioSetPublicNetwork(bool enable)
{
    RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

    RadioSetModem(MODEM_LORA);
    // if (enable == true)
    // {
    //     // Change LoRa modem SyncWord
    //     SX126xWriteRegister(REG_LR_SYNCWORD, (LORA_MAC_PUBLIC_SYNCWORD >> 8) & 0xFF);
    //     SX126xWriteRegister(REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF);
    // }
    // else
    // {
    //     // Change LoRa modem SyncWord
    //     SX126xWriteRegister(REG_LR_SYNCWORD, (LORA_MAC_PRIVATE_SYNCWORD >> 8) & 0xFF);
    //     SX126xWriteRegister(REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF);
    // }
}

uint32_t RadioGetWakeupTime(void)
{
    return 8;
}

void RadioOnTxTimeoutIrq(void *context)
{
    if ((RadioEvents != NULL) && (RadioEvents->TxTimeout != NULL))
    {
        RadioEvents->TxTimeout();
    }
}

void RadioOnRxTimeoutIrq(void *context)
{
    if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
    {
        RadioEvents->RxTimeout();
    }
}

void RadioOnDioIrq(void *context)
{
    IrqFired = true;
}

void labscim_radio_incoming_command(struct labscim_radio_command *cmd)
{
    labscim_ll_insert_at_back(&gReceivedCommands, cmd);
}

void RadioIrqProcess(void)
{
    while (gReceivedCommands.count > 0)
    {
        struct labscim_radio_response *cmd = (struct labscim_radio_response *)labscim_ll_pop_front(&gReceivedCommands);
        switch (cmd->radio_response_code)
        {
        case LORA_RADIO_SEND_COMPLETED:
        {
            TimerStop(&TxTimeoutTimer);
            if ((RadioEvents != NULL) && (RadioEvents->TxDone != NULL))
            {
                RadioEvents->TxDone();
            }
            free(cmd);
            break;
        }
        case LORA_RADIO_PACKET_RECEIVED:
        {
            struct lora_radio_payload *payload = (struct lora_radio_payload *)(cmd->radio_struct);
            if (payload->CRCError)
            {
                if (RxContinuous == false)
                {
                    RadioSleep();
                }
                if ((RadioEvents != NULL) && (RadioEvents->RxError))
                {
                    RadioEvents->RxError();
                }
                free(cmd);
            }
            else
            {
                uint8_t size;
                TimerStop(&RxTimeoutTimer);
                if (RxContinuous == false)
                {
                       RadioSleep();
                }
                if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL))
                {
                    //this ugly code matches the semtech snr report from sx126x radios
                    int8_t snr = 0;
                    if((int8_t)(payload->SNR_db) > 31)
                    {
                        snr = 0x7C;
                    } 
                    else if((int8_t)(payload->SNR_db) < -32)
                    {                        
                        snr = 0x80; 
                    } 
                    else
                    {
                        snr = (int8_t)(payload->SNR_db*4);
                    }                                          
                    int16_t rssi = (int16_t)(payload->RSSI_dbm);
                    if(rssi > 127)
                    {
                        //just to be sure
                        rssi = 127;
                    } else if(rssi < -128)
                    {
                        rssi = -128;
                    }
                    if(gLastRxPacket!=NULL)
                    {
                        free(gLastRxPacket);
                    }                    
                    gLastRxPacket = cmd;
                    RadioEvents->RxDone(payload->Message, payload->MessageSize_bytes, rssi, (  snr  ) >> 2);
                    gRSSI_dbm = payload->RSSI_dbm;
                }
                else
                {
                    free(cmd);
                }                
            }
            
            break;
        }
        case LORA_RADIO_IS_CHANNEL_FREE_RESULT:
        {
            bool ChannelIsFree = ((struct lora_is_channel_free_result *)cmd->radio_struct)->ChannelIsFree;
            if ((RadioEvents != NULL) && (RadioEvents->CadDone != NULL))
            {
                RadioEvents->CadDone(!ChannelIsFree);
            }
            free(cmd);
            break;
        }
        case LORA_RADIO_TX_TIMEOUT:
        {
            TimerStop(&TxTimeoutTimer);
            if ((RadioEvents != NULL) && (RadioEvents->TxTimeout != NULL))
            {
                RadioEvents->TxTimeout();
            }
            free(cmd);
            break;
        }
        case LORA_RADIO_RX_TIMEOUT:
        {

            TimerStop(&RxTimeoutTimer);
            if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
            {
                RadioEvents->RxTimeout();
            }
            free(cmd);
            break;
        }
        //unused sx126x irqs:
        //IRQ_PREAMBLE_DETECTED
        //IRQ_SYNCWORD_VALID
        //IRQ_HEADER_VALID
        //IRQ_HEADER_ERROR
        case LORA_RADIO_STATE_CHANGED: //not used for now
        default:
        {
            free(cmd);
            break;
        }
        }
    }
}