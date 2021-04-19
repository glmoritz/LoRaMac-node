/*
 * labscim-lora-radio-protocol.h
 *
 *  Created on: 9 de jun de 2020
 *      Author: root
 */

#ifndef DEV_LABSCIM_LORA_RADIO_PROTOCOL_H_
#define DEV_LABSCIM_LORA_RADIO_PROTOCOL_H_


#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include "omnet_radio_mode.h"
#include "sx126x_labscim.h"

#define LORA_RADIO_GET_STATE (0x1111)
#define LORA_RADIO_GET_STATE_RESULT (0x1112)
#define LORA_RADIO_STATE_CHANGED (0x1113)




/**
 * This sets the radio in the different operation modes
 * Timeout is used when the radio is set to RX
 */

struct lora_radio_status
{
	uint32_t RadioMode;        
	uint32_t ChannelIsFree;
} __attribute__((packed));


#define LORA_RADIO_SET_MODEM (0x2222)
/*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
struct lora_set_modem
{
	uint8_t Modem;
} __attribute__((packed));


#define LORA_RADIO_SET_CHANNEL (0x3333)
/*!
 * \brief Sets the channel frequency
 *
 * \param [IN] Frequency_Hz
 *          Channel RF frequency in Hz
 */
struct lora_set_frequency
{
	uint32_t Frequency_Hz;
} __attribute__((packed));


#define LORA_RADIO_IS_CHANNEL_FREE (0x4444)
#define LORA_RADIO_IS_CHANNEL_FREE_RESULT (0x4445)

/*!
 * \brief Checks if the channel is free for the given time
 *  
 * \param [IN] Frequency_Hz                Channel RF frequency in Hertz
 * \param [IN] RxBandWidth_Hz         Rx bandwidth in Hertz
 * \param [IN] RSSI_Threshold_dBm          RSSI threshold in dBm
 * \param [IN] MaxCarrierSenseTime_us Max time in microsseconds while the RSSI is measured
 * 
 */
struct lora_is_channel_free
{
	int32_t Frequency_Hz;
    int32_t RxBandWidth_Hz;
    int16_t RSSI_Threshold_dBm;
    uint32_t MaxCarrierSenseTime_us;
} __attribute__((packed));

/* \retval ChannelIsFree         [1: Channel is free, 0: Channel is not free]*/
struct lora_is_channel_free_result
{
	uint8_t ChannelIsFree;
} __attribute__((packed));


#define LORA_RADIO_SET_MODULATION_PARAMS (0x5555)
struct lora_set_modulation_params
{
	uint32_t TransmitPower_dBm;
	ModulationParams_t ModulationParams; //straight from SX1262 configuration	
} __attribute__((packed));

#define LORA_RADIO_SET_PACKET_PARAMS (0x6666)
struct lora_set_packet_params
{
	PacketParams_t PacketParams; //straight from SX1262 configuration	
} __attribute__((packed));


#define LORA_RADIO_SEND (0x7777)
#define LORA_RADIO_SEND_COMPLETED (0x7778)
#define LORA_RADIO_PACKET_RECEIVED (0x7779)
#define LORA_RADIO_TX_TIMEOUT (0x777A)
/**
 * This is a sent/received radio payload
 */
struct lora_radio_payload
{
	uint16_t MessageSize_bytes;
	float RSSI_dbm;
	float SNR_db;
	uint64_t RX_timestamp_us;
	float TxPower_dbm;
	uint32_t LoRaBandwidth_Hz;
	uint64_t Tx_Delay_us;
	uint32_t CenterFrequency_Hz;
	uint8_t CRCError;
	uint8_t IFChain;
	uint8_t LoRaSF;
	uint8_t LoRaCR;
	uint8_t Message[];
} __attribute__((packed));
#define FIXED_SIZEOF_LORA_RADIO_PAYLOAD (sizeof(uint16_t) + 2*sizeof(float) + sizeof(uint64_t) + sizeof(float) +  sizeof(uint32_t) + sizeof(uint64_t) + sizeof(uint32_t) + 4*sizeof(uint8_t))

#define LORA_RADIO_SET_RX (0x8888)
#define LORA_RADIO_RX_TIMEOUT (0x8889)
#define LORA_RADIO_SET_RX_DUTYCYCLE (0x888A)
#define LORA_RADIO_RX_DONE (0x888B)
/*!
 * \brief Configures the radio in RX mode
 *
 * \param [IN] Timeout_us - the time to listen. before that time the radio returns to idle and notifies an RX_TIMEOUT. 
 *
 * - 0x0000000000000000 No timeout. Rx Single mode. The radio will stay in RX Mode until a reception occurs and the devices return to RX_IDLE mode upon completion
 * - 0xFFFFFFFFFFFFFFFF Rx Continuous mode. The device remains in RX mode until the host sends a command to change the operation mode. The device can receive several packets. Each time a packet is received, a packet done
 * indication is given to the host and the device will automatically search for a new packet.
 * 
 */
struct lora_set_rx
{
	uint64_t Timeout_us;
} __attribute__((packed));

/*!
 *\brief When this command is sent the radio enters in a loop defined by the following steps:
 * -- The chip enters RX and listens for a packet for a period of time defined by Rx_window_us
 * -- The chip is looking for a preamble in either LoRa® or FSK
 * -- Upon preamble detection, the timeout is stopped and restarted with the value 2 * Rx_window_us + Sleep_window_us
 * -- If no packet is received during the RX window (defined by Rx_window_us), the radio goes into SLEEP mode for a period of time defined by Sleep_window_us
 * -- At the end of the Sleep_window_us, the radio automatically enters the RX mode. At any time, the host can stop the procedure.
 * The loop is terminated if either:
 * -- A packet is detected during the Rx_window_us, at which moment the radio sends a LORA_RADIO_RX_DONE message and returns to idle mode
 * -- The host issues a SetStandby command.
 *
 * \param [IN] Rx_window_us - the time to listen.
 * \param [IN] Sleep_window_us - the time to sleep. *
 */
struct lora_set_rx_dutycycle
{
	uint64_t Rx_window_us;
	uint64_t Sleep_window_us;
} __attribute__((packed));

#define LORA_RADIO_SET_IDLE (0x9999)
/*!
 * \brief Configures the radio in idle mode
 *
 * \param [IN] unused - dummy payload
 *  
 */
struct lora_set_idle
{
	uint64_t unused;
} __attribute__((packed));


#endif /* DEV_LABSCIM_CONTIKI_RADIO_PROTOCOL_H_ */
