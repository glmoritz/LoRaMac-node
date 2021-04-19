/*!
 * \file      delay-board.c
 *
 * \brief     Target board delay implementation
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
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#include "delay-board.h"
#include "labscim_socket.h"

#include "labscim_platform_socket.h"

#define LORAMAC_DELAYMS  0x9ABC

void DelayMsMcu( uint32_t ms )
{
    struct labscim_protocol_header * resp;
    uint8_t ctrue = 1;
    uint32_t cmd_seq;
    uint32_t seq_no = set_time_event(gNodeOutputBuffer, LORAMAC_DELAYMS, ctrue, ms*1000);
    protocol_yield(gNodeOutputBuffer);
	do
    {
        resp = (struct labscim_protocol_header*)socket_wait_for_command(0, 0);
        cmd_seq = resp->request_sequence_number;
        socket_process_command(resp);
    } while (cmd_seq!=seq_no);	
}
