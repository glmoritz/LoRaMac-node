/*!
 * \file  pam7q.h
 *
 * \brief PAM7Q GPS receiver driver implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __PAM7Q_H__
#define __PAM7Q_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

void PAM7QInit( void );

uint8_t PAM7QGetDeviceAddr( void );

void PAM7QSetDeviceAddr( uint8_t addr );

bool PAM7QGetGpsData( uint8_t *nmeaString, uint8_t *nmeaStringSize, uint16_t nmeaMaxStringSize );

uint8_t PAM7QWrite( uint8_t addr, uint8_t data );

uint8_t PAM7QWriteBuffer( uint8_t addr, uint8_t *data, uint8_t size );

uint8_t PAM7QRead( uint8_t addr, uint8_t *data );

uint8_t PAM7QReadBuffer( uint8_t addr, uint8_t *data, uint8_t size );

void GpsMcuOnPpsSignal( void );

/*
 * MPL3115A2 I2C address
 */
#define PAM7Q_I2C_ADDRESS                       0x42

#define MESSAGE_SIZE_1                          0xFD
#define MESSAGE_SIZE_2                          0xFE

#define PAYLOAD                                 0xFF


#ifdef __cplusplus
}
#endif

#endif // __PAM7Q_H__

