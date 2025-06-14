/*!
 * \file  mma8451.h
 *
 * \brief MMA8451 Accelerometer driver implementation
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
#ifndef __MMA8451_H__
#define __MMA8451_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "utilities.h"

/*
 * MMA8451 I2C address
 */ 
#define MMA8451_I2C_ADDRESS                          0x1C

/*
 * MMA8451 Registers
 */ 
#define MMA8451_STATUS                               0x00 //
#define MMA8451_OUT_X_MSB                            0x01 //
#define MMA8451_SYSMOD                               0x0B //
#define MMA8451_INT_SOURCE                           0x0C //
#define MMA8451_ID                                   0x0D //
#define MMA8451_PL_STATUS                            0x10 //
#define MMA8451_PL_CFG                               0x11 //
#define MMA8451_PL_COUNT                             0x12 // Orientation debounce
#define MMA8451_PL_BF_ZCOMP                          0x13 //
#define MMA8451_PL_THS_REG                           0x14 //
#define MMA8451_FF_MT_SRC                            0x16 //
#define MMA8451_TRANSIENT_CFG                        0x1D // Transient enable
#define MMA8451_TRANSIENT_SRC                        0x1E // Transient read/clear interrupt
#define MMA8451_TRANSIENT_THS                        0x1F // Transient threshold
#define MMA8451_TRANSIENT_COUNT                      0x20 // Transient debounce
#define MMA8451_PULSE_SRC                            0x22 //
#define MMA8451_CTRL_REG1                            0x2A //
#define MMA8451_CTRL_REG2                            0x2B //
#define MMA8451_CTRL_REG3                            0x2C // Interrupt control
#define MMA8451_CTRL_REG4                            0x2D // Interrupt enable
#define MMA8451_CTRL_REG5                            0x2E // Interrupt pin selection

/*!
 * \brief Initializes the device
 *
 * \retval status [LMN_STATUS_OK, LMN_STATUS_ERROR]
 */
LmnStatus_t MMA8451Init( void );

/*!
 * \brief Resets the device
 *
 * \retval status [LMN_STATUS_OK, LMN_STATUS_ERROR]
 */
LmnStatus_t MMA8451Reset( void );

/*!
 * \brief Initializes the orientation detection
 */
void MMA8451OrientDetect( void );

/*!
 * \brief Gets the orientation state.
 *
 * \retval orientation Bit 6 [1: Horizontal, 0: Vertical]
 *                     Bit 0 [1: Face down, 0: Face up]
 *                     Other bits don't care.
 */
uint8_t MMA8451GetOrientation( void );

#ifdef __cplusplus
}
#endif

#endif // __MMA8451_H__
