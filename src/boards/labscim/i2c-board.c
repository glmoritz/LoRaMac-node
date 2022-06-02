/*!
 * \file  i2c-board.c
 *
 * \brief Target board I2C driver implementation
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

#include "utilities.h"
#include "board-config.h"
#include "i2c-board.h"

/*!
 *  The value of the maximal timeout for I2C waiting loops
 */
#define TIMEOUT_MAX                                 0x8000

//static I2C_HandleTypeDef I2cHandle = { 0 };

static I2cAddrSize I2cInternalAddrSize = I2C_ADDR_SIZE_8;
I2c_t I2c;

void I2cMcuInit( I2c_t *obj, I2cId_t i2cId, PinNames scl, PinNames sda )
{
    
}

void I2cMcuFormat( I2c_t *obj, I2cMode mode, I2cDutyCycle dutyCycle, bool I2cAckEnable, I2cAckAddrMode AckAddrMode, uint32_t I2cFrequency )
{
    
}

void I2cMcuResetBus( I2c_t *obj )
{
    
}

void I2cMcuDeInit( I2c_t *obj )
{

    
}

void I2cSetAddrSize( I2c_t *obj, I2cAddrSize addrSize )
{
    I2cInternalAddrSize = addrSize;
}

LmnStatus_t I2cMcuWriteBuffer( I2c_t *obj, uint8_t deviceAddr, uint8_t *buffer, uint16_t size )
{
    LmnStatus_t status = LMN_STATUS_ERROR;
    

    return status;
}

LmnStatus_t I2cMcuReadBuffer( I2c_t *obj, uint8_t deviceAddr, uint8_t *buffer, uint16_t size )
{
    LmnStatus_t status = LMN_STATUS_ERROR;

    
    return status;
}

LmnStatus_t I2cMcuWriteMemBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    LmnStatus_t status = LMN_STATUS_ERROR;
    
    return status;
}

LmnStatus_t I2cMcuReadMemBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    LmnStatus_t status = LMN_STATUS_ERROR;
    
    return status;
}

LmnStatus_t I2cMcuWaitStandbyState( I2c_t *obj, uint8_t deviceAddr )
{
    LmnStatus_t status = LMN_STATUS_ERROR;

    

    return status;
}
