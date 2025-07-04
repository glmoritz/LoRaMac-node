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
#include <stdbool.h>
#include "utilities.h"
#include "i2c.h"
#include "mma8451.h"

extern I2c_t I2c;

static uint8_t I2cDeviceAddr = 0;

static bool MMA8451Initialized = false;

/*!
 * \brief Writes a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \retval status [LMN_STATUS_OK, LMN_STATUS_ERROR]
 */
LmnStatus_t MMA8451Write( uint8_t addr, uint8_t data );

/*!
 * \brief Writes a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \param [IN]: size
 * \retval status [LMN_STATUS_OK, LMN_STATUS_ERROR]
 */
LmnStatus_t MMA8451WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size );

/*!
 * \brief Reads a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \retval status [LMN_STATUS_OK, LMN_STATUS_ERROR]
 */
LmnStatus_t MMA8451Read( uint8_t addr, uint8_t *data );

/*!
 * \brief Reads a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \param [IN]: size
 * \retval status [LMN_STATUS_OK, LMN_STATUS_ERROR]
 */
LmnStatus_t MMA8451ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size );

/*!
 * \brief Sets the I2C device slave address
 *
 * \param [IN]: addr
 */
void MMA8451SetDeviceAddr( uint8_t addr );

/*!
 * \brief Gets the I2C device slave address
 *
 * \retval: addr Current device slave address
 */
uint8_t MMA8451GetDeviceAddr( void );

LmnStatus_t MMA8451Init( void )
{
    uint8_t regVal = 0;

    MMA8451SetDeviceAddr( MMA8451_I2C_ADDRESS );

    if( MMA8451Initialized == false )
    {
        MMA8451Initialized = true;

        MMA8451Read( MMA8451_ID, &regVal );
        if( regVal != 0x1A )   // Fixed Device ID Number = 0x1A
        {
            return LMN_STATUS_ERROR;
        }
        MMA8451Reset( );

        // INT pins on this chip default to push-pull output
        // set them to open drain.
        MMA8451Write( MMA8451_CTRL_REG3, 0x01 );
        MMA8451OrientDetect( );
    }
    return LMN_STATUS_OK;
}


LmnStatus_t MMA8451Reset( )
{
    if( MMA8451Write( 0x2B, 0x40 ) == LMN_STATUS_OK ) // Reset the MMA8451 with CTRL_REG2
    {
        return LMN_STATUS_OK;
    }
    return LMN_STATUS_ERROR;
}

LmnStatus_t MMA8451Write( uint8_t addr, uint8_t data )
{
    return MMA8451WriteBuffer( addr, &data, 1 );
}

LmnStatus_t MMA8451WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
    return I2cWriteMemBuffer( &I2c, I2cDeviceAddr << 1, addr, data, size );
}

LmnStatus_t MMA8451Read( uint8_t addr, uint8_t *data )
{
    return MMA8451ReadBuffer( addr, data, 1 );
}

LmnStatus_t MMA8451ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
    return I2cReadMemBuffer( &I2c, I2cDeviceAddr << 1, addr, data, size );
}

void MMA8451SetDeviceAddr( uint8_t addr )
{
    I2cDeviceAddr = addr;
}

uint8_t MMA8451GetDeviceAddr( void )
{
    return I2cDeviceAddr;
}

uint8_t MMA8451GetOrientation( void )
{
    uint8_t orientation = 0;

    MMA8451Read( MMA8451_PL_STATUS, &orientation );
    return orientation;
}

void MMA8451OrientDetect( void )
{
    uint8_t ctrlReg1 = 0;
    uint8_t tmpReg = 0;

    // Set device in standby mode
    MMA8451Read( MMA8451_CTRL_REG1, &ctrlReg1 );
    MMA8451Write( MMA8451_CTRL_REG1, ctrlReg1 & 0xFE );

    // Set the data rate to 50 Hz
    MMA8451Write( MMA8451_CTRL_REG1, ctrlReg1 | 0x20 );

    // Set enable orientation detection.
    MMA8451Read( MMA8451_PL_CFG, &tmpReg );
    MMA8451Write( MMA8451_PL_CFG, tmpReg | 0x40 );

    // Enable orientation interrupt
    MMA8451Write( MMA8451_CTRL_REG4, 0x10 );

    // Select orientation interrupt pin INT1
    MMA8451Write( MMA8451_CTRL_REG5, 0x10 );

    // Set the debounce counter 5 -> 100 ms at 50 Hz
    MMA8451Write( MMA8451_PL_COUNT, 0x05 );

    // Set device in active mode
    MMA8451Read( MMA8451_CTRL_REG1, &ctrlReg1 );
    MMA8451Write( MMA8451_CTRL_REG1, ctrlReg1 | 0x01 );
}
