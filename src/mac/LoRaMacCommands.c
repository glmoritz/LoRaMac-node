/*!
 * \file  LoRaMacCommands.c
 *
 * \brief LoRa MAC commands
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 * Copyright Stackforce 2021. All rights reserved.
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
#include <stddef.h>

#include "utilities.h"
#include "LoRaMacCommands.h"
#include "LoRaMacConfirmQueue.h"

#ifndef NUM_OF_MAC_COMMANDS
/*!
 * Number of MAC Command slots
 */
#define NUM_OF_MAC_COMMANDS 32
#endif

/*!
 * Size of the CID field of MAC commands
 */
#define CID_FIELD_SIZE 1

/*!
 *  Mac Commands list structure
 */
typedef struct sMacCommandsList
{
    /*
     * First element of MAC command list.
     */
    MacCommand_t* First;
    /*
     * Last element of MAC command list.
     */
    MacCommand_t* Last;
} MacCommandsList_t;

/*!
 * LoRaMac Commands Context structure
 */
typedef struct sLoRaMacCommandsCtx
{
    /*
     * List of MAC command elements
     */
    MacCommandsList_t MacCommandList;
    /*
     * Buffer to store MAC command elements
     */
    MacCommand_t MacCommandSlots[NUM_OF_MAC_COMMANDS];
    /*
     * Size of all MAC commands serialized as buffer
     */
    size_t SerializedCmdsSize;
} LoRaMacCommandsCtx_t;

/*!
 * Non-volatile module context.
 */
static LoRaMacCommandsCtx_t CommandsCtx;

/* Memory management functions */

/*!
 * \brief Determines if a MAC command slot is free
 *
 * \param[IN]     slot           - Slot to check
 * \retval                       - Status of the operation
 */
static bool IsSlotFree( const MacCommand_t* slot )
{
    uint8_t* mem = ( uint8_t* )slot;

    for( uint16_t size = 0; size < sizeof( MacCommand_t ); size++ )
    {
        if( mem[size] != 0x00 )
        {
            return false;
        }
    }
    return true;
}

/*!
 * \brief Allocates a new MAC command memory slot
 *
 * \retval                       - Pointer to slot
 */
static MacCommand_t* MallocNewMacCommandSlot( void )
{
    uint8_t itr = 0;

    while( IsSlotFree( ( const MacCommand_t* )&CommandsCtx.MacCommandSlots[itr] ) == false )
    {
        itr++;
        if( itr == NUM_OF_MAC_COMMANDS )
        {
            return NULL;
        }
    }

    return &CommandsCtx.MacCommandSlots[itr];
}

/*!
 * \brief Free memory slot
 *
 * \param[IN]     slot           - Slot to free
 *
 * \retval                       - Status of the operation
 */
static bool FreeMacCommandSlot( MacCommand_t* slot )
{
    if( slot == NULL )
    {
        return false;
    }

    memset1( ( uint8_t* )slot, 0x00, sizeof( MacCommand_t ) );

    return true;
}

/* Linked list functions */

/*!
 * \brief Initialize list
 *
 * \param[IN]     list           - List that shall be initialized
 * \retval                       - Status of the operation
 */
static bool LinkedListInit( MacCommandsList_t* list )
{
    if( list == NULL )
    {
        return false;
    }

    list->First = NULL;
    list->Last = NULL;

    return true;
}

/*!
 * \brief Add an element to the list
 *
 * \param[IN]     list           - List where the element shall be added.
 * \param[IN]     element        - Element to add
 * \retval                       - Status of the operation
 */
static bool LinkedListAdd( MacCommandsList_t* list, MacCommand_t* element )
{
    if( ( list == NULL ) || ( element == NULL ) )
    {
        return false;
    }

    // Check if this is the first entry to enter the list.
    if( list->First == NULL )
    {
        list->First = element;
    }

    // Check if the last entry exists and update its next point.
    if( list->Last )
    {
        list->Last->Next = element;
    }

    // Update the next point of this entry.
    element->Next = NULL;

    // Update the last entry of the list.
    list->Last = element;

    return true;
}

/*!
 * \brief Return the previous element in the list.
 *
 * \param[IN]     list           - List
 * \param[IN]     element        - Element where the previous element shall be searched
 * \retval                       - Status of the operation
 */
static MacCommand_t* LinkedListGetPrevious( MacCommandsList_t* list, MacCommand_t* element )
{
    if( ( list == NULL ) || ( element == NULL ) )
    {
        return NULL;
    }

    MacCommand_t* curElement;

    // Start at the head of the list
    curElement = list->First;

    // When current element is the first of the list, there's no previous element so we can return NULL immediately.
    if( element != curElement )
    {
        // Loop through all elements until the end is reached or the next of current is the current element.
        while( ( curElement != NULL ) && ( curElement->Next != element ) )
        {
            curElement = curElement->Next;
        }
    }
    else
    {
        curElement = NULL;
    }

    return curElement;
}

/*!
 * \brief Remove an element from the list
 *
 * \param[IN]     list           - List where the element shall be removed from.
 * \param[IN]     element        - Element to remove
 * \retval                       - Status of the operation
 */
static bool LinkedListRemove( MacCommandsList_t* list, MacCommand_t* element )
{
    if( ( list == NULL ) || ( element == NULL ) )
    {
        return false;
    }

    MacCommand_t* PrevElement = LinkedListGetPrevious( list, element );

    if( list->First == element )
    {
        list->First = element->Next;
    }

    if( list->Last == element )
    {
        list->Last = PrevElement;
    }

    if( PrevElement != NULL )
    {
        PrevElement->Next = element->Next;
    }

    element->Next = NULL;

    return true;
}

/*
 * \brief Determines if a MAC command is sticky or not
 *
 * \param[IN]   cid                - MAC command identifier
 *
 * \retval                     - Status of the operation
 */
static bool IsSticky( uint8_t cid )
{
    switch( cid )
    {
        case MOTE_MAC_RESET_IND:
        case MOTE_MAC_REKEY_IND:
        case MOTE_MAC_DEVICE_MODE_IND:
        case MOTE_MAC_DL_CHANNEL_ANS:
        case MOTE_MAC_RX_PARAM_SETUP_ANS:
        case MOTE_MAC_RX_TIMING_SETUP_ANS:
        case MOTE_MAC_TX_PARAM_SETUP_ANS:
        case MOTE_MAC_PING_SLOT_CHANNEL_ANS:
            return true;
        default:
            return false;
    }
}

LoRaMacCommandStatus_t LoRaMacCommandsInit( void )
{
    // Initialize with default
    memset1( ( uint8_t* )&CommandsCtx, 0, sizeof( CommandsCtx ) );

    LinkedListInit( &CommandsCtx.MacCommandList );

    return LORAMAC_COMMANDS_SUCCESS;
}

LoRaMacCommandStatus_t LoRaMacCommandsAddCmd( uint8_t cid, uint8_t* payload, size_t payloadSize )
{
    if( payload == NULL )
    {
        return LORAMAC_COMMANDS_ERROR_NPE;
    }
    MacCommand_t* newCmd;

    // Allocate a memory slot
    newCmd = MallocNewMacCommandSlot( );

    if( newCmd == NULL )
    {
        return LORAMAC_COMMANDS_ERROR_MEMORY;
    }

    // Add it to the list of Mac commands
    if( LinkedListAdd( &CommandsCtx.MacCommandList, newCmd ) == false )
    {
        return LORAMAC_COMMANDS_ERROR;
    }

    // Set Values
    newCmd->CID = cid;
    newCmd->PayloadSize = payloadSize;
    memcpy1( ( uint8_t* )newCmd->Payload, payload, payloadSize );
    newCmd->IsSticky = IsSticky( cid );

    CommandsCtx.SerializedCmdsSize += ( CID_FIELD_SIZE + payloadSize );

    return LORAMAC_COMMANDS_SUCCESS;
}

LoRaMacCommandStatus_t LoRaMacCommandsRemoveCmd( MacCommand_t* macCmd )
{
    if( macCmd == NULL )
    {
        return LORAMAC_COMMANDS_ERROR_NPE;
    }

    // Remove the Mac command element from MacCommandList
    if( LinkedListRemove( &CommandsCtx.MacCommandList, macCmd ) == false )
    {
        return LORAMAC_COMMANDS_ERROR_CMD_NOT_FOUND;
    }

    CommandsCtx.SerializedCmdsSize -= ( CID_FIELD_SIZE + macCmd->PayloadSize );

    // Free the MacCommand Slot
    if( FreeMacCommandSlot( macCmd ) == false )
    {
        return LORAMAC_COMMANDS_ERROR;
    }

    return LORAMAC_COMMANDS_SUCCESS;
}

LoRaMacCommandStatus_t LoRaMacCommandsGetCmd( uint8_t cid, MacCommand_t** macCmd )
{
    MacCommand_t* curElement;

    // Start at the head of the list
    curElement = CommandsCtx.MacCommandList.First;

    // Loop through all elements until we find the element with the given CID
    while( ( curElement != NULL ) && ( curElement->CID != cid ) )
    {
        curElement = curElement->Next;
    }

    // Update the pointer anyway
    *macCmd = curElement;

    // Handle error in case if we reached the end without finding it.
    if( curElement == NULL )
    {
        return LORAMAC_COMMANDS_ERROR_CMD_NOT_FOUND;
    }
    return LORAMAC_COMMANDS_SUCCESS;
}

LoRaMacCommandStatus_t LoRaMacCommandsRemoveNoneStickyCmds( void )
{
    MacCommand_t* curElement;
    MacCommand_t* nexElement;

    // Start at the head of the list
    curElement = CommandsCtx.MacCommandList.First;

    // Loop through all elements
    while( curElement != NULL )
    {
        if( curElement->IsSticky == false )
        {
            nexElement = curElement->Next;
            LoRaMacCommandsRemoveCmd( curElement );
            curElement = nexElement;
        }
        else
        {
            curElement = curElement->Next;
        }
    }

    return LORAMAC_COMMANDS_SUCCESS;
}

LoRaMacCommandStatus_t LoRaMacCommandsRemoveStickyAnsCmds( void )
{
    MacCommand_t* curElement;
    MacCommand_t* nexElement;

    // Start at the head of the list
    curElement = CommandsCtx.MacCommandList.First;

    // Loop through all elements
    while( curElement != NULL )
    {
        nexElement = curElement->Next;
        if( IsSticky( curElement->CID ) == true )
        {
            LoRaMacCommandsRemoveCmd( curElement );
        }
        curElement = nexElement;
    }

    return LORAMAC_COMMANDS_SUCCESS;
}

LoRaMacCommandStatus_t LoRaMacCommandsGetSizeSerializedCmds( size_t* size )
{
    if( size == NULL )
    {
        return LORAMAC_COMMANDS_ERROR_NPE;
    }
    *size = CommandsCtx.SerializedCmdsSize;
    return LORAMAC_COMMANDS_SUCCESS;
}

LoRaMacCommandStatus_t LoRaMacCommandsSerializeCmds( size_t availableSize, size_t* effectiveSize, uint8_t* buffer )
{
    MacCommand_t* curElement = CommandsCtx.MacCommandList.First;
    MacCommand_t* nextElement;
    uint8_t itr = 0;

    if( ( buffer == NULL ) || ( effectiveSize == NULL ) )
    {
        return LORAMAC_COMMANDS_ERROR_NPE;
    }

    // Loop through all elements which fits into the buffer
    while( curElement != NULL )
    {
        // If the next MAC command still fits into the buffer, add it.
        if( ( availableSize - itr ) >= ( CID_FIELD_SIZE + curElement->PayloadSize ) )
        {
            buffer[itr++] = curElement->CID;
            memcpy1( &buffer[itr], curElement->Payload, curElement->PayloadSize );
            itr += curElement->PayloadSize;
        }
        else
        {
            break;
        }
        curElement = curElement->Next;
    }

    // Remove all commands which do not fit into the buffer
    while( curElement != NULL )
    {
        // Store the next element before removing the current one
        nextElement = curElement->Next;
        LoRaMacCommandsRemoveCmd( curElement );
        curElement = nextElement;
    }

    // Fetch the effective size of the mac commands
    LoRaMacCommandsGetSizeSerializedCmds( effectiveSize );

    return LORAMAC_COMMANDS_SUCCESS;
}

uint8_t LoRaMacCommandsGetCmdSize( uint8_t cid )
{
    uint8_t cidSize = 0;

    // Decode Frame MAC commands
    switch( cid )
    {
        case SRV_MAC_RESET_CONF:
        {
            // cid + Serv_LoRaWAN_version
            cidSize = 2;
            break;
        }
        case SRV_MAC_LINK_CHECK_ANS:
        {
            // cid + Margin + GwCnt
            cidSize = 3;
            break;
        }
        case SRV_MAC_LINK_ADR_REQ:
        {
            // cid + DataRate_TXPower + ChMask (2) + Redundancy
            cidSize = 5;
            break;
        }
        case SRV_MAC_DUTY_CYCLE_REQ:
        {
            // cid + DutyCyclePL
            cidSize = 2;
            break;
        }
        case SRV_MAC_RX_PARAM_SETUP_REQ:
        {
            // cid + DLsettings + Frequency (3)
            cidSize = 5;
            break;
        }
        case SRV_MAC_DEV_STATUS_REQ:
        {
            // cid
            cidSize = 1;
            break;
        }
        case SRV_MAC_NEW_CHANNEL_REQ:
        {
            // cid + ChIndex + Frequency (3) + DrRange
            cidSize = 6;
            break;
        }
        case SRV_MAC_RX_TIMING_SETUP_REQ:
        {
            // cid + Settings
            cidSize = 2;
            break;
        }
        case SRV_MAC_TX_PARAM_SETUP_REQ:
        {
            // cid + EIRP_DwellTime
            cidSize = 2;
            break;
        }
        case SRV_MAC_DL_CHANNEL_REQ:
        {
            // cid + ChIndex + Frequency (3)
            cidSize = 5;
            break;
        }
        case SRV_MAC_REKEY_CONF:
        {
            // cid + Serv_LoRaWAN_version
            cidSize = 2;
            break;
        }
        case SRV_MAC_ADR_PARAM_SETUP_REQ:
        {
            // cid + ADRparam
            cidSize = 2;
            break;
        }
        case SRV_MAC_FORCE_REJOIN_REQ:
        {
            // cid + Payload (2)
            cidSize = 3;
            break;
        }
        case SRV_MAC_REJOIN_PARAM_REQ:
        {
            // cid + Payload (1)
            cidSize = 2;
            break;
        }
        case SRV_MAC_DEVICE_MODE_CONF:
        {
            // cid + Class
            cidSize = 2;
            break;
        }
        case SRV_MAC_DEVICE_TIME_ANS:
        {
            // cid + Seconds (4) + Fractional seconds (1)
            cidSize = 6;
            break;
        }
        case SRV_MAC_PING_SLOT_INFO_ANS:
        {
            // cid
            cidSize = 1;
            break;
        }
        case SRV_MAC_PING_SLOT_CHANNEL_REQ:
        {
            // cid + Frequency (3) + DR
            cidSize = 5;
            break;
        }
        case SRV_MAC_BEACON_TIMING_ANS:
        {
            // cid + TimingDelay (2) + Channel
            cidSize = 4;
            break;
        }
        case SRV_MAC_BEACON_FREQ_REQ:
        {
            // cid + Frequency (3)
            cidSize = 4;
            break;
        }
        default:
        {
            // Unknown command. ABORT MAC commands processing
            break;
        }
    }
    return cidSize;
}
