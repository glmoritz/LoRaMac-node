##
## \file  CMakeLists.txt
##
## \brief lora-mac CMakeLists.txt
##
## The Clear BSD License
## Copyright Semtech Corporation 2021. All rights reserved.
## Copyright Stackforce 2021. All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted (subject to the limitations in the disclaimer
## below) provided that the following conditions are met:
##     * Redistributions of source code must retain the above copyright
##       notice, this list of conditions and the following disclaimer.
##     * Redistributions in binary form must reproduce the above copyright
##       notice, this list of conditions and the following disclaimer in the
##       documentation and/or other materials provided with the distribution.
##     * Neither the name of the Semtech corporation nor the
##       names of its contributors may be used to endorse or promote products
##       derived from this software without specific prior written permission.
##
## NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
## THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
## CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
## NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
## PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.
##
project(lora-mac)
cmake_minimum_required(VERSION 3.6)

#---------------------------------------------------------------------------------------
# Options
#---------------------------------------------------------------------------------------

# Allow switching of target platform
set(BOARD_LIST NAMote72 NucleoL073 NucleoL152 NucleoL476 SAMR34 SKiM880B SKiM980A SKiM881AXL B-L072Z-LRWAN1 labscim)
set(BOARD NucleoL073 CACHE STRING "Default target platform is NucleoL073")
set_property(CACHE BOARD PROPERTY STRINGS ${BOARD_LIST})

# Allow switching of MBED shields
set(MBED_RADIO_SHIELD_LIST SX1272MB2DAS SX1276MB1LAS SX1276MB1MAS SX1261MBXBAS SX1262MBXCAS SX1262MBXDAS labscim)
set(MBED_RADIO_SHIELD SX1261MBXBAS CACHE STRING "Default MBED radio shield is SX1261MBXBAS")
set_property(CACHE MBED_RADIO_SHIELD PROPERTY STRINGS ${MBED_RADIO_SHIELD_LIST})

# Allow switching of secure-elements
set(SECURE_ELEMENT_LIST SOFT_SE LR1110_SE)
set(SECURE_ELEMENT SOFT_SE CACHE STRING "Default secure element is SOFT_SE")
set_property(CACHE SECURE_ELEMENT PROPERTY STRINGS ${SECURE_ELEMENT_LIST})

# Allow switching of Applications
set(APPLICATION_LIST LoRaMac ping-pong rx-sensi tx-cw )
set(APPLICATION LoRaMac CACHE STRING "Default Application is LoRaMac")
set_property(CACHE APPLICATION PROPERTY STRINGS ${APPLICATION_LIST})

# Switch for USB-Uart support, enable it for some Applications who needs it.
option(USE_USB_CDC "Use USB-Uart" OFF)

# Switch for debugger support.
option(USE_DEBUGGER "Use Debugger" ON)

# Switch for Class B support of LoRaMac.
option(CLASSB_ENABLED "Class B support of LoRaMac" OFF)

#---------------------------------------------------------------------------------------
# Target Boards
#---------------------------------------------------------------------------------------

if(BOARD STREQUAL NAMote72)
    # Configure toolchain for NAMote72
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/boards/NAMote72/cmsis/arm-gcc/stm32l152xc_flash.ld)
    include(stm32l1)

    # Build platform specific board implementation
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/NAMote72)

    # Configure radio
    set(RADIO sx1272 CACHE INTERNAL "Radio sx1272 selected")

elseif(BOARD STREQUAL NucleoL073)
    # Configure toolchain for NucleoL073
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/boards/NucleoL073/cmsis/arm-gcc/stm32l073xx_flash.ld)
    include(stm32l0)

    # Build platform specific board implementation
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/NucleoL073)

    # Configure radio
    if(MBED_RADIO_SHIELD STREQUAL SX1272MB2DAS)
        set(RADIO sx1272 CACHE INTERNAL "Radio sx1272 selected")
    elseif(MBED_RADIO_SHIELD STREQUAL SX1276MB1LAS OR MBED_RADIO_SHIELD STREQUAL SX1276MB1MAS)
        set(RADIO sx1276 CACHE INTERNAL "Radio sx1276 selected")
    elseif(MBED_RADIO_SHIELD STREQUAL SX1261MBXBAS OR MBED_RADIO_SHIELD STREQUAL SX1262MBXCAS OR MBED_RADIO_SHIELD STREQUAL SX1262MBXDAS)
        set(RADIO sx126x CACHE INTERNAL "Radio sx126x selected")
    elseif(MBED_RADIO_SHIELD STREQUAL LR1110MB1XXS)
        set(RADIO lr1110 CACHE INTERNAL "Radio lr1110 selected")
    else()
        message(STATUS "Please specify the MBED_RADIO_SHIELD!\nPossible values are: SX1272MB2DAS, SX1276MB1LAS, SX1276MB1MAS, SX1261MBXBAS, SX1262MBXCAS, SX1262MBXDAS and LR1110MB1XXS.")
    endif()

elseif(BOARD STREQUAL NucleoL152)
    # Configure toolchain for NucleoL152
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/boards/NucleoL152/cmsis/arm-gcc/stm32l152xe_flash.ld)
    include(stm32l1)

    # Build platform specific board implementation
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/NucleoL152)

    # Configure radio
    if(MBED_RADIO_SHIELD STREQUAL SX1272MB2DAS)
        set(RADIO sx1272 CACHE INTERNAL "Radio sx1272 selected")
    elseif(MBED_RADIO_SHIELD STREQUAL SX1276MB1LAS OR MBED_RADIO_SHIELD STREQUAL SX1276MB1MAS)
        set(RADIO sx1276 CACHE INTERNAL "Radio sx1276 selected")
    elseif(MBED_RADIO_SHIELD STREQUAL SX1261MBXBAS OR MBED_RADIO_SHIELD STREQUAL SX1262MBXCAS OR MBED_RADIO_SHIELD STREQUAL SX1262MBXDAS)
        set(RADIO sx126x CACHE INTERNAL "Radio sx126x selected")
    elseif(MBED_RADIO_SHIELD STREQUAL LR1110MB1XXS)
        set(RADIO lr1110 CACHE INTERNAL "Radio lr1110 selected")
    else()
        message(STATUS "Please specify the MBED_RADIO_SHIELD!\nPossible values are: SX1272MB2DAS, SX1276MB1LAS, SX1276MB1MAS, SX1261MBXBAS, SX1262MBXCAS, SX1262MBXDAS and  LR1110MB1XXS.")
    endif()

elseif(BOARD STREQUAL NucleoL476)
    # Configure toolchain for NucleoL476
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/boards/NucleoL476/cmsis/arm-gcc/stm32l476rgtx_flash.ld)
    include(stm32l4)

    # Build platform specific board implementation
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/NucleoL476)

    # Configure radio
    if(MBED_RADIO_SHIELD STREQUAL SX1272MB2DAS)
        set(RADIO sx1272 CACHE INTERNAL "Radio sx1272 selected")
    elseif(MBED_RADIO_SHIELD STREQUAL SX1276MB1LAS OR MBED_RADIO_SHIELD STREQUAL SX1276MB1MAS)
        set(RADIO sx1276 CACHE INTERNAL "Radio sx1276 selected")
    elseif(MBED_RADIO_SHIELD STREQUAL SX1261MBXBAS OR MBED_RADIO_SHIELD STREQUAL SX1262MBXCAS OR MBED_RADIO_SHIELD STREQUAL SX1262MBXDAS)
        set(RADIO sx126x CACHE INTERNAL "Radio sx126x selected")
    elseif(MBED_RADIO_SHIELD STREQUAL LR1110MB1XXS)
        set(RADIO lr1110 CACHE INTERNAL "Radio lr1110 selected")
    else()
        message(STATUS "Please specify the MBED_RADIO_SHIELD!\nPossible values are: SX1272MB2DAS, SX1276MB1LAS, SX1276MB1MAS, SX1261MBXBAS, SX1262MBXCAS, SX1262MBXDAS and  LR1110MB1XXS.")
    endif()

elseif(BOARD STREQUAL SAMR34)
    # Configure toolchain for SAMR34
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/boards/mcu/saml21/saml21b/gcc/gcc/saml21j18b_flash.ld)
    include(samr34)

    # Build platform specific board implementation
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/SAMR34)

    # Configure radio
    set(RADIO sx1276 CACHE INTERNAL "Radio sx1276 selected")

elseif(BOARD STREQUAL SKiM880B)
    # Configure toolchain for SKiM881AXL
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/boards/SKiM880B/cmsis/arm-gcc/stm32l151xba_flash.ld)
    include(stm32l1)

    # Build platform specific board implementation
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/SKiM880B)

    # Configure radio
    set(RADIO sx1272 CACHE INTERNAL "Radio sx1272 selected")

elseif(BOARD STREQUAL SKiM980A)
    # Configure toolchain for SKiM881AXL
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/boards/SKiM980A/cmsis/arm-gcc/stm32l151xba_flash.ld)
    include(stm32l1)

    # Build platform specific board implementation
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/SKiM980A)

    # Configure radio
    set(RADIO sx1272 CACHE INTERNAL "Radio sx1272 selected")

elseif(BOARD STREQUAL SKiM881AXL)
    # Configure toolchain for SKiM881AXL
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/boards/SKiM881AXL/cmsis/arm-gcc/stm32l081xx_flash.ld)
    include(stm32l0)

    # Build platform specific board implementation
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/SKiM881AXL)

    # Configure radio
    set(RADIO sx1272 CACHE INTERNAL "Radio sx1272 selected")

elseif(BOARD STREQUAL B-L072Z-LRWAN1)
    # Configure toolchain for B-L072Z-LRWAN1
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/boards/B-L072Z-LRWAN1/cmsis/arm-gcc/stm32l072xx_flash.ld)
    include(stm32l0)

    # Build platform specific board implementation
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/B-L072Z-LRWAN1)

    # Configure radio
    set(RADIO sx1276 CACHE INTERNAL "Radio sx1276 selected")
elseif(BOARD STREQUAL labscim)
    # Configure toolchain for labscim
    #set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/boards/B-L072Z-LRWAN1/cmsis/arm-gcc/stm32l072xx_flash.ld)
    #include(stm32l0)        
   
    # Build platform specific board implementation
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/labscim)

    # Configure radio
    set(RADIO labscim CACHE INTERNAL "labscim radio selected")
endif()

#---------------------------------------------------------------------------------------
# General Components
#---------------------------------------------------------------------------------------

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/radio)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/system)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/mac)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/peripherals)

#---------------------------------------------------------------------------------------
# Applications
#---------------------------------------------------------------------------------------

if(APPLICATION STREQUAL LoRaMac)

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/apps/LoRaMac)

elseif(APPLICATION STREQUAL ping-pong)

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/apps/ping-pong)

elseif(APPLICATION STREQUAL rx-sensi)

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/apps/rx-sensi)

elseif(APPLICATION STREQUAL tx-cw)

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/apps/tx-cw)

endif()
