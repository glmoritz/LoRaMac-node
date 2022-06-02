##
## \file  CMakeLists.txt
##
## \brief labscim board CMakeLists.txt
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
project(labscim)
cmake_minimum_required(VERSION 3.6)
#enable_language(ASM)

#---------------------------------------------------------------------------------------
# Target
#---------------------------------------------------------------------------------------

list(APPEND ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/adc-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/delay-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/eeprom-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/gpio-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/i2c-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lpm-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/rtc-board.c"    
    "${CMAKE_CURRENT_SOURCE_DIR}/sysIrqHandlers.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/uart-board.c"    
    "${CMAKE_CURRENT_SOURCE_DIR}/labscim_linked_list.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/labscim_socket.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/shared_mutex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/labscim_platform_socket.c"    
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/utilities.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/gps-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/radio_board.c"
)

add_definitions(-DIS_LABSCIM_CLIENT)
add_definitions(-DLABSCIM_LOG_COMMANDS)

if(MBED_RADIO_SHIELD STREQUAL LABSCIM_SHIELD)
    list(APPEND ${PROJECT_NAME}_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/sx126x_hal.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/ral_sx126x_bsp.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/radio_board.c"
    )
    set(RADIO_UPPERCASE "LABSCIM_RADIO")
else()
    message(STATUS " Please specify the MBED_RADIO_SHIELD !\nOnly possible for labscim is labscim.")
endif()

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

target_compile_definitions(${PROJECT_NAME} PUBLIC -D${MBED_RADIO_SHIELD} -D${RADIO_UPPERCASE})

# Add define if LR-FHSS modulation is supported
target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<BOOL:${LORAMAC_LR_FHSS_IS_ON}>:LORAMAC_LR_FHSS_IS_ON>)

# Add define if debbuger support is enabled
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_DEBUGGER}>:USE_DEBUGGER>)

# Add define if radio debug pins support is enabled
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_RADIO_DEBUG}>:USE_RADIO_DEBUG>)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:radio,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:peripherals,INTERFACE_INCLUDE_DIRECTORIES>
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
