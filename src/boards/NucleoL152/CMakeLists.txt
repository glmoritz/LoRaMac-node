##
## \file  CMakeLists.txt
##
## \brief NucleoL152 board CMakeLists.txt
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
project(NucleoL152)
cmake_minimum_required(VERSION 3.6)
enable_language(ASM)

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
    "${CMAKE_CURRENT_SOURCE_DIR}/spi-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/sysIrqHandlers.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/uart-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/utilities.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis/arm-gcc/startup_stm32l152xe.s"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis/system_stm32l1xx.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_adc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_adc_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_cortex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_dma.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_flash.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_flash_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_gpio.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_i2c.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_pwr.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_pwr_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rtc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rtc_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_spi.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_spi_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_uart.c"
)

if(MBED_RADIO_SHIELD STREQUAL SX1272MB2DAS)
    list(APPEND ${PROJECT_NAME}_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/sx127x_hal.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/ral_sx127x_bsp.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/radio_board.c"
    )
    set(RADIO_UPPERCASE "SX127X -DSX1272")
elseif(MBED_RADIO_SHIELD STREQUAL SX1276MB1LAS)
    list(APPEND ${PROJECT_NAME}_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/sx127x_hal.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/ral_sx127x_bsp.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/radio_board.c"
    )
    set(RADIO_UPPERCASE "SX127X -DSX1276")
elseif(MBED_RADIO_SHIELD STREQUAL SX1276MB1MAS)
    list(APPEND ${PROJECT_NAME}_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/sx127x_hal.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/ral_sx127x_bsp.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/radio_board.c"
    )
    set(RADIO_UPPERCASE "SX127X -DSX1276")
elseif(MBED_RADIO_SHIELD STREQUAL SX1261MBXBAS)
    list(APPEND ${PROJECT_NAME}_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/sx126x_hal.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/ral_sx126x_bsp.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/radio_board.c"
    )
    set(RADIO_UPPERCASE "SX126X")
elseif(MBED_RADIO_SHIELD STREQUAL SX1262MBXCAS)
    list(APPEND ${PROJECT_NAME}_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/sx126x_hal.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/ral_sx126x_bsp.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/radio_board.c"
    )
    set(RADIO_UPPERCASE "SX126X")
elseif(MBED_RADIO_SHIELD STREQUAL SX1262MBXDAS)
    list(APPEND ${PROJECT_NAME}_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/sx126x_hal.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/ral_sx126x_bsp.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/radio_board.c"
    )
    set(RADIO_UPPERCASE "SX126X")
elseif(MBED_RADIO_SHIELD STREQUAL LR1110MB1XXS)
    list(APPEND ${PROJECT_NAME}_SOURCES 
        "${CMAKE_CURRENT_SOURCE_DIR}/lr1110_hal.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/ral_lr1110_bsp.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/radio_board.c"
    )
    set(RADIO_UPPERCASE "LR1110")
else()
    message(STATUS " Please specify the MBED_RADIO_SHIELD !\nPossible values are: SX1272MB2DAS, SX1276MB1LAS, SX1276MB1MAS, SX1261MBXBAS, SX1262MBXCAS, SX1262MBXDAS and LR1110MB1XXS.")
endif()

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

target_compile_definitions(${PROJECT_NAME} PUBLIC -DUSE_HAL_DRIVER -DSTM32L152xE -D${MBED_RADIO_SHIELD} -D${RADIO_UPPERCASE})

# Add define if LR-FHSS modulation is supported
target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<BOOL:${LORAMAC_LR_FHSS_IS_ON}>:LORAMAC_LR_FHSS_IS_ON>)

# Add define if debbuger support is enabled
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_DEBUGGER}>:USE_DEBUGGER>)

# Add define if radio debug pins support is enabled
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_RADIO_DEBUG}>:USE_RADIO_DEBUG>)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/cmsis
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32L1xx_HAL_Driver/Inc
    $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:radio,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:peripherals,INTERFACE_INCLUDE_DIRECTORIES>
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
