cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME AxDr_L_Motor)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)
add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

include(cmake/threadx-eclipse.cmake)
include(cmake/usbx-eclipse.cmake)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Algo/PID.c
    Algo/Math.c
    Algo/Sin_LUT.c
    Algo/Voltage_Mod.c
    Identification/Identification.c
    Identification/Rs_Ls.c
    Identification/Flux.c
    Observer/Flux_Observer.c
    Observer/PLL.c
    Sensorless/IF_Start.c
    Sensorless/Sensorless.c
    Motor/Fast_Profile.c
    Motor/Motor_ADC.c
    Motor/Encoder.c
    Motor/Encoder_Drv/MT6816.c
    Motor/Encoder_Drv/MT6835.c
    Motor/Motor_Para.c
    Motor/Current_Loop.c
    Motor/Motion_Loop.c
    Motor/Align.c
    Motor/Open_Loop.c
    Motor/Motor_PWM.c
    Motor/Motor_Control.c
    BSP/RGB.c
    Comm/Protocol.c
    Comm/Plot.c
    Comm/USB_Thread.c
    AZURE_RTOS/App/motor_thread.c
    AZURE_RTOS/App/panel_thread.c
)

if(DEFINED STARM_TOOLCHAIN_CONFIG)
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:ASM>:-U__clang__>
    )
endif()

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Algo
    BSP
    Comm
    Identification
    Observer
    Sensorless
    Motor
    Motor/Encoder_Drv
    User
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)
