cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME AxDr_L_Motor)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Keep Eclipse middleware dependencies outside CubeMX-owned Middlewares/ST paths.
include(cmake/threadx-eclipse.cmake)
include(cmake/usbx-eclipse.cmake)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
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
    Motion/Motion.c
    Motion/Ramp.c
    Motion/Trapezoid.c
    Motor/Motor_ADC.c
    Motor/Encoder.c
    Motor/Encoder_Drv/MT6816.c
    Motor/Encoder_Drv/MT6835.c
    Motor/Protection.c
    Motor/Motor_Cal.c
    Motor/Motor_Config.c
    Motor/Motor_Para.c
    Motor/Current_Loop.c
    Motor/Motion_Loop.c
    Motor/Align.c
    Motor/Open_Loop.c
    Motor/Servo_Phase.c
    Motor/Motor_PWM.c
    Motor/Motor_Control.c
    Parameter/Parameter.c
    BSP/RGB.c
    Comm/Protocol.c
    Comm/Plot.c
    Comm/USB_Thread.c
    AZURE_RTOS/App/motor_thread.c
    AZURE_RTOS/App/panel_thread.c
    ThirdParty/Zephyr/subsys/kvss/nvs/nvs.c
    ThirdParty/Zephyr/subsys/crc/crc8_sw.c
    ThirdParty/Zephyr/subsys/crc/crc32_sw.c
)

# ST Arm Clang defines both __clang__ and GNU compatibility macros. The
# CubeMX ThreadX low-level source treats every Clang compiler as Arm Compiler
# 6 and consequently expects the __Vectors symbol, while the generated GNU
# startup file exports g_pfnVectors. Select the GNU ThreadX assembly branch
# when building the application assembly sources with starm-clang.
if(DEFINED STARM_TOOLCHAIN_CONFIG)
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:ASM>:-U__clang__>
    )
endif()

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Algo
    BSP
    Comm
    Identification
    Observer
    Sensorless
    Motion
    Motor
    Motor/Encoder_Drv
    Parameter
    ThirdParty/Zephyr_Port/include
    ThirdParty/Zephyr/subsys/kvss/nvs
    User
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
