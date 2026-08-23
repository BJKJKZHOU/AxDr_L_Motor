# Use the Eclipse ThreadX submodule while leaving STM32CubeMX free to regenerate
# its own middleware paths under Middlewares/ST.

set(THREADX_ECLIPSE_ROOT "${CMAKE_SOURCE_DIR}/ThirdParty/Eclipse/threadx")

if(NOT EXISTS "${THREADX_ECLIPSE_ROOT}/common/inc/tx_api.h")
    message(FATAL_ERROR
        "Eclipse ThreadX submodule is missing. Run: git submodule update --init --recursive")
endif()

target_include_directories(stm32cubemx BEFORE INTERFACE
    "${THREADX_ECLIPSE_ROOT}/common/inc"
    "${THREADX_ECLIPSE_ROOT}/ports/cortex_m4/gnu/inc"
)

get_target_property(THREADX_CUBEMX_SOURCES ThreadX SOURCES)
set(THREADX_ECLIPSE_SOURCES)

foreach(THREADX_SOURCE IN LISTS THREADX_CUBEMX_SOURCES)
    string(REPLACE
        "Middlewares/ST/threadx"
        "ThirdParty/Eclipse/threadx"
        THREADX_REMAP "${THREADX_SOURCE}")

    list(APPEND THREADX_ECLIPSE_SOURCES "${THREADX_REMAP}")
endforeach()

set_property(TARGET ThreadX PROPERTY SOURCES "${THREADX_ECLIPSE_SOURCES}")
