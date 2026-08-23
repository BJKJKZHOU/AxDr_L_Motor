# Use the Eclipse USBX submodule for the portable USBX core and CDC ACM class
# while keeping the STM32-specific DCD glue outside CubeMX-owned middleware paths.
#
# CubeMX owns cmake/stm32cubemx/CMakeLists.txt. Remap its USBX source list here
# so CubeMX regeneration does not overwrite the dependency selection.

set(USBX_ECLIPSE_ROOT "${CMAKE_SOURCE_DIR}/Middlewares/Eclipse/usbx")
set(USBX_ST_DCD_ROOT "${CMAKE_SOURCE_DIR}/Middlewares/ST/usbx_stm32_dcd")

if(NOT EXISTS "${USBX_ECLIPSE_ROOT}/common/core/inc/ux_api.h")
    message(FATAL_ERROR
        "Eclipse USBX submodule is missing. Run: git submodule update --init --recursive")
endif()

if(NOT EXISTS "${USBX_ST_DCD_ROOT}/ux_dcd_stm32.h")
    message(FATAL_ERROR
        "STM32 USBX DCD glue is missing from Middlewares/ST/usbx_stm32_dcd")
endif()

# Prefer Eclipse USBX headers and the relocated STM32 DCD glue.
target_include_directories(stm32cubemx BEFORE INTERFACE
    "${USBX_ECLIPSE_ROOT}/common/core/inc"
    "${USBX_ECLIPSE_ROOT}/ports/generic/inc"
    "${USBX_ECLIPSE_ROOT}/common/usbx_device_classes/inc"
    "${USBX_ST_DCD_ROOT}"
)

get_target_property(USBX_CUBEMX_SOURCES USBX SOURCES)
set(USBX_ECLIPSE_SOURCES)

foreach(USBX_SOURCE IN LISTS USBX_CUBEMX_SOURCES)
    set(USBX_REMAP "${USBX_SOURCE}")

    string(REPLACE
        "Middlewares/ST/usbx/common/core"
        "Middlewares/Eclipse/usbx/common/core"
        USBX_REMAP "${USBX_REMAP}")

    string(REPLACE
        "Middlewares/ST/usbx/common/usbx_device_classes"
        "Middlewares/Eclipse/usbx/common/usbx_device_classes"
        USBX_REMAP "${USBX_REMAP}")

    string(REPLACE
        "Middlewares/ST/usbx/common/usbx_stm32_device_controllers"
        "Middlewares/ST/usbx_stm32_dcd"
        USBX_REMAP "${USBX_REMAP}")

    list(APPEND USBX_ECLIPSE_SOURCES "${USBX_REMAP}")
endforeach()

# USBX 6.5 added an internal byte-pool allocator used by the portable memory
# manager. These files do not exist in the CubeMX 6.2 source list, so add them
# explicitly when building the Eclipse stack.
list(APPEND USBX_ECLIPSE_SOURCES
    "${USBX_ECLIPSE_ROOT}/common/core/src/ux_utility_memory_byte_pool_create.c"
    "${USBX_ECLIPSE_ROOT}/common/core/src/ux_utility_memory_byte_pool_search.c"
)

set_property(TARGET USBX PROPERTY SOURCES "${USBX_ECLIPSE_SOURCES}")
