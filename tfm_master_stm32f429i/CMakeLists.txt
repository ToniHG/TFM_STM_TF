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
set(CMAKE_PROJECT_NAME tfm_master_stm32f429i)

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

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Define the path to the STM32CubeF4 firmware package
set(ST_FW "C:/Users/tonih/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ../common/inc
    ${ST_FW}/Drivers/BSP/STM32F429I-Discovery
    ${ST_FW}/Drivers/BSP/Components/ili9341
    ${ST_FW}/Drivers/BSP/Components/stmpe811
    ${ST_FW}/Drivers/BSP/Components/Common
    ${ST_FW}/Utilities/Fonts
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ../common/src/can_protocol.c
    ../common/src/crc16.c
    ../common/src/fault_tolerance.c
    ${ST_FW}/Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.c
    ${ST_FW}/Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.c
    ${ST_FW}/Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_sdram.c
    ${ST_FW}/Drivers/BSP/Components/ili9341/ili9341.c
    ${ST_FW}/Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_ts.c
    ${ST_FW}/Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_io.c
    ${ST_FW}/Drivers/BSP/Components/stmpe811/stmpe811.c
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
