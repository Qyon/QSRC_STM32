cmake_minimum_required(VERSION 3.6)
include(CMakeForceCompiler)
SET(CMAKE_SYSTEM_NAME "Generic")
SET(CMAKE_SYSTEM_VERSION 1)

set(TOOLCHAIN_DIR D:/Programy/gcc-arm-none-eabi-6-2017-q1-update-win32/bin/)

CMAKE_FORCE_C_COMPILER(${TOOLCHAIN_DIR}/arm-none-eabi-gcc.exe GNU)
CMAKE_FORCE_CXX_COMPILER(${TOOLCHAIN_DIR}/arm-none-eabi-g++.exe GNU)

project(QSRC_STM32 C ASM CXX)


add_definitions(-DSTM32F103C8)
add_definitions(-DSTM32F10X_MD)
add_definitions(-DSTM32F103xB)
add_definitions(-DSUPPORT_CPLUSPLUS)
add_definitions(-D__ASSEMBLY__)



SET(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/STM32F103C8T6.ld)
SET(COMMON_FLAGS " -mcpu=cortex-m3 -mthumb -Wall -ffunction-sections -O3 -fdata-sections -fno-common -Wdouble-promotion ")
SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11 -fno-exceptions")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
#SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map -Wl,--print-gc-sections --specs=nano.specs -T ${LINKER_SCRIPT}")
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map -Wl,--gc-sections  --specs=nano.specs -T ${LINKER_SCRIPT}")

file(GLOB_RECURSE USER_SOURCES "*.c")
file(GLOB_RECURSE USER_SOURCES_CXX "*.cpp")
file(GLOB_RECURSE USER_HEADERS "*.h")
#file(GLOB_RECURSE HAL_SOURCES "stm_lib/src/*.c")

include_directories(
        Drivers/CMSIS/Device/ST/STM32F1xx/Include
        Drivers/CMSIS/Include
        Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
        Drivers/STM32F1xx_HAL_Driver/Inc
        Inc
        .)

add_executable(${PROJECT_NAME}.elf ${USER_SOURCES} ${USER_SOURCES_CXX} ${USER_HEADERS} ${HAL_SOURCES} Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map")
set(HEX_FILE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")
set(CMAKE_CXX_STANDARD 11)

add_custom_target(program
        DEPENDS ${PROJECT_NAME}.elf
        COMMAND D:/Programy/stlink-1.3.0-win64/bin/st-flash  --reset write ${BIN_FILE} 0x08000000
        COMMENT "flashing ${BIN_FILE}")