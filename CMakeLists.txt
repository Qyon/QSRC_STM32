#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.7)
set(TOOLCHAIN_DIR "D:/Programy/gcc-arm-none-eabi-9-2019-q4-major-win32/bin/")
LIST(APPEND CMAKE_PROGRAM_PATH ${TOOLCHAIN_DIR})


# specify cross compilers and tools
SET(CMAKE_C_COMPILER_WORKS 1)
SET(CMAKE_C_COMPILER arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER_WORKS 1)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR ${TOOLCHAIN_DIR}arm-none-eabi-ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_DIR}arm-none-eabi-objcopy.exe)
set(CMAKE_OBJDUMP ${TOOLCHAIN_DIR}arm-none-eabi-objdump)
set(SIZE ${TOOLCHAIN_DIR}arm-none-eabi-size)

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld)

#Uncomment for hardware floating point
#SET(FPU_FLAGS "-mfloat-abi=hard -mfpu=fpv4-sp-d16")
#add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1)

#Uncomment for software floating point
SET(FPU_FLAGS "-mfloat-abi=soft")

SET(COMMON_FLAGS
    "-mcpu=cortex-m3 ${FPU_FLAGS} -mthumb -mthumb-interwork -ffunction-sections -fdata-sections \
    -g -fno-common -fmessage-length=0 -specs=nosys.specs -specs=nano.specs -fno-exceptions")

add_link_options(-fno-exceptions -Wl,--gc-sections)


SET(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

PROJECT(QSRC_STM32 C CXX ASM)
set(CMAKE_CXX_STANDARD 11)

#add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1)
add_definitions(-D__weak=__attribute__\(\(weak\)\) -D__packed=__attribute__\(\(__packed__\)\) -DUSE_HAL_DRIVER -DSTM32F103xB)

file(GLOB_RECURSE SOURCES "startup/*.*" "Drivers/*.*" "Src/*.*" "Inc/*.*" Drivers/TMC-API/tmc/helpers/*.* Drivers/TMC-API/tmc/ic/TMC2160/*.*)

include_directories(Inc Drivers/STM32F1xx_HAL_Driver/Inc Drivers/STM32F1xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32F1xx/Include Drivers/CMSIS/Include Drivers/TMC-API/tmc/helpers  Drivers/TMC-API/tmc/ic/TMC2160)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
