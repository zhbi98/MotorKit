# No operating system
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.21)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
# skip compiler checks
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(ODriveV3 C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# You can try to annotate this statement to validate the UART floating-point printing problem
set(COMMON_FLAGS "-specs=nosys.specs -specs=nano.specs -u _printf_float -u _scanf_float")
# add_compile_options(-specs=nosys.specs -specs=nano.specs -u _printf_float -u _scanf_float)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork -mfloat-abi=hard -mfpu=fpv4-sp-d16) # soft/softfp/hard
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(
    ./
    ${CMAKE_SOURCE_DIR}/autogen
    ${CMAKE_SOURCE_DIR}/Board/v3/Inc
    ${CMAKE_SOURCE_DIR}/communication
    ${CMAKE_SOURCE_DIR}/communication/can
    ${CMAKE_SOURCE_DIR}/Drivers
    ${CMAKE_SOURCE_DIR}/Drivers/DRV8301
    ${CMAKE_SOURCE_DIR}/Drivers/STM32
    ${CMAKE_SOURCE_DIR}/fibre_cpp
    ${CMAKE_SOURCE_DIR}/fibre_cpp/include
    ${CMAKE_SOURCE_DIR}/fibre_cpp/include/fibre
    # ${CMAKE_SOURCE_DIR}/fibre_cpp/platform_support
    ${CMAKE_SOURCE_DIR}/MotorControl
    ${CMAKE_SOURCE_DIR}/ThirdParty/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/ThirdParty/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/include
    ${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_SOURCE_DIR}/ThirdParty/STM32_USB_Device_Library/Class/CDC/Inc
    ${CMAKE_SOURCE_DIR}/ThirdParty/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_SOURCE_DIR}/ThirdParty/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/ThirdParty/STM32F4xx_HAL_Driver/Inc/Legacy
)

add_definitions(-DUSE_HAL_DRIVER -D__MICROLIB -DSTM32F4 -DSTM32F4xx -DARM_MATH_CM4 -DSTM32F405xx)
add_definitions(-DFPU_FPV4 -DFIBRE_ENABLE_CLIENT=0 -DFIBRE_ENABLE_SERVER=1 -D__arm__)

aux_source_directory(${CMAKE_SOURCE_DIR}/autogen AUTOGEN)
aux_source_directory(${CMAKE_SOURCE_DIR}/Board/v3 BOARD_MAIN)
aux_source_directory(${CMAKE_SOURCE_DIR}/Board/v3/Src BOARD_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/communication COMMUNICATION_MAIN)
aux_source_directory(${CMAKE_SOURCE_DIR}/communication/can COMMUNICATION_CAN)
aux_source_directory(${CMAKE_SOURCE_DIR}/Drivers/DRV8301 DRV_DRV8301)
aux_source_directory(${CMAKE_SOURCE_DIR}/Drivers/STM32 DRV_STM32)
aux_source_directory(${CMAKE_SOURCE_DIR}/fibre_cpp FIBRE_CPP_MAIN)
# aux_source_directory(${CMAKE_SOURCE_DIR}/fibre_cpp/platform_support PLATFORM_SUPPORT)
aux_source_directory(${CMAKE_SOURCE_DIR}/MotorControl MOTOR_CONTROL)
aux_source_directory(${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source RTOS_MAIN)
aux_source_directory(${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/CMSIS_RTOS CMSIS_RTOS)
aux_source_directory(${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F GCC_RTOS)
aux_source_directory(${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/portable/MemMang MEMMANG_RTOS)
aux_source_directory(${CMAKE_SOURCE_DIR}/ThirdParty/STM32_USB_Device_Library/Class/CDC/Src USB_CLASS)
aux_source_directory(${CMAKE_SOURCE_DIR}/ThirdParty/STM32_USB_Device_Library/Core/Src USB_CORE)
aux_source_directory(${CMAKE_SOURCE_DIR}/ThirdParty/STM32F4xx_HAL_Driver/Src HAL_DRIVER)
aux_source_directory(${CMAKE_SOURCE_DIR}/ FIRMWARE_MAIN)

set(STARTUP       ${CMAKE_SOURCE_DIR}/Board/v3/startup_stm32f405xx.s)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Board/v3/STM32F405RGTx_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork -mfloat-abi=hard -mfpu=fpv4-sp-d16) # soft/softfp/hard
add_link_options(-T ${LINKER_SCRIPT})

link_directories("ThirdParty/CMSIS/Lib/GCC")
link_libraries("libarm_cortexM4lf_math.a")

add_executable(${PROJECT_NAME}.elf 
    ${AUTOGEN}
    ${BOARD_MAIN}
    ${BOARD_SRC}
    ${COMMUNICATION_MAIN}
    ${COMMUNICATION_CAN}
    ${DRV_DRV8301}
    ${DRV_STM32}
    ${FIBRE_CPP_MAIN}
    # ${PLATFORM_SUPPORT}
    ${MOTOR_CONTROL}
    ${RTOS_MAIN}
    ${CMSIS_RTOS}
    ${GCC_RTOS}
    ${MEMMANG_RTOS}
    ${USB_CLASS}
    ${USB_CORE}
    ${HAL_DRIVER}
    ${FIRMWARE_MAIN}
    ${STARTUP} 
    ${LINKER_SCRIPT}
)

# target_link_libraries(
#     ${PROJECT_NAME}.elf 
#     ${CMAKE_SOURCE_DIR}/ThirdParty/CMSIS/Lib/GCC/libarm_cortexM4lf_math.a
# )

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
