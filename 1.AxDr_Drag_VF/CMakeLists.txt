# =============================================================================
#  AxDr — STM32G431RBTx FOC 无刷电机驱动
#  CLion / CMake 工程 (arm-none-eabi-gcc)
#
#  配置方法 (CLion 或命令行):
#     cmake -B build/Debug -G Ninja ^
#           -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake ^
#           -DCMAKE_BUILD_TYPE=Debug
#     cmake --build build/Debug
#
#  产物: AxDr.elf / AxDr.hex / AxDr.bin
# =============================================================================
cmake_minimum_required(VERSION 3.22)

project(AxDr C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# 目标芯片参数: Cortex-M4 + 单精度 FPU (硬浮点), 对应 Keil 的 FPU2
# -----------------------------------------------------------------------------
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# -----------------------------------------------------------------------------
# 编译选项
# -----------------------------------------------------------------------------
add_compile_options(
    ${MCU_FLAGS}
    -Wall
    -Wextra
    -Wno-unused-parameter
    -fdata-sections
    -ffunction-sections
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-g3>
    $<$<CONFIG:Release>:-Os>
)

add_compile_definitions(
    STM32G431xx
    USE_HAL_DRIVER
)

# -----------------------------------------------------------------------------
# 链接选项
# -----------------------------------------------------------------------------
add_link_options(
    ${MCU_FLAGS}
    -T${CMAKE_SOURCE_DIR}/STM32G431RBTx_FLASH.ld
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,--no-warn-rwx-segments
    -specs=nano.specs
)

# -----------------------------------------------------------------------------
# 头文件搜索路径 (和 Keil 工程保持一致)
# -----------------------------------------------------------------------------
include_directories(
    Core/Inc
    USB_Device/App
    USB_Device/Target
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
    User/motor
    User/moldue/inc
)

# -----------------------------------------------------------------------------
# 源文件
# -----------------------------------------------------------------------------
# HAL 驱动: 这个目录里只放了工程实际用到的那 26 个文件 (CubeMX 生成的)
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS
    Drivers/STM32G4xx_HAL_Driver/Src/*.c
)

set(APP_SOURCES
    # --- CubeMX 生成的 Core ---
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/adc.c
    Core/Src/dma.c
    Core/Src/fdcan.c
    Core/Src/spi.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Src/stm32g4xx_it.c
    Core/Src/stm32g4xx_hal_msp.c
    Core/Src/system_stm32g4xx.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c

    # --- USB Device ---
    USB_Device/Target/usbd_conf.c
    USB_Device/App/usb_device.c
    USB_Device/App/usbd_desc.c
    USB_Device/App/usbd_cdc_if.c

    # --- USB Device Library ---
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

    # --- 用户代码 ---
    User/moldue/vofa.c
    User/motor/foc_calc.c
    User/motor/foc_ctrl.c
    User/motor/foc_drv.c
    User/motor/foc_loop.c
    User/motor/foc_protect.c
    User/motor/mt6701.c
    User/motor/ntc.c
    User/motor/pid.c

    # --- 启动文件 (GCC 语法) ---
    startup_stm32g431xx.S
)

add_executable(${CMAKE_PROJECT_NAME} ${APP_SOURCES} ${HAL_SOURCES})

# 让产物叫 AxDr.elf (OpenOCD / CLion 调试配置都按这个名字找)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")

# -----------------------------------------------------------------------------
# 生成 .hex / .bin 并打印占用
# -----------------------------------------------------------------------------
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "生成 ${CMAKE_PROJECT_NAME}.hex / ${CMAKE_PROJECT_NAME}.bin"
)
