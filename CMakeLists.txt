# 指定系统名称为 Generic（裸机/RTOS 环境）
set(CMAKE_SYSTEM_NAME Generic)

# CMake 最低版本要求
cmake_minimum_required(VERSION 3.22)

#
# 本文件仅在首次生成时创建，
# 若多次调用 CubeMX 转换器不会自动覆盖。
#
# 用户可根据需要自由修改。
#

# 配置 C 语言标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# ===================== 【STM32F407IG 硬件浮点 FPU 配置】 =====================
# 1. 全局编译选项：指定 Cortex-M4 内核 + Thumb 指令集
add_compile_options(-mcpu=cortex-m4 -mthumb)

# 2. 硬件浮点选项：启用单精度 FPU 并采用硬浮点 ABI
#   - mfpu=fpv4-sp-d16  : F407 的单精度浮点单元
#   - mfloat-abi=hard   : 通过 FPU 寄存器传递浮点参数
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# 3. 全局宏定义：启用 FPU 使用标记（必须定义，否则可能 HardFault）
#   __FPU_USED=1  : 告知 CMSIS 头文件启用 FPU 指令生成与上下文保存
# add_compile_definitions(__FPU_USED=1)// 该定义已在 CubeMX 生成的代码中包含，通常不需要重复定义

# 4. （可选）若项目使用了 CMSIS-DSP 数学库，请保留以下定义
# add_compile_definitions(ARM_MATH_CM4)

# 5. 链接选项：与编译选项保持一致（防御性编程，避免链接异常）
add_link_options(-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)

# ============================================================================

# 设置默认构建类型（若未指定）
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# 定义项目名称变量
set(CMAKE_PROJECT_NAME robot_arm)

# 开启导出编译命令（方便 clangd 等工具进行代码索引）
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 声明项目名称
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 启用 C 和汇编语言支持
enable_language(C ASM)

# 创建可执行文件目标
add_executable(${CMAKE_PROJECT_NAME})

# 添加 STM32CubeMX 生成的源文件子目录
add_subdirectory(cmake/stm32cubemx)

# 设置链接库搜索路径（用户自定义部分）
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 在此添加用户自定义库路径
)

# 添加用户源代码文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # 示例：添加用户自己的驱动文件
    Includes/Bottom_layer/uart_driver.c
    Includes/Bottom_layer/Double_buffer.c
    Includes/Bottom_layer/RingBuffer.c
    Includes/Bottom_layer/CRC_check.c
    Includes/Bottom_layer/uart_driver.c
    Includes/Bottom_layer/Motor_driver.c
    Includes/Application_layer/pid.c
    Includes/Application_layer/uart_task.c
    Includes/Application_layer/robotic_arm.c
    Includes/Application_layer/Arm_control_task.c
)

# 添加用户头文件搜索路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 示例：用户底层驱动头文件路径
    ${CMAKE_CURRENT_SOURCE_DIR}/Includes/Bottom_layer
    # 示例：用户应用层头文件路径
    ${CMAKE_CURRENT_SOURCE_DIR}/Includes/Application_layer
)

# 添加项目级符号定义（用户自定义宏）
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 在此添加用户自定义宏
)

# 修复 C++ 文件链接时可能出现的 ob 库依赖问题
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 链接必要的库
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx               # CubeMX 生成的硬件抽象层库
    # 在此添加用户自定义库（如 -lm 等）
    m
)
