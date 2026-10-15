cmake_minimum_required(VERSION 3.22)

#
# TDCRControl — STM32H743 + FreeRTOS
# 本文件由 STM32CubeMX 转换器生成，之后由用户手工维护。
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
set(CMAKE_PROJECT_NAME ControlSystem_STM32H7)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# ---------------------------------------------------------------------------
# 应用层源码 (APP / BSP)
# ---------------------------------------------------------------------------
set(TDCR_APP_SRC
        # --- APP/Task : 任务层 ---
        APP/Task/CmdCtrlTask.c
        APP/Task/SensorDataTask.c
        APP/Task/DataParserTask.c

        # --- APP/CR : 连续体机器人模型与控制 ---
        APP/CR/CR.c
        APP/CR/SDM.c
        APP/CR/SDM_vc.c
        APP/CR/Perception.c
        APP/CR/ForceBalance.c

        # --- BSP/Communicate : 通信与协议 ---
        BSP/Communicate/proto.c
        BSP/Communicate/can_driver.c
        BSP/Communicate/cmd_packer.c
        BSP/Communicate/motor_cmd_parser.c
        BSP/Communicate/pc_cmd_parser.c

        # --- BSP/Motor : 步进闭环驱动 ---
        BSP/Motor/Motor.c
        BSP/Motor/X_V2.c

        # --- BSP/Sensor : 传感器驱动 ---
        BSP/Sensor/Sensor.c
        BSP/Sensor/ForceFilter.c
        BSP/Sensor/CMCU-06.c
        BSP/Sensor/IMU.c
        BSP/Sensor/WT_IMU.c
)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME} ${TDCR_APP_SRC})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        APP
        APP/Common
        APP/CR
        APP/Task
        BSP
        BSP/Communicate
        BSP/Motor
        BSP/Sensor
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

# ---------------------------------------------------------------------------
# 烧录 / 调试（OpenOCD）
#   cmake --build build/Debug --target flash
#   换探针: cmake --preset Debug -DOPENOCD_INTERFACE=cmsis-dap.cfg
# ---------------------------------------------------------------------------
find_program(OPENOCD_EXE NAMES openocd
        HINTS "$ENV{USERPROFILE}/xpack-openocd-0.12.0-6/bin")

set(OPENOCD_INTERFACE "stlink.cfg"   CACHE STRING "OpenOCD interface 配置")
set(OPENOCD_TARGET    "stm32h7x.cfg" CACHE STRING "OpenOCD target 配置")

if(OPENOCD_EXE)
    # program 只传文件名 + WORKING_DIRECTORY：源码路径含空格，绝对路径会被 make/ninja 拆参
    add_custom_target(flash
            COMMAND "${OPENOCD_EXE}"
                    -f "interface/${OPENOCD_INTERFACE}"
                    -f "target/${OPENOCD_TARGET}"
                    -c "program $<TARGET_FILE_NAME:${CMAKE_PROJECT_NAME}> verify reset exit"
            DEPENDS ${CMAKE_PROJECT_NAME}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            USES_TERMINAL
            COMMENT ">>> OpenOCD flashing ${CMAKE_PROJECT_NAME}.elf")

    add_custom_target(ocd_reset
            COMMAND "${OPENOCD_EXE}"
                    -f "interface/${OPENOCD_INTERFACE}"
                    -f "target/${OPENOCD_TARGET}"
                    -c "init; reset run; exit"
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            USES_TERMINAL
            COMMENT ">>> OpenOCD reset & run")
else()
    message(WARNING "openocd not found: flash / ocd_reset targets unavailable")
endif()
