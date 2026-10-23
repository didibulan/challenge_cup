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
set(CMAKE_PROJECT_NAME MiniArm_Test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

file(GLOB_RECURSE UserLib_SOURCES
        #        "./User_Lib/IntraBoards_Comm/*.c"
#        "./UserLib/ChassisCtrl/*.c"
        #        "./User_Lib/Dwt/*.c"
        #        "./User_Lib/UI/*.c"
        "./UserLib/dev_ws2812/*.c"
        #        "./User_Lib/Power_Control/*.c"
        #        "./User_Lib/BMI088/Device_BMI088/*.c"
        #        "./User_Lib/BMI088/Alg/EKF/*.c"
        #        "./User_Lib/BMI088/Alg/kalman/*.c"      # 添加 Kalman 滤波源文件
        #        "./User_Lib/BMI088/Alg/IMU_PID/*.c"     # 添加 IMU PID 源文件（如果需要）
        #        "./User_Lib/Power_Control/*.c"

)

file(GLOB_RECURSE Lib_SOURCES
        "./Phoenix_Lib/Algorithm/Chassis_calc/*.c"
        "./Phoenix_Lib/Algorithm/Crc/*.c"
        "./Phoenix_Lib/Algorithm/Pid/*.c"
        "./Phoenix_Lib/Algorithm/Fliters/*.c"
        "./Phoenix_Lib/Bsp/Fdcan/*.c"
        "./Phoenix_Lib/Bsp/Log/*.c"
        "./Phoenix_Lib/Bsp/Uart/*.c"
        "./Phoenix_Lib/Module/Dr16/*.c"
        "./Phoenix_Lib/Module/Referee/*.c"
        "./Phoenix_Lib/Module/Motor/Dji/*.c"
        "./Phoenix_Lib/Module/Motor/Dm/*.c"
        "./Phoenix_Lib/Module/Daemon/*.c"
        "./Phoenix_Lib/Module/SEGGER/*.c"
        "./Phoenix_Lib/Module/Servo/*.c"
)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        App/Alg_Task.c
        App/App_ArmPosTrace.c
        App/App_ArmTask.c
        App/App_CommTask.c
        App/App_IMUTask.c
        App/App_InitTask.c
        App/App_RefereeTask.c
        App/App_StateMachine.c
        App/engineer_config.h
        App/robot_config.h
        App/App_ArmPosTrace.h
        Phoenix_Lib/Bsp/Fdcan/bsp_fdcan.c
        Phoenix_Lib/Bsp/Log/bsp_log.c
        UserLib/WS2812/dev_ws2812.h
        Phoenix_Lib/Module/Motor/Dm/dev_motor_dm.c
        Phoenix_Lib/Algorithm/Pid/alg_pid.c
        Phoenix_Lib/Module/SEGGER/Jlink_RTT/SEGGER_RTT_printf.c
        Phoenix_Lib/Module/SEGGER/Jlink_RTT/SEGGER_RTT.c
        Phoenix_Lib/Module/SEGGER/Jlink_RTT/SEGGER_SYSVIEW.c
        Phoenix_Lib/Module/SEGGER/dev_segger.c
        UserLib/Robotics/Alg_calc.cpp
        UserLib/Robotics/Alg_calc.h
        UserLib/Robotics/dev_matrix.cpp
        UserLib/Robotics/dev_matrix.h
        UserLib/Robotics/dev_robotics.cpp
        UserLib/Robotics/dev_robotics.h
        UserLib/C_Robotics/c_robotics.c
        UserLib/C_Robotics/c_robotics.h
        UserLib/Dwt/Bsp_dwt.c
        UserLib/Dwt/Bsp_dwt.h
        UserLib/Planning/dev_planning.c
        UserLib/Planning/dev_planning.h
        UserLib/Robotics/dev_dynamics_identification.c
        UserLib/Robotics/dev_dynamics_identification.h
        Phoenix_Lib/Bsp/Uart/bsp_uart.c
        Phoenix_Lib/Module/Referee/dev_referee.c
        Phoenix_Lib/Algorithm/Crc/alg_crc.c
        Phoenix_Lib/Module/Daemon/dev_daemon.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths

        ./App
        ./UserLib/
        ./UserLib/Dwt
        ./UserLib/Robotics
        ./UserLib/C_Robotics
        ./UserLib/UserRobtics
        ./UserLib/MiniPC
        ./UserLib/Planning
        ./UserLib/Compliance
        ./UserLib/IntraBoards_Comm
        ./UserLib/WS2812
        ./UserLib/BMI088/Device_BMI088
        ./UserLib/BMI088/Alg/EKF
        ./UserLib/BMI088/Alg/kalman
        ./UserLib/BMI088/Alg/IMU_PID
        ./Phoenix_Lib/Module/SEGGER/
        ./Phoenix_Lib/Module/SEGGER/Jlink_RTT/
        ./Phoenix_Lib/Algorithm/Pid/
        ./Phoenix_Lib/Algorithm/Crc/
        ./Phoenix_Lib/Bsp/Fdcan/
        ./Phoenix_Lib/Bsp/Log/
        ./Phoenix_Lib/Bsp/Uart/
        ./Phoenix_Lib/Module/Dr16/
        ./Phoenix_Lib/Module/Referee/
        ./Phoenix_Lib/Module/Daemon/
        ./Phoenix_Lib/Module/Motor/Dm/
        ./Phoenix_Lib/Module/Motor/Dji/
        ./Phoenix_Lib/Module/Motor/RobStride/
        ./Phoenix_Lib/Module/Servo/
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

target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/GCC/libarm_cortexM7lfsp_math.a)
