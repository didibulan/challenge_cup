//
// Created by didib on 2026/7/2.
//
/**
*   @file App_InitTask.c
*   @brief 车载模块初始化，包含电机，舵机等
*   @author Zhong Kena
*   @date 2026/7/2
*   @note
*/
#include "App_InitTask.h"
/************************************Register**************************************/
//备用电机占位
DjiMotorInstance_s* wheel_motors[4] = {NULL};
//摇臂电机
DmMotorInstance_s* leg_motors[4] = {NULL};
//存矿模块
ServoInstance_s* storage_servo[2] = {NULL};
//底盘实例
ChassisInstance_s *chassis_instance = NULL;
//是否初始化完成
bool istrackInitialized = false;
/************************************Init_Settings**************************************/
//备用电机占位

//摇臂电机
DmMotorInitConfig_s leg_motor1_config = {
    .topic_name = "leg_motor1",
    .type = J4340,
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
        .DLC = FDCAN_DLC_BYTES_8,
        .can_number = 2,
        .tx_id = 0x01,
        .rx_id = 0x11,
    },
    .parameters = {
        .pos_max = 12.5f,
        .vel_max = 10,
        .tor_max = 28,
        .kd_int = 0.5f,
        .kp_int = 50.f,
        .kd_max = 5.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 5.0f,
        .ki = 0.f,
        .kd = 0.0f,
        .i_max = 2.0f,
        .out_max = 25.0f,
    },
    .angle_pid_config = {
        .kp = 38.0f,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 0.0f,
        .out_max = LEG_MAX_SPEED,
        .angle_max = 0.0f,
    }
};

DmMotorInitConfig_s leg_motor2_config = {
    .topic_name = "leg_motor2",
    .type = J4340,
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
        .DLC = FDCAN_DLC_BYTES_8,
        .can_number = 2,
        .tx_id = 0x02,
        .rx_id = 0x12,
    },
    .parameters = {
        .pos_max = 12.5f,
        .vel_max = 10,
        .tor_max = 28,
        .kd_int = 0.5f,
        .kp_int = 50.f,
        .kd_max = 5.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 5.0f,
        .ki = 0.f,
        .kd = 0.0f,
        .i_max = 2.0f,
        .out_max = 25.0f,
    },
    .angle_pid_config = {
        .kp = 38.f,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 0.0f,
        .out_max = LEG_MAX_SPEED,
        .angle_max = 0.0f,
    }
};

DmMotorInitConfig_s leg_motor3_config = {
    .topic_name = "leg_motor3",
    .type = J4340,
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
        .DLC = FDCAN_DLC_BYTES_8,
        .can_number = 2,
        .tx_id = 0x03,
        .rx_id = 0x13,
    },
    .parameters = {
        .pos_max = 12.5f,
        .vel_max = 10,
        .tor_max = 28,
        .kd_int = 0.5f,
        .kp_int = 50.f,
        .kd_max = 5.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 5.0f,
        .ki = 0.f,
        .kd = 0.0f,
        .i_max = 2.0f,
        .out_max = 25.0f,
    },
    .angle_pid_config = {
        .kp = 38.f,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 0.0f,
        .out_max = LEG_MAX_SPEED,
        .angle_max = 0.0f,
    }
};

DmMotorInitConfig_s leg_motor4_config = {
    .topic_name = "leg_motor4",
    .type = J4340,
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
        .DLC = FDCAN_DLC_BYTES_8,
        .can_number = 2,
        .tx_id = 0x04,
        .rx_id = 0x14,
    },
    .parameters = {
        .pos_max = 12.5f,
        .vel_max = 10,
        .tor_max = 28,
        .kd_int = 0.5f,
        .kp_int = 50.f,
        .kd_max = 5.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 5.0f,
        .ki = 0.15f,
        .kd = 0.0f,
        .i_max = 2.0f,
        .out_max = 25.0f,
    },
    .angle_pid_config = {
        .kp = 38.0f,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 0.0f,
        .out_max = LEG_MAX_SPEED,
        .angle_max = 0.0f,
    }
};

//存矿模块
ServoInitConfig_s servo_config_1 = {
    .channel = TIM_CHANNEL_1,
    .tim_handle = &htim1,
    .type = SERVO_NORMAL
};

ServoInitConfig_s servo_config_2 = {
    .channel = TIM_CHANNEL_1,
    .tim_handle = &htim1,
    .type = SERVO_NORMAL
};

//底盘实例
ChassisInitConfig_s chassis_config = {
    .type = Mecanum_Wheel,
    .motor_config = {
        {
            .topic_name = "chassis_motor1",
            .type = M3508,
            .control_mode = DJI_VELOCITY,
            .can_config = {
                .can_number = 1,
                .tx_id = 0x200,
                .rx_id = 0x204,
            },
            .reduction_ratio = 19.0f,
            .velocity_pid_config = {
                .kp = 100.0f,
                .ki = 10.0f,
                .kd = 0.0f,
                .i_max = 3000.0f,
                .out_max = 15000.0f,
            },
        },
        {
            .topic_name = "chassis_motor2",
            .type = M3508,
            .control_mode = DJI_VELOCITY,
            .can_config = {
                .can_number = 1,
                .tx_id = 0x200,
                .rx_id = 0x203,
            },
            .reduction_ratio = 19.0f,
            .velocity_pid_config = {
                .kp = 120.0f,
                .ki = 50.0f,
                .kd = 0.0f,
                .i_max = 1500.0f,
                .out_max = 15000.0f,
            },
        },
        {
            .topic_name = "chassis_motor3",
            .type = M3508,
            .control_mode = DJI_VELOCITY,
            .can_config = {
                .can_number = 1,
                .tx_id = 0x200,
                .rx_id = 0x201,
            },
            .reduction_ratio = 19.0f,
            .velocity_pid_config = {
                .kp = 120.0f,
                .ki = 40.0f,
                .kd = 0.0f,
                .i_max = 1500.0f,
                .out_max = 15000.0f,
            },
        },
        {
            .topic_name = "chassis_motor4",
            .type = M3508,
            .control_mode = DJI_VELOCITY,
            .can_config = {
                .can_number = 1,
                .tx_id = 0x200,
                .rx_id = 0x202,
            },
            .reduction_ratio = 19.0f,
            .velocity_pid_config = {
                .kp = 120.0f,
               .ki = 20.0f,
                .kd = 0.0f,
                .i_max = 3000.0f,
                .out_max = 15000.0f,
            },
        }
    },
    .mecanum_message = {
        .length_a = 0.319f,
        .length_b = 0.446f,
        .wheel_radius = 0.075f,
    },
    .omni_steering_message = {
        .wheel_radius = 0.075f,
        .chassis_radius = 0.5483f,
    }
};

/************************************Task**************************************/
void App_Init(void const* argument){
    //备用电机占位

    //摇臂电机
    while (leg_motors[0] == NULL)leg_motors[0] = Motor_DM_Register(&leg_motor1_config);
    while (leg_motors[1] == NULL)leg_motors[1] = Motor_DM_Register(&leg_motor2_config);
    while (leg_motors[2] == NULL)leg_motors[2] = Motor_DM_Register(&leg_motor3_config);
    while (leg_motors[3] == NULL)leg_motors[3] = Motor_DM_Register(&leg_motor4_config);
    //存矿模块
    while (storage_servo[0] == NULL) storage_servo[0] = Servo_Register(&servo_config_1);
    while (storage_servo[1] == NULL) storage_servo[1] = Servo_Register(&servo_config_2);
    //底盘实例
    while (chassis_instance == NULL) chassis_instance = Chassis_Register(&chassis_config);
    //是否初始化完成
    istrackInitialized = true;
    //删除初始化任务
    vTaskDelete(NULL);
}