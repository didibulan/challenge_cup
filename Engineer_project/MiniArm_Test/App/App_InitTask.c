//
// Created by didib on 2026/7/6.
//
/**
*   @file App_InitTask.c
*   @brief
*   @author Zhong Kena
*   @date 2026/7/6
*   @note
*/
#include "App_InitTask.h"
/************************************宏定义开关**************************************/

/************************************Register_extern_variable**************************************/
DmMotorInstance_s* arm_motors[9] = {NULL};
/************************************Init_Settings**************************************/
DmMotorInitConfig_s arm_motor0_config = {
    .topic_name = "arm_motor0",
    .type = J6248,
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
        .DLC = FDCAN_DLC_BYTES_8,
        .can_number = 1,
        .tx_id = 0x01,
        .rx_id = 0x11,
    },
    .parameters = {
        .pos_max = 3.1415927f,
        .vel_max = 45,
        .tor_max = 20,
        .kd_int = 1.3f,
        .kp_int = 100.8f,
        .kd_max = 5.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 1.f,
        .ki = 0.02f,
        .kd = 0.001f,
        .kf = 0.f,
        .dead_zone = 0,
        .i_max = 1.0f,
        .out_max = 10.0f,
    },
    .angle_pid_config = {
        .kp = 5.5f,
        .kd = 0.0f,
        .ki = 0.0f,
        .dead_zone = 0.05f,
        .i_max = 3.0f,
        .out_max = 3.0f,
        .angle_max = 6.2831853f,
    }
};

DmMotorInitConfig_s arm_motor1_config = {
    .topic_name = "arm_motor1",
    .type = J6248,                //J10010l
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = CAN_STANDARD_MODE,
        .DLC = FDCAN_DLC_BYTES_32,
        .can_number = 1,
        .tx_id = 0x02,
        .rx_id = 0x12,
    },
    .parameters = {
        .pos_max = 3.1415927f,
        .vel_max = 25,
        .tor_max = 200,
        .kd_int = 2.8f,
        .kp_int = 290.f,
        .kd_max = 5.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 9.0f,
        .ki = 0.3f,
        .kd = 0.0f,
        .i_max = 4.0f,
        .out_max = 10.0f,
        .i_variable_max = 0,
        .i_variable_min = 0.1f
    },
    .angle_pid_config = {
        .kp = 10,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 3.0f,
        .out_max = ARM_MAX_SPEED,
        .angle_max = 0.0f,
    }
};

DmMotorInitConfig_s arm_motor2_config = {
    .topic_name = "arm_motor2",
    .type = J6248,
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
        .DLC = FDCAN_DLC_BYTES_8,
        .can_number = 1,
        .tx_id = 0x03,
        .rx_id = 0x13,
    },
    .parameters = {
        .pos_max = 3.1415927f,
        .vel_max = 20,
        .tor_max = 120,
        .kd_int = 0.15f,
        .kp_int = 80.8f,
        .kd_max = 5.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 15.0f,
        .ki = 0.4f,
        .kd = 0.0f,
        .i_max = 3.0f,
        .out_max = 10.0f,
        .i_variable_max = 0,
        .i_variable_min = 0.1f
    },
    .angle_pid_config = {
        .kp = 4.5f,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 0.5f,
        .out_max = ARM_MAX_SPEED,
        .angle_max = 0.0f,
    }
};

DmMotorInitConfig_s arm_motor3_config = {
    .topic_name = "arm_motor3",
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
        .pos_max = 3.1415927f,
        .vel_max = 30,
        .tor_max = 10,
        .kd_int = 0.8f,
        .kp_int = 80.8f,
        .kd_max = 5.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 0.5f,
        .ki = 0.02f,
        .kd = 0.0f,
        .i_max = 3.0f,
        .out_max = 10.0f,
    },
    .angle_pid_config = {
        .kp = 7.0f,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 0.15f,
        .out_max = ARM_MAX_SPEED,
        .angle_max = PI,
    }
};

DmMotorInitConfig_s arm_motor4_config = {
    .topic_name = "arm_motor4",
    .type = J4310,
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
        .DLC = FDCAN_DLC_BYTES_8,
        .can_number = 2,
        .tx_id = 0x05,
        .rx_id = 0x15,
    },
    .parameters = {
        .pos_max = 3.141593f,
        .vel_max = 30,
        .tor_max = 10,
        .kd_int = 0.5f,
        .kp_int = 5.8f,
        .kd_max = 3.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 3.7f,
        .ki = 0.15f,
        .kd = 0.0f,
        .i_max = 0.25f,
        .out_max = 10.0f,
    },
    .angle_pid_config = {
        .kp = 6.8f,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 0.0f,
        .out_max = ARM_MAX_SPEED,
        .angle_max = 0.0f,
    }
};

DmMotorInitConfig_s arm_motor5_config = {
    .topic_name = "arm_motor5",
    .type = J4340,
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
        .DLC = FDCAN_DLC_BYTES_8,
        .can_number = 2,
        .tx_id = 0x06,
        .rx_id = 0x16,
    },
    .parameters = {
        .pos_max = 3.1415927f,
        .vel_max = 10,
        .tor_max = 28,
        .kd_int = 1.3f,
        .kp_int = 65.8f,
        .kd_max = 3.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 0.55f,
        .ki = 0.01f,
        .kd = 0.0f,
        .i_max = 3.0f,
        .out_max = 10.0f,
    },
    .angle_pid_config = {
        .kp = 4.5f,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 3.0f,
        .out_max = ARM_MAX_SPEED,
        .angle_max = 0.0f,
    }
};

DmMotorInitConfig_s arm_motor6_config = {
    .topic_name = "arm_motor6",
    .type = J4310,
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
        .DLC = FDCAN_DLC_BYTES_8,
        .can_number = 2,
        .tx_id = 0x07,
        .rx_id = 0x17,
    },
    .parameters = {
        .pos_max = 3.141593f,
        .vel_max = 45,
        .tor_max = 18,
        .kd_int = 0.35f,
        .kp_int = 12.f,
        .kd_max = 5.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 0.55f,
        .ki = 0.01f,
        .kd = 0.0f,
        .i_max = 3.0f,
        .out_max = 10.0f,
    },
    .angle_pid_config = {
        .kp = 4.5f,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 3.0f,
        .out_max = ARM_MAX_SPEED,
        .angle_max = 0.0f,
    }
};

DmMotorInitConfig_s arm_motor7_config = {
    .topic_name = "arm_motor7",
    .type = J4340,
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
        .DLC = FDCAN_DLC_BYTES_8,
        .can_number = 2,
        .tx_id = 0x08,
        .rx_id = 0x18,
    },
    .parameters = {
        .pos_max = 3.141593f,
        .vel_max = 30,
        .tor_max = 10,
        .kd_int = 2.5f,
        .kp_int = 50.f,
        .kd_max = 5.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 0.55f,
        .ki = 0.01f,
        .kd = 0.0f,
        .i_max = 3.0f,
        .out_max = 10.0f,
    },
    .angle_pid_config = {
        .kp = 4.5f,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 3.0f,
        .out_max = ARM_MAX_SPEED,
        .angle_max = 0.0f,
    }
};
DmMotorInitConfig_s arm_motor8_config = {
    .topic_name = "arm_motor8",
    .type = J4340,
    .control_mode = DM_POSITION,
    .can_config = {
        .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
        .DLC = FDCAN_DLC_BYTES_8,
        .can_number = 2,
        .tx_id = 0x09,
        .rx_id = 0x19,
    },
    .parameters = {
        .pos_max = 3.14159f,
        .vel_max = 30,
        .tor_max = 10,
        .kd_int = 0.3f,
        .kp_int = 8.f,
        .kd_max = 5.0f,
        .kp_max = 500.0f,
    },
    .velocity_pid_config = {
        .kp = 0.55f,
        .ki = 0.01f,
        .kd = 0.0f,
        .i_max = 3.0f,
        .out_max = 10.0f,
    },
    .angle_pid_config = {
        .kp = 4.5f,
        .kd = 0.0f,
        .ki = 0.0f,
        .i_max = 3.0f,
        .out_max = ARM_MAX_SPEED,
        .angle_max = 0.0f,
    }
};

/************************************Task**************************************/
void App_Init(void const * argument){
    while (arm_motors[0] == NULL) arm_motors[0] = Motor_DM_Register(&arm_motor0_config);
    while (arm_motors[1] == NULL) arm_motors[1] = Motor_DM_Register(&arm_motor1_config);
    while (arm_motors[2] == NULL) arm_motors[2] = Motor_DM_Register(&arm_motor2_config);
    while (arm_motors[3] == NULL) arm_motors[3] = Motor_DM_Register(&arm_motor3_config);
    while (arm_motors[4] == NULL) arm_motors[4] = Motor_DM_Register(&arm_motor7_config);
    while (arm_motors[5] == NULL) arm_motors[5] = Motor_DM_Register(&arm_motor5_config);
    while (arm_motors[6] == NULL) arm_motors[6] = Motor_DM_Register(&arm_motor6_config);

    while (arm_motors[7] == NULL) arm_motors[7] = Motor_DM_Register(&arm_motor4_config);
    while (arm_motors[8] == NULL) arm_motors[8] = Motor_DM_Register(&arm_motor8_config);

    //删除初始化任务
    vTaskDelete(NULL);
}