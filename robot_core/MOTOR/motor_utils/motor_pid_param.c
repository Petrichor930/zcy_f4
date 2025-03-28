
#include "motor_headers.h"
#include "drv_conf.h"
#include "motor_pid_param.h"
#include "pid.h"

pid_struct_t wheel1_pid = { .kp = 34.f,
                            .ki = 0.04f,
                            .kd = 0.f,
                            .i_max = 200,
                            .out_max = C620_CURR_DATA_MAX };

pid_struct_t wheel2_pid = { .kp = 10.f,
                            .ki = 0.05f,
                            .kd = 0.f,
                            .i_max = 200,
                            .out_max = C620_CURR_DATA_MAX };

pid_struct_t wheel3_pid = { .kp = 7.0f,
                            .ki = 0.04f,
                            .kd = 0.f,
                            .i_max = 200,
                            .out_max = C620_CURR_DATA_MAX };

pid_struct_t wheel4_pid = { .kp = 10.5f,
                            .ki = 0.03f,
                            .kd = 0.f,
                            .i_max = 200,
                            .out_max = C620_CURR_DATA_MAX };


pid_struct_t chassis_vx_pid = { .kp = 35.0f,
                                .ki = 0.15f,
                                .kd = 0.8f,
                                .i_max = C620_CURR_DATA_MAX / 50,
                                .out_max = C620_CURR_DATA_MAX,
                                .k_deadband = 0.005f };

pid_struct_t chassis_vy_pid = { .kp = 35.0f,
                                .ki = 0.15f,
                                .kd = 0.8f,
                                .i_max = C620_CURR_DATA_MAX / 50,
                                .out_max = C620_CURR_DATA_MAX,
                                .k_deadband = 0.005f };

pid_struct_t chassis_yaw_pid = { .kp = 30.0f,
                                 .ki = 0.15f,
                                 .kd = 0.8f,
                                 .i_max = C620_CURR_DATA_MAX / 50,
                                 .out_max = C620_CURR_DATA_MAX,
                                 .k_deadband = 0.005f };

pid_struct_t chassis_follow_pid = { .kp = 0.4f,
                                    .ki = 0.004f,
                                    .kd = 0.8f,
                                    .i_max = 2,
                                    .out_max = 6,
                                    .k_deadband = 0.5f,
                                    .deadband_zero_output = 1

};

pid_struct_t joint1_pid = { .kp = 0.02f,
                            .ki = 0.0001f,
                            .kd = 0.f,
                            .i_max = 0.1,
                            .out_max = 0.6,
                            .k_deadband = 0.005f };

pid_struct_t chassis_wz_pid = {
    .kp = 10.0f,
    .ki = 0.02f,
    .kd = 0.0f,
    .i_max = C620_CURR_DATA_MAX / 80,
    .out_max = C620_CURR_DATA_MAX,
    .k_deadband = 0.001f,
};


// const pid_struct_t joint7_ang_pid = {
//     .kp = 150.f,
//     .ki = 0.0f,
//     .kd = 100.f,
//     .i_max = 0,
//     .out_max = 200.f,
//     .k_deadband = 0.01f,
// };

// const pid_struct_t joint7_rpm_pid = {
//     .kp = 0.05f,
//     .ki = 0.015f,
//     .kd = 0.0f,
//     .i_max = 4,
//     .out_max = GM6020_VOLT_MAX,
//     .k_deadband = 0.1f,
// };

const pid_struct_t joint7_ang_pid = {
    .kp = 100.f,
    .ki = 0.0f,
    .kd = 100.f,
    .i_max = 0,
    .out_max = 200.f,
    .k_deadband = 0.01f,
};

const pid_struct_t joint7_rpm_pid = {
    .kp = 0.06f,
    .ki = 0.005f,
    .kd = 0.0f,
    .i_max = 2,
    .out_max = GM6020_VOLT_MAX,
    .k_deadband = 0.1f,
};
