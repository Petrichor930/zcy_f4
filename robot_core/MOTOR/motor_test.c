#include "dev_UT.h"
#include "motor_ctrl.h"
#include "motor_def.h"
#include "pid.h"
#include "Remote_Control.h"
// #include "dwt.h"
// #include "filter.h"
// #include "kalman.h"
// #include "pid_leso.h"   ?
#include "drv_conf.h"
#include "motor_headers.h"
//   ?#include "vofa.h"


#include HAL_INCLUDE

extern UART_HandleTypeDef UNITREE_UART_HANDLE;

#if USE_TEST_FILE

#define TEST_FDCAN hfdcan3
#define TEST_ID    4
#define TEST_ID_2  7

//DM
DM_motor_transmit_PDESVDES_msg_t tx_msg = { 0, 1 };

//RM
kalman1_state KF;
leso_para_t LESO;
pid_struct_t pid[2];
float delta_angle;
float yaw_current_rpm;
RM_QUAD_motor_transmit_msg_t rm_tx_msg = { 0 };

//UT
UT_motor_transmit_msg_t ut_tx_msg = { 0 };
float Unitree_last_kp = 0;
float Unitree_kp = 0.3f;
feedforwar_struct_t ff = { 1000, 10, 0 };


void motor_test_init()
{
    motors_t *motors = get_motors_ptr();

    // DM test
    // DM_motor_register(&motors->test, motor_model_DM_DM4310, PDESVDES, COM_FDCAN,
    //                   (uint32_t *)&TEST_FDCAN, TEST_ID, RR_DM4310, &joint5_lpf_val, M_GIMBAL);

    // read_single_DM_motor_data(&TEST_FDCAN, TEST_ID, 0x23, COM_FDCAN);

    // write_single_DM_motor_data(&TEST_FDCAN, 0x04, 0x23, 0x09);
    // storage_single_DM_motor_output(&TEST_FDCAN, 0x04, 0x23); //fdcan 5M


    // dwt_delay_us(30000);

    // read_single_DM_motor_data(&TEST_FDCAN, 0x04, 0x23);
    // enable_single_DM_motor_output(&TEST_FDCAN, TEST_ID, COM_FDCAN);

    //Unitree test
    UT_motor_register(&motors->test, motor_model_UT_M8010_6, &UNITREE_UART_HANDLE, 1,
                      &joint_lpf_val, M_GIMBAL, 0.05, 0.0, 0, 2);

    //RM test
    //  7    0x20B   0x2FF (电压控)COM_CAN
    // RM_QUAD_motor_register(&motors->test, motor_model_RM_QUAD_GM6020, QUAD_CURR, COM_CAN,
    //                        (uint32_t *)&TEST_FDCAN, TEST_ID_2, RR_GM6020, &joint_lpf_val,
    //                        M_GIMBAL);
    // delta_angle = 0;
    // yaw_current_rpm = 0;
    // motors->test.P_des = motors->test.real.rel_angle;
    // memcpy(&pid[ANG_LOOP], &joint7_ang_pid, sizeof(pid_struct_t));
    // memcpy(&pid[RPM_LOOP], &joint7_rpm_pid, sizeof(pid_struct_t));
    // kalman1_init(&KF, 0, 0.1, 60);
    // leso_6020_init(&LESO, 0.2f, 5.0f, 1.5f);
}


void motor_test_loop(rc_ctrl_t *rc, RC_ctrl_t *rc_ctrl)
{
    motors_t *motors = get_motors_ptr();


    rocker->lx += CLAMP((float)rc->rc.ch0 * T_ACC_CNT / 660.0f - rocker->rx, 2.0f);

    // DM test
    // if (rc->rc.switch_right == RC_SW_DOWN || is_rc_offline()) {
    //     motors->test.AUX.EN = 0;
    //     disable_single_DM_motor_output(&TEST_FDCAN, TEST_ID, COM_FDCAN);
    //     return;
    // } else if (motors->test.AUX.EN == 0) {
    //     enable_single_DM_motor_output(&TEST_FDCAN, TEST_ID, motors->test.com_type);
    //     motors->test.AUX.EN = 1;
    // }
    // if (rc->rc.switch_left == RC_SW_UP || rc->rc.switch_left == RC_SW_DOWN) {
    //     motors->test.P_des += rocker->rx / 3140;
    //     tx_msg.expt_scale = motors->test.P_des;
    //     tx_msg.expt_vel = 2;
    //     _set_DM_motor_output_PDESVDES(&TEST_FDCAN, &tx_msg, TEST_ID, motors->test.com_type);
    // }

    // Unitree test
    if (rc->rc.switch_right == RC_SW_DOWN || is_rc_offline()) {
        disable_UT_motor_output(&motors->test);
        motors->test.AUX.Kp = 0;
        ff.last_in = motors->test.real.abs_angle;
        return;
    } else if (motors->test.AUX.EN == 0) {
        enable_UT_motor_output(&motors->test);
        motors->test.V_des = 0;
        motors->test.T_ff = 0;
    }
    if (rc->rc.switch_left == RC_SW_MID) {
        // static bool state = 0;
        // if (state == 0) {
        //     motors->test.P_des = 1.2;
        //     set_single_UT_motor_output(&motors->test, &ut_tx_msg);
        //     if (fabs(motors->test.real.abs_angle - 3.14) < 0.1) {
        //         set_UT_zero_angle(&motors->test);
        //         state = 1;
        //     }
        //
        // } else {

        /*缓启动*/
        motors->test.AUX.Kp = s_curve_acc(Unitree_last_kp, Unitree_kp, 100, 1);
        Unitree_last_kp = motors->test.AUX.Kp;

        rocker->lx += CLAMP((float)rc->rc.ch2 * T_ACC_CNT / 660.0f - rocker->lx, 2);
        motors->test.P_des += rocker->lx / 314 / 100;
        motors->test.T_ff = forward_feed(motors->test.real.abs_angle, &ff);

        convert_UT_tx_data(&motors->test, &ut_tx_msg);
        // set_single_UT_motor_output(&motors->test, &ut_tx_msg);
        // }
    }


    //RM test
    // if (rc->rc.switch_right == RC_SW_DOWN || is_rc_offline()) {
    //     pid_reset(&pid[0]);
    //     pid_reset(&pid[1]);
    //     shut_RM_QUAD_motor_output(&TEST_FDCAN, M6020_ID_EXTEND, motors->test.com_type);
    //     return;
    // }
    // if (rc->rc.switch_left == RC_SW_MID || rc->rc.switch_left == RC_SW_DOWN) {
    //     motors->test.P_des += rocker->rx / 8192 * 2 * PI / 2;

    //     float yaw_reverse_EMF = motors->test.real.rpm / Ke_GM6020;

    //     delta_angle = motors->test.P_des - motors->test.real.abs_angle;

    //     yaw_current_rpm = iir_filter_2(motors->test.real.rpm, 0);

    //     kalman1_filter(&KF, motors->test.real.current + yaw_reverse_EMF);

    //     motors->test.T_ff = pid_leso_dualloop(pid, &LESO, delta_angle, yaw_current_rpm, KF.x);
    //     motors->test.T_ff = CLAMP(motors->test.T_ff, 10); //GM6020_VOLT_MAX

    //     Vofa_FireWater("e_ang,r_ang,T: %f,%f,%f\n", motors->test.P_des, motors->test.real.abs_angle,
    //                    motors->test.T_ff);

    //     rm_tx_msg.D[2] = (int)(motors->test.T_ff / GM6020_VOLT_MAX * GM6020_VOLT_DATA_MAX);
    //     set_OnePack_RM_motor_output(&TEST_FDCAN, &rm_tx_msg, M6020_ID_EXTEND,
    //                                 motors->test.com_type);
    // }
}
#endif
