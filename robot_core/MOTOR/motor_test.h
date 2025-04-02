#ifndef MOTOR_TEST
#define MOTOR_TEST

#include "Remote_Control.h"
#ifdef __cplusplus
extern "C" {
#endif

void motor_test_init();
void motor_test_loop(rc_info_t,RC_ctrl_t *rc_ctrl);

#ifdef __cplusplus
}
#endif




#endif