#ifndef MOTOR_TEST
#define MOTOR_TEST

#include "rc.h"
#ifdef __cplusplus
extern "C" {
#endif

void motor_test_init();
void motor_test_loop(rc_ctrl_t* rc,rocker_t *rocker);

#ifdef __cplusplus
}
#endif




#endif