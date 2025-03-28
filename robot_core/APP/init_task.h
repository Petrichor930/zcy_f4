#ifndef INIT_TASK_H
#define INIT_TASK_H

#include "drv_can.h"
#include "main.h"
#include "chassis.h"

extern SemaphoreHandle_t xChassisSemaphore;

void robot_init(void);

#endif // INIT_TASK_H
