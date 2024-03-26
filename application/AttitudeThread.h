#ifndef __AttitudeThread_H
#define __AttitudeThread_H
#include "struct_typedef.h"
#include "stdint.h"

/**
 * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void AttitudeThread(void const *pvParameters);

void GetCurrentQuaternion(float q[4]);
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU);

#endif
