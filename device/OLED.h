#ifndef OLED_H
#define OLED_H
#include <stdint.h>
#include "InterruptService.h" //存放结构体OfflineMonitor_t的头文件
/**
 * @brief          initialize the oled device
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          初始化OLED模块，
 * @param[in]      none
 * @retval         none
 */
void OLED_init(void);
/**
 * @brief display offline device through OLED(IIC)
 */
/**
 * @brief 通过OLED(IIC)显示离线设备
 */
void Display_Error(const OfflineMonitor_t *offline_data);

#endif
