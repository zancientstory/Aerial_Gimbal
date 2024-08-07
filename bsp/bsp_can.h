#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "stdint.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void);

void CanSendMessage(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t dlc, uint8_t *message);

#endif