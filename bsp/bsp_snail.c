#include "bsp_snail.h"
#include "tim.h"

uint16_t pulse = 1730;

void Init_Snail(void)
{
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
  HAL_Delay(3000);
}
void Enable_Snail(void)
{
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pulse);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pulse);
}
void Disable_Snail(void)
{
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
}
