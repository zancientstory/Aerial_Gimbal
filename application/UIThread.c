#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "RefereeBehaviour.h"
#include "Client_UI.h"
#include "CanPacket.h"
#include "CalculateThread.h"
#include "UIThread.h"

// 固定ui数据
String_Data Vulnerability_text_fixed;
String_Data fly_text_fixed;
String_Data dog_text_fixed;
// 改变的ui数据
Float_Data Vulnerability_data_change;
Float_Data fly_data_change;
Float_Data dog_data_change;
// ui数据改变的标志位
uint8_t Vulnerability_flag = 0;
uint8_t fly_flag = 0;
uint8_t dog_flag = 0;
// UI添加数据的标志位
uint8_t ui_add_flag = 0;

void UIThread(void const *argument)
{
	osDelay(4000);

	// 初始化固定ui数据
	Char_Draw(&Vulnerability_text_fixed, "VUL", UI_Graph_ADD, 0, UI_Color_Main, 18, 4, 2, 1521, 854, "VUL\n");
	Char_Draw(&fly_text_fixed, "FLY", UI_Graph_ADD, 0, UI_Color_Main, 18, 4, 2, 1521, 776, "FLY\n");
	Char_Draw(&dog_text_fixed, "DOG", UI_Graph_ADD, 0, UI_Color_Main, 18, 4, 2, 1521, 698, "DOG\n");

	Char_ReFresh(Vulnerability_text_fixed);
	Char_ReFresh(fly_text_fixed);
	Char_ReFresh(dog_text_fixed);

	Float_Draw(&Vulnerability_data_change, "vul", UI_Graph_ADD, 0, UI_Color_Main, 30, 1, 2, 1668, 854, 0);
	Float_Draw(&fly_data_change, "fly", UI_Graph_ADD, 0, UI_Color_Main, 30, 1, 2, 1668, 776, 0);
	Float_Draw(&dog_data_change, "dog", UI_Graph_ADD, 0, UI_Color_Main, 30, 1, 2, 1668, 698, 0);

	UI_ReFresh(1, Vulnerability_data_change);
	UI_ReFresh(2, fly_data_change, dog_data_change);
	while (1)
	{
		if ((Gimbal.StateMachine == GM_TEST || Gimbal.StateMachine == GM_MATCH) && (ui_add_flag == 0))
		{
			Char_Draw(&Vulnerability_text_fixed, "VUL", UI_Graph_ADD, 0, UI_Color_Main, 18, 4, 2, 1521, 854, "VUL\n");
			Char_Draw(&fly_text_fixed, "FLY", UI_Graph_ADD, 0, UI_Color_Main, 18, 4, 2, 1521, 776, "FLY\n");
			Char_Draw(&dog_text_fixed, "DOG", UI_Graph_ADD, 0, UI_Color_Main, 18, 4, 2, 1521, 698, "DOG\n");

			Char_ReFresh(Vulnerability_text_fixed);
			Char_ReFresh(fly_text_fixed);
			Char_ReFresh(dog_text_fixed);

			Float_Draw(&Vulnerability_data_change, "vul", UI_Graph_ADD, 0, UI_Color_Main, 30, 1, 2, 1668, 854, 0);
			Float_Draw(&fly_data_change, "fly", UI_Graph_ADD, 0, UI_Color_Main, 30, 1, 2, 1668, 776, 0);
			Float_Draw(&dog_data_change, "dog", UI_Graph_ADD, 0, UI_Color_Main, 30, 1, 2, 1668, 698, 0);

			UI_ReFresh(1, Vulnerability_data_change);
			UI_ReFresh(2, fly_data_change, dog_data_change);

			ui_add_flag = 1;
		}
		else if (Gimbal.StateMachine == GM_NO_FORCE)
		{
			ui_add_flag = 0;
		}
		// 根据标志位判断是否刷新数据
		if (Vulnerability_flag == 1)
		{
			Float_Draw(&Vulnerability_data_change, "vul", UI_Graph_Change, 0, UI_Color_Main, 30, 1, 2, 1668, 854, 1000.0f);
		}
		else
		{
			Float_Draw(&Vulnerability_data_change, "vul", UI_Graph_Change, 0, UI_Color_Main, 30, 1, 2, 1668, 854, 0.0f);
		}
		if (fly_flag == 1)
		{
			Float_Draw(&fly_data_change, "fly", UI_Graph_Change, 0, UI_Color_Main, 30, 1, 2, 1668, 776, 1000.0f);
		}
		else
		{
			Float_Draw(&fly_data_change, "fly", UI_Graph_Change, 0, UI_Color_Main, 30, 1, 2, 1668, 776, 0.0f);
		}
		if (dog_flag == 1)
		{
			Float_Draw(&dog_data_change, "dog", UI_Graph_Change, 0, UI_Color_Main, 30, 1, 2, 1668, 698, 1000.0f);
		}
		else
		{
			Float_Draw(&dog_data_change, "dog", UI_Graph_Change, 0, UI_Color_Main, 30, 1, 2, 1668, 698, 0.0f);
		}
		// 刷新数据
		UI_ReFresh(1, Vulnerability_data_change);
		UI_ReFresh(2, fly_data_change, dog_data_change);

		osDelay(1);
	}
}
