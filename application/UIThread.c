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

//Graph_Data imagex, imagey, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14;
//Graph_Data x15, x16, x17, x18, x19, x20, x21, x22, x23, x24, x25, x26, x27, x28, x29, x30, x31, x32, x33;
//String_Data s1, s2, s3, s4, s5, s6, s7, s8, s9;
//Float_Data f1;
//extern RC_ctrl_t Remote;
//uint8_t temp_flag_5_22=0;
//float a = 0.0;
void UIThread(void const *argument)
{
	while (1)
	{
		
//		if (student_interactive_data_t.user_data[0] != 0x00 && student_interactive_data_t.data_cmd_id == 0x0209)
//		{
//			Char_Draw(&s1, "zim", UI_Graph_ADD, 0x01, UI_Color_Green, 20, 1, 10, 1608, 651, "1");
//		}
//		else{
//			UI_Delete(UI_Graph_Del,1);
//		}

//		if(temp_flag_5_22 == 1)
//		{
//			a = 1.0;
//		}
//		else
//		{
//			a = 0.0;
//		}
//		Float_Draw(&f1,"btv",UI_Graph_ADD,0,UI_Color_Green,20, 1, 10, 1608, 651, a);
////		Float_Draw(&f1,"btv",UI_Graph_Change,0,UI_Color_Purplish_red,30,5,2,920,208,a);

//		UI_ReFresh(1,f1);
		osDelay(100);
	}
}
