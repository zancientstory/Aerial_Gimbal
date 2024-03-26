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

Graph_Data imagex, imagey, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14;
Graph_Data x15, x16, x17, x18, x19, x20, x21, x22, x23, x24, x25, x26, x27, x28, x29, x30, x31, x32, x33;
String_Data s1, s2, s3, s4, s5, s6, s7, s8, s9;
int flag = 1;
uint8_t data = 0;
extern RC_ctrl_t Remote;

void UIThread(void const *argument)
{
	while (1)
	{

		osDelay(1);
	}
}
