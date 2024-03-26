/*
* ����������FIFO���CAN����FIFO������������
* ʵ�ʲ��Բ��������������ϵ�CAN����Ȼ����Ļ�������FIFO
*/
#include "RefereeBehaviour.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "CanPacket.h"


Queue_t SendBuffer[2];

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;


ext_game_robot_status_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_musk_t buff_musk_t;
aerial_robot_energy_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_rfid_status_t rfid_status_t;
ext_student_interactive_data_t student_interactive_data_t;

ext_dart_client_cmd_t dart_client_cmd_t;
ext_dart_remaining_time_t dart_remaining_time_t;

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    //memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));


    memset(&robot_state, 0, sizeof(ext_game_robot_status_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));
		memset(&rfid_status_t, 0, sizeof(ext_rfid_status_t));


    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
						EnQueue(&SendBuffer[0], send_game_status(),11);
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
						EnQueue(&SendBuffer[1], send_enemy_information(),11);
        }
        break;
        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
        break;
				case DART_FIRE_COUNTDOWN_CMD_ID:
        {
            memcpy(&dart_remaining_time_t, frame + index, sizeof(ext_dart_remaining_time_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
						EnQueue(&SendBuffer[1], send_bullet_limit(),11);
						EnQueue(&SendBuffer[1], send_robot_information(),11);
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
						EnQueue(&SendBuffer[0], send_power_heat_data(),22);
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
						EnQueue(&SendBuffer[0], send_bullet_speed(),11);
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
				case ROBOT_RFID_STATUS_ID:
				{
						memcpy(&rfid_status_t, frame + index, sizeof(ext_rfid_status_t));
				}
				case DART_CLIENT_CMD_ID:
        {
            memcpy(&dart_client_cmd_t, frame + index, sizeof(ext_dart_client_cmd_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
        }
        break;
        default:
        {
            break;
        }
    }
}
/*-----------------------------------------����FIFO---------------------------------------*/
void QueueInit(Queue_t *q)
{
    q->front = 0;
    q->rear = -1;
    q->counter = 0;
} 


uint8_t IsFull(Queue_t *q)
{
	if(q->counter >= SIZE)
    return 1;
	return 0;
}

void EnQueue(Queue_t *q, uint8_t *val,uint8_t lenth)
{
	uint8_t i = 0;
    if(IsFull(q))
        return;
		if(q->counter > (SIZE - 11))
			return;
	
	for(;i<lenth;i++)
	{
		q->rear = (q->rear + 1) % SIZE;
		q->arr[q->rear] = *(val+i);
		q->counter++;
	}
}


uint8_t IsEmpty(Queue_t *q)
{
	if(q->counter <= 0)
    return 1;
	return 0;
}

//0����� -1�������
int Pop(Queue_t *buffer1,Queue_t *buffer2,uint8_t data[11])
{
	int i = 0;
	if(IsEmpty(buffer1))
	{
		if(IsEmpty(buffer2))
			return 0;
		else if(buffer2->counter %11 == 0)
		{
			for(;i<11;i++)
			{
				data[i] = buffer2->arr[buffer2->front];
				buffer2->front = (buffer2->front + 1) % SIZE;
				buffer2->counter--;
			}
		}
		else
		{
			return -1;
		}
			
	}
	else if(buffer1->counter %11 == 0)
	{
		for(;i<11;i++)
			{
				data[i] = buffer1->arr[buffer1->front];
				buffer1->front = (buffer1->front + 1) % SIZE;
				buffer1->counter--;
			}
	}
	else
	{
		return -1;
	}
	return 1;
}


/*
* ��������Ҫ�����������λ��������������
*/


uint8_t *send_power_heat_data(void);
uint8_t *send_bullet_speed(void);
uint8_t *send_bullet_limit(void);
uint8_t *send_power_limit(void);
uint8_t *send_robot_information(void);
uint8_t *send_enemy_information(void);


//���̹��ʺ�ǹ������
//��ʵʱ�ԣ����ռ���
uint8_t PowerHeatData[22];
uint8_t *send_power_heat_data(void)
{
	memset(PowerHeatData,0,sizeof(PowerHeatData));
	for(int i = 0;i<2;i++)
	{
		PowerHeatData[0+i*11] = (uint8_t)((0x120+i)>>8);
		PowerHeatData[1+i*11] = (uint8_t)(0x120+i);
		PowerHeatData[2+i*11] = 8;
	}
	memcpy(PowerHeatData+3,&power_heat_data_t.chassis_voltage,8);
	memcpy(PowerHeatData+14,&power_heat_data_t.buffer_energy,8);
	return PowerHeatData;
}
//ǹ������
uint8_t BulletSpeed[11];
uint8_t *send_bullet_speed(void)
{
	memset(BulletSpeed,0,sizeof(BulletSpeed));
	switch (shoot_data_t.shooter_number)
	{
		case 1://������Сǹ��0
		{
			BulletSpeed[0] = (uint8_t)(0x122>>8);
			BulletSpeed[1] = (uint8_t)(0x122);
			BulletSpeed[2] = 4;
			memcpy(BulletSpeed+3,&shoot_data_t.initial_speed,4);
			break;
		}
		case 2://������Сǹ��1
		{
			BulletSpeed[0] = (uint8_t)(0x123>>8);
			BulletSpeed[1] = (uint8_t)(0x123);
			BulletSpeed[2] = 4;
			memcpy(BulletSpeed+3,&shoot_data_t.initial_speed,4);
			break;
		}
		case 3://�����˴�ǹ��
		{
			BulletSpeed[0] = (uint8_t)(0x124>>8);
			BulletSpeed[1] = (uint8_t)(0x124);
			BulletSpeed[2] = 4;
			memcpy(BulletSpeed+3,&shoot_data_t.initial_speed,4);
			break;
		}
	}
	return BulletSpeed;
}

//�����������
uint8_t BulletLimit[11];
uint8_t *send_bullet_limit(void)
{
	memset(BulletLimit,0,sizeof(BulletLimit));
		BulletLimit[0] = (uint8_t)((0x125)>>8);
		BulletLimit[1] = (uint8_t)(0x125);
		BulletLimit[2] = 4;
	memcpy((BulletLimit+3),&robot_state.shooter_barrel_cooling_value,4);
	return BulletLimit;
}
//���̹�������
uint8_t PowLimit[11];
uint8_t *send_power_limit(void)
{
	memset(PowLimit,0,sizeof(PowLimit));
	PowLimit[0] = (uint8_t)(0x128>>8);
	PowLimit[1] = (uint8_t)(0x128);
	PowLimit[2] = 2;
	memcpy(PowLimit+3,&robot_state.chassis_power_limit,2);
	return PowLimit;
}
//������������Ϣ
robot_information_t robot_information;
uint8_t RobotInformation[11];
uint8_t *send_robot_information(void)
{
	robot_information.robot_id = robot_state.robot_id;
	robot_information.power_output = (uint8_t)(((0x01 & robot_state.power_management_shooter_output) << 2) |
											((0x01 & robot_state.power_management_chassis_output) << 2) |
											((0x01 & robot_state.power_management_gimbal_output) << 2));
	robot_information.remain_HP = robot_state.current_HP;
	robot_information.max_HP = robot_state.maximum_HP;
	memset(RobotInformation,0,sizeof(RobotInformation));
	RobotInformation[0] = (uint8_t)(0x129>>8);
	RobotInformation[1] = (uint8_t)(0x129);
	RobotInformation[2] = 6;
	memcpy(RobotInformation+3,&robot_information,6);
	return RobotInformation;
}

//���͵з�Ѫ��
enemy_information_t enemy_information;
uint8_t EnemyInformation[11];
uint8_t *send_enemy_information(void)
{
	if(robot_state.robot_id & 0x100)//����
	{
		enemy_information.hero_remain_HP = game_robot_HP_t.red_1_robot_HP;
		enemy_information.infantry3_remain_HP = game_robot_HP_t.red_3_robot_HP;
		enemy_information.infantry4_remain_HP = game_robot_HP_t.red_4_robot_HP;
		enemy_information.infantry5_remain_HP = game_robot_HP_t.red_5_robot_HP;
	}
	else
	{
		enemy_information.hero_remain_HP = game_robot_HP_t.red_1_robot_HP;
		enemy_information.infantry3_remain_HP = game_robot_HP_t.blue_3_robot_HP;
		enemy_information.infantry4_remain_HP = game_robot_HP_t.blue_4_robot_HP;
		enemy_information.infantry5_remain_HP = game_robot_HP_t.blue_5_robot_HP;
	}
	memset(EnemyInformation,0,sizeof(EnemyInformation));
	EnemyInformation[0] = (uint8_t)(0x12A >> 8);
	EnemyInformation[1] = (uint8_t)(0x12A);
	EnemyInformation[2] = 8;
	memcpy((EnemyInformation+3),&enemy_information,8);
	return EnemyInformation;
}

//���ͱ���״̬��Ϣ
uint8_t GAMESTATUS[11];
send_game_status_t SendGameStatus;
uint8_t *send_game_status(void)
{
	SendGameStatus.game_status = game_state.game_type<<4;
	SendGameStatus.game_status |= game_state.game_progress;
	SendGameStatus.end_time = game_state.stage_remain_time;
	
	memset(GAMESTATUS,0,sizeof(GAMESTATUS));
	GAMESTATUS[0] = (uint8_t)(0x12B >> 8);
	GAMESTATUS[1] = (uint8_t)(0x12B);
	GAMESTATUS[2] = 3;
	memcpy((GAMESTATUS+3),&SendGameStatus,3);
	return GAMESTATUS;
}
