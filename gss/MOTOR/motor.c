#include "system.h"
#include "motor_data.h"
#include "user_can.h"
#include "485_address.h"
int Motor_Number = 4;
MOTOR_TPDO mtd[MAX_MOTOR_NUMBER];//< 发送pdo
MOTOR_RPDO mrd[MAX_MOTOR_NUMBER];//< 接收pdo
static uint16_t* pdu_data;
static MOTOR_DATA motor_data[MAX_MOTOR_NUMBER];
uint8_t can_id_map[4] = 
{
	LB_CAN_ID,//< 左前
	LF_CAN_ID,//< 左后
	RB_CAN_ID,//< 右前
	RF_CAN_ID,//< 右后
};
uint16_t can_model_map[4] =
{
	motor1_model,//< 左前
	motor2_model,//< 左后
	motor3_model,//< 右前
	motor4_model,//< 右后
};
typedef union
{
	float fd;
	uint32_t ud;
}FTOU;
/**********************************************************
 * 函数功能： 伺服电机初始化。
 * 参数：     i为电机顺序。
 * 参数：     ID为从机地址。
 * 说明：     无。
 **********************************************************/
void New_Servo_Motor_Init(int i,uint8_t ID)
{
	switch (pdu_data[can_model_map[i]])
	{
	case 1:// 万泽伺服
		WANZER_PDO_Config(ID);
		break;
	default://中菱伺服
		ZLAC8015_PDO_Config(ID);
		break;
	}
	NMT_Control(ID, 0x01, ID); //开启PDO1传输数据
	//delay_ms(1);
}

void MotorDataRefreshInit(uint16_t* pd)
{
	pdu_data = pd;
}

void MotorDataRefresh()
{

	int i = 0;
	uint16_t n = motor1_state_word;
	uint8_t k = 0;
	MOTOR_TPDO* mt;
	MOTOR_RPDO* mr;
	MOTOR_DATA* md;
	pdu_data[R_motor_CAN_map] = can_id_map[0] | can_id_map[1] << 8;
	pdu_data[L_motor_CAN_map] = can_id_map[2] | can_id_map[3] << 8;

	for (i = 0; i < Motor_Number; i++)
	{
		mt = &mtd[k];
		mr = &mrd[i];
		md = &motor_data[k];
		pdu_data[n++] = mt->d.status.sd;	
		pdu_data[n++] = ((uint16_t)md->d.step << 2 | md->d.on&3) | ((uint16_t)mr->d.online << 8);
		pdu_data[n++] = 0;
		pdu_data[n++] = 0;
		pdu_data[n++] = 0;
		pdu_data[n++] = 0;
		pdu_data[n++] = (uint16_t)(mt->d.current_velocity >> 16);
		pdu_data[n++] = (uint16_t)(mt->d.current_velocity);
		pdu_data[n++] = (uint16_t)(mt->d.current_pos >> 16);
		pdu_data[n++] = (uint16_t)(mt->d.current_pos);
		k++;
	}

}

void motor_init_task(void* pvParameters)
{
	TickType_t lastWakeTime = (TickType_t)getSysTickCnt();
	int i;
	MOTOR_RPDO* m_ctrl;
	MOTOR_TPDO* m_states;
	MOTOR_DATA* md;
	for (i = 0; i < Motor_Number; i++)
	{
		motor_data[i].d.step = 0;
		mrd[i].d.online = 0;
		mrd[i].d.mapping = can_id_map[i];
		mtd[i].d.mapping = can_id_map[i];
	}
	pdu_data[motor1_model] = 0;
	pdu_data[R_motor_default_CAN_id] = can_id_map[0]<<8 | can_id_map[1];
	pdu_data[L_motor_default_CAN_id] = can_id_map[2]<<8 | can_id_map[3];
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));//此任务以1000Hz的频率运行
		if (pdu_data[can_reinitialize] == 5)
		{
			pdu_data[can_reinitialize] = 0;
			for (i = 0; i < Motor_Number; i++)
			{
				NMT_Control(can_id_map[i], 0x00, can_id_map[i]); //暂停传输数据
				mrd[i].d.online = 0;
				mtd[i].d.heartbeat = 0;
				motor_data[i].d.step = 0;
			}
		}
		else
		{

			for (i = 0; i < Motor_Number; i++)
			{
				m_ctrl = &mrd[i];
				m_states = &mtd[i];
				md = &motor_data[i];
				switch (md->d.step)
				{
				case 0:
					New_Servo_Motor_Init(i,can_id_map[i]);
					md->d.step = 1;
					break;
				case 1:
					if (m_states->d.heartbeat > 5)
					{
						m_ctrl->d.online = 0;
						m_states->d.heartbeat = 0;
						md->d.step = 0;
					}
					else m_states->d.heartbeat++;
					break;
				default:break;
				}
			}

		}
	}

}
void motor_task(void* pvParameters)
{
	TickType_t lastWakeTime = (TickType_t)getSysTickCnt();
	int i;
	MOTOR_RPDO* m_ctrl;
	MOTOR_TPDO* m_states;
	MOTOR_DATA* md;


	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));//此任务以1000Hz的频率运行
		MotorDataRefresh();
		for (i = 0; i < Motor_Number; i++)
		{
			m_ctrl = &mrd[i];
			m_states = &mtd[i];
			md = &motor_data[i];
			md->d.on = 0;
			//if (m_ctrl->d.online == 0)
			//{
			//	m_ctrl->d.ctrl.cd = 0;
			//	continue;
			//}
			if (m_states->d.status.sd == 592 ||
				m_states->d.status.sd == 0x1040 ||
				m_states->d.status.sd == 0x1060 ||
				m_states->d.status.sd == 0x5650)
			{
				m_ctrl->d.ctrl.cd = 6;
			}
			else if (m_states->d.status.sd == 561 ||
				m_states->d.status.sd == 0x1021 ||
				m_states->d.status.sd == 0x5631)
			{
				m_ctrl->d.ctrl.cd = 7;
			}
			else if (m_states->d.status.sd == 563 ||
				m_states->d.status.sd == 0x3023 ||
				m_states->d.status.sd == 0x5633)
			{
				m_ctrl->d.ctrl.cd = 15;
			}
			else if (m_states->d.status.sd == 567||
				m_states->d.status.sd == 0x5637)
			{
				md->d.on = 1;
			}
			if (g_ucRemote_Flag == 0)
			{
				m_ctrl->d.ctrl.cd = 0;
			}
			else if (pdu_data[error_get_and_clear] == 1)
			{//< 伺服清除报警
				m_ctrl->d.ctrl.cd = 0x80;
			}
		}
		if (pdu_data[error_get_and_clear] == 1)pdu_data[error_get_and_clear] = 0;
	}

}
