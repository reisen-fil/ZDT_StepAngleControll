#ifndef __ZDT_X42_V2_H
#define __ZDT_X42_V2_H

#include "usart.h"
#include "main.h"
#include "fifo.h"


/**********************************************************
*** ZDT_X42_V2.0�����ջ���������
*** ��д���ߣ�ZHANGDATOU
*** ����֧�֣��Ŵ�ͷ�ջ��ŷ�
*** �Ա����̣�https://zhangdatou.taobao.com
*** CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
*** qq����Ⱥ��262438510
**********************************************************/

#define		ABS(x)		((x) > 0 ? (x) : -(x)) 

typedef enum {
  S_VER   = 0,      /* ��ȡ�̼��汾�Ͷ�Ӧ��Ӳ���汾 */
  S_RL    = 1,      /* ��ȡ��ȡ���������� */
  S_PID   = 2,      /* ��ȡPID���� */
  S_ORG   = 3,      /* ��ȡ������� */
  S_VBUS  = 4,      /* ��ȡ���ߵ�ѹ */
  S_CBUS  = 5,      /* ��ȡ���ߵ��� */
  S_CPHA  = 6,      /* ��ȡ����� */
  S_ENC   = 7,      /* ��ȡ������ԭʼֵ */
  S_CPUL  = 8,      /* ��ȡʵʱ������������ʵʱλ�ü���õ����������� */
  S_ENCL  = 9,      /* ��ȡ�������Ի�У׼��ı�����ֵ */
  S_TPUL  = 10,     /* ��ȡ���������� */
  S_TPOS  = 11,     /* ��ȡ���Ŀ��λ�� */
  S_OPOS  = 12,     /* ��ȡ���ʵʱ�趨��Ŀ��λ�ã�����ģʽ��ʵʱλ�ã� */
  S_VEL   = 13,     /* ��ȡ���ʵʱת�� */
  S_CPOS  = 14,     /* ��ȡ���ʵʱλ�ã����ڽǶȱ������ۼӵĵ��ʵʱλ�ã� */
  S_PERR  = 15,     /* ��ȡ���λ����� */
  S_TEMP  = 16,     /* ��ȡ���ʵʱ�¶� */
  S_SFLAG = 17,     /* ��ȡ״̬��־λ */
  S_OFLAG = 18,     /* ��ȡ����״̬��־λ */
  S_Conf  = 19,     /* ��ȡ�������� */
  S_State = 20,     /* ��ȡϵͳ״̬���� */
}SysParams_t;

extern __IO bool rxFrameFlag;
extern __IO uint8_t u_rxCmd[FIFO_SIZE];
extern __IO uint8_t u_rxCount;

void usart_SendCmd(uint8_t *cmd, uint16_t len);
void usart_SendByte(uint8_t data);

void ZDT_X42_V2_Reset_CurPos_To_Zero(uint8_t addr); // ����ǰλ������
void ZDT_X42_V2_Reset_Clog_Pro(uint8_t addr); // �����ת����
void ZDT_X42_V2_Read_Sys_Params(uint8_t addr, SysParams_t s); // ��ȡ����
void ZDT_X42_V2_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode); // ���������л�����/�ջ�����ģʽ
void ZDT_X42_V2_En_Control(uint8_t addr, bool state, uint8_t snF); // ���ʹ�ܿ���
void ZDT_X42_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF); // ����ģʽ����
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF); // �ٶ�ģʽ����
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF); // ֱͨ����λ��ģʽ����
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF); // �������߼Ӽ���λ��ģʽ����
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF); // �õ������ֹͣ�˶�
void ZDT_X42_V2_Synchronous_motion(uint8_t addr); // �������ͬ����ʼ�˶�
void ZDT_X42_V2_Origin_Set_O(uint8_t addr, bool svF); // ���õ�Ȧ��������λ��
void ZDT_X42_V2_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // �޸Ļ������
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF); // �������������
void ZDT_X42_V2_Origin_Interrupt(uint8_t addr); // ǿ���жϲ��˳�����
void ZDT_X42_V2_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount); // �������ݽ��պ���


/* My Pack */ 
void ZDT_X42_Controll_StepAngle(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF);
void ZDT_X42_Origin_Set_O(uint8_t addr);
void ZDT_X42_triggle_to_zero(uint8_t addr);
void ZDT_X42_V2_Set_TurnToZero_Params(uint8_t addr,uint8_t o_dir,uint16_t o_vel,uint32_t o_tm,uint16_t sl_vel,uint16_t sl_ma,uint16_t sl_ms);


#endif
