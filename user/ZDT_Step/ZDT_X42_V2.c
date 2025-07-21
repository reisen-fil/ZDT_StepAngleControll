#include "ZDT_X42_V2.h"

/**********************************************************
*** ZDT_X42_V2.0�����ջ���������
*** ��д���ߣ�ZHANGDATOU
*** ����֧�֣��Ŵ�ͷ�ջ��ŷ�
*** �Ա����̣�https://zhangdatou.taobao.com
*** CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
*** qq����Ⱥ��262438510
**********************************************************/

__IO bool rxFrameFlag = false;
__IO uint8_t u_rxCmd[FIFO_SIZE] = {0};
__IO uint8_t u_rxCount = 0;

/**
	* @brief   USART1�жϺ���
	* @param   ��
	* @retval  ��
	*/

/**********************************************************
***	���ڽ����ж�
**********************************************************/

//Receive Byte

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 1. ���յ����ֽڶ��� FIFO���ȼ��� fifo_enQueue�� */
        fifo_enQueue(rx_one_byte);

        /* 2. ����������һ�� 1 �ֽ��жϽ��� */
        HAL_UART_Receive_IT(&huart1, &rx_one_byte, sizeof(rx_one_byte));
    }
}

/**********************************************************
***	���ڿ����ж�
**********************************************************/

//Read Variable-length Data

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        /* Size ������һ֡�ĳ��� */
				u_rxCount = Size;
				// ��ȡһ֡��������
				u_rxCount = fifo_queueLength(); for(uint8_t i=0; i < u_rxCount; i++) { u_rxCmd[i] = fifo_deQueue(); }
				// һ֡���ݽ�����ɣ���λ֡��־λ
				rxFrameFlag = true;			

        /* ����������һ�Ρ�����ֱ�����С� */
				HAL_UARTEx_ReceiveToIdle_IT(&huart1, u_rxBuf, RX_BUF_LEN);
    }
}

/**
	* @brief   USART���Ͷ���ֽ�
	* @param   ��
	* @retval  ��
	*/
void usart_SendCmd(uint8_t *cmd, uint16_t len)
{
    HAL_UART_Transmit(&huart1, cmd, len, HAL_MAX_DELAY);
}

/**
  * @brief    ����ǰλ������
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x0A;                       // ������
  cmd[2] =  0x6D;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    �����ת����
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x0E;                       // ������
  cmd[2] =  0x52;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    ��ȡϵͳ����
  * @param    addr  �������ַ
  * @param    s     ��ϵͳ��������
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ

  switch(s)                             // ������
  {
    case S_VER   : cmd[1] = 0x1F; break;                  /* ��ȡ�̼��汾�Ͷ�Ӧ��Ӳ���汾 */
    case S_RL    : cmd[1] = 0x20; break;                  /* ��ȡ��ȡ���������� */
    case S_PID   : cmd[1] = 0x21; break;                  /* ��ȡPID���� */
    case S_ORG   : cmd[1] = 0x22; break;                  /* ��ȡ������� */
    case S_VBUS  : cmd[1] = 0x24; break;                  /* ��ȡ���ߵ�ѹ */
    case S_CBUS  : cmd[1] = 0x26; break;                  /* ��ȡ���ߵ��� */
    case S_CPHA  : cmd[1] = 0x27; break;                  /* ��ȡ����� */
    case S_ENC   : cmd[1] = 0x29; break;                  /* ��ȡ������ԭʼֵ */
    case S_CPUL  : cmd[1] = 0x30; break;                  /* ��ȡʵʱ������������ʵʱλ�ü���õ����������� */
    case S_ENCL  : cmd[1] = 0x31; break;                  /* ��ȡ�������Ի�У׼��ı�����ֵ */
    case S_TPUL  : cmd[1] = 0x32; break;                  /* ��ȡ���������� */
    case S_TPOS  : cmd[1] = 0x33; break;                  /* ��ȡ���Ŀ��λ�� */
    case S_OPOS  : cmd[1] = 0x34; break;                  /* ��ȡ���ʵʱ�趨��Ŀ��λ�ã�����ģʽ��ʵʱλ�ã� */
    case S_VEL   : cmd[1] = 0x35; break;                  /* ��ȡ���ʵʱת�� */
    case S_CPOS  : cmd[1] = 0x36; break;                  /* ��ȡ���ʵʱλ�ã����ڽǶȱ������ۼӵĵ��ʵʱλ�ã� */
    case S_PERR  : cmd[1] = 0x37; break;                  /* ��ȡ���λ����� */
    case S_TEMP  : cmd[1] = 0x39; break;                  /* ��ȡ���ʵʱ�¶� */
    case S_SFLAG : cmd[1] = 0x3A; break;                  /* ��ȡ״̬��־λ */
    case S_OFLAG : cmd[1] = 0x3B; break;                  /* ��ȡ����״̬��־λ */
    case S_Conf  : cmd[1] = 0x42; cmd[2] = 0x6C; break;   /* ��ȡ�������� */
    case S_State : cmd[1] = 0x43; cmd[2] = 0x7A; break;   /* ��ȡϵͳ״̬���� */
    default: break;
  }

  // ��������
  if(s >= S_Conf)
  {
    cmd[3] = 0x6B; usart_SendCmd(cmd, 4);
  }
  else
  {
    cmd[2] = 0x6B; usart_SendCmd(cmd, 3);
  }
}

/**
  * @brief    �޸Ŀ���/�ջ�����ģʽ
  * @param    addr     �������ַ
  * @param    svF      ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  * @param    ctrl_mode������ģʽ����Ӧ��Ļ�ϵ�P_Pul�˵�����0�ǹر������������ţ�1�ǿ���ģʽ��2�Ǳջ�ģʽ��3����En�˿ڸ���Ϊ��Ȧ��λ�����������ţ�Dir�˿ڸ���Ϊ��λ����ߵ�ƽ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x46;                       // ������
  cmd[2] =  0x69;                       // ������
  cmd[3] =  svF;                        // �Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  cmd[4] =  ctrl_mode;                  // ����ģʽ����Ӧ��Ļ�ϵ�Ctrl_Mode�˵�����0�ǿ���ģʽ��1��FOCʸ���ջ�ģʽ
  cmd[5] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 6);
}

/**
  * @brief    ʹ���źſ���
  * @param    addr  �������ַ
  * @param    state ��ʹ��״̬     ��trueΪʹ�ܵ����falseΪ�رյ��
  * @param    snF   �����ͬ����־ ��0Ϊ�����ã�����ֵ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_En_Control(uint8_t addr, bool state, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xF3;                       // ������
  cmd[2] =  0xAB;                       // ������
  cmd[3] =  (uint8_t)state;             // ʹ��״̬
  cmd[4] =  snF;                        // ���ͬ���˶���־
  cmd[5] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 6);
}

/**
  * @brief    ����ģʽ
  * @param    addr  �������ַ
  * @param    sign  ������         ��0Ϊ��������ֵΪ��
  * @param    t_ramp��б��(Ma/s)   ����Χ0 - 65535Ma/s
  * @param    torque������(Ma)     ����Χ0 - 4000Ma
  * @param    snF   �����ͬ����־ ��0Ϊ�����ã�����ֵ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xF5;                       // ������
  cmd[2] =  sign;                       // ���ţ�����
  cmd[3] =  (uint8_t)(t_ramp >> 8);     // ����б��(Ma/s)��8λ�ֽ�
  cmd[4] =  (uint8_t)(t_ramp >> 0);     // ����б��(Ma/s)��8λ�ֽ�
  cmd[5] =  (uint8_t)(torque >> 8);     // ����(Ma)��8λ�ֽ�
  cmd[6] =  (uint8_t)(torque >> 0);     // ����(Ma)��8λ�ֽ�
  cmd[7] =  snF;                        // ���ͬ���˶���־
  cmd[8] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 9);
}

/**
  * @brief    �ٶ�ģʽ
  * @param    addr  	�������ַ
  * @param    dir     ������         ��0ΪCW������ֵΪCCW
  * @param    v_ramp  ��б��(RPM/s)  ����Χ0 - 65535RPM/s
  * @param    velocity���ٶ�(RPM)    ����Χ0.0 - 4000.0RPM
  * @param    snF     �����ͬ����־ ��0Ϊ�����ã�����ֵ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF)
{
  uint8_t cmd[16] = {0}; uint16_t vel = 0;

  // ���ٶȷŴ�10�����͹�ȥ
  vel = (uint16_t)ABS(velocity * 10.0f);

  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xF6;                       // ������
  cmd[2] =  dir;                        // ���ţ�����
  cmd[3] =  (uint8_t)(v_ramp >> 8);     // �ٶ�б��(RPM/s)��8λ�ֽ�
  cmd[4] =  (uint8_t)(v_ramp >> 0);     // �ٶ�б��(RPM/s)��8λ�ֽ�
  cmd[5] =  (uint8_t)(vel >> 8);        // �ٶ�(RPM)��8λ�ֽ�
  cmd[6] =  (uint8_t)(vel >> 0);        // �ٶ�(RPM)��8λ�ֽ�
  cmd[7] =  snF;                        // ���ͬ���˶���־
  cmd[8] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 9);
}

/**
  * @brief    ֱͨ����λ��ģʽ
  * @param    addr  	�������ַ
  * @param    dir     ������										��0ΪCW������ֵΪCCW
  * @param    velocity������ٶ�(RPM)					����Χ0.0 - 4000.0RPM
  * @param    position��λ��(��)								����Χ0.0��- (2^32 - 1)��
  * @param    raf     ����λλ��/����λ�ñ�־	��0Ϊ���λ�ã�����ֵΪ����λ��
  * @param    snF     �����ͬ����־						��0Ϊ�����ã�����ֵ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF)
{
  uint8_t cmd[16] = {0}; uint16_t vel = 0; uint32_t pos = 0;

  // ���ٶȺ�λ�÷Ŵ�10�����͹�ȥ
  vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

  // װ������
  cmd[0]  =  addr;                      // ��ַ
  cmd[1]  =  0xFB;                      // ������
  cmd[2]  =  dir;                       // ���ţ�����
  cmd[3]  =  (uint8_t)(vel >> 8);       // ����ٶ�(RPM)��8λ�ֽ�
  cmd[4]  =  (uint8_t)(vel >> 0);       // ����ٶ�(RPM)��8λ�ֽ� 
  cmd[5]  =  (uint8_t)(pos >> 24);      // λ��(bit24 - bit31)
  cmd[6]  =  (uint8_t)(pos >> 16);      // λ��(bit16 - bit23)
  cmd[7]  =  (uint8_t)(pos >> 8);       // λ��(bit8  - bit15)
  cmd[8]  =  (uint8_t)(pos >> 0);       // λ��(bit0  - bit7 )
  cmd[9]  =  raf;                       // ��λλ��/����λ�ñ�־
  cmd[10] =  snF;                       // ���ͬ���˶���־
  cmd[11] =  0x6B;                      // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 12);
}

/**
  * @brief    ��������λ��ģʽ
  * @param    addr  	�������ַ
  * @param    dir     ������										��0ΪCW������ֵΪCCW
  * @param    acc     �����ټ��ٶ�(RPM/s)			��0ΪCW������ֵΪCCW
  * @param    dec     �����ټ��ٶ�(RPM/s)			��0ΪCW������ֵΪCCW
  * @param    velocity������ٶ�(RPM)					����Χ0.0 - 4000.0RPM
  * @param    position��λ��(��)								����Χ0.0��- (2^32 - 1)��
  * @param    raf     ����λλ��/����λ�ñ�־	��0Ϊ���λ�ã�����ֵΪ����λ��
  * @param    snF     �����ͬ����־						��0Ϊ�����ã�����ֵ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF)
{
  uint8_t cmd[32] = {0}; uint16_t vel = 0; uint32_t pos = 0;

  // ���ٶȺ�λ�÷Ŵ�10�����͹�ȥ
  vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

  // װ������
  cmd[0]  =  addr;                      // ��ַ
  cmd[1]  =  0xFD;                      // ������
  cmd[2]  =  dir;                       // ���ţ�����
  cmd[3]  =  (uint8_t)(acc >> 8);       // ���ټ��ٶ�(RPM/s)��8λ�ֽ�
  cmd[4]  =  (uint8_t)(acc >> 0);       // ���ټ��ٶ�(RPM/s)��8λ�ֽ�  
  cmd[5]  =  (uint8_t)(dec >> 8);       // ���ټ��ٶ�(RPM/s)��8λ�ֽ�
  cmd[6]  =  (uint8_t)(dec >> 0);       // ���ټ��ٶ�(RPM/s)��8λ�ֽ�  
  cmd[7]  =  (uint8_t)(vel >> 8);       // ����ٶ�(RPM)��8λ�ֽ�
  cmd[8]  =  (uint8_t)(vel >> 0);       // ����ٶ�(RPM)��8λ�ֽ� 
  cmd[9]  =  (uint8_t)(pos >> 24);      // λ��(bit24 - bit31)
  cmd[10] =  (uint8_t)(pos >> 16);      // λ��(bit16 - bit23)
  cmd[11] =  (uint8_t)(pos >> 8);       // λ��(bit8  - bit15)
  cmd[12] =  (uint8_t)(pos >> 0);       // λ��(bit0  - bit7 )
  cmd[13] =  raf;                       // ��λλ��/����λ�ñ�־
  cmd[14] =  snF;                       // ���ͬ���˶���־
  cmd[15] =  0x6B;                      // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 16);
}

/**
  * @brief    ����ֹͣ�����п���ģʽ��ͨ�ã�
  * @param    addr  �������ַ
  * @param    snF   �����ͬ����־��0Ϊ�����ã�����ֵ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xFE;                       // ������
  cmd[2] =  0x98;                       // ������
  cmd[3] =  snF;                        // ���ͬ���˶���־
  cmd[4] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    ���ͬ���˶�
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xFF;                       // ������
  cmd[2] =  0x66;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    ���õ�Ȧ��������λ��
  * @param    addr  �������ַ
  * @param    svF   ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x93;                       // ������
  cmd[2] =  0x88;                       // ������
  cmd[3] =  svF;                        // �Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  cmd[4] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    �޸Ļ������
  * @param    addr   �������ַ
  * @param    svF    ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  * @param    o_mode ������ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  * @param    o_dir  �����㷽��0ΪCW������ֵΪCCW
  * @param    o_vel  �������ٶȣ���λ��RPM��ת/���ӣ�
  * @param    o_tm   �����㳬ʱʱ�䣬��λ������
  * @param    sl_vel ������λ��ײ������ת�٣���λ��RPM��ת/���ӣ�
  * @param    sl_ma  ������λ��ײ�������������λ��Ma��������
  * @param    sl_ms  ������λ��ײ������ʱ�䣬��λ��Ms�����룩
  * @param    potF   ���ϵ��Զ��������㣬falseΪ��ʹ�ܣ�trueΪʹ��
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x4C;                       // ������
  cmd[2] =  0xAE;                       // ������
  cmd[3] =  svF;                        // �Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  cmd[4] =  o_mode;                     // ����ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  cmd[5] =  o_dir;                      // ���㷽��
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // �����ٶ�(RPM)��8λ�ֽ�
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // �����ٶ�(RPM)��8λ�ֽ� 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // ���㳬ʱʱ��(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // ���㳬ʱʱ��(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // ���㳬ʱʱ��(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // ���㳬ʱʱ��(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // ����λ��ײ������ת��(RPM)��8λ�ֽ�
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // ����λ��ײ������ת��(RPM)��8λ�ֽ� 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // ����λ��ײ���������(Ma)��8λ�ֽ�
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // ����λ��ײ���������(Ma)��8λ�ֽ� 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // ����λ��ײ������ʱ��(Ms)��8λ�ֽ�
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // ����λ��ײ������ʱ��(Ms)��8λ�ֽ�
  cmd[18] =  potF;                      // �ϵ��Զ��������㣬falseΪ��ʹ�ܣ�trueΪʹ��
  cmd[19] =  0x6B;                      // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 20);
}

/**
  * @brief    ��������
  * @param    addr   �������ַ
  * @param    o_mode ������ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  * @param    snF    �����ͬ����־��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x9A;                       // ������
  cmd[2] =  o_mode;                     // ����ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  cmd[3] =  snF;                        // ���ͬ���˶���־��falseΪ�����ã�trueΪ����
  cmd[4] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    ǿ���жϲ��˳�����
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void ZDT_X42_V2_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x9C;                       // ������
  cmd[2] =  0x48;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    ��������
  * @param    rxCmd   : ���յ������ݻ����ڸ�����
  * @param    rxCount : ���յ������ݳ���
  * @retval   ��
  */
void ZDT_X42_V2_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount)
{
	uint16_t i = 0, timeout = 0;
	
	// rxFrameFlag��usart.c�Ĵ��ڿ����ж��н�����λ
	// STM32���ڣ�ÿ���յ�һ֡���ݣ��ͻ����һ�������ж�
	// ���ԣ�ÿ����һ�������жϣ��Ϳ��ԶԴ��ڽ��յ������ݽ��н���
	while(rxFrameFlag == false)
	{
		// �ȴ�100���뻹δ��λ�����ж�Ϊû�����ݷ��أ���ʱ�˳�
		HAL_Delay(1); ++timeout; if(timeout > 100) { u_rxCount = 0; break; }
	}
	rxFrameFlag = false;
	
	// ��ȡ�������ݳ���
	*rxCount = u_rxCount;
	
	// ��ȡ��������
	for(i=0; i < u_rxCount; i++) { rxCmd[i] = u_rxCmd[i]; }
}

//_Control Step roll Angle(roll step
void ZDT_X42_Controll_StepAngle(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF)
{
  // ֱͨ����λ��ģʽ��ת��2000RPM�����λ���˶�-3600.0��
  ZDT_X42_V2_Bypass_Position_LV_Control(addr, dir, velocity, position, raf, snF);

  // �ȴ���������������ݻ���������rxCmd�ϣ�����ΪrxCount
  ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

  // ��֤У���ֽ�
  if(rxCmd[rxCount - 1] == 0x6B) { } else { }		
}


// Set now position as zero angle
void ZDT_X42_Origin_Set_O(uint8_t addr)
{
		// ���õ�ǰλ��Ϊ��Ȧ��������λ�ã����洢���ջ�������
		ZDT_X42_V2_Origin_Set_O(addr, 1);

		// �ȴ���������������ݻ���������rxCmd�ϣ�����ΪrxCount
		ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

		// ��֤У���ֽ�
		if(rxCmd[rxCount - 1] == 0x6B) {  } else {  }		
}


// �������״̬��־λ��������ɺ��Ƿ����ɹ���־λ
uint8_t s_oflag = 0; bool s_osuc = false;

//triggle_to zero angle
void ZDT_X42_triggle_to_zero(uint8_t addr)
{
		// �����������Ȧ����λ��ײ���㣨ע�⣺0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ��λ���ػ��㣩
		// ���Ҫ������Ȧ���㣬����Ҫ�����õ�Ȧ��������λ�ã�ʹ�����̡����õ�Ȧ��������λ�á���С��Ļ�˵�O_Set��������
		// ���Ҫ������Ȧ��λ���㣬����Ҫ������En������Ϊ��λ���ص����루P_PUL����ΪESI_RCO��������˵����Ӻ���λ����
		ZDT_X42_V2_Origin_Trigger_Return(addr, 0, 0);

		// �ȴ���������������ݻ���������rxCmd�ϣ�����ΪrxCount
		ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

		// 1ms��ѯ��ȡ����״̬��־λ - ��ѯ�Ƿ������ɣ��ǻ���ɹ����ǻ���ʧ��
		while(1)
		{
			// ��ȡ����״̬��־λ
			ZDT_X42_V2_Read_Sys_Params(addr, S_CPOS);

			// �ȴ���������������ݻ���������rxCmd�ϣ�����ΪrxCount
			ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

			// ������״̬��־λ���ж��Ƿ������ɣ��ǻ���ɹ����ǻ���ʧ��
			s_oflag = (rxCmd[2] & 0x0C); // ȡ���ڻ����־λ0x04�ͻ���ʧ�ܱ�־λ0x08
			if(s_oflag == 0x00) { s_osuc = true;	break; } // ���ڻ����־λ�ͻ���ʧ�ܱ�־λ��Ϊ0�������ɹ����˳�ѭ��
			if(s_oflag  & 0x08) { s_osuc = false;	break; } // ����ʧ�ܱ�־λλΪ1�������ʧ�ܣ��˳�ѭ��

			// ��ʱ1ms��ѯ��ȡ
			HAL_Delay(1);
		}

		// ��֤������ɺ��Ƿ����ɹ�
		if(s_osuc) {  } else {  }		
}

//Set_Turn To Zero_Params
void ZDT_X42_V2_Set_TurnToZero_Params(uint8_t addr,uint8_t o_dir,uint16_t o_vel,uint32_t o_tm,uint16_t sl_vel,uint16_t sl_ma,uint16_t sl_ms)
{
		/**********************************************************
		***  �޸�ԭ����������
			* @param    addr   �������ַ
			* @param    svF    ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
			* @param    o_mode ������ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
			* @param    o_dir  �����㷽��0ΪCW������ֵΪCCW
			* @param    o_vel  �������ٶȣ���λ��RPM��ת/���ӣ�
			* @param    o_tm   �����㳬ʱʱ�䣬��λ������
			* @param    sl_vel ������λ��ײ������ת�٣���λ��RPM��ת/���ӣ�
			* @param    sl_ma  ������λ��ײ�������������λ��Ma��������
			* @param    sl_ms  ������λ��ײ������ʱ�䣬��λ��Ms�����룩
			* @param    potF   ���ϵ��Զ��������㣬falseΪ��ʹ�ܣ�trueΪʹ��
		**********************************************************/ 
		ZDT_X42_V2_Origin_Modify_Params(addr, false, 0, o_dir, o_vel, o_tm, sl_vel, sl_ma, sl_ms, false);

		// �ȴ���������������ݻ���������rxCmd�ϣ�����ΪrxCount
		ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);
		
		// ��֤У���ֽ�
		if(rxCmd[rxCount - 1] == 0x6B) { } else { }		
}




