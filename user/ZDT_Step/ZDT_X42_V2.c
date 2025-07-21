#include "ZDT_X42_V2.h"

/**********************************************************
*** ZDT_X42_V2.0步进闭环控制例程
*** 编写作者：ZHANGDATOU
*** 技术支持：张大头闭环伺服
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN博客：http s://blog.csdn.net/zhangdatou666
*** qq交流群：262438510
**********************************************************/

__IO bool rxFrameFlag = false;
__IO uint8_t u_rxCmd[FIFO_SIZE] = {0};
__IO uint8_t u_rxCount = 0;

/**
	* @brief   USART1中断函数
	* @param   无
	* @retval  无
	*/

/**********************************************************
***	串口接收中断
**********************************************************/

//Receive Byte

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 1. 把收到的字节丢进 FIFO（等价于 fifo_enQueue） */
        fifo_enQueue(rx_one_byte);

        /* 2. 重新启动下一次 1 字节中断接收 */
        HAL_UART_Receive_IT(&huart1, &rx_one_byte, sizeof(rx_one_byte));
    }
}

/**********************************************************
***	串口空闲中断
**********************************************************/

//Read Variable-length Data

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        /* Size 就是这一帧的长度 */
				u_rxCount = Size;
				// 提取一帧数据命令
				u_rxCount = fifo_queueLength(); for(uint8_t i=0; i < u_rxCount; i++) { u_rxCmd[i] = fifo_deQueue(); }
				// 一帧数据接收完成，置位帧标志位
				rxFrameFlag = true;			

        /* 重新启动下一次“接收直到空闲” */
				HAL_UARTEx_ReceiveToIdle_IT(&huart1, u_rxBuf, RX_BUF_LEN);
    }
}

/**
	* @brief   USART发送多个字节
	* @param   无
	* @retval  无
	*/
void usart_SendCmd(uint8_t *cmd, uint16_t len)
{
    HAL_UART_Transmit(&huart1, cmd, len, HAL_MAX_DELAY);
}

/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 功能码
  cmd[2] =  0x52;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址

  switch(s)                             // 功能码
  {
    case S_VER   : cmd[1] = 0x1F; break;                  /* 读取固件版本和对应的硬件版本 */
    case S_RL    : cmd[1] = 0x20; break;                  /* 读取读取相电阻和相电感 */
    case S_PID   : cmd[1] = 0x21; break;                  /* 读取PID参数 */
    case S_ORG   : cmd[1] = 0x22; break;                  /* 读取回零参数 */
    case S_VBUS  : cmd[1] = 0x24; break;                  /* 读取总线电压 */
    case S_CBUS  : cmd[1] = 0x26; break;                  /* 读取总线电流 */
    case S_CPHA  : cmd[1] = 0x27; break;                  /* 读取相电流 */
    case S_ENC   : cmd[1] = 0x29; break;                  /* 读取编码器原始值 */
    case S_CPUL  : cmd[1] = 0x30; break;                  /* 读取实时脉冲数（根据实时位置计算得到的脉冲数） */
    case S_ENCL  : cmd[1] = 0x31; break;                  /* 读取经过线性化校准后的编码器值 */
    case S_TPUL  : cmd[1] = 0x32; break;                  /* 读取输入脉冲数 */
    case S_TPOS  : cmd[1] = 0x33; break;                  /* 读取电机目标位置 */
    case S_OPOS  : cmd[1] = 0x34; break;                  /* 读取电机实时设定的目标位置（开环模式的实时位置） */
    case S_VEL   : cmd[1] = 0x35; break;                  /* 读取电机实时转速 */
    case S_CPOS  : cmd[1] = 0x36; break;                  /* 读取电机实时位置（基于角度编码器累加的电机实时位置） */
    case S_PERR  : cmd[1] = 0x37; break;                  /* 读取电机位置误差 */
    case S_TEMP  : cmd[1] = 0x39; break;                  /* 读取电机实时温度 */
    case S_SFLAG : cmd[1] = 0x3A; break;                  /* 读取状态标志位 */
    case S_OFLAG : cmd[1] = 0x3B; break;                  /* 读取回零状态标志位 */
    case S_Conf  : cmd[1] = 0x42; cmd[2] = 0x6C; break;   /* 读取驱动参数 */
    case S_State : cmd[1] = 0x43; cmd[2] = 0x7A; break;   /* 读取系统状态参数 */
    default: break;
  }

  // 发送命令
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
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的Ctrl_Mode菜单），0是开环模式，1是FOC矢量闭环模式
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_En_Control(uint8_t addr, bool state, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 6);
}

/**
  * @brief    力矩模式
  * @param    addr  ：电机地址
  * @param    sign  ：符号         ，0为正，其余值为负
  * @param    t_ramp：斜率(Ma/s)   ，范围0 - 65535Ma/s
  * @param    torque：力矩(Ma)     ，范围0 - 4000Ma
  * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF5;                       // 功能码
  cmd[2] =  sign;                       // 符号（方向）
  cmd[3] =  (uint8_t)(t_ramp >> 8);     // 力矩斜率(Ma/s)高8位字节
  cmd[4] =  (uint8_t)(t_ramp >> 0);     // 力矩斜率(Ma/s)低8位字节
  cmd[5] =  (uint8_t)(torque >> 8);     // 力矩(Ma)高8位字节
  cmd[6] =  (uint8_t)(torque >> 0);     // 力矩(Ma)低8位字节
  cmd[7] =  snF;                        // 多机同步运动标志
  cmd[8] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 9);
}

/**
  * @brief    速度模式
  * @param    addr  	：电机地址
  * @param    dir     ：方向         ，0为CW，其余值为CCW
  * @param    v_ramp  ：斜率(RPM/s)  ，范围0 - 65535RPM/s
  * @param    velocity：速度(RPM)    ，范围0.0 - 4000.0RPM
  * @param    snF     ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF)
{
  uint8_t cmd[16] = {0}; uint16_t vel = 0;

  // 将速度放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f);

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 符号（方向）
  cmd[3] =  (uint8_t)(v_ramp >> 8);     // 速度斜率(RPM/s)高8位字节
  cmd[4] =  (uint8_t)(v_ramp >> 0);     // 速度斜率(RPM/s)低8位字节
  cmd[5] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[6] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[7] =  snF;                        // 多机同步运动标志
  cmd[8] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 9);
}

/**
  * @brief    直通限速位置模式
  * @param    addr  	：电机地址
  * @param    dir     ：方向										，0为CW，其余值为CCW
  * @param    velocity：最大速度(RPM)					，范围0.0 - 4000.0RPM
  * @param    position：位置(°)								，范围0.0°- (2^32 - 1)°
  * @param    raf     ：相位位置/绝对位置标志	，0为相对位置，其余值为绝对位置
  * @param    snF     ：多机同步标志						，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF)
{
  uint8_t cmd[16] = {0}; uint16_t vel = 0; uint32_t pos = 0;

  // 将速度和位置放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFB;                      // 功能码
  cmd[2]  =  dir;                       // 符号（方向）
  cmd[3]  =  (uint8_t)(vel >> 8);       // 最大速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 最大速度(RPM)低8位字节 
  cmd[5]  =  (uint8_t)(pos >> 24);      // 位置(bit24 - bit31)
  cmd[6]  =  (uint8_t)(pos >> 16);      // 位置(bit16 - bit23)
  cmd[7]  =  (uint8_t)(pos >> 8);       // 位置(bit8  - bit15)
  cmd[8]  =  (uint8_t)(pos >> 0);       // 位置(bit0  - bit7 )
  cmd[9]  =  raf;                       // 相位位置/绝对位置标志
  cmd[10] =  snF;                       // 多机同步运动标志
  cmd[11] =  0x6B;                      // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 12);
}

/**
  * @brief    梯形曲线位置模式
  * @param    addr  	：电机地址
  * @param    dir     ：方向										，0为CW，其余值为CCW
  * @param    acc     ：加速加速度(RPM/s)			，0为CW，其余值为CCW
  * @param    dec     ：减速加速度(RPM/s)			，0为CW，其余值为CCW
  * @param    velocity：最大速度(RPM)					，范围0.0 - 4000.0RPM
  * @param    position：位置(°)								，范围0.0°- (2^32 - 1)°
  * @param    raf     ：相位位置/绝对位置标志	，0为相对位置，其余值为绝对位置
  * @param    snF     ：多机同步标志						，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF)
{
  uint8_t cmd[32] = {0}; uint16_t vel = 0; uint32_t pos = 0;

  // 将速度和位置放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 符号（方向）
  cmd[3]  =  (uint8_t)(acc >> 8);       // 加速加速度(RPM/s)高8位字节
  cmd[4]  =  (uint8_t)(acc >> 0);       // 加速加速度(RPM/s)低8位字节  
  cmd[5]  =  (uint8_t)(dec >> 8);       // 减速加速度(RPM/s)高8位字节
  cmd[6]  =  (uint8_t)(dec >> 0);       // 减速加速度(RPM/s)低8位字节  
  cmd[7]  =  (uint8_t)(vel >> 8);       // 最大速度(RPM)高8位字节
  cmd[8]  =  (uint8_t)(vel >> 0);       // 最大速度(RPM)低8位字节 
  cmd[9]  =  (uint8_t)(pos >> 24);      // 位置(bit24 - bit31)
  cmd[10] =  (uint8_t)(pos >> 16);      // 位置(bit16 - bit23)
  cmd[11] =  (uint8_t)(pos >> 8);       // 位置(bit8  - bit15)
  cmd[12] =  (uint8_t)(pos >> 0);       // 位置(bit0  - bit7 )
  cmd[13] =  raf;                       // 相位位置/绝对位置标志
  cmd[14] =  snF;                       // 多机同步运动标志
  cmd[15] =  0x6B;                      // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 16);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 功能码
  cmd[2] =  0x88;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr   ：电机地址
  * @param    svF    ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 功能码
  cmd[2] =  0xAE;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
  cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
  cmd[19] =  0x6B;                      // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF    ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 功能码
  cmd[2] =  0x48;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    接收数据
  * @param    rxCmd   : 接收到的数据缓存在该数组
  * @param    rxCount : 接收到的数据长度
  * @retval   无
  */
void ZDT_X42_V2_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount)
{
	uint16_t i = 0, timeout = 0;
	
	// rxFrameFlag在usart.c的串口空闲中断中进行置位
	// STM32串口，每接收到一帧数据，就会产生一个空闲中断
	// 所以，每产生一个空闲中断，就可以对串口接收到的数据进行解析
	while(rxFrameFlag == false)
	{
		// 等待100毫秒还未置位，就判定为没有数据返回，超时退出
		HAL_Delay(1); ++timeout; if(timeout > 100) { u_rxCount = 0; break; }
	}
	rxFrameFlag = false;
	
	// 获取返回数据长度
	*rxCount = u_rxCount;
	
	// 获取返回数据
	for(i=0; i < u_rxCount; i++) { rxCmd[i] = u_rxCmd[i]; }
}

//_Control Step roll Angle(roll step
void ZDT_X42_Controll_StepAngle(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF)
{
  // 直通限速位置模式：转速2000RPM，相对位置运动-3600.0°
  ZDT_X42_V2_Bypass_Position_LV_Control(addr, dir, velocity, position, raf, snF);

  // 等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
  ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

  // 验证校验字节
  if(rxCmd[rxCount - 1] == 0x6B) { } else { }		
}


// Set now position as zero angle
void ZDT_X42_Origin_Set_O(uint8_t addr)
{
		// 设置当前位置为单圈回零的零点位置，并存储到闭环驱动上
		ZDT_X42_V2_Origin_Set_O(addr, 1);

		// 等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
		ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

		// 验证校验字节
		if(rxCmd[rxCount - 1] == 0x6B) {  } else {  }		
}


// 定义回零状态标志位、回零完成后是否回零成功标志位
uint8_t s_oflag = 0; bool s_osuc = false;

//triggle_to zero angle
void ZDT_X42_triggle_to_zero(uint8_t addr)
{
		// 发送命令触发多圈无限位碰撞回零（注意：0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈限位开关回零）
		// 如果要触发单圈回零，则需要先设置单圈回零的零点位置，使用例程“设置单圈回零的零点位置”或小屏幕菜单O_Set进行设置
		// 如果要触发多圈限位回零，则需要先设置En引脚作为限位开关的输入（P_PUL设置为ESI_RCO），并按说明书接好限位开关
		ZDT_X42_V2_Origin_Trigger_Return(addr, 0, 0);

		// 等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
		ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

		// 1ms轮询读取回零状态标志位 - 查询是否回零完成，是回零成功还是回零失败
		while(1)
		{
			// 读取回零状态标志位
			ZDT_X42_V2_Read_Sys_Params(addr, S_CPOS);

			// 等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
			ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

			// 检查回零状态标志位，判断是否回零完成，是回零成功还是回零失败
			s_oflag = (rxCmd[2] & 0x0C); // 取正在回零标志位0x04和回零失败标志位0x08
			if(s_oflag == 0x00) { s_osuc = true;	break; } // 正在回零标志位和回零失败标志位都为0，则回零成功，退出循环
			if(s_oflag  & 0x08) { s_osuc = false;	break; } // 回零失败标志位位为1，则回零失败，退出循环

			// 延时1ms轮询读取
			HAL_Delay(1);
		}

		// 验证回零完成后是否回零成功
		if(s_osuc) {  } else {  }		
}

//Set_Turn To Zero_Params
void ZDT_X42_V2_Set_TurnToZero_Params(uint8_t addr,uint8_t o_dir,uint16_t o_vel,uint32_t o_tm,uint16_t sl_vel,uint16_t sl_ma,uint16_t sl_ms)
{
		/**********************************************************
		***  修改原点回零参数：
			* @param    addr   ：电机地址
			* @param    svF    ：是否存储标志，false为不存储，true为存储
			* @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
			* @param    o_dir  ：回零方向，0为CW，其余值为CCW
			* @param    o_vel  ：回零速度，单位：RPM（转/分钟）
			* @param    o_tm   ：回零超时时间，单位：毫秒
			* @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
			* @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
			* @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
			* @param    potF   ：上电自动触发回零，false为不使能，true为使能
		**********************************************************/ 
		ZDT_X42_V2_Origin_Modify_Params(addr, false, 0, o_dir, o_vel, o_tm, sl_vel, sl_ma, sl_ms, false);

		// 等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
		ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);
		
		// 验证校验字节
		if(rxCmd[rxCount - 1] == 0x6B) { } else { }		
}




