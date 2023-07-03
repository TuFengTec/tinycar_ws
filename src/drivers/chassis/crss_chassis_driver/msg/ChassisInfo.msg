## Time stamped drive command for robots with Ackermann steering.
#  $Id$

Header          header

uint16  WARMIGM
uint16  ERROR
uint8   MODE            # 底盘控制模式
int16   Vol             # 电池电压(mV)
int16   Curr            # 电池电流(mA)
int16   Quantity        # 电池电量
int16   ENCODER         # 备份编码器

int16   CURR_STEER      # 转向电流(mA)
int16   CURR_WHEEL_L    # 左驱动电流(mA)
int16   CURR_WHEEL_R    # 右驱动电流(mA)
int16   VEL_WHEEL_L     # 左后轮转速(rpm)
int16   VEL_WHEEL_R     # 右后轮转速(rpm)
int16   ENC_WHEEL_L     # 左后轮编码器
int16   ENC_WHEEL_R     # 右后轮编码器

uint16[8] UL            # 超声波阵列值


