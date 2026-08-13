#include "app_chassis.h"

#include "CP_System.h"
#include "DM.h"
#include "PID.h"
#include "RM.h"
#include "RM_Lib.h"
#include "gpio.h"
#include "spi.h"

#include <math.h>
#include <stdint.h>

typedef float f;
typedef uint8_t u8;

#define PROTECT_MODE 0
#define ONLY_GIMBAL  1
#define ONLY_CHASSIC 2
#define CONTROL_MODE 3
#define XTL_MODE     4
#define SHOOT_MODE   5
#define PLAYER_MODE  6
#define FAST_CHASSIC 7

#define CHASSIC_TOTAL_OUTPUT_MAX 48000

// CAN2 -> gimbal feed-forward frame:
//   word 0: signed spin speed (wheel-mixer command units)
//   word 1: PLAYER small-gyro enable
//   word 2: sin(gimbal relative yaw) * 10000
//   word 3: cos(gimbal relative yaw) * 10000
#define CHASSIS_SPIN_FF_CAN_ID 0x116
#define CHASSIS_SPIN_PHASE_SCALE 10000.0f

#define XTL_RANDOM_SPEED_RATIO       0.10f
#define XTL_RANDOM_UPDATE_PERIOD_MS  10U
#define XTL_RANDOM_TARGET_MIN_MS     300U
#define XTL_RANDOM_TARGET_MAX_MS     900U
#define XTL_RANDOM_STEP_MIN          1.0f
#define XTL_RANDOM_STEP_MAX          8.0f

#define Motor_Yaw_front -2.0534f//1.12776f//-1.95791f//1.81122f // -1.95791f

#define YAW_ERROR_MASK     (0x0001 << 0)
#define YAW_BACK_FLAG_MASK (0x0001 << 1)
#define BUFF_FLAG_MASK     (0x0001 << 2)

#ifndef PI
#define PI 3.1415926f
#endif

#define cater_RY 5
#define cater_RX 10

typedef struct
{
  float PID_1_out;
  float PID_2_out;
  float PID_3_out;
  float PID_4_out;
  float PID_min_out;
  float Klimit;
  float limit_output;
  float TotalOutput;
  float KlimitGain;
  float used_power;
} DP_Power_Limit_t;

struct DP_Motor_FP
{
  u8 M3508_Flag;
  u8 DM_Flag;
  u8 TIM3_Flag;
  DP_Motor_FP() : M3508_Flag(0), DM_Flag(0), TIM3_Flag(0) {}
};

static DBUS YK(&huart6);

static USER_CAN CAN_Motor(&hcan1, 0);
static USER_CAN CAN_Communicate(&hcan2, 1);

static MOTOR_RM M3508_MOTOR_QZ(0x201, &CAN_Motor),
                M3508_MOTOR_HY(0x202, &CAN_Motor),
                M3508_MOTOR_QY(0x203, &CAN_Motor),
                M3508_MOTOR_HZ(0x204, &CAN_Motor);

static MOTOR_DM YAW(0x12, &CAN_Communicate);

static PID_class PID_Chassis(7, 0, 0, 350, 0, 0, 500);
static PID_class PID_Chassis_FeiPo(20, 0, 10, 500, 0, 200, 500);

static PID_class M3508_MOTOR_QZ_sp(5, 0, 0, 13000, 0, 0, 13000),
                 M3508_MOTOR_HZ_sp(5, 0, 0, 13000, 0, 0, 13000),
                 M3508_MOTOR_HY_sp(5, 0, 0, 13000, 0, 0, 13000),
                 M3508_MOTOR_QY_sp(5, 0, 0, 13000, 0, 0, 13000);

static MOTOR_QXL_DiPan DP;

static DP_Power_Limit_t DP_Power_Limit;
static DP_Motor_FP Motor_Flag;

static UpDown_check_class UD_E(0), UD_Q(0), UD_Chase(0), UD_Speed_Cut(0),
                          UD_XTL(0), UD_SpeedUp(0), UD_SpeedDown(0),
                          UD_FeiPo(0), UD_yaogan(0), UD_MID(0);

static uint8_t UD_E_buf, UD_Q_buf, UD_Mid_buf;

static uint32_t M3508_1 = 0, M3508_2 = 0, M3508_3 = 0, M3508_4 = 0;
static uint16_t Chase_Flag, Chase_Time;

static uint8_t YAW_Error = 1;
static uint8_t Yaw_Back;
static f XTL_PID_OUT;

static float Gimbal_Roll, Gimbal_Pitch, Gimbal_Roll_Acc, Gimbal_Pitch_Acc;
static float V_Bat, V_Cap, V_Load;
static u8 v_can_buff;
static float V_Bat_Real, V_Cap_Real, V_Load_Real;

static uint8_t XTL_Flag = 0, Reset_flag = 0;
static uint16_t speed = 0, XTL_speed = 0;
static int16_t SpeedChange = 0;
static f XTL_SPEED_OUT = 0;
static f XTL_RANDOM_SPEED_OUT = 0;
static f XTL_RANDOM_SPEED_TARGET = 0;
static uint32_t XTL_RANDOM_STATE = 0x6D2B79F5U;
static uint32_t XTL_RANDOM_LAST_UPDATE = 0;
static uint32_t XTL_RANDOM_NEXT_TARGET = 0;
static uint8_t XTL_RANDOM_ACTIVE = 0;
static uint16_t XTL_SPEED_FLAG = 8;

static uint8_t Chassic_3508_Flag;
static float FB_Speed, LR_Speed, FB_Real_Speed, LR_Real_Speed, Target_Speed;

static uint16_t Communicate_Send_Flag_1, Communicate_Rx_Flag_1;

static uint8_t UI_step = 1, UI_step_ten = 1;
static bool cp_state;

static int16_t Driver_OUT_PID, MCL_SPEED;
static float theta_rad;
static float sin_theta, cos_theta;
static uint8_t Buff_Flag;
static uint16_t E_Char[5] = {Color_Yellow, 50, 5, 850, 150};
static uint16_t Q_Char[5] = {Color_Yellow, 50, 5, 700, 150};
static uint16_t MID_Char[5] = {Color_Green, 40, 4, 900, 850};
static uint8_t E_Flag, Q_Flag, MID_Flag;
static uint8_t YK_Mode;
static u8 mid_state;

static float fb_add_sp = 5, fb_cut_sp = 5.5, lr_add_sp = 5, lr_cut_sp = 5.5;
static float Chassic_Ch0_Real, Chassic_Ch1_Real, Chassic_Ch2_Real;

static uint32_t name1[5] = {1, 2, 3, 4, 5};
static graphic_tpyedef graphic_tpye1[7] = {Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line};
static Color_tpyedef color1[7] = {Color_Yellow, Color_Pink, Color_Yellow, Color_Green, Color_Yellow, Color_Yellow, Color_Red_Or_Bule};
static uint16_t d1[8][5] = {{0},
                            {0},
                            {1, 2, 2, 2, 2},
                            {0, 945 , 945, 945, 966 },
                            {0, 511, 501, 491, 540},
                            {0, 0, 0, 0, 0},
                            {0, 985 , 985 , 985 , 966},
                            {0, 511, 501, 491, 440}};

static float sin_theta_L, sin_theta_R, sin_theta_down, cos_theta_L, cos_theta_R, cos_theta_down;
static uint32_t name2[7] = {6, 7, 8, 9, 10, 11, 12};
static graphic_tpyedef graphic_tpye2[7] = {Graphic_Circle, Graphic_Line, Graphic_Line, Graphic_Rectangle, Graphic_Rectangle, Graphic_Rectangle, Graphic_Rectangle};
static Color_tpyedef color2[7] = {Color_Green, Color_Pink, Color_White, Color_Orange, Color_Orange, Color_Orange, Color_Orange};
static uint16_t d2[8][7] = {{0},
                            {0},
                            {2, 6, 0, 2, 2, 2, 2},
                            {480, 480, 700, 0, 0, 0, 0},
                            {750, 750, 80, 0, 0, 0, 0},
                            {75, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 80, 0, 0, 0, 0}};

static uint32_t name4[5] = {15, 16, 17, 18};
static graphic_tpyedef graphic_tpye4[5] = {Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line};
static Color_tpyedef color4[5] = {Color_Yellow, Color_Yellow, Color_Yellow, Color_Yellow};
static uint16_t d4[8][5] = {{0}, 
                            {0}, 
                            {2, 2, 2, 2}, 
                            {400, 700, 1570, 1270}, 
                            {20, 434, 20, 434}, 
                            {0, 0, 0, 0}, 
                            {700, 720, 1270, 1250}, 
                            {434, 434, 434, 434}};

static uint32_t name5[2] = {19, 20};
static graphic_tpyedef graphic_tpye5[2] = {Graphic_Line, Graphic_Line};
static Color_tpyedef color5[2] = {Color_Yellow, Color_Yellow};
static uint16_t d5[8][2] = {{0, 0},
                            {0, 0},
                            {2, 2},
                            {1026, 1026},
                            {591, 611},
                            {0, 0},
                            {1056, 1056},
                            {611, 591}};

static void Slow(float *rec, float target, float slow_Inc)
{
  if (fabsf(*rec) - fabsf(target) > 0)
    slow_Inc = slow_Inc * 8;

  if (fabsf(*rec - target) < slow_Inc)
    *rec = target;
  else
  {
    if ((*rec) > target)
      (*rec) -= slow_Inc;
    if ((*rec) < target)
      (*rec) += slow_Inc;
  }
}

static void F_slow(float *in, float target, float add_inc, float cut_inc, float stop_err)
{
  if (fabsf(*in - target) <= stop_err)
    *in = target;
  else
  {
    float in_buf = 0;
    if (*in < target)
    {
      in_buf = *in + add_inc;
      *in = (in_buf > target) ? target : in_buf;
    }
    else
    {
      in_buf = *in - cut_inc;
      *in = (in_buf < target) ? target : in_buf;
    }
  }
}

static void Chassic_axis_slow(float *real, float target, float add_sp, float cut_sp)
{
  if (*real > 0)
    F_slow(real, target, add_sp, cut_sp, cut_sp);
  else
    F_slow(real, target, cut_sp, add_sp, cut_sp);
}

static void KEY_Forback_Ctrl1(void)
{
  FB_Speed = (YK.Pressed_Check(KEY_PRESSED_W) - YK.Pressed_Check(KEY_PRESSED_S)) * Target_Speed;
  LR_Speed = (YK.Pressed_Check(KEY_PRESSED_D) - YK.Pressed_Check(KEY_PRESSED_A)) * Target_Speed;

  if (FB_Real_Speed > 0)
    F_slow(&FB_Real_Speed, FB_Speed, fb_add_sp, fb_cut_sp, fb_cut_sp);
  else
    F_slow(&FB_Real_Speed, FB_Speed, fb_cut_sp, fb_add_sp, fb_cut_sp);

  if (LR_Real_Speed > 0)
    F_slow(&LR_Real_Speed, LR_Speed, lr_add_sp, lr_cut_sp, lr_cut_sp);
  else
    F_slow(&LR_Real_Speed, LR_Speed, lr_cut_sp, lr_add_sp, lr_cut_sp);
}

static void Chassic_Forback_Ctrl(void)
{
  float ch0_target = 0;
  float ch1_target = 0;
  float ch2_target = 0;

  if (YK_Mode == ONLY_CHASSIC || YK_Mode == FAST_CHASSIC || YK_Mode == CONTROL_MODE || YK_Mode == XTL_MODE)
  {
    ch0_target = YK.yaogan.ch1;
    ch1_target = YK.yaogan.ch0;
    ch2_target = YK.yaogan.ch2;
  }

  Chassic_axis_slow(&Chassic_Ch0_Real, ch0_target, fb_add_sp, fb_cut_sp);
  Chassic_axis_slow(&Chassic_Ch1_Real, ch1_target, lr_add_sp, lr_cut_sp);
  Chassic_axis_slow(&Chassic_Ch2_Real, ch2_target, lr_add_sp, lr_cut_sp);
}

static void MODE_DEAL(void)
{
  if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = PROTECT_MODE;
  else if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = ONLY_GIMBAL;
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = ONLY_CHASSIC;
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = CONTROL_MODE;
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = XTL_MODE;
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_DOWN)
    YK_Mode = SHOOT_MODE;
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_DOWN)
    YK_Mode = PLAYER_MODE;
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = FAST_CHASSIC;
  else
    YK_Mode = PROTECT_MODE;
}

static void M3508_Limit_deal(void)
{
  if (YK_Mode == PROTECT_MODE || YK_Mode == SHOOT_MODE)
  {
    CAN_Motor.Send_RM(0x200, 0, 0, 0, 0);
  }
  else
  {
    DP_Power_Limit.Klimit = (float)(V_Cap_Real / 20.0f);
    switch (v_can_buff)
    {
      case 0:
        DP_Power_Limit.KlimitGain = 1;
        V_Cap_Real = 0.0f;
        break;
      case 1:
        if (V_Cap_Real >= 12.0f)
          DP_Power_Limit.KlimitGain = 1;
        else if (V_Cap_Real >= 10.0f)
          DP_Power_Limit.KlimitGain = YK.Pressed_Check(KEY_PRESSED_SHIFT) ? DP_Power_Limit.Klimit * 1.5f : DP_Power_Limit.Klimit;
        else if (V_Cap_Real >= 8.0f)
          DP_Power_Limit.KlimitGain = YK.Pressed_Check(KEY_PRESSED_SHIFT) ? DP_Power_Limit.Klimit * DP_Power_Limit.Klimit * 1.5f : (DP_Power_Limit.Klimit * DP_Power_Limit.Klimit) * 5 / 4;
        else
          DP_Power_Limit.KlimitGain = YK.Pressed_Check(KEY_PRESSED_SHIFT) ? DP_Power_Limit.Klimit * DP_Power_Limit.Klimit * DP_Power_Limit.Klimit * 1.5f : DP_Power_Limit.Klimit * DP_Power_Limit.Klimit * DP_Power_Limit.Klimit;
        break;
    }

    DP_Power_Limit.limit_output = DP_Power_Limit.KlimitGain * CHASSIC_TOTAL_OUTPUT_MAX;
    DP_Power_Limit.TotalOutput = fabsf(M3508_MOTOR_QZ_sp.OUT_PID) + fabsf(M3508_MOTOR_HZ_sp.OUT_PID) + fabsf(M3508_MOTOR_HY_sp.OUT_PID) + fabsf(M3508_MOTOR_QY_sp.OUT_PID);

    if (DP_Power_Limit.TotalOutput >= DP_Power_Limit.limit_output)
    {
      DP_Power_Limit.PID_1_out = M3508_MOTOR_QZ_sp.OUT_PID / DP_Power_Limit.TotalOutput * DP_Power_Limit.limit_output;
      DP_Power_Limit.PID_2_out = M3508_MOTOR_HY_sp.OUT_PID / DP_Power_Limit.TotalOutput * DP_Power_Limit.limit_output;
      DP_Power_Limit.PID_3_out = M3508_MOTOR_QY_sp.OUT_PID / DP_Power_Limit.TotalOutput * DP_Power_Limit.limit_output;
      DP_Power_Limit.PID_4_out = M3508_MOTOR_HZ_sp.OUT_PID / DP_Power_Limit.TotalOutput * DP_Power_Limit.limit_output;
    }
    else
    {
      M3508_MOTOR_QZ_sp.LIMIT_PID = 12000;
      M3508_MOTOR_HZ_sp.LIMIT_PID = 12000;
      M3508_MOTOR_HY_sp.LIMIT_PID = 12000;
      M3508_MOTOR_QY_sp.LIMIT_PID = 12000;

      DP_Power_Limit.PID_1_out = M3508_MOTOR_QZ_sp.OUT_PID;
      DP_Power_Limit.PID_2_out = M3508_MOTOR_HY_sp.OUT_PID;
      DP_Power_Limit.PID_3_out = M3508_MOTOR_QY_sp.OUT_PID;
      DP_Power_Limit.PID_4_out = M3508_MOTOR_HZ_sp.OUT_PID;
    }
    CAN_Motor.Send_RM(0x200, DP_Power_Limit.PID_1_out, DP_Power_Limit.PID_2_out, DP_Power_Limit.PID_3_out, DP_Power_Limit.PID_4_out);
  }
}

static void Communicate_deal(void)
{
  if (CAN_Communicate.RxHeader.StdId == 0x110)
    YK.can_receive_data_deal(CAN_Communicate.rx_buf);

  if (CAN_Communicate.RxHeader.StdId == 0x113)
  {
    YK.V_can_receive_data_deal(CAN_Communicate.rx_buf);
    Communicate_Rx_Flag_1 = CAN_Communicate.rx_buf[3] | CAN_Communicate.rx_buf[2] << 8;
    Driver_OUT_PID = CAN_Communicate.rx_buf[5] | CAN_Communicate.rx_buf[4] << 8;
    MCL_SPEED = CAN_Communicate.rx_buf[7] | CAN_Communicate.rx_buf[6] << 8;
  }

  if (CAN_Communicate.RxHeader.StdId == 0x123)
  {
    union
    {
      float f;
      uint8_t c[4];
    } angle;
    angle.c[0] = CAN_Communicate.rx_buf[1];
    angle.c[1] = CAN_Communicate.rx_buf[2];
    angle.c[2] = CAN_Communicate.rx_buf[3];
    angle.c[3] = CAN_Communicate.rx_buf[4];
    if (CAN_Communicate.rx_buf[0] == 0)
      Gimbal_Pitch = angle.f;
    else if (CAN_Communicate.rx_buf[0] == 1)
      Gimbal_Roll = angle.f;
    else if (CAN_Communicate.rx_buf[0] == 2)
      Gimbal_Pitch_Acc = angle.f;
    else if (CAN_Communicate.rx_buf[0] == 3)
      Gimbal_Roll_Acc = angle.f;
  }

  if (CAN_Communicate.RxHeader.StdId == 0x120)
  {
    V_Bat = CAN_Communicate.rx_buf[1] | CAN_Communicate.rx_buf[0] << 8;
    V_Cap = CAN_Communicate.rx_buf[3] | CAN_Communicate.rx_buf[2] << 8;
    V_Load = CAN_Communicate.rx_buf[5] | CAN_Communicate.rx_buf[4] << 8;
    v_can_buff = CAN_Communicate.rx_buf[7] | CAN_Communicate.rx_buf[6] << 8;
    V_Bat_Real = V_Bat / 100;
    V_Cap_Real = V_Cap / 100;
    V_Load_Real = V_Load / 100;
  }
}

static uint32_t XTL_random_next(void)
{
  XTL_RANDOM_STATE ^= XTL_RANDOM_STATE << 13;
  XTL_RANDOM_STATE ^= XTL_RANDOM_STATE >> 17;
  XTL_RANDOM_STATE ^= XTL_RANDOM_STATE << 5;
  return XTL_RANDOM_STATE;
}

static float XTL_random_range(float min_value, float max_value)
{
  const float random_ratio = (float)(XTL_random_next() & 0xFFFFU) / 65535.0f;
  return min_value + (max_value - min_value) * random_ratio;
}

static void XTL_random_speed_deal(void)
{
  const uint32_t now = HAL_GetTick();
  const bool random_xtl_enable = (YK_Mode == XTL_MODE) || (YK_Mode == PLAYER_MODE && XTL_Flag);

  if (!random_xtl_enable)
  {
    XTL_RANDOM_ACTIVE = 0;
    XTL_RANDOM_SPEED_OUT = 0;
    XTL_RANDOM_SPEED_TARGET = 0;
    return;
  }

  const float base_speed = (float)XTL_speed;
  const float speed_span = base_speed * XTL_RANDOM_SPEED_RATIO;
  const float min_speed = base_speed - speed_span;
  const float max_speed = base_speed + speed_span;

  if (XTL_RANDOM_ACTIVE == 0)
  {
    XTL_RANDOM_STATE ^= now ^ ((uint32_t)XTL_speed << 16) ^ (uint32_t)(fabsf(YAW.mang) * 1000.0f);
    if (XTL_RANDOM_STATE == 0)
      XTL_RANDOM_STATE = 0x6D2B79F5U;

    XTL_RANDOM_ACTIVE = 1;
    XTL_RANDOM_SPEED_OUT = 0;
    XTL_RANDOM_LAST_UPDATE = now;
    XTL_RANDOM_NEXT_TARGET = 0;
  }

  if ((int32_t)(now - XTL_RANDOM_NEXT_TARGET) >= 0 ||
      XTL_RANDOM_SPEED_TARGET < min_speed || XTL_RANDOM_SPEED_TARGET > max_speed)
  {
    XTL_RANDOM_SPEED_TARGET = XTL_random_range(min_speed, max_speed);
    XTL_RANDOM_NEXT_TARGET = now + XTL_RANDOM_TARGET_MIN_MS +
                            (XTL_random_next() % (XTL_RANDOM_TARGET_MAX_MS - XTL_RANDOM_TARGET_MIN_MS + 1U));
  }

  if ((uint32_t)(now - XTL_RANDOM_LAST_UPDATE) >= XTL_RANDOM_UPDATE_PERIOD_MS)
  {
    const float random_step = XTL_random_range(XTL_RANDOM_STEP_MIN, XTL_RANDOM_STEP_MAX);
    if (XTL_RANDOM_SPEED_OUT < XTL_RANDOM_SPEED_TARGET)
      XTL_RANDOM_SPEED_OUT = fminf(XTL_RANDOM_SPEED_OUT + random_step, XTL_RANDOM_SPEED_TARGET);
    else
      XTL_RANDOM_SPEED_OUT = fmaxf(XTL_RANDOM_SPEED_OUT - random_step, XTL_RANDOM_SPEED_TARGET);

    XTL_RANDOM_LAST_UPDATE = now;
  }
}

static void XTLkeyboarddeal(void)
{
  if (UD_XTL.updata(YK.Pressed_Check(KEY_PRESSED_Q)) == UpDown_check_rising)
  {
    if (XTL_Flag == 0)
    {
      XTL_Flag = 1;
      Communicate_Send_Flag_1 &= ~(0x0001 << 4);
      Communicate_Send_Flag_1 |= (0x0001 << 3);
    }
    else
    {
      XTL_Flag = 0;
      XTL_SPEED_OUT = 0;
      SpeedChange = 0;
      XTL_PID_OUT = 0;
      Communicate_Send_Flag_1 &= ~(0x0001 << 3);
    }
  }

  f xtl_speed_target = (XTL_Flag || YK_Mode == XTL_MODE) ? XTL_speed : 0;
  Slow(&XTL_SPEED_OUT, xtl_speed_target, 10);
  XTL_random_speed_deal();
  UD_Q_buf = UD_Q.updata(XTL_Flag);
  if (UD_Q_buf == UpDown_check_rising)
  {
    Q_Flag = 1;
    Q_Char[0] = Color_Pink;
  }
  else if (UD_Q_buf == UpDown_check_falling)
  {
    Q_Flag = 1;
    Q_Char[0] = Color_Yellow;
  }
}

static void chassic_power_deal(void)
{
  if (CP.ext_game_robot_status_t.chassis_power_limit > 1 && CP.ext_game_robot_status_t.chassis_power_limit < 45)
  {
    speed = 200 + SpeedChange;
    XTL_speed = 250 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 45 && CP.ext_game_robot_status_t.chassis_power_limit < 50)
  {
    speed = 230 + SpeedChange;
    XTL_speed = 280 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 50 && CP.ext_game_robot_status_t.chassis_power_limit < 55)
  {
    speed = 260 + SpeedChange;
    XTL_speed = 300 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 55 && CP.ext_game_robot_status_t.chassis_power_limit < 60)
  {
    speed = 300 + SpeedChange;
    XTL_speed = 340 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 60 && CP.ext_game_robot_status_t.chassis_power_limit < 65)
  {
    speed = 330 + SpeedChange;
    XTL_speed = 360 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 65 && CP.ext_game_robot_status_t.chassis_power_limit < 70)
  {
    speed = 360 + SpeedChange;
    XTL_speed = 400 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 70 && CP.ext_game_robot_status_t.chassis_power_limit < 75)
  {
    speed = 390 + SpeedChange;
    XTL_speed = 440 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 75 && CP.ext_game_robot_status_t.chassis_power_limit < 80)
  {
    speed = 400 + SpeedChange;
    XTL_speed = 450 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 80 && CP.ext_game_robot_status_t.chassis_power_limit < 85)
  {
    speed = 425 + SpeedChange;
    XTL_speed = 450 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 85 && CP.ext_game_robot_status_t.chassis_power_limit < 90)
  {
    speed = 440 + SpeedChange;
    XTL_speed = 470 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 90 && CP.ext_game_robot_status_t.chassis_power_limit < 95)
  {
    speed = 460 + SpeedChange;
    XTL_speed = 480 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 95 && CP.ext_game_robot_status_t.chassis_power_limit < 100)
  {
    speed = 470 + SpeedChange;
    XTL_speed = 480 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 100 && CP.ext_game_robot_status_t.chassis_power_limit < 120)
  {
    speed = 480 + SpeedChange;
    XTL_speed = 490 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 120 && CP.ext_game_robot_status_t.chassis_power_limit < 200)
  {
    speed = 500 + SpeedChange;
    XTL_speed = 500 + SpeedChange;
  }
  else if (CP.ext_game_robot_status_t.chassis_power_limit >= 200)
  {
    speed = 500 + SpeedChange;
    XTL_speed = 520 + SpeedChange;
  }
  else
  {
    speed = 550 + SpeedChange;
    XTL_speed = 500 + SpeedChange;
  }
}

static void QXL_Field_Data_Deal(float forward, float right, float turn, int max_rate)
{
  const float body_forward = forward * cos_theta + right * sin_theta;
  const float body_right = right * cos_theta - forward * sin_theta;
  DP.QXL_Data_Deal(body_forward, body_right, turn, 0.0f, max_rate);
}

static bool Chassis_spin_feedforward_enabled(void)
{
  // Feed-forward is intentionally limited to PLAYER mode.  XTL_MODE is a
  // separate operator mode and must not make the gimbal compensate unless
  // the normal PLAYER-mode small gyro has explicitly been enabled.
  return (YK_Mode == PLAYER_MODE && XTL_Flag);
}

static int16_t Chassis_spin_feedforward_speed(void)
{
  if (!Chassis_spin_feedforward_enabled())
    return 0;

  float signed_speed = XTL_RANDOM_SPEED_OUT;
  // PLAYER_deal() uses the historical fixed positive spin direction.

  if (signed_speed > 32767.0f)
    signed_speed = 32767.0f;
  else if (signed_speed < -32768.0f)
    signed_speed = -32768.0f;
  return (int16_t)signed_speed;
}

static void PLAYER_deal(void)
{
  if (YK.Pressed_Check(KEY_PRESSED_SHIFT))
  {
    Target_Speed = speed + 200 + XTL_Flag * (CP.ext_game_robot_status_t.chassis_power_limit - 20) * 1.2f;
    YK.yaogan.ch1 = LR_Real_Speed;
    YK.yaogan.ch0 = FB_Real_Speed;
  }
  else
  {
    Target_Speed = speed + XTL_Flag * (CP.ext_game_robot_status_t.chassis_power_limit - 20) * 0.4f;
    YK.yaogan.ch1 = LR_Real_Speed;
    YK.yaogan.ch0 = FB_Real_Speed;
  }

  if (YAW_Error == 0)
  {
    if (XTL_Flag)
    {
      QXL_Field_Data_Deal(FB_Real_Speed,
                          LR_Real_Speed,
                          XTL_RANDOM_SPEED_OUT + 0.5f * ((fabsf(-LR_Real_Speed) < fabsf(-FB_Real_Speed)) ? fabsf(-LR_Real_Speed) : fabsf(-FB_Real_Speed)),
                          8000);
    }
    else if (Yaw_Back == 1)
    {
      QXL_Field_Data_Deal(FB_Real_Speed, LR_Real_Speed, 0, 8000);
    }
    else
    {
      QXL_Field_Data_Deal(FB_Real_Speed, LR_Real_Speed, PID_Chassis.OUT_PID, 8000);
    }
  }
}

static void Key_deal(void)
{
  chassic_power_deal();

  mid_state = (YAW.mang < Motor_Yaw_front + 1.0f && YAW.mang > Motor_Yaw_front - 1.0f) ? 1 : 0;

  Yaw_Back = (Communicate_Rx_Flag_1 & YAW_BACK_FLAG_MASK) ? 1 : 0;
  Buff_Flag = (Communicate_Rx_Flag_1 & BUFF_FLAG_MASK) ? 1 : 0;

  UD_Mid_buf = UD_MID.updata(mid_state);
  if (UD_Mid_buf == UpDown_check_rising)
  {
    MID_Flag = 1;
    MID_Char[0] = Color_Green;
  }
  else if (UD_Mid_buf == UpDown_check_falling)
  {
    MID_Flag = 1;
    MID_Char[0] = Color_White;
  }

  if (CP.ext_game_robot_status_t.robot_id < 10)
    Communicate_Send_Flag_1 |= (0x0001 << 0);
  else
    Communicate_Send_Flag_1 &= ~(0x0001 << 0);

  if (Buff_Flag != 0)
    Communicate_Send_Flag_1 |= (0x0001 << 2);
  else
    Communicate_Send_Flag_1 &= ~(0x0001 << 2);

  if (UD_Chase.updata(YK.shubiao.press_l) == UpDown_check_rising || YK.shubiao.press_l)
  {
    Chase_Flag = 1;
    Chase_Time = 0;
  }

  if (UD_yaogan.updata(YK.yaogan.v > 600) == UpDown_check_rising && YK_Mode == XTL_MODE)
  {
    XTL_SPEED_FLAG += 2;
    if (XTL_SPEED_FLAG >= 20)
      XTL_SPEED_FLAG = 2;
    SpeedChange = (XTL_SPEED_FLAG - 8) * 50;
  }

  if (UD_SpeedUp.updata(YK.Pressed_Check(KEY_PRESSED_C)) == UpDown_check_rising && (YK.jianpan & KEY_PRESSED_CTRL))
  {
    if (SpeedChange < 500)
      SpeedChange += 100;
  }

  if (UD_SpeedDown.updata(YK.Pressed_Check(KEY_PRESSED_C)) == UpDown_check_rising && !(YK.jianpan & KEY_PRESSED_CTRL))
  {
    if (SpeedChange > -200)
      SpeedChange -= 100;
  }

  if (YK_Mode == PLAYER_MODE)
  {
    UD_E_buf = UD_E.updata(Buff_Flag);
    if (UD_E_buf == UpDown_check_rising)
    {
      E_Flag = 1;
      E_Char[0] = Color_Pink;
    }
    else if (UD_E_buf == UpDown_check_falling)
    {
      E_Flag = 1;
      E_Char[0] = Color_Yellow;
    }
  }

  XTLkeyboarddeal();

  if (YK.Pressed_Check(KEY_PRESSED_Z) && YK.Pressed_Check(KEY_PRESSED_CTRL))
  {
    Reset_flag = 1;
    HAL_Delay(10);
    for (uint8_t temp = 0; temp < 20; temp++)
    {
      M3508_MOTOR_QZ_sp.PID_update(0, M3508_MOTOR_QZ.sp);
      M3508_MOTOR_HZ_sp.PID_update(0, M3508_MOTOR_HZ.sp);
      M3508_MOTOR_HY_sp.PID_update(0, M3508_MOTOR_HY.sp);
      M3508_MOTOR_QY_sp.PID_update(0, M3508_MOTOR_QY.sp);
      CAN_Motor.Send_RM(0x200, M3508_MOTOR_QZ_sp.OUT_PID, M3508_MOTOR_HY_sp.OUT_PID, M3508_MOTOR_QY_sp.OUT_PID, M3508_MOTOR_HZ_sp.OUT_PID);
      HAL_Delay(10);
    }
    CAN_Motor.Send_RM(0x200, 0, 0, 0, 0);
    HAL_Delay(100);

    __set_FAULTMASK(1);
    NVIC_SystemReset();
  }
}

void App_Chassis_Init(void)
{
  HAL_Delay(2000);
  CAN_Motor.Init(0, 0);
  HAL_Delay(5);
  CAN_Communicate.Init(1, 1);
  HAL_Delay(10);
  CP_System_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim7);
}

void App_Chassis_Loop(void)
{
  MODE_DEAL();
  YAW_Error = 0;
  Key_deal();
  cp_state = Judge_IF_DUM_Normal();
}

void App_Chassis_CAN1_RxFifo0Callback(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
  if (CAN_Motor.Receive(&hcan1) == HAL_OK)
  {
    if (M3508_MOTOR_QZ.update() == HAL_OK)
    {
      M3508_1++;
      M3508_MOTOR_QZ_sp.PID_update(DP.QXL.qz, M3508_MOTOR_QZ.sp);
      Chassic_3508_Flag |= 0x01;
    }
    if (M3508_MOTOR_HZ.update() == HAL_OK)
    {
      M3508_2++;
      M3508_MOTOR_HZ_sp.PID_update(DP.QXL.hz, M3508_MOTOR_HZ.sp);
      Chassic_3508_Flag |= 0x02;
    }
    if (M3508_MOTOR_HY.update() == HAL_OK)
    {
      M3508_3++;
      M3508_MOTOR_HY_sp.PID_update(DP.QXL.hy, M3508_MOTOR_HY.sp);
      Chassic_3508_Flag |= 0x04;
    }
    if (M3508_MOTOR_QY.update() == HAL_OK)
    {
      M3508_4++;
      M3508_MOTOR_QY_sp.PID_update(DP.QXL.qy, M3508_MOTOR_QY.sp);
      Chassic_3508_Flag |= 0x08;
    }
  }
}

void App_Chassis_CAN2_RxFifo1Callback(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
  if (CAN_Communicate.Receive(&hcan2) == HAL_OK)
  {
    Communicate_deal();

    if (YAW.DM_update() == HAL_OK)
    {
      theta_rad = YAW.mang - Motor_Yaw_front;
      sin_theta = sinf(theta_rad);
      cos_theta = cosf(theta_rad);

      if (cos_theta > 0)
        PID_Chassis.PID_update(0, sin_theta * 70);
      else
        PID_Chassis.PID_update(0, -sin_theta * 70);
    }

    switch (YK_Mode)
    {
      case ONLY_CHASSIC:
      case FAST_CHASSIC:
        DP.QXL_Data_Deal(Chassic_Ch0_Real, Chassic_Ch1_Real, Chassic_Ch2_Real, 0.0f, 8000);
        break;
      case CONTROL_MODE:
        QXL_Field_Data_Deal(Chassic_Ch0_Real, Chassic_Ch1_Real, PID_Chassis.OUT_PID, 8000);
        break;
      case XTL_MODE:
      {
        f xtl_dir = (YK.yaogan.ch3 <= -600) ? 1.0f : -1.0f;
        QXL_Field_Data_Deal(Chassic_Ch0_Real, Chassic_Ch1_Real, xtl_dir * XTL_RANDOM_SPEED_OUT, 8000);
        break;
      }
      case PLAYER_MODE:
        PLAYER_deal();
        break;
      default:
        DP.QXL_Data_Deal(0, 0, 0, 0.0f, 0);
        break;
    }
  }
}

static void UI_TIM2_deal(void)
{
  const float ui_theta = -theta_rad;
  const float ui_sin_theta = sinf(ui_theta);
  const float ui_cos_theta = cosf(ui_theta);

  sin_theta_L = sinf(ui_theta - PI / 2.0f);
  cos_theta_L = cosf(ui_theta - PI / 2.0f);
  sin_theta_down = sinf(ui_theta - PI);
  cos_theta_down = cosf(ui_theta - PI);
  sin_theta_R = sinf(ui_theta + PI / 2.0f);
  cos_theta_R = cosf(ui_theta + PI / 2.0f);

  d2[6][1] = (uint16_t)(410 + (1 + ui_sin_theta) * 70);
  d2[7][1] = (uint16_t)(680 + (1 + ui_cos_theta) * 70);

  d2[3][3] = (uint16_t)(410 - cater_RX + (1 - ui_sin_theta) * 70);
  d2[4][3] = (uint16_t)(680 + cater_RY + (1 - ui_cos_theta) * 70);
  d2[6][3] = (uint16_t)(410 + cater_RX + (1 - ui_sin_theta) * 70);
  d2[7][3] = (uint16_t)(680 - cater_RY + (1 - ui_cos_theta) * 70);

  d2[3][4] = (uint16_t)(410 - cater_RX + (1 - sin_theta_L) * 70);
  d2[4][4] = (uint16_t)(680 + cater_RY + (1 - cos_theta_L) * 70);
  d2[6][4] = (uint16_t)(410 + cater_RX + (1 - sin_theta_L) * 70);
  d2[7][4] = (uint16_t)(680 - cater_RY + (1 - cos_theta_L) * 70);

  d2[3][5] = (uint16_t)(410 - cater_RX + (1 - sin_theta_down) * 70);
  d2[4][5] = (uint16_t)(680 + cater_RY + (1 - cos_theta_down) * 70);
  d2[6][5] = (uint16_t)(410 + cater_RX + (1 - sin_theta_down) * 70);
  d2[7][5] = (uint16_t)(680 - cater_RY + (1 - cos_theta_down) * 70);

  d2[3][6] = (uint16_t)(410 - cater_RX + (1 - sin_theta_R) * 70);
  d2[4][6] = (uint16_t)(680 + cater_RY + (1 - cos_theta_R) * 70);
  d2[6][6] = (uint16_t)(410 + cater_RX + (1 - sin_theta_R) * 70);
  d2[7][6] = (uint16_t)(680 - cater_RY + (1 - cos_theta_R) * 70);

  static uint32_t ten_flag = 0;
  if (ten_flag > 600)
    ten_flag = 1;
  if (Q_Flag)
  {
    CP_DrawOrDelete_Char(35, Modify_Graphic, 0, (Color_tpyedef)Q_Char[0], Q_Char[1], Q_Char[2], Q_Char[3], Q_Char[4], (uint8_t *)"Q");
    Q_Flag++;
    if (Q_Flag > 3)
      Q_Flag = 0;
  }
  else if (E_Flag)
  {
    CP_DrawOrDelete_Char(36, Modify_Graphic, 0, (Color_tpyedef)E_Char[0], E_Char[1], E_Char[2], E_Char[3], E_Char[4], (uint8_t *)"V");
    E_Flag++;
    if (E_Flag > 3)
      E_Flag = 0;
  }
  else if (MID_Flag)
  {
    CP_DrawOrDelete_Char(38, Modify_Graphic, 0, (Color_tpyedef)MID_Char[0], MID_Char[1], MID_Char[2], MID_Char[3], MID_Char[4], (uint8_t *)"MID");
    MID_Flag++;
    if (MID_Flag > 3)
      MID_Flag = 0;
  }
  else if (ten_flag % 30 == 0)
  {
    switch (UI_step_ten)
    {
      case 1:
        CP_DrawOrDelete_Seven_Graphic(name4, Increase_Graphic, graphic_tpye4, 1, color4, d4[0], d4[1], d4[2], d4[3], d4[4], d4[5], d4[6], d4[7]);
        UI_step_ten++;
        break;
      case 2:
        CP_DrawOrDelete_Five_Graphic(name1, Increase_Graphic, graphic_tpye1, 2, color1, d1[0], d1[1], d1[2], d1[3], d1[4], d1[5], d1[6], d1[7]);
        UI_step_ten++;
        break;
      case 3:
        CP_DrawOrDelete_Seven_Graphic(name2, Increase_Graphic, graphic_tpye2, 2, color2, d2[0], d2[1], d2[2], d2[3], d2[4], d2[5], d2[6], d2[7]);
        UI_step_ten++;
        break;
      case 4:
        CP_DrawOrDelete_One_Number(33, Increase_Graphic, Graphic_Int_number, 0, Color_Green, 25, 1, 5, 1150, 600, CP.ext_shoot_data_t.bullet_speed);
        UI_step_ten++;
        break;
      case 5:
        CP_DrawOrDelete_One_Number(34, Increase_Graphic, Graphic_Int_number, 0, Color_Green, 25, 1, 5, 1150, 550, Gimbal_Roll);
        UI_step_ten++;
        break;
      case 6:
        CP_DrawOrDelete_Char(35, Increase_Graphic, 0, (Color_tpyedef)Q_Char[0], Q_Char[1], Q_Char[2], Q_Char[3], Q_Char[4], (uint8_t *)"Q");
        UI_step_ten++;
        break;
      case 7:
        CP_DrawOrDelete_Char(36, Increase_Graphic, 0, (Color_tpyedef)E_Char[0], E_Char[1], E_Char[2], E_Char[3], E_Char[4], (uint8_t *)"V");
        UI_step_ten++;
        break;
      case 8:
        CP_DrawOrDelete_One_Number(32, Increase_Graphic, Graphic_Float_number, 0, Color_Green, 25, 1, 5, 1150, 500, V_Cap_Real);
        UI_step_ten++;
        break;
      case 9:
        CP_DrawOrDelete_Two_Graphic(name5, Increase_Graphic, graphic_tpye5, 2, color5, d5[0], d5[1], d5[2], d5[3], d5[4], d5[5], d5[6], d5[7]);
        UI_step_ten++;
        break;
      case 10:
        CP_DrawOrDelete_Char(38, Increase_Graphic, 0, (Color_tpyedef)MID_Char[0], MID_Char[1], MID_Char[2], MID_Char[3], MID_Char[4], (uint8_t *)"MID");
        UI_step_ten = 1;
        break;
      default:
        CP_DrawOrDelete_Five_Graphic(name1, Increase_Graphic, graphic_tpye1, 2, color1, d1[0], d1[1], d1[2], d1[3], d1[4], d1[5], d1[6], d1[7]);
        UI_step_ten = 1;
        break;
    }
  }
  else
  {
    switch (UI_step)
    {
      case 1:
        CP_DrawOrDelete_Seven_Graphic(name2, Modify_Graphic, graphic_tpye2, 2, color2, d2[0], d2[1], d2[2], d2[3], d2[4], d2[5], d2[6], d2[7]);
        UI_step++;
        break;
      case 2:
        CP_DrawOrDelete_One_Number(32, Modify_Graphic, Graphic_Float_number, 0, Color_Green, 25, 1, 5, 1150, 500, V_Cap_Real);
        UI_step++;
        break;
      case 3:
        CP_DrawOrDelete_Seven_Graphic(name2, Modify_Graphic, graphic_tpye2, 2, color2, d2[0], d2[1], d2[2], d2[3], d2[4], d2[5], d2[6], d2[7]);
        UI_step++;
        break;
      case 4:
      {
        static uint8_t count_g = 0;
        if (count_g % 2 == 0)
        {
          count_g = 1;
          CP_DrawOrDelete_One_Number(33, Modify_Graphic, Graphic_Int_number, 0, Color_Green, 25, 1, 5, 1150, 600, CP.ext_shoot_data_t.bullet_speed);
        }
        else
        {
          count_g = 2;
          CP_DrawOrDelete_One_Number(34, Modify_Graphic, Graphic_Int_number, 0, Color_Green, 25, 1, 5, 1150, 550, Gimbal_Roll);
        }
        UI_step++;
        break;
      }
      case 5:
        CP_DrawOrDelete_Seven_Graphic(name2, Modify_Graphic, graphic_tpye2, 2, color2, d2[0], d2[1], d2[2], d2[3], d2[4], d2[5], d2[6], d2[7]);
        UI_step = 1;
        break;
      default:
        UI_step = 1;
        break;
    }
  }
  ten_flag++;
}

void App_Chassis_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3)
  {
    KEY_Forback_Ctrl1();
    Chassic_Forback_Ctrl();
    if (Motor_Flag.TIM3_Flag == 0)
    {
      M3508_Limit_deal();
      Motor_Flag.TIM3_Flag = 1;
    }
    else
    {
      Motor_Flag.TIM3_Flag = 0;
    }
  }

  if (htim == &htim5)
  {
    // Publish speed and mechanical phase at 200 Hz.  The phase lets the
    // gimbal use different feed-forward gains for the front and rear halves
    // of one chassis rotation.
    const bool spin_enabled = Chassis_spin_feedforward_enabled();
    const int16_t spin_speed = Chassis_spin_feedforward_speed();
    const int16_t spin_phase_sin = spin_enabled ?
        (int16_t)(sin_theta * CHASSIS_SPIN_PHASE_SCALE) : 0;
    const int16_t spin_phase_cos = spin_enabled ?
        (int16_t)(cos_theta * CHASSIS_SPIN_PHASE_SCALE) : (int16_t)CHASSIS_SPIN_PHASE_SCALE;
    CAN_Communicate.Send_RM(CHASSIS_SPIN_FF_CAN_ID, spin_speed,
                            spin_enabled ? 1 : 0,
                            spin_phase_sin, spin_phase_cos);

    static uint8_t CNC_flag = 1;
    if (CNC_flag)
    {
      CAN_Communicate.Send_RM(0x558, Communicate_Send_Flag_1,
                              0,
                              CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_heat_limit,
                              CP.ext_power_heat_data_t.shooter_id1_17mm_cooling_heat);
      CNC_flag = 0;
    }
    else
    {
      CAN_Communicate.Send_RM(0x121, CP.ext_game_robot_status_t.chassis_power_limit,
                              1,
                              0,
                              0);
      if(CP.ext_game_robot_status_t.chassis_power_limit == 0) CP.ext_game_robot_status_t.chassis_power_limit = 65;
      CNC_flag = 1;
    }
  }

  if (htim == &htim2)
  {
    UI_TIM2_deal();
  }
}

void App_Chassis_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &CP_SYSTEM_USART_HANDLE)
  {
    CP_System_DMACplt_DataDeal();
  }
}
