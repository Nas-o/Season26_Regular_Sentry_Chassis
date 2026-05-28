#include "remote_control.hpp"
#include "Global_status.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "ramp_generator.hpp"
#include "referee_system.hpp"

#include "DT7.hpp"
#include "VT13.hpp"
#include "FSI6X.hpp"
#include "Power_switch.hpp"
#include "IMU_updata.hpp"

#include "Stm32_time.hpp"
#include "Power_switch.hpp"

#include "cmsis_os2.h"




/*----------------------------------- 按键消抖 --------------------------------------*/

int16_t wait_time[SIZE_OF_WAIT] = {0}; // 键盘消抖用时间

/**
 * @brief 统一消抖
 *
 * @param key 按键宏
 * @return uint8_t 0未到时间，1到时间
 */
uint8_t Wait(uint8_t key)
{
    if (wait_time[key] >= 2)
    {
        wait_time[key]--;
        return 0;
    }
    else if ((wait_time[key] == 0) || (wait_time[key] == 1))
    {
        wait_time[key]--;
        return 1;
    }
    else
    {
        return 0;
    }
}

void SetWait(uint8_t key)
{
    wait_time[key] = 300;
}


/*----------------------------------- 遥控数据更新 --------------------------------------*/
/**
 * @brief 遥控数据来源于DT7遥控器
 *
 */
void DT7toRCdata()
{
    /*遥控器数据*/
    RC_data.rc.ch[0] = DT7_data.rc.ch[0];
    RC_data.rc.ch[1] = DT7_data.rc.ch[1];
    RC_data.rc.ch[2] = DT7_data.rc.ch[2];
    RC_data.rc.ch[3] = DT7_data.rc.ch[3];
    RC_data.rc.ch[4] = DT7_data.rc.ch[4];
    RC_data.rc.s[0] = DT7_data.rc.s[0];
    RC_data.rc.s[1] = DT7_data.rc.s[1];
    /*键鼠数据 */
    RC_data.key.v = DT7_data.key.v;
    RC_data.mouse.x = DT7_data.mouse.x;
    RC_data.mouse.y = DT7_data.mouse.y;
    RC_data.mouse.z = DT7_data.mouse.z;
    RC_data.mouse.press_l = DT7_data.mouse.press_l;
    RC_data.mouse.press_r = DT7_data.mouse.press_r;
    RC_data.mouse.press_mid = 0;
    DT7_data.online--;
    RC_data.online = DT7_data.online;
}

/**
 * @brief 来自图传的遥控数据
 *
 */
void VT13toRCdata()
{
    /*遥控器数据*/
    RC_data.rc.ch[0] = VT13_data.rc.ch[0];
    RC_data.rc.ch[1] = VT13_data.rc.ch[1];
    RC_data.rc.ch[2] = VT13_data.rc.ch[3];
    RC_data.rc.ch[3] = VT13_data.rc.ch[2];
    if (VT13_data.rc.shutter == 1) // 扳机键与开火相对应
        RC_data.rc.ch[4] = 660;
    else
        RC_data.rc.ch[4] = 0;
    // 挡位与拨杆映射
    if (VT13_data.rc.mode_sw == 1) // N
        RC_data.rc.s[0] = RC_SW_MID;
    if (VT13_data.rc.mode_sw == 0) // C
        RC_data.rc.s[0] = RC_SW_DOWN;
    if (VT13_data.rc.mode_sw == 2) // S
    {
        RC_data.rc.s[0] = RC_SW_UP;
        RC_data.rc.s[1] = RC_SW_UP;
    }
    // 滚轮与拨杆映射
    if (VT13_data.rc.wheel < -330)
        RC_data.rc.s[1] = RC_SW_DOWN;
    if ((VT13_data.rc.wheel > -330) && (VT13_data.rc.wheel < 330) && (VT13_data.rc.mode_sw != 2))
        RC_data.rc.s[1] = RC_SW_MID;
    if (VT13_data.rc.wheel >= 330)
        RC_data.rc.s[1] = RC_SW_UP;
    if (VT13_data.rc.left_button == 1)
        Power_TurnOff(power2);
    else
        Power_TurnOn(power2);
    /* if (VT13_data.rc.right_button == 1)
        GIMBALMotor_setzero(YAW_ZERO + 135.0f, YAWMotor);
    else
        GIMBALMotor_setzero(YAW_ZERO, YAWMotor); */

    /*键鼠数据 */
    RC_data.key.v = VT13_data.key.v;
    RC_data.mouse.x = VT13_data.mouse.x;
    RC_data.mouse.y = VT13_data.mouse.y;
    RC_data.mouse.z = VT13_data.mouse.z;
    RC_data.mouse.press_l = VT13_data.mouse.press_l;
    RC_data.mouse.press_r = VT13_data.mouse.press_r;
    RC_data.mouse.press_mid = VT13_data.mouse.middle;
    VT13_data.online--;
    RC_data.online = VT13_data.online;
}

/**
 * @brief 遥控数据来源于FS_I6X遥控器
 *
 */
void FSI6XtoRCdata()
{ 
   RC_data.rc.ch[0]=FSI6X_data.CH1;
   RC_data.rc.ch[1]=FSI6X_data.CH2;
   RC_data.rc.ch[2]=FSI6X_data.CH4;
   RC_data.rc.ch[3]=FSI6X_data.CH3;

   for(int i = 0 ; i<4 ; i++) // 死区判断
   {
    if(fabs(RC_data.rc.ch[i]) < 10)
    RC_data.rc.ch[i] = 0;
   }
   
   if(FSI6X_data.CH8==FS_DOWN)//LOCK
   {
    RC_data.rc.s[0]=RC_SW_DOWN;
    RC_data.rc.s[1]=RC_SW_DOWN;
   }
   else if (FSI6X_data.CH8==FS_UP)
   {
     if (FSI6X_data.CH7==FS_MID && FSI6X_data.CH6 == FS_UP)// 正小陀螺
     {
        RC_data.rc.s[0]=RC_SW_UP ;
        RC_data.rc.s[1]=RC_SW_MID;
     }
     else if (FSI6X_data.CH7==FS_DOWN && FSI6X_data.CH6 == FS_UP)// 逆小陀螺
     {
        RC_data.rc.s[1]=RC_SW_DOWN;
        RC_data.rc.s[0]=RC_SW_MID;
     }
     else if(FSI6X_data.CH6 == FS_DOWN && FSI6X_data.CH7 == FS_UP)// 开启摩擦轮
     {
       RC_data.rc.s[1]=RC_SW_UP; 
       RC_data.rc.s[0]=RC_SW_MID;
       
     }
     else
     {
       RC_data.rc.s[1]=RC_SW_MID; 
       RC_data.rc.s[0]=RC_SW_MID;
     }
     if(FSI6X_data.CH5 == FS_DOWN) // 开火
       {
        RC_data.rc.ch[4] = 660;
       }
       else if(FSI6X_data.CH5 == FS_UP)
       {
        RC_data.rc.ch[4] = 0;
       }
   }
   FSI6X_data.online--;
   RC_data.online = FSI6X_data.online;
}

/**
 * @brief 根据数据来源更新遥控数据
 *
 */
void RCdata_Updater()
{
    if (DT7_data.online >= 0)
      DT7toRCdata();
    else if (FSI6X_data.online >= 0)
      FSI6XtoRCdata();
    else if (VT13_data.online >= 0)
      VT13toRCdata(); 
}


/*----------------------------------- 遥控器控制逻辑 --------------------------------------*/
/**
 * @brief 遥控器控制
 *
 */
void RC_Controller()
{
    if (RC_data.online >= 0)
        RC_data.online--;
    /*控制模式选择*/
    if ((RC_data.rc.s[0]==RC_SW_DOWN && RC_data.rc.s[1]==RC_SW_DOWN) || (RC_data.online <= 0)) // 左下右下，锁死
        Global.Control.mode = LOCK;
    else
        Global.Control.mode = RC;
    if (Global.Control.mode != RC)
        return;
    /*底盘控制*/
    if (RC_data.rc.s[0]==RC_SW_UP && RC_data.rc.s[1]==RC_SW_MID) // 右上,左中||左上，正小陀螺
        Global.Chassis.mode = SPIN_P;
    else if (RC_data.rc.s[1]==RC_SW_DOWN && RC_data.rc.s[0]==RC_SW_MID ) // 左下,右中||右上，負小陀螺
        Global.Chassis.mode = SPIN_N;
    else
        Global.Chassis.mode = FLOW_Chassis;
    Chassis_SetX(RC_data.rc.ch[0] / 40.0f);
    Chassis_SetY(RC_data.rc.ch[1] / 40.0f);
    /*云台控制*/
    Gimbal_SetPitchAngle(Global.Gimbal.input.pitch + RC_data.rc.ch[3] / 2000.0f);
    Gimbal_SetYawAngle(Global.Gimbal.input.yaw - RC_data.rc.ch[2] / 2000.0f);
    /*发弹机构控制*/
    if (RC_data.rc.s[1]==RC_SW_UP && (RC_data.rc.s[0]==RC_SW_MID || RC_data.rc.s[0]==RC_SW_DOWN)) // 左上，右中||右下，开启摩擦轮
        Global.Shoot.shoot_mode = READY;
    else
        Global.Shoot.shoot_mode = CLOSE;
    if (RC_data.rc.ch[4] >= 300 &&
        RC_data.rc.ch[4] <= 660 &&
        Global.Shoot.shoot_mode != CLOSE)
        Global.Shoot.trigger_mode = HIGH;
    else if (RC_data.rc.ch[4] >= 50 &&
             RC_data.rc.ch[4] <= 300 &&
             Global.Shoot.shoot_mode != CLOSE)
        Global.Shoot.trigger_mode = LOW;
    else if (RC_data.rc.ch[4] > 660 &&
             Global.Shoot.shoot_mode != CLOSE)
    {
        Global.Shoot.shoot_mode = DEBUG_SHOOT;
        Global.Shoot.trigger_mode = DEBUG_TRIGGER;
    }
    else
        Global.Shoot.trigger_mode = TRIGGER_CLOSE;
}

/*----------------------------------- 键鼠控制逻辑 --------------------------------------*/
/**
 * @brief 遥控器控制
 *
 */
void Keyboard_MouseController(void)
{
}


/*------------------------------------ Task -------------------------------------*/

/**
 * @brief          遥控任务
 * @param          none
 * @retval         none
 */
void Remote_Tasks(void)
{
	// 遥控数据更新
	RCdata_Updater();
    // 遥控器控制
    RC_Controller();
    // 键鼠控制
    Keyboard_MouseController();
}