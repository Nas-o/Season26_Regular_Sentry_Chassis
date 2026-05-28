/**
 * @file Stm32_time.h
 * @author sethome 
 * @brief STM32F4的开机时间计算
 * @version 0.1
 * @date 2022-11-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#define __STM32_TIME_H
#ifdef __STM32_TIME_H

#include "stm32h7xx_hal.h"
#include "stdint.h"

// 获取系统时间
uint32_t Get_SysTime_ms(void); // recommend
uint32_t Get_SysTime_us(void);

float Get_SysTime_s(void);

void TIM_Count_100kHz(void);
#endif
// end of file
