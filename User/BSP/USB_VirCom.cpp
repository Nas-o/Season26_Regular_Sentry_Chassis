/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2025-12-24 00:06:06
 * @FilePath: \Reserve_Sentry\User\BSP\USB_VirCom.c
 */
/**
 * @file USB_VirCom.c
 * @author sethome
 * @brief 虚拟串口数据发送
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "usbd_cdc_if.h"
#include "USB_VirCom.hpp"
#include "CRC8_CRC16.hpp"
#include "Stm32_time.hpp"
#include "fifo.hpp"

#include "Global_status.hpp"
#include "Auto_control.hpp"



void Vircom_Send(uint8_t data[], uint16_t len)
{
/*   if (CDC_Transmit_HS(data, len) == 1) // 判断数据是否发送
  {
    // USB忙碌数据转入缓冲区

    fifo_s_puts(&USB_send_fifo, (char *)data, (int)len);
  } */
  CDC_Transmit_HS(data, len);
}

void Vircom_Rev(uint8_t data[], uint16_t len)
{
/*   if(data[0]==0xff)
  {
    Global.Auto.input.Auto_control_online=1;
    decodeMINIPCdata(&fromMINIPC,data,len);
    MINIPC_to_STM32();
  } */
/*     if (data[0] == 0xAA && data[1] == 19)
    {
        if (data[18] == 0x01 || data[18] == 0x00)
        {
          decodeNAVdata(&Navigation_receive_1, data, 19);
        }
    } */
   if(data[0] == 'S' && data[1] == 'P')
   {
      decodeMINIPCdata(&fromMINIPC,data,len);
      MINIPC_to_STM32();
   }
}

#include "stdio.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  Vircom_Send((uint8_t *)&ch, 1);

  return ch;
}