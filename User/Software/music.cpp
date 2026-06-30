/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-06-03 14:15:36
 * @FilePath: \Season26_Regular_Sentry_Chassis\User\Software\music.cpp
 */
/**
 * @file music.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief 音乐播放模块
 * @version 0.1
 * @date 2025-04-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "music.hpp"

// ===== 《两只老虎》旋律数组 =====
Note melody[] = {
    // 第一段（中音区）
    {kMidDo, 500},
    {kMidRe, 500},
    {kMidMi, 500},
    {kMidDo, 500}, // 1 2 3 1
    {kMidDo, 500},
    {kMidRe, 500},
    {kMidMi, 500},
    {kMidDo, 500}, // 1 2 3 1
    {kMidMi, 500},
    {kMidFa, 500},
    {kMidSol, 1000}, // 3 4 5 -
    {kMidMi, 500},
    {kMidFa, 500},
    {kMidSol, 1000}, // 3 4 5 -

    // 第二段（中音区 + 高音区）
    {kMidSol, 250},
    {kMidLa, 250},
    {kMidSol, 250},
    {kMidFa, 250},
    {kMidMi, 500},
    {kMidDo, 500}, // 5 6 5 4 3 1
    {kMidSol, 250},
    {kMidLa, 250},
    {kMidSol, 250},
    {kMidFa, 250},
    {kMidMi, 500},
    {kMidDo, 500}, // 5 6 5 4 3 1

    // 结尾部分（中音Do + 高音Sol）
    {kMidDo, 500},
    {kHighSol, 500},
    {kMidDo, 1000}, // 1 5 1 -
    {kMidDo, 500},
    {kHighSol, 500},
    {kMidDo, 1000}, // 1 5 1 -
    {kMusicFinish, 500},
};

void Music_Init()
{
    Buzzer_Init();
}

void Music_Play(Note *note)
{
    for (int i = 0; note[i].fre != kMusicFinish; i++)
    {
        MUSIC_SET_FREQUENCY(note[i].fre);
        MUSIC_ON();
        MUSIC_DELAY(note[i].time);
    }
    MUSIC_OFF();
}
