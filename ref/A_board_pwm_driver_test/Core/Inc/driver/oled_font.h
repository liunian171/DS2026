/**
 * @file    oled_font.h
 * @brief   字库声明 — 8×16 ASCII + 16×16 汉字
 */

#ifndef __OLED_FONT_H__
#define __OLED_FONT_H__

#include <stdint.h>

/* ---- ASCII 8×16 字库（参考 1-4 OLED驱动函数模块）---- */
/* 95 字符 (0x20~0x7E)，每个 16 字节，下标 = (ch - 0x20) × 16 */
extern const uint8_t OLED_F8x16[][16];

/* ---- 汉字 16×16 字库 — 流年 ---- */
#define CN_INDEX_LIU  0
#define CN_INDEX_NIAN 1
extern const uint8_t font_cn_16x16[2][32];

#endif
