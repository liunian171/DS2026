/**
 * @file    oled_driver.cpp
 * @brief   SSD1315 OLED 驱动实现 — 8×16 ASCII + 16×16 汉字
 *
 * @note I2C 适配说明
 * ───────────────────────────────────────────────────────────
 * 本工程使用软件 I2C（useri2c.h 的 I2C_Handle + ops 表）。
 * Bluepill 移植版改为直接调 HAL_I2C_Mem_Write（硬件 I2C2），
 * 等 i2c_hardware_ops.c 实现后应改回 ops 接口以保持一致性。
 * ───────────────────────────────────────────────────────────
 */

#include "oled_driver.h"
#include "oled_font.h"
#include "useri2c.h"
#include "stm32f4xx_hal.h"

extern I2C_Handle g_i2c_dev;

static const uint8_t kOledAddr = 0x3C;

/* ---------- 回调 ---------- */
static volatile uint8_t g_done;
static int8_t           g_result;

static void oled_cb(void *ctx, int8_t result) {
    (void)ctx;
    g_result = result;
    g_done   = 1;
}

static void oled_wait(void) {
    uint32_t timeout = 500 * 100000;
    while (!g_done && --timeout) { }
    if (!g_done) g_result = -2;
    g_done = 0;
}

static void write(uint8_t reg, const uint8_t *data, uint16_t len) {
    g_done = 0;
    i2c_write_reg_async(&g_i2c_dev, kOledAddr, reg,
                        (uint8_t *)data, len, oled_cb);
    oled_wait();
}

/* ========================================================================== */

void OledDriver::write_cmd(uint8_t cmd)        { write(0x00, &cmd, 1); }
void OledDriver::write_cmd_multi(const uint8_t *cmds, uint16_t len) { write(0x00, cmds, len); }
void OledDriver::write_data(uint8_t data)       { write(0x40, &data, 1); }

void OledDriver::set_pos(uint8_t page, uint8_t col_byte) {
    uint8_t cmd[3] = {
        (uint8_t)(0xB0 + page),
        (uint8_t)(col_byte & 0x0F),
        (uint8_t)(0x10 + (col_byte >> 4)),
    };
    write_cmd_multi(cmd, 3);
}

/* ========================================================================== */

OledDriver::OledDriver() {}

int8_t OledDriver::init() {
    const uint8_t cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00,
        0x40, 0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8,
        0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB,
        0x40, 0xA4, 0xA6, 0xAF,
    };
    write_cmd_multi(cmds, sizeof(cmds));
    clear();
    return 0;
}

void OledDriver::clear() {
    for (uint8_t pg = 0; pg < 8; pg++) {
        set_pos(pg, 0);
        for (uint16_t i = 0; i < 128; i++) write_data(0x00);
    }
}

/* ==========================================================================
 *  ASCII 8×16 字符显示
 *
 *  每个字符 16 字节:
 *     [0..7]  = 上半部分 (8 列)
 *     [8..15] = 下半部分 (8 列)
 *
 *  row(0~3) 对应:
 *     上 page = row * 2
 *     下 page = row * 2 + 1
 * ========================================================================== */

void OledDriver::show_string(uint8_t row, uint8_t col, const char *str) {
    if (row > 3 || !str) return;

    while (*str && col < 16) {
        uint8_t idx = (uint8_t)(*str) - 0x20;
        if (idx > 94) { str++; continue; }

        uint8_t byte_col = col * 8;   // 每个字符 8 列像素

        /* ---- 上半（row × 2）---- */
        set_pos(row * 2, byte_col);
        write(0x40, OLED_F8x16[idx], 8);           // 前 8 字节

        /* ---- 下半（row × 2 + 1）---- */
        set_pos(row * 2 + 1, byte_col);
        write(0x40, OLED_F8x16[idx] + 8, 8);       // 后 8 字节

        col++;
        str++;
    }
}

/* ==========================================================================
 *  16×16 汉字显示 — 占用两行 (上+下)
 * ========================================================================== */

void OledDriver::show_chinese(uint8_t row, uint8_t col, uint8_t index) {
    if (row > 2 || col > 14) return;  // col 在 ASCII 8 像素单位下
    if (index > 1) return;

    uint8_t byte_col = col * 8;  // 汉字宽 16 像素 = 2 个 ASCII 列

    /* 上半：row */
    set_pos(row * 2, byte_col);
    write(0x40, font_cn_16x16[index], 16);         // 前 16 字节

    /* 下半：row + 1 */
    set_pos(row * 2 + 1, byte_col);
    write(0x40, font_cn_16x16[index] + 16, 16);    // 后 16 字节
}
