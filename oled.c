/*
 * oled.c - SSD1306 128x64 驱动（I2C1：PB2=SCL / PB3=SDA，地址 0x3C）
 *
 * 三个约束决定了写法：
 *   1. MSPM0 的 I2C TX FIFO 只有 8 字节，所以每次事务最多「1 控制字节 + 7 数据」，
 *      这里按「1 控制字节 + 4 列」分包，留足余量；
 *   2. 主循环 10 ms 一拍，不能整屏刷新（128x64 = 1 KB ≈ 23 ms），
 *      所以只重画变化的字符，且每拍最多一个，实测一帧约 1 ms；
 *   3. 这些小函数必须 noinline —— 否则 LLVM 会把 put_uint/set_line 复制到每个
 *      调用点，OLED 驱动会从 2 KB 膨胀到 6.8 KB。
 */
#include "ti_msp_dl_config.h"
#include "oled.h"
#include "car_config.h"

#define OLED_NOINLINE    __attribute__((noinline))

#define OLED_ADDR        (0x3CU)
#define OLED_COLS        (16U)      /* 128 / 8 */
#define OLED_LINES       (4U)       /* 64 / 16 */
#define I2C_CHUNK        (4U)       /* 每包 4 列：1 + 4 = 5 字节 < FIFO 8 */

/* 标准 128x64 初始化序列（页寻址模式） */
static const uint8_t k_init_seq[] = {
    0xAE,             /* display off */
    0xD5, 0x80,       /* clock div */
    0xA8, 0x3F,       /* multiplex 64 */
    0xD3, 0x00,       /* display offset */
    0x40,             /* start line 0 */
    0x8D, 0x14,       /* charge pump on */
    0x20, 0x00,       /* page addressing */
    0xA1,             /* segment remap */
    0xC8,             /* COM scan dec */
    0xDA, 0x12,       /* COM pins */
    0x81, 0xCF,       /* contrast */
    0xD9, 0xF1,       /* precharge */
    0xDB, 0x40,       /* VCOMH */
    0xA4,             /* resume RAM content */
    0xA6,             /* normal (not inverted) */
    0xAF              /* display on */
};

static char    s_want[OLED_LINES][OLED_COLS];
static char    s_disp[OLED_LINES][OLED_COLS];
static bool    s_dirty[OLED_LINES];
static bool    s_online;
static uint8_t s_scan;
static uint8_t s_pos;

/* ---------------- 底层 I2C ---------------- */

static OLED_NOINLINE bool i2c_write(const uint8_t *buf, uint8_t len)
{
    uint32_t guard;

    if (!s_online) {
        return false;
    }

    for (guard = 0U; guard < 20000U; guard++) {
        if ((DL_I2C_getControllerStatus(OLED_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            break;
        }
    }
    if (guard >= 20000U) {
        s_online = false;
        return false;
    }

    (void)DL_I2C_fillControllerTXFIFO(OLED_I2C_INST, (uint8_t *)buf, (uint32_t)len);
    DL_I2C_startControllerTransfer(OLED_I2C_INST, OLED_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, (uint32_t)len);

    for (guard = 0U; guard < 20000U; guard++) {
        if ((DL_I2C_getControllerStatus(OLED_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            break;
        }
    }
    if (guard >= 20000U) {
        s_online = false;
        return false;
    }
    if ((DL_I2C_getControllerStatus(OLED_I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        s_online = false;   /* 没接屏 / 地址不对，之后直接放弃 */
        return false;
    }
    return true;
}

/* 一包命令：0x00 + 最多 7 条 */
static OLED_NOINLINE bool oled_cmd(const uint8_t *cmds, uint8_t n)
{
    uint8_t buf[8];
    uint8_t i;

    if (n > 7U) {
        n = 7U;
    }
    buf[0] = 0x00U;
    for (i = 0U; i < n; i++) {
        buf[i + 1U] = cmds[i];
    }
    return i2c_write(buf, (uint8_t)(n + 1U));
}

static OLED_NOINLINE bool oled_cmd_list(const uint8_t *cmds, uint16_t n)
{
    while (n > 0U) {
        uint8_t step = (n > 7U) ? 7U : (uint8_t)n;

        if (!oled_cmd(cmds, step)) {
            return false;
        }
        cmds += step;
        n = (uint16_t)(n - step);
    }
    return true;
}

/* 设定页 + 列（页寻址模式下写完自动列递增） */
static OLED_NOINLINE bool oled_set_pos(uint8_t page, uint8_t col)
{
    uint8_t cmds[3];

    cmds[0] = (uint8_t)(0xB0U | (page & 0x07U));
    cmds[1] = (uint8_t)(col & 0x0FU);
    cmds[2] = (uint8_t)(0x10U | ((col >> 4) & 0x0FU));
    return oled_cmd(cmds, 3U);
}

/* 一包数据：0x40 + 最多 4 列 */
static OLED_NOINLINE bool oled_data(const uint8_t *data, uint8_t n)
{
    uint8_t buf[I2C_CHUNK + 1U];
    uint8_t i;

    if (n > I2C_CHUNK) {
        n = I2C_CHUNK;
    }
    buf[0] = 0x40U;
    for (i = 0U; i < n; i++) {
        buf[i + 1U] = data[i];
    }
    return i2c_write(buf, (uint8_t)(n + 1U));
}

/* ---------------- 画字符 ---------------- */

static OLED_NOINLINE void oled_draw_char(uint8_t line, uint8_t col, char ch)
{
    const uint8_t *glyph;
    uint8_t page = (uint8_t)(line * 2U);
    uint8_t x    = (uint8_t)(col * 8U);
    uint8_t i;

    if ((ch < 32) || (ch > 126)) {
        ch = ' ';
    }
    glyph = oled_font8x16[(uint8_t)ch - 32U];

    for (i = 0U; i < 8U; i = (uint8_t)(i + I2C_CHUNK)) {
        if (!oled_set_pos(page, (uint8_t)(x + i))) {
            return;
        }
        if (!oled_data(&glyph[i], I2C_CHUNK)) {
            return;
        }
        if (!oled_set_pos((uint8_t)(page + 1U), (uint8_t)(x + i))) {
            return;
        }
        if (!oled_data(&glyph[i + 8U], I2C_CHUNK)) {
            return;
        }
    }
}

/* ---------------- 对外接口 ---------------- */

bool oled_init(void)
{
    uint8_t line;
    uint8_t col;

    s_online = true;
    s_scan   = 0U;
    s_pos    = 0U;

    for (line = 0U; line < OLED_LINES; line++) {
        s_dirty[line] = false;
        for (col = 0U; col < OLED_COLS; col++) {
            s_want[line][col] = ' ';
            s_disp[line][col] = '?';   /* 和 want 不同，保证首次上电全刷一遍 */
        }
    }

    if (!oled_cmd_list(k_init_seq, (uint16_t)sizeof(k_init_seq))) {
        return false;
    }

    for (line = 0U; line < OLED_LINES; line++) {
        s_dirty[line] = true;
    }
    return true;
}

void oled_task(void)
{
    uint8_t n;

    if (!s_online) {
        return;
    }

    /* 轮询找到第一个需要刷新的行 */
    for (n = 0U; n < OLED_LINES; n++) {
        if (s_dirty[s_scan]) {
            break;
        }
        s_scan = (uint8_t)((s_scan + 1U) % OLED_LINES);
    }
    if (!s_dirty[s_scan]) {
        return;
    }

    /* 找该行第一个还没画对的字符 */
    for (s_pos = 0U; s_pos < OLED_COLS; s_pos++) {
        if (s_disp[s_scan][s_pos] != s_want[s_scan][s_pos]) {
            break;
        }
    }
    if (s_pos >= OLED_COLS) {
        s_dirty[s_scan] = false;
        s_scan = (uint8_t)((s_scan + 1U) % OLED_LINES);
        return;
    }

    oled_draw_char(s_scan, s_pos, s_want[s_scan][s_pos]);
    s_disp[s_scan][s_pos] = s_want[s_scan][s_pos];
}

/* ---------------- 四行状态文本 ---------------- */

static OLED_NOINLINE char *put_str(char *p, const char *s)
{
    while (*s != '\0') {
        *p = *s;
        p++;
        s++;
    }
    return p;
}

static OLED_NOINLINE char *put_uint(char *p, uint32_t v)
{
    char tmp[10];
    uint8_t n = 0U;

    if (v == 0U) {
        *p = '0';
        return p + 1;
    }
    while ((v > 0U) && (n < sizeof(tmp))) {
        tmp[n] = (char)('0' + (v % 10U));
        v /= 10U;
        n++;
    }
    while (n > 0U) {
        n--;
        *p = tmp[n];
        p++;
    }
    return p;
}

static OLED_NOINLINE char *put_int(char *p, int32_t v)
{
    if (v < 0) {
        *p = '-';
        p++;
        return put_uint(p, (uint32_t)(-v));
    }
    return put_uint(p, (uint32_t)v);
}

static OLED_NOINLINE char *put_pad(char *p, uint8_t n)
{
    while (n > 0U) {
        *p = ' ';
        p++;
        n--;
    }
    return p;
}

static OLED_NOINLINE uint8_t str_len(const char *s)
{
    uint8_t n = 0U;

    while (s[n] != '\0') {
        n++;
    }
    return n;
}

static OLED_NOINLINE void set_line(uint8_t line, const char *text)
{
    uint8_t col;

    for (col = 0U; col < OLED_COLS; col++) {
        char c = text[col];

        if (c == '\0') {
            break;
        }
        s_want[line][col] = c;
    }
    for (; col < OLED_COLS; col++) {
        s_want[line][col] = ' ';
    }
    s_dirty[line] = true;
}

static OLED_NOINLINE void fmt_uint(char *buf, uint32_t v)
{
    char *p = put_uint(buf, v);

    *p = '\0';
}

static OLED_NOINLINE void fmt_int(char *buf, int32_t v)
{
    char *p = put_int(buf, v);

    *p = '\0';
}

/* 一位小数：tenths = 实际值 × 10，例如 267 表示 26.7 */
static OLED_NOINLINE void fmt_dec1(char *buf, uint32_t tenths)
{
    char *p = put_uint(buf, tenths / 10U);

    *p = '.';
    p++;
    *p = (char)('0' + (tenths % 10U));
    p++;
    *p = '\0';
}

/* 「标签 + 右对齐数字 + 后缀」 */
static OLED_NOINLINE void build_line(uint8_t line, const char *prefix,
                                     const char *num, const char *suffix,
                                     uint8_t num_width)
{
    char buf[24];
    char *p = buf;
    uint8_t len = str_len(num);

    p = put_str(p, prefix);
    if (num_width > len) {
        p = put_pad(p, (uint8_t)(num_width - len));
    }
    p = put_str(p, num);
    if (suffix != 0) {
        p = put_str(p, suffix);
    }
    *p = '\0';
    set_line(line, buf);
}

void oled_status(uint8_t item, const char *state, uint32_t elapsed_ms,
                 int32_t dist_mm, int32_t dev_mm, int32_t yaw_deg10)
{
    char buf[24];
    char num[14];
    char *p;
    uint8_t len;

    /* 第 1 行：ITEM n  STATE */
    p = put_str(buf, "ITEM ");
    p = put_uint(p, (uint32_t)item);
    p = put_str(p, "  ");
    p = put_str(p, state);
    *p = '\0';
    set_line(0U, buf);

    /* 第 2 行：T     26.7 s */
    fmt_dec1(num, elapsed_ms / 100U);
    build_line(1U, "T", num, " s", 9U);

    /* 第 3 行：D     6140 mm */
    fmt_uint(num, (uint32_t)((dist_mm < 0) ? 0 : dist_mm));
    build_line(2U, "D", num, " mm", 9U);

    /* 第 4 行：DEV  -30 YAW 0（两个数字，单独拼） */
    p = put_str(buf, "DEV");
    fmt_int(num, dev_mm);
    len = str_len(num);
    if (len < 5U) {
        p = put_pad(p, (uint8_t)(5U - len));
    }
    p = put_str(p, num);
    p = put_str(p, " YAW ");
    fmt_int(num, yaw_deg10 / 10);
    p = put_str(p, num);
    *p = '\0';
    set_line(3U, buf);
}
