/*
 * car_config.h - C 题固件的全部可调参数
 *
 * 只有两段：
 *   1. 底盘（编码器、轮速环、电机方向）
 *   2. 比赛（赛道几何、灰度、循迹、6 个项目）
 *
 * 上车标定只需要改这个文件，改完重新编译即可。
 */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#define CAR_FW_VERSION            "2.0"

/* ===================== 底盘 ===================== */

/* PWM 周期，必须和 car.syscfg 里的 timerCount 一致 */
#define CAR_PWM_PERIOD_LD         (1600U)

/* 控制/采样周期（ms），速度环和任务状态机都按这个节拍跑 */
#define CAR_CTRL_PERIOD_MS        (10U)

/* 每个周期占空比最多变化几个百分点（软启动/软刹车） */
#define CAR_RAMP_STEP_PCT         (4)

/* 某个轮子转向反了就把对应项改成 1 */
#define CAR_LEFT_INVERT           (0)
#define CAR_RIGHT_INVERT          (0)

/* 编码器线数（电机轴每转脉冲数）、减速比、轮径 */
#define CAR_ENCODER_PPR           (13.0f)
#define CAR_GEAR_RATIO            (30.0f)
#define CAR_WHEEL_DIAMETER_MM     (65.0f)

/* 编码器方向反了就改成 1 */
#define CAR_LEFT_ENCODER_INVERT   (0)
#define CAR_RIGHT_ENCODER_INVERT  (0)

/* 100% 占空比对应的轮速（mm/s）。
 * 这是所有 mm/s 指令的换算基准，必须按实车标定，否则速度会系统性偏快/偏慢。 */
#define CAR_MAX_SPEED_MM_S        (600.0f)

/* 轮速环 PID */
#define CAR_SPEED_KP              (0.15f)
#define CAR_SPEED_KI              (0.50f)
#define CAR_SPEED_KD              (0.0f)

/* 堵转保护：目标在动但轮子长时间不转就切故障停车，防止烧电机 */
#define CAR_STALL_SPEED_MM_S      (15.0f)
#define CAR_STALL_MIN_PCT         (5)
#define CAR_STALL_TIMEOUT_MS      (1200U)

/* 轮速低于这个值认为车已经停稳（mm/s） */
#define CAR_STOP_SPEED_MM_S       (30.0f)

/* ===================== 赛道几何（题目给定） ===================== */

/* 环路中心线周长：1.5 m + πR + 1.5 m + πR，R = 0.5 m */
#define CAR_LAP_LENGTH_MM         (6140)

/* A→B 直道长度 */
#define CAR_STRAIGHT_AB_MM        (1500)

/* 左右轮中心距（mm）：原地转 90° 靠它换算角度，必须实测 */
#define CAR_WHEEL_BASE_MM         (130.0f)

/* ===================== 八路灰度循迹 ===================== */

/* 压到黑线时输出低电平填 0，输出高电平填 1 */
#define CAR_LINE_BLACK_LEVEL      (0)

/* 8 路左右顺序反了就改 1（现象：一上车就往线外冲） */
#define CAR_LINE_SENSOR_REVERSE   (0)

/* 相邻两路探头间距（mm） */
#define CAR_LINE_PITCH_MM         (11.0f)

/* 启停线判定：同时压到几路以上算“宽黑线”
 * 1.8 cm 线宽只有 1~2 路，5 cm 长的启停线有 4 路以上 */
#define CAR_LINE_MARKER_MIN       (4)

/* ===================== 循迹 PD ===================== */

/* kp：百分比/mm（偏差 11 mm 约修 12%） */
#define CAR_LINE_KP               (1.10f)

/* kd：百分比/(mm/s) */
#define CAR_LINE_KD               (0.015f)

/* 转向修正上限（%），太大容易画龙 */
#define CAR_LINE_CORR_MAX_PCT     (55)

/* 偏差每 1 mm 降低多少 mm/s 巡航速度（弯道自动减速） */
#define CAR_LINE_SLOWDOWN         (3.5f)

/* 丢线后原地找线的速度（%） */
#define CAR_LINE_SEARCH_PCT       (22)

/* 连续丢线超过这么久判失败停车（ms） */
#define CAR_LINE_FAIL_MS          (1000U)

/* ===================== 各项目速度（mm/s） ===================== */

#define CAR_SPEED_ITEM1_MM_S      (260)   /* 1 圈 ≤40 s */
#define CAR_SPEED_ITEM2_MM_S      (420)   /* 1 圈 ≤20 s */
#define CAR_SPEED_ITEM3_MM_S      (560)   /* 3 圈 ≤40 s */
#define CAR_SPEED_ITEM4_MM_S      (300)   /* 往返 ≤30 s */
#define CAR_SPEED_ITEM6_MM_S      (320)   /* 找线 + 1 圈 ≤30 s */

/* 停车前的爬行速度：从 0.3 m/s 直接刹会冲 5 cm 以上 */
#define CAR_SPEED_CREEP_MM_S      (110)

/* 弯道降速下限 */
#define CAR_SPEED_MIN_MM_S        (160)

/* 距离目标还有这么远就切爬行（mm） */
#define CAR_STOP_SLOW_ZONE_MM     (260)

/* 停车标定：正数 = 多跑一点再停。按 $R 帧的 dev 值取反填写 */
#define CAR_STOP_TRIM_MM          (0)

/* ===================== 启停线识别 ===================== */

/* 连续几个节拍满足才算一条启停线 */
#define CAR_MARKER_CONFIRM_TICKS  (2)

/* 两条启停线之间至少隔多远，防止同一条数两次 */
#define CAR_MARKER_REFRACT_MM     (150)

/* A 点窗口：里程落在周长整数倍 ± 这个值内算到 A */
#define CAR_MARKER_WINDOW_MM      (400)

/* 起步先跑这么远再开始数启停线（车本来就压在 A 线上） */
#define CAR_MARKER_BLANK_MM       (300)

/* ===================== 第 4 项：倒车直行 ===================== */

/* 倒车用编码器航向纠偏（百分比/度），以及纠偏限幅 */
#define CAR_BACK_YAW_KP           (0.55f)
#define CAR_BACK_CORR_MAX_PCT     (25.0f)

/* ===================== 第 5 项：原地转向 ===================== */

/* 转多少度（顺时针为负）。方向反了改成 +90.0f */
#define CAR_TURN_ITEM5_DEG        (-90.0f)

/* 角度环：比例 + 角速度阻尼（只有比例会在目标角两侧来回摆，停不住） */
#define CAR_TURN_KP               (1.00f)
#define CAR_TURN_KD               (0.10f)

/* 最小/最大转向占空比，最小值要能克服静摩擦 */
#define CAR_TURN_MIN_PCT          (12)
#define CAR_TURN_MAX_PCT          (30)

/* 角度到位判据（度）与稳定节拍数 */
#define CAR_TURN_TOL_DEG          (2.5f)
#define CAR_TURN_SETTLE_TICKS     (15U)

/* ===================== 第 6 项：白区找线 ===================== */

/* 原地扫描找线：速度（%）与每次扫过的角度 */
#define CAR_SEEK_ROT_PCT          (16)
#define CAR_SEEK_SWEEP_DEG        (370.0f)

/* 扫一圈没找到就直行一段再扫 */
#define CAR_SEEK_FWD_PCT          (18)
#define CAR_SEEK_FWD_MM           (80)

/* 最多尝试几轮（6 轮约覆盖 40 cm 起步距离） */
#define CAR_SEEK_MAX_TRIES        (6U)

/* 找到线后偏差小于这个值认为已“拉正”（mm） */
#define CAR_ALIGN_TOL_MM          (10.0f)

/* 拉正后连续稳定多少个节拍才切正常巡线 */
#define CAR_APPROACH_SETTLE_TICKS (10U)

/* 拉正过程中丢线超过这么久，回去重新扫 */
#define CAR_APPROACH_LOST_MS      (400U)

/* 找到线后遇到第几条启停线停车。A、B 启停线完全一样，
 * 按“裁判把车放在 A 附近”取第 1 条；放远了改 2。 */
#define CAR_ITEM6_STOP_MARKERS    (1U)

/* ===================== 停留与超时 ===================== */

#define CAR_HOLD_MS               (3000U)

/* 题目限时 + 8 s 余量，超时自动停车 */
#define CAR_TIMEOUT_ITEM1_MS      (48000U)
#define CAR_TIMEOUT_ITEM2_MS      (28000U)
#define CAR_TIMEOUT_ITEM3_MS      (48000U)
#define CAR_TIMEOUT_ITEM4_MS      (38000U)
#define CAR_TIMEOUT_ITEM5_MS      (28000U)
#define CAR_TIMEOUT_ITEM6_MS      (38000U)

/* ===================== 启动按键（PA16） ===================== */

/* 连续几个 10 ms 采样一致才认（30 ms 消抖） */
#define CAR_KEY_DEBOUNCE_TICKS    (3U)

/* 短按最短时间 / 长按判定时间（ms） */
#define CAR_KEY_SHORT_MIN_MS      (30U)
#define CAR_KEY_LONG_MS           (1200U)

/* 上电后这么久内忽略按键事件。
 * 按键模块上并联了 0.1 uF 消抖电容，冷上电瞬间电容是 0 V，会被读成一次
 * “按下”，30~40 ms 后才释放，正好越过短按门槛 —— 不加这个窗口车会自己开跑。 */
#define CAR_KEY_MASK_MS           (500U)

/* ===================== OLED 显示 ===================== */

/* 状态刷新周期（ms）；真正写到屏上由 oled_task 分摊，每拍最多一个字符 */
#define CAR_OLED_PERIOD_MS        (200U)

#endif /* CAR_CONFIG_H */
