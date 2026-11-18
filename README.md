# MSPM0G3507 循迹小车固件 · 电赛 C 题

**自主循迹与定点停车运动控制系统** —— 用一块 MSPM0G3507、八路红外灰度和双编码器，把电赛 C 题的 6 个测试项放进同一套固件里跑完。

![MCU](https://img.shields.io/badge/MCU-MSPM0G3507-blue)
![SDK](https://img.shields.io/badge/MSPM0--SDK-2.11-orange)
![IDE](https://img.shields.io/badge/CCS%20Theia-21.0.1-brightgreen)
![License](https://img.shields.io/badge/License-MIT-success)

---

## 这是什么

小车沿 1.5 m 直道 + R=0.5 m 半圆组成的环路（中心线周长 6140 mm）自动循迹，按题目要求在 A 点定点停车、往返、原地转向、线外找线。整套代码就是一条控制链：

```
八路灰度 → 黑线偏差 pos(mm) → 循迹 PD → 左右轮目标速度
                                              ↓
按键/串口 → 6 项任务状态机 → 轮速闭环 PID → TB6612 → 电机
                                              ↑
编码器 → 里程 d(mm) / 航向 ψ(deg) ────────────┘
```

没有遥控协议、没有 Flash 存参、没有电池检测——上电就是比赛状态。

## 特性

- **一套代码跑六项任务**：长按 PA16 在 1→6 之间切换，短按开跑、运行中再按即中止；串口 `1`~`6` / `P` / `X` / `S` 也能完全控制。
- **里程是停车主判据**：编码器四倍频 1560 计数/转、周长 204.2 mm，合 0.13 mm/计数。目标里程 = 圈数 × 6140 mm + 标定值，启停线只用来核对圈数——不受“灰度条比车头靠后 3~4 cm”的几何误差影响。
- **循迹 PD + 弯道自动降速 + 丢线找回**：偏差每 1 mm 降 3.5 mm/s 巡航速度，转向修正限幅 55% 防画龙，连续丢线 1 s 判失败。
- **轮速闭环 + 软启动 + 堵转保护**：10 ms 节拍，占空比每拍变化 ≤4%；目标在动而轮子 1.2 s 不转就切故障停车。
- **可观测**：SSD1306 OLED 显示项目号/状态/用时/里程/偏差；串口 2 Hz 状态帧 + 每次结束的结果帧（带停车偏差 `dev`，标定就看它）。
- **体积小**：约 16.8 KB Flash / 1.2 KB RAM（0x4300 / 0x4C9），Cortex-M0+ 上编译零告警。

## 硬件

| 部件 | 规格 |
|---|---|
| 主控 | MSPM0G3507（WHEELTEC 地猛星小车板，兼容 LP-MSPM0G3507） |
| 电机驱动 | TB6612 双路（PWM + 方向 + STBY） |
| 底盘 | 两轮差速 + 万向轮，轮径 65 mm，减速比 30:1，编码器 13 线/转 |
| 循迹 | 八路红外灰度，探头间距 11 mm，总跨度 77 mm |
| 显示 | 0.96" SSD1306 OLED，I²C 地址 0x3C（可选，不接也能跑） |
| 按键 | PA16 轻触按键，内部上拉，按下接 GND |

## 接线

| 功能 | 引脚 | 说明 |
|---|---|---|
| 左/右电机 PWM | PA12 / PA13（TIMG0，20 kHz） | TB6612 PWMA / PWMB |
| 左电机方向 | PA8 (AIN1) / PA9 (AIN2) | 正转 1-0 |
| 右电机方向 | PB18 (BIN1) / PA7 (BIN2) | 正转 1-0 |
| 驱动使能 | PB24 (STBY) | 高电平使能 |
| 板载 LED | PA14 | 心跳：待机 500 ms / 待启动 250 ms / 运行 100 ms |
| 串口 | PA28 (TX) / PA31 (RX) | UART0，115200 8N1 |
| OLED | PB2 (SCL) / PB3 (SDA) | I2C1，400 kHz |
| 编码器 1 | PA21 (A) / PA22 (B) | 内部上拉，双边沿中断 |
| 编码器 2 | PB19 (A) / PB20 (B) | 同上 |
| 八路灰度 CH1~CH8 | PA17 / PB8 / PB9 / PA24 / PA2 / PA0 / PA1 / PA23 | 数字量输入 |
| 启动按键 | PA16 | 内部上拉 |

> **PA0 / PA1 是开漏焊盘**，SysConfig 不允许开内部上拉，灰度模块必须是推挽输出或板上自带上拉。
> **PA18 是 BSL 引导脚**，上电被拉低会进不了用户程序，不要接东西。
> 改引脚：双击 `car.syscfg` 改完 Ctrl+S，**不要直接改生成的 `build/ti_msp_dl_config.*`**。

## 六项任务

| 项目 | 内容 | 题目限时 | 判定 / 主要参数 |
|---|---|---|---|
| 1 | 顺时针 1 圈停 A，偏差 ≤5 cm | 40 s | 里程 ≥6140 mm 或识别到 A 点启停线，`CAR_SPEED_ITEM1_MM_S=260` |
| 2 | 逆时针 1 圈停 A，偏差 ≤5 cm | 20 s | 巡线本身不分方向，`CAR_SPEED_ITEM2_MM_S=420` |
| 3 | 顺时针连续 3 圈停 A | 40 s | 第 3 次 A 点启停线或 ≥18420 mm，`CAR_SPEED_ITEM3_MM_S=560` |
| 4 | A→B 停 3 s，倒车回 A | 30 s | 前进 ≥1500 mm 停 → 停 3 s → 倒回 0 mm 停，`CAR_SPEED_ITEM4_MM_S=300` |
| 5 | 原地顺时针 90°，停 3 s，再逆时针 90° 转回 | 20 s | 编码器差速积分角，`CAR_TURN_ITEM5_DEG=-90` |
| 6 | 线外 20~40 cm 白区自主找线，顺时针到 A 停 | 30 s | 原地扫 370° → 没找到就前进 80 mm 再扫，最多 6 轮 |

各项都带“限时 + 8 s”的软超时，超时自动停车，不会把车顶到赛道外面。

### 停车精度是怎么做到的（≤5 cm）

| 措施 | 作用 |
|---|---|
| 停车前 26 cm 切爬行（110 mm/s） | 从 0.56 m/s 直接刹会冲 5 cm 以上；爬行时刹车距离只有 2~3 mm |
| 里程主判据 | 车头最前端是判定点，灰度条在车头后 3~4 cm；等灰度条压到启停线时车头早就过线了 |
| 启停线只核对圈数 | A 点在周长整数倍处，B 点在 1.5 m（顺时针）或 4.64 m（逆时针）处，用“到周长整数倍的距离 <400 mm”把两者分开 |
| 编码器 0.13 mm/计数 | 里程量化误差可忽略，精度取决于“计数 → 毫米”的标定 |

## 编译与烧写

### 方式一：CCS（推荐）

```text
CCS Theia 21.0.1 + MSPM0-SDK 2.11.00.07 + SysConfig 1.26
```

1. `File → Import Project → CCS Projects`，选择**本仓库根目录**（工程名 `wheeltec_car_g3507`）。
2. 双击 `car.syscfg` 检查引脚，`Ctrl+S` 重新生成配置。
3. `Project → Build`，然后 `Run → Debug`（或直接 Flash 烧写 `build/wheeltec_car.hex`）。

仓库根目录就是 CCS 工程目录：`.project` / `.cproject` / `.ccsproject` / `targetConfigs/` 都在这里，导入时别选成子目录。

### 方式二：命令行

不打开 CCS 也能验证代码：

```powershell
powershell -ExecutionPolicy Bypass -File .\build.ps1
```

产物在 `build/`：`wheeltec_car.out` / `.hex` / `.bin`。`build.ps1` 里默认按以下路径找工具链，装到别处就改开头三行：

```powershell
$sdk       = "C:\ti\mspm0_sdk_2_11_00_07"
$sysconfig = "C:\ti\sysconfig_1.26.2\sysconfig_cli.bat"
$compiler  = "C:\ti\ti_cgt_arm_llvm_4.0.2.LTS"
```

### 上车标定顺序

1. `CAR_MAX_SPEED_MM_S`：给 100% 占空比量实际轮速，所有 mm/s 指令都按它换算。
2. `CAR_WHEEL_BASE_MM`：量左右轮中心距，决定第 5 项 90° 的准确度。
3. `CAR_LINE_BLACK_LEVEL` / `CAR_LINE_SENSOR_REVERSE`：压黑线时电平、8 路左右顺序（现象是一上车就往线外冲）。
4. `CAR_STOP_TRIM_MM`：跑一次看 `$R` 帧的 `dev`，取反填进去再跑一次。
5. `CAR_TURN_ITEM5_DEG`：转向方向反了就改成 `+90.0f`。

## 操作方式

| 操作 | 效果 |
|---|---|
| 短按 PA16 | 开始当前项目；运行中按下 = 中止并停车 |
| 长按 PA16（1.2 s） | 切换项目 1→2→…→6→1（运行中无效） |
| 串口 `1`~`6` | 直接选项目 |
| 串口 `P` / `X` / `S` | 开始 / 中止 / 打印一次状态 |

> 上电 500 ms 内的按键事件会被丢弃（`CAR_KEY_MASK_MS`）：按键模块上的 0.1 µF 电容冷上电时会被读成一次假按下，不屏蔽会导致车一上电就自己开跑。

### 串口协议（115200 8N1）

运行中 2 Hz 输出状态帧：

```
$D,item=1,st=2,ph=1,t=12340,d=3210,pos=-8,blk=2,corr=12,lap=0,mk=0
```

- `st`：0 待机 / 1 待启动 / 2 运行 / 3 完成 / 4 失败
- `t` 已用时间 ms，`d` 里程 mm，`pos` 黑线偏差 mm，`blk` 压线路数
- `corr` 转向修正量 %，`lap` A 点圈数，`mk` 启停线计数

每次结束输出结果帧（**标定停车就看 `dev`**）：

```
$R,item=1,ok=1,why=0,t=26750,d=6110,dev=-30,yaw10=0,lap=1,mk=2
```

- `dev`：实际停车点相对目标的偏差（mm）
- `yaw10`：结束时车头角度 ×10（第 5 项看这个）
- `why`：1 超时 / 2 堵转 / 3 丢线 / 4 找不到线 / 6 人为中止

## 参数速查（`car_config.h`）

| 参数 | 默认 | 含义 |
|---|---|---|
| `CAR_MAX_SPEED_MM_S` | 600 | 100% 占空比对应的轮速，**必须实测** |
| `CAR_WHEEL_BASE_MM` | 130 | 轮距，决定 90° 精度 |
| `CAR_STOP_TRIM_MM` | 0 | 停车标定，按 `$R` 的 `dev` 取反填写 |
| `CAR_LINE_KP / KD` | 1.10 / 0.015 | 循迹 PD（%/mm、%/(mm/s)） |
| `CAR_SPEED_KP / KI / KD` | 0.15 / 0.50 / 0 | 轮速环 |
| `CAR_SPEED_ITEM1..6_MM_S` | 260 / 420 / 560 / 300 / — / 320 | 各项巡航速度 |
| `CAR_SPEED_CREEP_MM_S` | 110 | 停车前爬行速度 |
| `CAR_TURN_ITEM5_DEG` | -90.0 | 第 5 项转向角，方向反了改 `+90` |
| `CAR_TIMEOUT_ITEM1..6_MS` | 48000 … 38000 | 各项软超时 |

## 目录结构

| 文件 | 作用 |
|---|---|
| `car.syscfg` | SysConfig：电机 PWM、调试串口、编码器中断、八路灰度、启动按键 |
| `main.c` | 1 ms 节拍、10 ms 调度、串口指令、心跳灯 |
| `mission.c/.h` | **6 项任务状态机**：跑圈 / 往返 / 原地转 90° / 白区找线 |
| `line_sensor.c/.h` | 八路灰度采样、黑线位置、启停线（宽黑线）识别 |
| `line_follow.c/.h` | 循迹 PD、弯道自动降速、丢线找回 |
| `odom.c/.h` | 编码器里程计：行驶距离、车头转角 |
| `button.c/.h` | PA16 消抖，短按开跑/中止，长按换项目 |
| `control.c/.h` | 轮速闭环：编码器测速 + 左右轮 PID + 软启动 + 堵转保护 |
| `motor.c/.h` | TB6612 双路驱动（PWM + 方向 + STBY） |
| `encoder.c/.h` | 编码器 4 倍频正交解码 |
| `pid.c/.h` | 通用位置式 PID |
| `oled.c/.h` + `oled_font.c` | SSD1306 128×64 显示，8×16 ASCII 点阵（脚本生成，勿手改） |
| `car_config.h` | **所有可调参数**（底盘 + 比赛） |
| `car_time.h` / `car_irq.h` | 系统节拍 / 临界区 |
| `car_uart.c/.h` | 不依赖 printf 的串口发送与整数拼接 |
| `build.ps1` | 命令行编译（产出 `build/wheeltec_car.hex`） |
| `docs/implementation-notes.md` | 题目要求 → 代码的逐条对照、时间预算、标定步骤 |

## 已知限制

1. **OLED 需要外接**：PB2/PB3 + 3V3/GND，接 0.96" SSD1306（0x3C）。没接屏时驱动自动放弃，其它功能照常。
2. **没有陀螺仪**：90° 靠编码器差速积分，轮距填错会按比例偏。
3. **A/B 启停线不可区分**（尺寸完全一样），第 6 项按“车放在 A 附近”取第 1 条；放远了改 `CAR_ITEM6_STOP_MARKERS`。
4. **参数只能改 `car_config.h` 重编译**：不再支持串口在线调 PID（现场比赛允许改程序）。

## License

[MIT](LICENSE) © 2026 13716710623
