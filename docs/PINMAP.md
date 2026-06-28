# PINMAP.md — MSPM0G3507 引脚占用全表

> **生成依据**: `ml_pwm.c`、`ml_motor.c`、`encoder.c`、`xunji.c`、`IMU.c`、`isr.c`、`ti_msp_dl_config.c`、`buzzer_light.c`
> 
> **用途**: 在新增云台舵机 PWM 等功能前，先查此表确认引脚空闲，避免冲突。

---

## 1. 已占用引脚总表

| 引脚 | 功能 | 使用方式 | 来源文件 |
|------|------|----------|---------|
| **PB6** | 左电机 PWM 输出 (TIMG6 CC0) | TIMG6 PWM | `ml_pwm.c` → `motor_init()` |
| **PB7** | 右电机 PWM 输出 (TIMG6 CC1) | TIMG6 PWM | `ml_pwm.c` → `motor_init()` |
| **PB24** | 左电机方向 AIN1 | GPIO OUT | `ml_motor.c` |
| **PB20** | 左电机方向 AIN2 | GPIO OUT | `ml_motor.c` |
| **PB19** | 右电机方向 BIN1 | GPIO OUT | `ml_motor.c` |
| **PB18** | 右电机方向 BIN2 | GPIO OUT | `ml_motor.c` |
| **PB16** | TB6612 STBY（常高） | GPIO OUT | `ml_motor.c` |
| **PA12** | 左编码器 A 相 (RISING EXTI) | GPIO EXTI IN | `encoder.c` |
| **PA13** | 左编码器 B 相 (FALLING EXTI) | GPIO EXTI IN | `encoder.c` |
| **PA24** | 右编码器 A 相 (RISING EXTI) | GPIO EXTI IN | `encoder.c` |
| **PA15** | 右编码器 B 相 (FALLING EXTI) | GPIO EXTI IN | `encoder.c` |
| **PA17** | 按键1 (mode)（EXTI + 轮询） | GPIO EXTI IN | `encoder.c` + `key.c` |
| **PA27** | 按键2 (set)（EXTI + 轮询） | GPIO EXTI IN | `encoder.c` + `key.c` |
| **PB13** | 灰度 D1 | GPIO IN (上拉) | `xunji.c` |
| **PB15** | 灰度 D2 | GPIO IN (上拉) | `xunji.c` |
| **PA31** | 灰度 D3 | GPIO IN (上拉) | `xunji.c` |
| **PA28** | 灰度 D4 | GPIO IN (上拉) | `xunji.c` |
| **PB1** | 灰度 D5 | GPIO IN (上拉) | `xunji.c` |
| **PB4** | 灰度 D6 | GPIO IN (上拉) | `xunji.c` |
| **PB17** | 灰度 D7 | GPIO IN (上拉) | `xunji.c` |
| **PB12** | 灰度 D8 | GPIO IN (上拉) | `xunji.c` |
| **PA8** | IMU JY61P UART1 TX | UART1 | `IMU.c` |
| **PA9** | IMU JY61P UART1 RX | UART1 | `IMU.c` |
| **PB2** | OLED SDA (I2C) | Software I2C | `ml_i2c.c` |
| **PB3** | OLED SCL (I2C) | Software I2C | `ml_i2c.c` |
| **PB8** | 蜂鸣器 | GPIO OUT | `buzzer_light.c` |
| **PB26** | LED | GPIO OUT | `buzzer_light.c` |
| **PA0/PA1** | HFXT 晶振 | Analog | `ti_msp_dl_config.c` |

---

## 2. Timer 实例占用情况

| Timer | 当前用途 | PWM 输出引脚 | 状态 |
|-------|---------|------------|------|
| **TIMG6** | 电机 PWM | CC0→PB6, CC1→PB7 | ⛔ 已满（两通道均占用） |
| **TIMG7** | 蜂鸣器/LED 自动关闭定时（中断模式） | CC0→PA17, CC1→PA18（代码中未作 PWM 用） | ⚠️ 已用于定时中断；PA17 已被按键占用 |
| **TIMG8** | 10ms 主控 ISR（定时中断） | CC0→PA26, CC1→PA27（代码中未作 PWM 用） | ⚠️ 已用于 PID 调度中断；PA27 已被按键占用 |
| **TIMG0** | 未使用（ISR 有空 stub） | CC0→PA12, CC1→PA13 | ⚠️ PA12/PA13 已被编码器占用 |
| **TIMG12** | 未使用（ISR 有空 stub） | CC0→PA14, CC1→PA24 | 🟡 **PA14 空闲，PA24 已被编码器占用** |

> **结论**：TIMG6 两个通道全被电机占满，PB6/PB7 **不可用于云台**。

---

## 3. 候选云台 PWM 引脚分析

### 方案 A：新增独立 Timer（推荐）
使用 MSPM0G3507 尚未在代码中初始化的 TIM 实例，但 `ml_pwm.c` 目前只支持 TIMG0/6/7/8/12：

| 候选引脚 | Timer | 通道 | 冲突风险 | 建议 |
|---------|-------|------|---------|------|
| **PA14** | TIMG12 CC0 | 独立通道 | ✅ 无冲突（PA14 未被使用） | **✅ 推荐（舵机1 yaw）** |
| **PA18** | TIMG7 CC1 | 需确认 TIMG7 是否可同时 PWM+中断 | ⚠️ TIMG7 已用于定时中断，不建议复用 | ❌ 不推荐 |
| **PA26** | TIMG8 CC0 | 需确认 TIMG8 是否可同时 PWM+中断 | ⚠️ TIMG8 已用于 PID 主中断，不建议复用 | ❌ 不推荐 |

### 方案 B：使用未在 ml_pwm.c 中定义的 TIMA 实例
MSPM0G3507 还有 TIMA0、TIMA1（高级 Timer），可提供更多 PWM 通道。但当前 `ml_pwm.c` 不支持，需要扩展。

### 当前最优结论

```
云台 yaw 舵机 PWM → PA14 (TIMG12 CC0)   [需在 SysConfig/ml_pwm.c 中确认 TIMG12 的 PWM 功能]
云台 pitch 舵机 PWM → 待定，需查 TIMA0/TIMA1 可用引脚
```

**在动手写 `gimbal.c` 之前，请先执行以下步骤**：
1. 打开 `user/empty.syscfg`，查看哪些 Timer 实例和引脚已在 SysConfig 中被占用
2. 确认 TIMG12 CC0 对应 PA14 的 PWM 功能未被 SysConfig 分配给其他模块
3. 如需第二个舵机通道，查 TIMA0/TIMA1 的引脚映射

---

## 4. 空闲引脚参考（未见于任何源文件的引脚）

以下引脚在当前代码中**未被使用**，可考虑用于扩展功能（仍需确认 PCB 是否已连接到外部电路）：

`PA2`, `PA3`, `PA4`, `PA5`, `PA6`, `PA7`, `PA10`, `PA11`, `PA14`(推荐), `PA16`, `PA18`, `PA19`, `PA20`, `PA21`, `PA22`, `PA23`, `PA25`, `PA26`(TIMG8 PWM备用), `PB0`, `PB5`, `PB6`(被TIMG6占用), `PB9`, `PB10`, `PB11`, `PB14`, `PB21`, `PB22`, `PB23`, `PB25`, `PB27`

> ⚠️ 以上"空闲"仅指代码层面未初始化，不代表 PCB 上无连接。实际使用前需对照原理图确认。
