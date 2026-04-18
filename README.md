# xiaozhi-esp32 — Deskrobots qebabe-xiaoche 定制固件

> 基于 qebabe/xiaozhi-esp32 项目 fork 修改，为 Deskrobots qebabe-xiaoche 机器人小车定制的智能对话机器人固件。

修改起始日期：2026-03-08 | 修改人：pudite

![产品图](待更新)

---

## 硬件平台

| 模块 | 规格 |
|------|------|
| **主控** | ESP32-S3-N16R8 (16MB Flash, 8MB PSRAM) |
| **电机驱动** | L298N 模块，N20 直流电机 x2（差速驱动） |
| **摄像头** | OV2640 DVP 接口，640x480 JPEG @ 25fps |
| **显示屏** | SSD1306 OLED 128x64 (I2C) |
| **音频** | I2S 麦克风 + I2S 功放（Duplex 半双工模式） |
| **LED** | WS2812 内置 LED (GPIO48) |
| **按键** | BOOT 按钮 (GPIO0) |

### 引脚分配

| 功能 | GPIO | 说明 |
|------|------|------|
| 电机-左前 (LF) | GPIO45 | PWM 控制 |
| 电机-左后 (LB) | GPIO47 | PWM 控制 |
| 电机-右前 (RF) | GPIO21 | PWM 控制 |
| 电机-右后 (RB) | GPIO14 | PWM 控制 |
| OLED SDA | GPIO43 | I2C |
| OLED SCL | GPIO44 | I2C |
| 摄像头 XCLK | GPIO15 | 20MHz |
| 摄像头 D0~D7 | GPIO11, 9, 8, 10, 12, 18, 17, 16 | DVP 8-bit |
| 摄像头 VSYNC / HREF / PCLK | GPIO6 / GPIO7 / GPIO13 | DVP 控制信号 |
| 摄像头 SCCB (SIOD/SIOC) | GPIO4 / GPIO5 | 独立 I2C 总线 |
| 麦克风 WS / SCK / DIN | GPIO42 / GPIO41 / GPIO1 | I2S 输入（Duplex 模式与扬声器共享 WS/BCLK） |
| 扬声器 DOUT / BCLK / LRCK | GPIO2 / GPIO41 / GPIO42 | I2S 输出（Duplex 模式与麦克风共享 WS/BCLK） |
| BOOT 按钮 | GPIO0 | 唤醒/对话切换 |
| 状态 LED | GPIO48 | WS2812 |

> 注：当前使用 Duplex I2S 模式，麦克风与扬声器共享 WS/BCLK 引脚。若需独立音频引脚，可启用 Simplex 模式（config.h 中取消 `AUDIO_I2S_METHOD_SIMPLEX` 注释）。

---

## 功能特性

### 1. 智能对话

- 支持 **WebSocket / MQTT** 协议连接小智服务器
- 语音唤醒（"你好小智"）+ 按键唤醒双模式
- TTS 语音合成播报

### 2. 视频流

- OV2640 摄像头实时 MJPEG 流（640x480 @ 25fps）
- 独立流媒体服务器（端口 81），不阻塞控制 API
- Web 端实时观看，支持拍照下载
- 摄像头水平/垂直翻转可调，画质可调

### 3. 电机控制

- **WebSocket 长连接**实时遥控（`/ws/control`），低延迟不断连
- 虚拟摇杆拖拽控制，连续速度调节
- 预设动作按钮：前进/后退/左转/右转/转圈
- 情感表达动作：唤醒/开心/悲伤/思考/倾听/说话/摆动/跳舞
- 高级情感：兴奋/爱慕/生气/惊讶/困惑
- 电机动作参数可通过 Web 配置页面在线调整（时长、速度）

### 4. MCP 工具服务器

设备暴露以下 MCP 工具供远程调用：

| 工具 | 功能 |
|------|------|
| `self.motor.move_forward` | 前进（速度、时长可调） |
| `self.motor.move_backward` | 后退 |
| `self.motor.turn_left` / `turn_right` | 左转/右转 |
| `self.motor.spin_around` | 原地转圈 |
| `self.motor.quick_forward` / `quick_backward` | 快速冲刺 0.5s |
| `self.motor.wiggle` / `dance` | 摆动/跳舞 |
| `self.motor.stop` | 紧急停止 |
| `self.motor.wake_up` ~ `confused` | 12 种情感动画 |
| `self.network.get_ip` | 获取 IP 地址 |

### 5. Web 控制界面

- 手机/PC 浏览器访问设备 IP 即可控制
- 视频流 + 摇杆遥控一体化界面
- 动作控制面板（默认折叠，点击展开）
- 电机动作参数配置页面（`/config`）

---

## 构建目标

| 目标名称 | 屏幕 |
|----------|------|
| `qebabe-xiaoche` | SSD1306 128x32 |
| `qebabe-xiaoche-128x64` | SSD1306 128x64（默认） |

## 编译与烧录

### 环境准备

本项目最低支持 **ESP-IDF v5.4**，推荐使用 **v5.5.2**。

1. 在 VSCode 扩展市场中搜索并安装 **ESP-IDF** 扩展
2. 安装完成后，按 `Ctrl+Shift+P` 打开命令面板，输入 `ESP-IDF: Configure ESP-IDF Extension` 打开 **ESP-IDF 安装管理器**
3. 在管理器中选择安装 ESP-IDF 版本 `5.5.2`（或更高兼容版本）

### 编译

环境配置完成后，VSCode 底部状态栏会出现 ESP-IDF 工具栏，选择好 ESP-IDF 版本后，点击 ▶ **Build** 按钮即可编译。

> 也可点击底部 ESP-IDF 工具栏中的 **终端图标** 打开 ESP-IDF 终端，在终端中执行编译命令。
>
> 或在已有的 `build` 目录下直接调用 Ninja（适用于 AI 辅助编译等场景）：
> ```bash
> cd build
> "C:\Espressif/tools/ninja/1.12.1/ninja.exe"
> ```
> （删除 `build` 目录后需先通过 VSCode 重新编译一次生成构建目录）

### 烧录

1. USB 数据线连接开发板的 **USB-OTG 口**
2. 点击 VSCode 底部 ESP-IDF 工具栏，配置烧录参数：
   - **烧录方式**：选择 `UART`
   - **端口**：选择设备管理器中插入开发板后出现的 COM 端口
   - **目标芯片**：选择 `ESP32-S3`（后续选项选带 `***USB-JTAG***` 的那个）
   - **SDK 配置编辑器（menuconfig）**：可跳过，默认设置已就绪
3. 点击 ▶ **Flash（烧录）** 即可

### 关键配置

以下设置已写入 `sdkconfig.defaults`，删除 `sdkconfig` 后重建依然生效：

```
CONFIG_CAMERA_OV2640=y
CONFIG_CAMERA_OV2640_DVP_JPEG_640X480_25FPS=y
CONFIG_XIAOZHI_CAMERA_ALLOW_JPEG_INPUT=y
CONFIG_FREERTOS_MAX_TASK_NAME_LEN=16
```

ESP32-S3 板级默认配置（`sdkconfig.defaults.esp32s3`）：

```
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y    # 16MB Flash
CONFIG_SPIRAM=y                       # 8MB PSRAM
CONFIG_SPIRAM_MODE_OCT=y              # 八线 PSRAM
CONFIG_HTTPD_WS_SUPPORT=y             # WebSocket 支持
CONFIG_SR_WN_WN9_NIHAOXIAOZHI_TTS=y   # "你好小智" 唤醒词
```

---

## 主要更新记录（2026-03-08 起）

- **电机控制系统**：全新网页遥控器，PWM 调速升级，WebSocket 实时低延迟控制
- **视频流功能**：OV2640 摄像头 MJPEG 直播，独立流媒体服务器，拍照/翻转/画质调节
- **情感动作系统**：12 种情感动画，每种表情对应独特电机动作
- **MCP 工具扩展**：9 个电机控制工具 + 网络查询工具
- **Web 配置页面**：在线调整电机动作时长、速度参数
- **看门狗机制**：视频流与电机控制双重看门狗保护
- **板级配置优化**：电机引脚适配，OLED 显示修复

---

## 注意事项

- 电机使用 PWM 控制，5V 供电时最低 50% 速度保证启动，3.3V 供电时需提高至 90%
- 视频流与电机控制可同时运行，互不干扰（独立端口 + 独立 socket）
- 摄像头 SCCB (GPIO4/5) 使用独立 I2C 总线，与显示 I2C (GPIO43/44) 不冲突

---

## 开源地址

[pudite/xiaozhi-esp32](https://github.com/pudite/xiaozhi-esp32)（fork 自 [qebabe/xiaozhi-esp32](https://github.com/qebabe/xiaozhi-esp32)）
