# agent_link

[English](README.md) | **简体中文**

把嵌入式设备接入 **Deotaland Agent 平台**的连接层。设备声明自己有哪些能力（麦克风、喇叭、屏幕、传感器等），再注册几个回调，蓝牙广播、GATT、帧编码都交给库处理，设备侧不写任何蓝牙或协议代码。

本仓库是一个完整的 ESP-IDF 工程：可复用的 SDK 在 `components/agent_link/`，其余是可直接编译烧录的参考固件。

## 你要实现什么

1. **能力**：用一组 `AGENT_CAP_*` 位描述你的硬件。
2. **输出回调**：Agent 通过这些回调驱动你的硬件（放音频、显示文字、震动）。实现你支持的，其余留 `NULL`。
3. **上报调用**：设备用 `agent_link_push_*` 把数据往上送，比如麦克风音频、传感器读数、电量。

能力层以下（BLE、GATT、L2CAP、组帧）都由 SDK 负责。

## 设计

| 原则 | 带来的结果 |
|---|---|
| 能力驱动 | 设备用一个 `caps` 位掩码声明自己有哪些硬件，只实现对应的回调，没有的功能就不开。传感器与执行器还会在连接时发一份自描述清单。 |
| 回调进、push 出 | 下行（Agent 到设备）以回调到达，上行（设备到 Agent）走 `agent_link_push_*`。SDK 不碰任何 GPIO，硬件动作都在你的代码里，因此可跨硬件复用。 |
| 传输藏在接口之后 | 核心只对接抽象的 `agent_transport_t`。目前实现了 BLE；WiFi、USB 后端就绪后，同一套核心不改设备代码即可运行。 |
| 板子就是一个类 | 每块板是一个实现 `Board` 的小 C++ 类（`PlayAudio`、`ShowText`、`Vibrate`、电量）。加板或换板只动它自己的目录。 |

## 架构

```
              云端 Agent  <-->  手机 App
                      |
                   ( BLE )
   ============================================================
    components/agent_link/        SDK（与芯片无关）
      transport_ble.cpp           NimBLE 广播 / GATT / L2CAP
      protocol.cpp                帧编解码
      agent_link.cpp              能力路由 + 生命周期
   ============================================================
      上行  push_*/report_*  ^      v  下行  on_* 回调
   ============================================================
    main/app_main.cpp             共享 app（与板无关）
      把 agent_link 回调绑定到 Board::GetInstance()
   ============================================================
    boards/<板>/                  你的硬件：引脚 + 驱动
      config.h   config.json   <板>.cc   （实现 Board）
   ============================================================
```

上行：板子调 `agent_link_push_*`，核心组帧，传输发出。下行：传输收到帧，核心按能力路由，你的回调驱动硬件。

## 编译与烧录

需要 ESP-IDF v5.0 或更新版本，以及一块带 BLE 射频的开发板。在工程根目录：

```bash
idf.py set-target esp32s3     # 对应你板子的芯片（见 boards/<板>/config.json 的 "target"）
idf.py menuconfig             # Agent Link Device -> Board Type（默认 rorolee-s3）
idf.py build
idf.py -p <串口> flash monitor
```

启动日志大致是这样：

```
agent_link.app: board = ROROLEE_S3, caps = 0x023f
agent_link: init: name='ROROLEE_S3' caps=0x023f proto=v1 transport=ble io=0
agent_link.ble: BLE started — Service C 0xFFC0 registered; advertising on sync
agent_link.ble: advertising as 'ROROLEE_S3'
```

用 nRF Connect 或 LightBlue 扫描，能按名字看到设备、并看到控制服务 `0xFFC0`，就说明链路通了。

## 能力位

`agent_cap_t` 每个能力占一位，按位或组合：

```
MIC  SPEAKER  SCREEN  BUTTON  HAPTIC  BATTERY  LED  SENSOR  ACTUATOR  RECORDING  CAMERA
```

按你的硬件置位，再实现与之对应的回调和上报调用。

## 输出回调（Agent 到设备）

`agent_output_cb_t` 的字段。实现你支持的，其余留 `NULL`。

| 回调 | 何时被调 |
|---|---|
| `on_audio_out(pcm16, bytes)` / `on_audio_end()` | 收到 TTS 音频（PCM16，16 kHz，单声道），分段到达 |
| `on_show_text(utf8)` / `on_show_image(rgb565_be, w, h)` | 屏幕显示文字或图像 |
| `on_haptic(ms)` / `on_led(rgb)` / `on_actuate(ch, val)` | 震动、LED、通用执行器 |
| `on_agent_list(json)` | 平台下发可用 Agent 列表，供设备做选择界面 |
| `on_command(cmd, payload, len, resp, cap, *resp_len)` | 需要返回数据的查询（各类 `Get*`）：填好 `resp` 并返回 `true`，SDK 带上它回响应。`0x03 GetChargingStatus` 由 SDK 用缓存的电量直接应答 |
| `on_custom(cmd, payload, len)` | 设备私有命令的兜底通道，只收不回 |

## 上报数据（设备到 Agent）

目前已实现：

- `agent_link_push_voice` / `agent_link_voice_end`：上传麦克风 PCM（16 kHz，16 bit，单声道）。
- `agent_link_register_io` / `agent_link_push_reading`：声明传感器与执行器并上报读数，执行器命令会回到你注册的回调。见 [`docs/device-io.md`](docs/device-io.md)。
- `agent_link_report_battery`：电量百分比与充电状态，内部去重后才真正上报。

已声明但尚未接到传输（见[现状](#现状)）：`agent_link_push_event`、`agent_link_report_selected_agent`、`agent_link_recording_start/data/end`、`agent_link_push_video`。

## 控制面与数据面

| | 控制面 | 数据面 |
|---|---|---|
| 承载 | 命令、响应、事件（小而可靠） | 语音、录音、文件（高吞吐） |
| BLE 通道 | 服务 `0xFFC0` 上的 GATT 写/通知 | 语音上行：GATT Notify `0xFFA1`（事件 `0x40`）。TTS 下行、录音、文件：L2CAP CoC（PSM `0x0081`） |
| 传输方法 | `send_ctrl` | `stream_start` / `send_stream` / `stream_end` |
| 设备 API | `report_battery`、`push_reading`、`push_event`、`on_command`、`on_custom` | `push_voice` / `voice_end`、`on_audio_out`、`recording_*`、`push_video` |

## 最小接入示例

```c
#include "agent_link.h"

static void my_play(const uint8_t* pcm, size_t n, void* ctx) { /* 写喇叭 codec */ }
static void my_draw(const char* utf8, void* ctx)            { /* 渲染到屏幕 */ }

void app_start(void) {
    static agent_output_cb_t out = { .on_audio_out = my_play, .on_show_text = my_draw };
    agent_link_config_t cfg = {
        .device_name = "MyThing",
        .caps        = AGENT_CAP_MIC | AGENT_CAP_SPEAKER | AGENT_CAP_SCREEN,
        .output      = &out,
    };
    agent_link_init(&cfg);
    agent_link_start();
    // 之后：麦克风数据交给 agent_link_push_voice(pcm, n)
}
```

本仓库里这层胶水已经写在 `main/app_main.cpp`，它把音频、文字、震动几个回调转发给所选的 `Board`。你只需写自己的板子。

## 添加一块板子

一块板就是三个文件，最省事的办法是照 `boards/rorolee-s3/` 抄：

```
boards/my-board/
├── config.h      # 引脚与特性宏
├── config.json   # 目标芯片、sdkconfig 片段、依赖
└── my_board.cc   # class MyBoard : public Board { ... };  DECLARE_BOARD(MyBoard);
```

再登记两处：在 `main/Kconfig.projbuild` 加一个 `BOARD_TYPE_MY_BOARD` 选项，在 `main/CMakeLists.txt` 的选板链里加一个 `elseif` 分支。`main/app_main.cpp` 和 `main/board.h` 都不用改。完整步骤见 [`boards/README.md`](boards/README.md)。

## 支持的芯片

目前只实现了 BLE（NimBLE）传输，所以目标芯片需要带片上 BLE 射频：ESP32、ESP32-S3、ESP32-C3、ESP32-C6、ESP32-H2 等。没有原生射频的芯片（ESP32-S2、ESP32-P4）要等 WiFi 或 USB 后端，那部分目前只有骨架（`transport_wifi.cpp`），尚未实现。芯片间的差异全部收在 `boards/<板>/` 里；换芯片就是 `idf.py set-target <chip>` 再选对应的板。

## 目录结构

```
agent_link/
├── CMakeLists.txt               顶层工程
├── sdkconfig.defaults
├── main/                        共享 app，与板无关
│   ├── app_main.cpp             把 agent_link 回调绑定到所选 Board
│   ├── board.h                  抽象 Board 接口 + DECLARE_BOARD
│   ├── Kconfig.projbuild        Board Type 选择
│   └── CMakeLists.txt           编译所选板的源码
├── boards/                      每块板一个目录
│   ├── README.md                如何加一块板
│   ├── common/                  跨板复用的驱动（codec、屏）
│   ├── rorolee-s3/              参考板（ESP32-S3）
│   └── ...                      esp32p4Waveshare、m5stack、tem_monitor
├── components/
│   ├── agent_link/              SDK（协议 + 传输），有自己的 README
│   └── esp_lcd_sh8501/          参考板用到的 SH8501 屏驱动
└── docs/                        协议与设计说明
```

## 现状

| 部分 | 状态 |
|---|---|
| BLE 广播、可连接、控制服务 `0xFFC0` | 已完成 |
| 控制面：App 到设备的命令（回 ACK + 路由）、设备到 App 的事件（如电量 `0x14`）、连接状态回 `on_state` | 已完成 |
| 语音上行：`push_voice`/`voice_end` 到 GATT Notify `0xFFA1`（`0x40 VoiceChunk`），含按 MTU 切片与背压 | 已实现，尚未真机验证 |
| 语音下行：App 经 L2CAP CoC `0x0081` 推 PCM 到 `on_audio_out` | 已实现，尚未真机验证 |
| 通用 I/O：`register_io`、`push_reading`、执行器下行（manifest `0x18`、读数 `0x19`、执行 `0x33`、拉取 `0x34`） | 已实现，尚未真机验证 |
| 录音、文件/OTA 走 L2CAP 上行 | 未实现 |
| 视频（`push_video`） | 只有接口，需要 WiFi/WebRTC 后端 |
| WiFi / USB 传输后端 | 仅骨架（`transport_wifi.cpp`） |
| 参考板 `rorolee-s3` | 屏幕、ES8311/ES7210 codec、按键说话（PTT）、BQ27220 电量计都已接上。屏上文字暂为占位（纯色填充，无字体渲染）；震动仅打日志 |

大致推进顺序：控制面（完成）→ 语音（完成）→ 通用 I/O（完成）→ L2CAP 录音/文件 → WiFi 后端 → 视频。

## BLE 协议

帧是 6 字节头加负载：`version (0x01)`、`message_type`、`command_id`、`sequence`、`payload_len`（小端 uint16），后面跟负载。`message_type` 低 7 位为 `0x01` 命令、`0x02` 响应、`0x03` 事件；最高位 `0x80` 表示负载加密（加密尚未启用，因此帧是明文）。

GATT 布局：

- 服务 `0xFFC0`（控制）：`0xFFC1` 命令（App 写，设备通知回响应）、`0xFFC4` 事件（设备通知）。
- 服务 `0xFFA0`（语音）：`0xFFA1` 通知，承载语音上行事件 `0x40 VoiceChunk`。
- 标准服务 `0x180F` 电量（`0x2A19`）与 `0x180A` 设备信息（`0x2A29` 厂商、`0x2A24` 型号、`0x2A26` 固件版本）。
- L2CAP CoC，PSM `0x0081`：接收下行 TTS 音频并转给 `on_audio_out`。同一通道上的录音、文件、OTA 上行尚未实现。

通用 I/O 走控制面：自描述清单（事件 `0x18`）、带类型的读数（事件 `0x19`）、执行器命令（`0x33`）、清单拉取（`0x34`），格式见 [`docs/device-io.md`](docs/device-io.md)。语音上行帧是 6 字节头后接 `session_id`（4）、`sequence`（4）、`flags`（1）和 PCM；整帧控制在 220 字节以内，每次通知的 PCM 取 `min(MTU - 3 - 15, 205)` 并向下取偶数。

## 仅复用 SDK

`components/agent_link/` 是一个独立的 ESP-IDF 组件。把它拷进你工程的 `components/`，在 `main/CMakeLists.txt` 的 `REQUIRES` 里加上 `agent_link`，再 `#include "agent_link.h"` 即可。细节见[组件 README](components/agent_link/README.md)。

## 许可证

本项目以 MIT 许可证发布，见 [LICENSE](LICENSE)。Copyright (c) 2026 DEOTALAND LIMITED（德奧塔文化科技有限公司）。

板级目录布局参考了 [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)（同为 MIT）。`components/esp_lcd_sh8501/` 与 `managed_components/` 下的乐鑫驱动为 Apache-2.0，各自保留其许可证声明。
