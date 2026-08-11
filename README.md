# agent_link

**English** | [简体中文](README.zh-CN.md)

A connectivity layer that puts embedded devices on the **Deotaland Agent platform**. A device declares which capabilities it has (microphone, speaker, screen, sensors, and so on) and registers a few callbacks; the library handles BLE advertising, GATT, and frame encoding. There is no Bluetooth or wire-protocol code to write on the device side.

The repository is a complete ESP-IDF project. The reusable SDK lives in `components/agent_link/`; everything around it is reference firmware you can build and flash as-is.

## What you implement

1. **Capabilities**: a bitmask of `AGENT_CAP_*` flags describing the hardware you have.
2. **Output callbacks**: the Agent calls these to drive your hardware (play audio, show text, vibrate). Implement the ones you support and leave the rest `NULL`.
3. **Push calls**: the device calls `agent_link_push_*` to send data up: microphone audio, sensor readings, battery level.

Everything below that (BLE, GATT, L2CAP, framing) is the SDK's job.

## Design

| Principle | Consequence |
|---|---|
| Capability-driven | The device declares its hardware as a `caps` bitmask and implements only the matching callbacks; unsupported features stay off. Sensors and actuators additionally announce themselves in a manifest sent on connect. |
| Callbacks in, push-calls out | Downlink (Agent to device) arrives as callbacks; uplink (device to Agent) goes through `agent_link_push_*`. The SDK never touches a GPIO, which is what keeps it portable. |
| Transport behind an interface | The core talks to an abstract `agent_transport_t`. BLE is implemented today; the same core is meant to run over WiFi or USB once those backends exist, with no change to device code. |
| Board as a class | Each board is a small C++ class implementing `Board` (`PlayAudio`, `ShowText`, `Vibrate`, battery). Adding or swapping a board touches only its own directory. |

## Architecture

```
              Cloud Agent  <-->  Phone App
                      |
                   ( BLE )
   ============================================================
    components/agent_link/        the SDK (chip-independent)
      transport_ble.cpp           NimBLE advertising / GATT / L2CAP
      protocol.cpp                frame encode / decode
      agent_link.cpp              capability routing + lifecycle
   ============================================================
      uplink  push_*/report_*  ^      v  downlink  on_* callbacks
   ============================================================
    main/app_main.cpp             shared app (board-independent)
      binds agent_link callbacks to Board::GetInstance()
   ============================================================
    boards/<board>/               your hardware: pins + drivers
      config.h   config.json   <board>.cc   (implements Board)
   ============================================================
```

Uplink: the board calls `agent_link_push_*`, the core builds a frame, and the transport sends it. Downlink: the transport receives a frame, the core routes it by capability, and your callback drives the hardware.

## Build and flash

You need ESP-IDF v5.0 or newer and a board with an on-chip BLE radio. From the project root:

```bash
idf.py set-target esp32s3     # match your board's chip (see "target" in boards/<board>/config.json)
idf.py menuconfig             # Agent Link Device -> Board Type (defaults to rorolee-s3)
idf.py build
idf.py -p <port> flash monitor
```

On boot the log looks roughly like this:

```
agent_link.app: board = ROROLEE_S3, caps = 0x023f
agent_link: init: name='ROROLEE_S3' caps=0x023f proto=v1 transport=ble io=0
agent_link.ble: BLE started — Service C 0xFFC0 registered; advertising on sync
agent_link.ble: advertising as 'ROROLEE_S3'
```

Scan with a BLE tool such as nRF Connect or LightBlue. If the device shows up by name and exposes control service `0xFFC0`, the link is up.

## Capabilities

`agent_cap_t` is a bit per capability, combined with bitwise OR:

```
MIC  SPEAKER  SCREEN  BUTTON  HAPTIC  BATTERY  LED  SENSOR  ACTUATOR  RECORDING  CAMERA
```

Set the bits for the hardware you have, then implement the callbacks and push calls that match them.

## Output callbacks (Agent to device)

Fields of `agent_output_cb_t`. Implement what you support; the rest stay `NULL`.

| Callback | Called when |
|---|---|
| `on_audio_out(pcm16, bytes)` / `on_audio_end()` | TTS audio arrives (PCM16, 16 kHz, mono), segment by segment |
| `on_show_text(utf8)` / `on_show_image(rgb565_be, w, h)` | Text or an image should be shown on screen |
| `on_haptic(ms)` / `on_led(rgb)` / `on_actuate(ch, val)` | Vibration, LED, or a generic actuator |
| `on_agent_list(json)` | The platform sends the list of available Agents for a selection UI |
| `on_command(cmd, payload, len, resp, cap, *resp_len)` | A query that must return data (the various `Get*`): fill `resp`, return `true`, and the SDK replies with it. `0x03 GetChargingStatus` is answered by the SDK from its cached battery value |
| `on_custom(cmd, payload, len)` | Escape hatch for device-private commands, fire-and-forget |

## Pushing data up (device to Agent)

Implemented today:

- `agent_link_push_voice` / `agent_link_voice_end`: stream microphone PCM (16 kHz, 16-bit, mono).
- `agent_link_register_io` / `agent_link_push_reading`: declare sensors and actuators, then report readings. Actuator commands are routed back to the callback you registered. See [`docs/device-io.md`](docs/device-io.md).
- `agent_link_report_battery`: battery percentage and charging state, de-duplicated internally.

Declared but not yet wired to the transport (see [Status](#status)): `agent_link_push_event`, `agent_link_report_selected_agent`, `agent_link_recording_start/data/end`, `agent_link_push_video`.

## Control plane vs data plane

| | Control plane | Data plane |
|---|---|---|
| Carries | Commands, responses, events (small, reliable) | Voice, recording, files (high throughput) |
| BLE channel | GATT write/notify on service `0xFFC0` | Voice uplink: GATT Notify `0xFFA1` (event `0x40`). TTS downlink, recording, files: L2CAP CoC (PSM `0x0081`) |
| Transport call | `send_ctrl` | `stream_start` / `send_stream` / `stream_end` |
| Device API | `report_battery`, `push_reading`, `push_event`, `on_command`, `on_custom` | `push_voice` / `voice_end`, `on_audio_out`, `recording_*`, `push_video` |

## A minimal integration

```c
#include "agent_link.h"

static void my_play(const uint8_t* pcm, size_t n, void* ctx) { /* write to the speaker codec */ }
static void my_draw(const char* utf8, void* ctx)            { /* render to the screen */ }

void app_start(void) {
    static agent_output_cb_t out = { .on_audio_out = my_play, .on_show_text = my_draw };
    agent_link_config_t cfg = {
        .device_name = "MyThing",
        .caps        = AGENT_CAP_MIC | AGENT_CAP_SPEAKER | AGENT_CAP_SCREEN,
        .output      = &out,
    };
    agent_link_init(&cfg);
    agent_link_start();
    // then: feed the mic with agent_link_push_voice(pcm, n)
}
```

In this repository that glue already exists in `main/app_main.cpp`, which forwards the audio, text, and haptic callbacks to the selected `Board`. You only write the board.

## Adding a board

A board is three files, most easily copied from `boards/rorolee-s3/`:

```
boards/my-board/
├── config.h      # pins and feature macros
├── config.json   # target chip, sdkconfig fragment, dependencies
└── my_board.cc   # class MyBoard : public Board { ... };  DECLARE_BOARD(MyBoard);
```

Then register it in two places: add a `BOARD_TYPE_MY_BOARD` choice in `main/Kconfig.projbuild`, and one `elseif` branch in the board-select chain of `main/CMakeLists.txt`. `main/app_main.cpp` and `main/board.h` do not change. The full walkthrough is in [`boards/README.md`](boards/README.md).

## Supported targets

The only transport implemented so far is BLE (NimBLE), so the target needs an on-chip BLE radio: ESP32, ESP32-S3, ESP32-C3, ESP32-C6, ESP32-H2, and similar. Parts without a native radio (ESP32-S2, ESP32-P4) will depend on the WiFi or USB backend, which is scaffolded (`transport_wifi.cpp`) but not implemented. Per-chip differences live entirely under `boards/<board>/`; changing chip is `idf.py set-target <chip>` plus selecting the matching board.

## Repository layout

```
agent_link/
├── CMakeLists.txt               top-level project
├── sdkconfig.defaults
├── main/                        shared app, board-independent
│   ├── app_main.cpp             binds agent_link callbacks to the selected Board
│   ├── board.h                  abstract Board interface + DECLARE_BOARD
│   ├── Kconfig.projbuild        Board Type selection
│   └── CMakeLists.txt           compiles the selected board's sources
├── boards/                      one directory per board
│   ├── README.md                how to add a board
│   ├── common/                  drivers shared across boards (codec, panels)
│   ├── rorolee-s3/              reference board (ESP32-S3)
│   └── ...                      esp32p4Waveshare, m5stack, tem_monitor
├── components/
│   ├── agent_link/              the SDK (protocol + transport); has its own README
│   └── esp_lcd_sh8501/          SH8501 panel driver used by the reference board
└── docs/                        protocol and design notes
```

## Status

| Area | State |
|---|---|
| BLE advertising, connectable, control service `0xFFC0` | Done |
| Control plane: App-to-device commands (ACK + routing), device-to-App events (e.g. battery `0x14`), connection state to `on_state` | Done |
| Voice uplink: `push_voice`/`voice_end` to GATT Notify `0xFFA1` (`0x40 VoiceChunk`), with MTU-aware slicing and backpressure | Implemented, not yet validated on hardware |
| Voice downlink: App pushes PCM over L2CAP CoC `0x0081` to `on_audio_out` | Implemented, not yet validated on hardware |
| Device I/O: `register_io`, `push_reading`, actuator downlink (manifest `0x18`, reading `0x19`, actuate `0x33`, pull `0x34`) | Implemented, not yet validated on hardware |
| Recording and file/OTA transfer over L2CAP uplink | Not implemented |
| Video (`push_video`) | Interface only; needs the WiFi/WebRTC backend |
| WiFi / USB transport backends | Scaffolding only (`transport_wifi.cpp`) |
| Reference board `rorolee-s3` | Display, ES8311/ES7210 codec, push-to-talk mic, and BQ27220 fuel gauge are wired. On-screen text is a placeholder (solid fill, no font rendering); haptic logs only |

Rough order of work: control plane (done) → voice (done) → device I/O (done) → L2CAP recording/file → WiFi backend → video.

## BLE protocol

Frames are a 6-byte header plus payload: `version (0x01)`, `message_type`, `command_id`, `sequence`, `payload_len` (little-endian uint16), then the payload. The low 7 bits of `message_type` are `0x01` command, `0x02` response, `0x03` event; the top bit `0x80` marks an encrypted payload (encryption is not enabled yet, so frames are plaintext).

GATT layout:

- Service `0xFFC0` (control): `0xFFC1` command (App writes, device notifies the response), `0xFFC4` event (device notifies).
- Service `0xFFA0` (voice): `0xFFA1` notify, carrying voice-uplink event `0x40 VoiceChunk`.
- Standard services `0x180F` Battery (`0x2A19`) and `0x180A` Device Information (`0x2A29` manufacturer, `0x2A24` model, `0x2A26` firmware revision).
- L2CAP CoC on PSM `0x0081` receives downlink TTS audio and forwards it to `on_audio_out`. Recording, file, and OTA uplink over the same channel are not implemented yet.

Device I/O rides the control plane: a self-describing manifest (event `0x18`), typed readings (event `0x19`), an actuator command (`0x33`), and a manifest pull (`0x34`). Formats are specified in [`docs/device-io.md`](docs/device-io.md). A voice-uplink frame is the 6-byte header followed by `session_id` (4), `sequence` (4), `flags` (1), and PCM; the whole frame is kept at or below 220 bytes, with per-notify PCM sized to `min(MTU - 3 - 15, 205)` and rounded down to an even length.

## Using only the SDK

`components/agent_link/` is a standalone ESP-IDF component. Copy it into your project's `components/`, add `agent_link` to `REQUIRES` in your `main/CMakeLists.txt`, and `#include "agent_link.h"`. Details are in the [component README](components/agent_link/README.md).

## License

Released under the MIT License; see [LICENSE](LICENSE). Copyright (c) 2026 DEOTALAND LIMITED (德奧塔文化科技有限公司).

The board directory layout takes inspiration from [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32), which is also MIT. The Espressif drivers in `components/esp_lcd_sh8501/` and under `managed_components/` are licensed Apache-2.0 and keep their own notices.
