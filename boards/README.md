# Boards

A board is a set of hardware (codec, screen, buttons, motor, sensors) plus the capabilities it declares. The shared app ([`../main/app_main.cpp`](../main/app_main.cpp)) and the [`agent_link`](../components/agent_link/) SDK depend only on the abstract [`Board`](../main/board.h) interface, not on any particular board or chip, so adding or swapping a board leaves both untouched.

## A board is three files

Copy `boards/rorolee-s3/` as a starting point:

```
boards/rorolee-s3/
├── config.h        pins and feature macros (GPIO_NUM_*, resolution, I2C addresses)
├── config.json     manufacturer/type/target metadata (which chip -> which `idf.py set-target`)
└── rorolee_s3.cc   the board class: subclass Board, implement capabilities, end with DECLARE_BOARD(...)
```

`config.json` carries `manufacturer`, `type`, and `target` (the chip, e.g. `esp32s3`) — informational metadata; it tells you which `idf.py set-target` to run and isn't parsed by the build. Actual sdkconfig requirements go in two places (see "Adding a board" below): settings every board on a chip needs (flash size, PSRAM on/off, PSRAM mode/speed if the whole board family shares one module — every ESP32-S3 board here does, Octal @ 80MHz) in `sdkconfig.defaults.<target>`; settings only _this specific_ board needs, where the wanted value is itself a member of a Kconfig `choice` (an external RTC crystal on rorolee-s3/rorolee-basic/tem_monitor but not the others, say), patched directly into `sdkconfig` by the top of [`../CMakeLists.txt`](../CMakeLists.txt) — plain Kconfig `select` cannot force those (kconfiglib ignores `select`/`imply` aimed at a choice member), so don't reach for it there.

## The Board interface

`Board` (in [`../main/board.h`](../main/board.h)) is capability-level, not driver-level. `Name()` and `Capabilities()` are required; the rest are optional overrides that default to no-ops.

| Method                                   | Direction       | Override when the board has                                |
| ---------------------------------------- | --------------- | ---------------------------------------------------------- |
| `Name()`                                 | —               | required: display and BLE advertising name                 |
| `Capabilities()`                         | —               | required: bitwise OR of the `AGENT_CAP_*` bits you support |
| `PlayAudio(pcm16, bytes)` / `AudioEnd()` | Agent to device | a speaker                                                  |
| `ShowText(utf8)`                         | Agent to device | a screen                                                   |
| `Vibrate(ms)`                            | Agent to device | a motor                                                    |
| `GetBatteryLevel()` / `IsCharging()`     | device to Agent | a fuel gauge                                               |

Set a capability bit only when you implement its method; anything you leave out keeps the base no-op.

## What goes in `<board>.cc`

Three kinds of thing:

1. **Hardware bring-up**, in the constructor. Use the public registry components (`esp_codec_dev`, `esp_lcd_*`, `esp_driver_i2c`, `LEDC`) or the shared drivers in [`common/`](common/). Keep pin numbers in `config.h`.
2. **Capability methods.** Override the ones your hardware supports and set the matching `AGENT_CAP_*` bits in `Capabilities()`.
3. **Board-specific logic.** For example, a task that reads a sensor and reports it through the device-I/O API:

   ```c
   agent_link_io_desc_t t = { .id = "temp0", .dir = AGENT_IO_IN, .kind = "temperature",
                              .value = AGENT_VAL_F32, .unit = "C" };
   agent_link_register_io(&t, NULL, NULL);          // once, before agent_link_start()
   ...
   float c = read_sensor();
   agent_link_push_reading("temp0", &c, sizeof c);  // periodically
   ```

   `#include "agent_link.h"` and call `agent_link_push_*` directly. Actuators register a callback with `agent_link_register_io` and act when the Agent drives them. See [`../docs/device-io.md`](../docs/device-io.md) for the full sensor/actuator model, and `boards/tem_monitor/` for a worked example. If a board grows, add more `.cc`/`.h` files in its directory; they are compiled automatically.

> The device stays thin. What the Agent says and how it decides live in the cloud, not on the device, so a board is usually hardware plus a little glue rather than a full dialog state machine.

## Adding a board

Say `my-board` on an ESP32-C6:

1. Copy a directory: `cp -r boards/rorolee-s3 boards/my-board`.
2. Edit `config.h` with your pins.
3. Edit `config.json`: set `"target": "esp32c6"` (metadata only, for humans deciding `idf.py set-target`).
4. Edit the `.cc`: rename the class, set `Name()` and `Capabilities()`, implement your methods, and end with `DECLARE_BOARD(MyBoard);`.
5. Register it in three places:
   - add `config BOARD_TYPE_MY_BOARD` to the `choice BOARD_TYPE` in [`../main/Kconfig.projbuild`](../main/Kconfig.projbuild). Do **not** add `select` lines for board-specific sdkconfig needs there — see the next bullet for why and what to do instead;
   - add `elseif(CONFIG_BOARD_TYPE_MY_BOARD) set(BOARD_DIR "my-board")` to the board-select chain in [`../main/CMakeLists.txt`](../main/CMakeLists.txt). If the board needs a value that only _this_ board wants and that value is itself a member of a Kconfig `choice` (an external RTC crystal, a non-default console, ...), also add `BOARD_TYPE_MY_BOARD` (and its wanted `CONFIG_..._SRC=y`-style lines) to the per-board tables at the top of the repo-root [`../CMakeLists.txt`](../CMakeLists.txt) — plain Kconfig `select` on a choice member silently does nothing (kconfiglib: "select/imply has no effect on choice symbols"; verified against this project's own tree), so the values are patched directly into `sdkconfig` before Kconfig reads it instead. This reruns on every configure, so it self-corrects after switching `Board Type` in menuconfig, no manual `sdkconfig` editing needed. If instead the whole chip family shares the value (like PSRAM mode/speed for every ESP32-S3 board here), put it in `sdkconfig.defaults.<target>` instead — see the last bullet below.
   - If you use peripherals beyond what is already required, add their driver components (`esp_driver_i2c`, `esp_lcd`, `esp_codec_dev`) to `REQUIRES` in `main/CMakeLists.txt`.
   - If the board is on a chip target that isn't built yet (a new `esp32c6`, say), add a `sdkconfig.defaults.<target>` at the repo root for whatever _every_ board on that chip needs (flash size, PSRAM on/off, ...) — ESP-IDF merges it automatically on `idf.py set-target <target>`, the same way [`../sdkconfig.defaults.esp32p4`](../sdkconfig.defaults.esp32p4) already does for the P4 board. Plain `select` in `main/Kconfig.projbuild` is still the right tool for a board-specific value that is a plain bool, not a choice member (not common so far in this repo).

Then build:

```bash
idf.py set-target esp32c6
idf.py menuconfig        # Agent Link Device -> Board Type -> My Board
idf.py build flash
```

`../main/app_main.cpp` and `../main/board.h` do not change; they bind to the abstract `Board` and pick up the selected board automatically.

## Boards in this repo

The Board Type menu currently offers:

| Directory           | Target   | Notes                                                                                                                                             |
| ------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `rorolee-s3/`       | ESP32-S3 | Reference board: SH8501 AMOLED, ES8311/ES7210 codec, push-to-talk mic, BQ27220 fuel gauge, external 32kHz crystal                                 |
| `rorolee-basic/`    | ESP32-S3 | Like `rorolee-s3` plus SD card and buttons; external 32kHz crystal                                                                                |
| `tem_monitor/`      | ESP32-S3 | Sensor board: SPA06 pressure/temperature and SHT30 temperature/humidity over I2C; a worked example of the device-I/O path; external 32kHz crystal |
| `es8311-voice/`     | ESP32-S3 | Minimal example: one ES8311 codec doing full-duplex speaker + mic                                                                                 |
| `es8311-asr/`       | ESP32-S3 | Minimal example: one ES8311 codec, mic-only, streams PCM to the App for live ASR                                                                  |
| `gc2145-camera/`    | ESP32-S3 | GC2145 DVP camera live preview on an ST7789 240x240 LCD                                                                                           |
| `esp32p4Waveshare/` | ESP32-P4 | Waveshare board with a CO5300 466x466 AMOLED                                                                                                      |

Drivers used by more than one board live in [`common/`](common/)

## Reference

The layout (`boards/<board>/{config.h, config.json, <board>.cc}`, a Kconfig board choice, and a `target` in `config.json`) follows [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32). The difference is that agent_link owns networking and transport, so `Board` here describes only local hardware and does not split into `WifiBoard` / `Ml307Board` the way xiaozhi does; it exposes capability-level operations instead of driver types.
