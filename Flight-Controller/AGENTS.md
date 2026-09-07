# Repository Guidelines

## Project Structure & Module Organization
This repository is an ESP-IDF firmware project for a flight controller.

- `main/src/main.c` is the application entry point.
- `components/` contains the reusable firmware modules, organized by subsystem such as `drone/`, `tasks/`, `state_machine/`, `transmitter/`, `gnss/`, `lsm6dso/`, `bmi160/`, `bmp390/`, `pwm/`, and `controllers/`.
- `spiffs/` stores SPIFFS filesystem content used by the firmware.
- `partitions.csv` defines the flash partition layout.
- `build/` is generated output and should not be edited manually.

## Build, Test, and Development Commands
Use ESP-IDF from a configured shell with `IDF_PATH` available.

- `idf.py build` compiles the firmware.
- `idf.py flash` writes the image to the connected ESP32 board.
- `idf.py monitor` opens the serial monitor after flashing.
- `idf.py build flash monitor` is the common edit-test loop.
- `idf.py fullclean` removes generated build artifacts when the build becomes inconsistent.

## Coding Style & Naming Conventions
The codebase follows C conventions with ESP-IDF component structure.

- Use 4-space indentation and keep alignment consistent with surrounding code.
- Prefer descriptive `snake_case` for functions, files, and variables.
- Type names and configuration structs often use a project prefix, such as `drone_t` or `pid_cfg_t`.
- Keep headers focused on declarations and constants; place implementation in the matching `src/` file.
- Preserve existing logging style with `ESP_LOGx()` and clear tags.

## Testing Guidelines
There is no dedicated automated test suite in the repository. Validation is typically done by:

- Building with `idf.py build`.
- Flashing to hardware and verifying behavior through `idf.py monitor`.
- Checking sensor, state-machine, and control-loop changes on the target board before merging.

If you add tests, keep names close to the module under test, for example `test_state_machine.c`.

## Commit & Pull Request Guidelines
Git history uses short, descriptive commits, often in Spanish, with a summary of the functional change and affected subsystem.

- Write commits in the imperative mood and keep them focused on one change.
- Mention hardware, control, or sensor impact when relevant.
- Pull requests should describe the change, note any firmware/config updates, and include board-test results.
- Add screenshots or serial logs only when they help explain a behavior change.

## Configuration Notes
Do not commit generated build output or local editor files. Keep board-specific configuration in source-controlled project files such as `sdkconfig`, `partitions.csv`, and component headers.
