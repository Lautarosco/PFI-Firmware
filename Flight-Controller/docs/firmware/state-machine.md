# State Machine

## Overview

The flight controller uses a table-driven state machine to select the active
drone behavior. `vTaskStateMachine_Run()` initializes the machine and repeats
the following cycle:

1. Read pending button or requested-transition events.
2. Find the first matching `(current state, event)` row in
   `state_trans_matrix`.
3. Update `curr_state` to the row's `next_state`.
4. Execute the function registered for the new state.
5. Delay for `drone->attributes.ts_ms` milliseconds.

The initial state is `ST_IDLE`. The transition and handler definitions are in
`components/state_machine/include/state_machine.h` and
`components/state_machine/src/state_machine.c`.

## Current States

| State | Handler | Responsibility |
| --- | --- | --- |
| `ST_IDLE` | `StIdleFunc` | Safe idle state; no active work. |
| `ST_INIT` | `StInitFunc` | Initialize the drone and average initial roll measurements. |
| `ST_WAITING` | `StWaitingFunc` | Hold before calibration, control, or motor testing. |
| `ST_CALIBRATION` | `StCalibrationFunc` | Calibrate ESCs through a user-guided throttle sequence. |
| `ST_CONTROL` | `StControlFunc` | Run cascaded PID control and update motor outputs. |
| `ST_PROPELLER_CALIBRATION` | `StVibrationCheck` | Test motors and report vibration measurements. |
| `ST_RESET` | `StResetFunc` | Set PWM outputs to minimum and restart the MCU. |

## Transitions

| Current state | Event | Next state | Meaning |
| --- | --- | --- | --- |
| `ST_IDLE` | `EV_CROSS` | `ST_INIT` | Begin initialization. |
| `ST_IDLE` | `EV_PS` | `ST_RESET` | Reset from idle. |
| `ST_IDLE` | `EV_ANY` | `ST_IDLE` | Remain idle. |
| `ST_INIT` | `EV_ANY` | `ST_WAITING` | Finish initialization cycle. |
| `ST_INIT` | `EV_PS` | `ST_RESET` | Abort with reset. |
| `ST_WAITING` | `EV_TRIANGLE` | `ST_CALIBRATION` | Start ESC calibration. |
| `ST_WAITING` | `EV_CIRCLE` | `ST_CONTROL` | Start flight control. |
| `ST_WAITING` | `EV_SQUARE` | `ST_PROPELLER_CALIBRATION` | Start motor/vibration testing. |
| `ST_WAITING` | `EV_PS` | `ST_RESET` | Reset while waiting. |
| `ST_CALIBRATION` | `EV_ANY` | `ST_WAITING` | Return after calibration. |
| `ST_CALIBRATION` | `EV_PS` | `ST_RESET` | Reset from calibration. |
| `ST_CONTROL` | `EV_ANY` | `ST_CONTROL` | Run the next control iteration. |
| `ST_CONTROL` | `EV_PS` | `ST_RESET` | Stop control and reset. |
| `ST_PROPELLER_CALIBRATION` | `EV_ANY` | `ST_PROPELLER_CALIBRATION` | Continue testing. |
| `ST_PROPELLER_CALIBRATION` | `EV_SQUARE` | `ST_WAITING` | Exit motor testing. |
| `ST_PROPELLER_CALIBRATION` | `EV_PS` | `ST_RESET` | Reset during testing. |

## Important Runtime Details

- `EV_ANY` is an actual default event value, not a wildcard. It is selected
  when no recognized button event is present.
- `EV_PS` is checked in every normal state and routes to `ST_RESET`.
- Calibration is synchronous and waits inside its handler for button presses.
- Vibration testing can request `EV_SQUARE` through
  `request_state_transition`; that request is consumed before normal button
  processing on the next loop.
- `EV_START`, directional buttons, and shoulder buttons are collected by the
  event reader, but currently have no transition rows outside the vibration
  handler's direct button checks.
