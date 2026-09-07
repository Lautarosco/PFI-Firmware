# Controller Architecture

This document describes the controller implementation as it exists in the
firmware. The main implementation is in
`components/controllers/src/controllers.c`; its public data model is in
`components/controllers/include/controllers_structs.h`.

## System Position

The controller component produces corrections for the state-machine control
loop. The runtime path is:

```text
sensor task -> drone.attributes.states
                         |
                         v
StControlFunc() -> PID controllers -> MMA mixer -> PWM channels -> ESCs
                         ^
                         |
                 drone.attributes.sp
```

`StControlFunc()` runs in `vTaskStateMachine_Run()`. It snapshots the current
state and setpoints, updates the cascaded controllers, sends selected outputs
to the motor-mixing algorithm, and writes the mixer outputs to PWM.

## Controller Storage

Every controller is stored by value in the drone object:

```c
typedef struct drone_components {
    pid_controller_t controllers[8];
} drone_components_t;
```

The array index is the `states_t` enum value and is part of the architecture:

| Index | Controller | Role |
| ---: | --- | --- |
| `0` | `Z` | Outer altitude/position loop |
| `1` | `Z_D` | Inner vertical-rate loop |
| `2` | `ROLL` | Outer roll-angle loop |
| `3` | `PITCH` | Outer pitch-angle loop |
| `4` | `YAW` | Outer yaw-angle loop |
| `5` | `ROLL_D` | Inner roll-rate loop |
| `6` | `PITCH_D` | Inner pitch-rate loop |
| `7` | `YAW_D` | Inner yaw-rate loop |

The controller is not dynamically allocated. A caller passes the address of
one array element to all constructors, initializers, updates, and action
functions.

## `pid_controller_t` Data Model

`pid_controller_t` combines configuration, runtime state, and function
pointers:

- `error` and `prev_error` hold the current and previous error.
- `tag` identifies the controller and must match its array index.
- `p`, `i`, and `d` expose the most recently stored action values; `out` stores
  the saturated final output.
- `ts_ms` is the controller sampling period.
- `gain` contains `kp`, `ki`, `kd`, and `kb` (back-calculation gain).
- `integrator` is the accumulated integral state.
- `derivative_lpf` stores the low-pass derivative output and `alpha` value.
- `integral_limits` and `pid_out_limits` define saturation boundaries.
- `init_ok` prevents use before initialization.

The function pointers make the structure object-like C. `init` initializes one
instance, `pidUpdate` runs it, and `PidSetActionP/I/D` replace the selected
action functions. The three action pointers share this signature:

```c
typedef float ControllerFunction(pid_controller_t *pid, float error);
```

The action receives the controller pointer, so it can read gains and mutate
state without global controller variables.

## Construction and Initialization

`Drone()` first zeroes the complete `drone_t`, constructs each PID object with
the default action set, and assigns the global gain pointers used by command
and transmitter code:

```text
Drone()
  -> Pid(controller[i], P_Basic, I_Clamping, D_Basic)
       -> memset(controller, 0, sizeof(pid_controller_t))
       -> assign init, pidUpdate, setter, and action pointers
  -> get_drone_params(drone)
```

When the state machine enters `ST_INIT`, `drone_init()` initializes each
controller with:

```text
tag       = array index
ts_ms     = drone.attributes.ts_ms (currently 1 ms)
gains     = { kp=0, ki=0, kd=0, kb=0 }
limits    = DroneConfigs.pid_cfgs[i]
integrator/error/prev_error = 0
init_ok   = true
```

After initialization, `read_from_flash()` loads persisted parameter values.
Therefore, the runtime gains normally come from NVS, while the default config
provides limits and other static settings. If no persisted gains exist, the
controllers remain at the zero-gain values passed by `drone_init()`.

## Function-Pointer Wiring

The default wiring is:

```text
pFunc -> P_Basic
iFunc -> I_Clamping
dFunc -> D_Basic
```

`PidSetActionP()`, `PidSetActionI()`, and `PidSetActionD()` simply replace one
pointer. The command task exposes the same mechanism through the action table:

```text
<pid actions,roll_d,D_LPF>
  -> PidActionsCmdFunc()
  -> resolve "roll_d" to ROLL_D
  -> resolve "D_LPF" to D_LPF
  -> PidSetActionD(&controllers[ROLL_D], D_LPF)
```

Available actions are `P_Basic`, `P_Quadratic` (implemented but not included
in the command table), `I_Basic`, `I_Clamping`, `I_BackCalc`, `D_Basic`,
and `D_LPF`. The command parser accepts four-field framed input and dispatches
`pid actions`, `var update`, `sp update`, and `nvs_store` commands.

## PID Update Algorithm

`pidUpdate(self, pv, sp)` performs these steps:

1. Reject an uninitialized object and restart the MCU.
2. Compute `error = sp - pv`.
3. Convert the error from degrees to radians for every controller except `Z`.
4. Call `pFunc`, then `iFunc`, then `dFunc`.
5. Store the returned actions in `p`, `i`, and `d`.
6. Sum `P + I + D`.
7. Saturate the sum to `pid_out_limits` and store it in `out`.

The integral strategies are stateful:

- `I_Basic` always adds `error * ts_ms` to `integrator`.
- `I_Clamping` estimates a temporary output using `pFunc` and `dFunc`, skips
  integration when the output is saturated in the same direction as the
  error, and otherwise updates the integrator.
- `I_BackCalc` estimates an unsaturated output, computes the saturation error,
  applies `kb`, and limits the resulting integral action.

The derivative strategies are also stateful. `D_Basic` calculates
`error - prev_error` and immediately writes the new `prev_error`. `D_LPF`
calculates a time-scaled derivative and updates `derivative_lpf.out` through
`FirstOrderIIR()`.

## Cascaded Control and Mixer Integration

`StControlFunc()` calls controllers in cascaded pairs:

```text
ROLL angle PID  -> sp.roll_dot
ROLL_D rate PID -> CRolld -> MMA C_ROLL

PITCH angle PID -> sp.pitch_dot
PITCH_D rate PID -> CPitchd -> MMA C_PITCH (currently multiplied by 0)

Z position PID  -> sp.z_dot
Z_D rate PID    -> CZd (currently multiplied by 0)
```

Yaw controllers are not currently called by `StControl.c`, so no yaw
correction reaches the MMA input. The mixer is capable of combining roll,
pitch, yaw, and base thrust corrections into four outputs, clamps each duty
cycle to PWM limits, and the control function writes the outputs to the four
PWM objects at most once every 20 ms.

## Configuration and External Pointers

`drone_cfg_t.pid_cfgs[8]` stores per-controller integral and output limits.
The `.tag` field in each configuration entry documents the intended mapping,
but initialization passes the loop index directly, so array order remains the
actual source of identity.

`get_drone_params()` builds a table of names and addresses for NVS and command
updates. It binds `roll/P`, `roll_d/D`, `pitch/I`, `yaw_d/D_IIR`, and the
other controller parameters directly to fields inside the controller array. A
parameter update therefore changes the live controller immediately; NVS
storage is separate and requires the `nvs_store` command.

The global pointers `GlobalRollGains`, `GlobalRoll_dGains`, `GlobalPitchGains`,
`GlobalPitch_dGains`, `GlobalYawGains`, and `GlobalYaw_dGains` point directly
to the corresponding `gain` structs. The Z gain pointers are declared but
their assignments are currently commented out.

`D_Basic()` also uses a `container_of` expression to derive the enclosing
`drone_t` from a controller pointer and its `tag`. The derived pointer is not
currently used, so the controller still depends on the drone layout without
gaining behavior from that relationship.

## Current Implementation Risks

These are documented observations from the current code, not design
assumptions:

- `I_Clamping()` calls `dFunc()` while estimating saturation, and
  `pidUpdate()` calls it again for the final output. Because `D_Basic()` writes
  `prev_error`, the final derivative can become zero. See
  `docs/troubleshooting/derivative-term-zero.md`.
- `I_BackCalc()` has the same nested derivative-call pattern.
- `D_LPF()` updates the filtered value but returns the unfiltered derivative and
  does not update `prev_error`, so selecting it changes the state semantics.
- `I_Basic()` and `I_Clamping()` multiply by `ts_ms` in milliseconds, while
  `D_LPF()` explicitly converts milliseconds to seconds. Gain tuning must account
  for this inconsistency.
- `I_Clamping()` contains unreachable legacy logic after an unconditional
  return; its `int_update` variable is therefore unused in the active path.
- The Z_D controller is called but its output is multiplied by zero, and pitch
  and yaw contributions are currently disabled or omitted downstream.
- The controller component declares a build-time dependency on `drone` and
  includes `drone.h` for `D_Basic()`, while `drone` also requires
  `controllers`. This coupling should be considered before moving either
  component.

## Extension Checklist

When adding a controller or action:

1. Add or preserve the `states_t` index and the matching controller array slot.
2. Add limits and the matching `pid_cfgs` entry.
3. Initialize or bind gains through the NVS parameter table if they are
   runtime-configurable.
4. Decide whether the action is stateless; action functions that mutate
   history must not be called speculatively without accounting for that state
   change.
5. Connect the controller output to the appropriate cascade, mixer input, and
   telemetry field.
