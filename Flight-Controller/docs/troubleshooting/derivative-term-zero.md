# Derivative Term Always Zero

## Symptom

The PID derivative contribution (`D`) is zero during control, even when the
tracking error changes between control cycles. This makes the controller act
like a PI controller for the active `D_Basic` path.

## Root Cause

`pidUpdate()` calculates the integral action before the derivative action. The
active integral implementation, `I_Clamping()`, performs its own temporary
output calculation and calls `obj->dFunc()` once. `D_Basic()` updates
`obj->prev_error` as a side effect of that call. `pidUpdate()` then calls the
same derivative function again for the final output, using the same error:

```c
float derivative = error - obj->prev_error;
obj->prev_error = error;
```

By the second call, `prev_error == error`, so `derivative == 0`. The first
derivative result is used only for the anti-windup estimate and is discarded
from the final PID sum.

## Program Flow

The runtime path is:

```text
app_main()
  -> vTaskStateMachine_Run()
    -> StateMachine_RunIteration()
      -> StControlFunc()
        -> pidUpdate(controller, process_value, setpoint)
          -> self->error = setpoint - process_value
          -> self->iFunc(...) = I_Clamping(...)
               -> pFunc(...)
               -> dFunc(...) = D_Basic(...)
                    derivative = error - prev_error
                    prev_error = error
               -> anti-windup estimate
          -> self->dFunc(...) = D_Basic(...)
               derivative = error - prev_error = 0
          -> output = P + I + D
```

For example, with `prev_error = 0.20` and `error = 0.30`, the first call
computes `0.10` and stores `0.30`. The second call computes `0.30 - 0.30 = 0`.
Consequently, `kd` cannot contribute to the final output for that cycle.

## Evidence in Code

- `components/controllers/src/controllers.c`: `pidUpdate()` calls `iFunc()`
  and then `dFunc()`; `I_Clamping()` also calls `dFunc()`.
- `components/controllers/src/controllers.c`: `D_Basic()` both calculates the
  difference and mutates `prev_error`.
- `components/state_machine/src/StControl.c`: the control state invokes
  `pidUpdate()` for the cascaded roll, pitch, and altitude controllers.

## Related Observation

`D_LPF()` updates the filtered output but returns the unfiltered derivative and
does not update `prev_error`. It should be reviewed separately if that option
is enabled; it is not the cause of the confirmed `D_Basic` zero-output bug.
