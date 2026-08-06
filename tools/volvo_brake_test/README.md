# Volvo brake pulse diagnostic

This development-only diagnostic tests whether a Volvo accepts a longitudinal deceleration command when neither sunnypilot nor the stock ACC reports a lead vehicle. The phone page offers a short pulse and a persistent red-light-stop simulation.

It does **not** directly send CAN. The phone trigger enters `controlsd`, which produces the normal `carControl` command consumed by the Volvo `CarController` and panda safety. During the explicit stop window, `CarController` also echoes one centered Delphi ESR target immediately after the radar's native Target24 frame. Panda permits that narrowly bounded bus-1 frame only while the stop-only FSM4 transform is freshly armed and controls remain allowed with both pedals released.

## Guardrails

- Volvo with openpilot longitudinal control only
- armed from the comma while offroad; arming clears on reboot or the next offroad transition
- normal `controlsd` only; joystick and maneuver modes are disabled when armed
- sunnypilot and longitudinal control must already be active
- stock cruise must be enabled
- 30–40 km/h only
- no lead reported by `longitudinalPlan`
- gas and brake must both be released
- the raw radar target is limited to 4–90 m, -15.0–0.5 m/s relative speed,
  and -6.0–4.0 m/s² relative acceleration
- one test per onroad cycle; either a 2.25-second pulse or a red-light stop, both peaking at -1.5 m/s²
- gas, brake, disengagement, a detected lead, or exceeding 40 km/h cancels immediately; the pulse also cancels below 5 km/h

The red-light stop is allowed to continue below 5 km/h. It holds its stopping request at standstill until the driver presses a pedal, disengages, or presses **SIMULATE GREEN**. If the car does not reach standstill within 12 seconds, the diagnostic aborts and requests cruise cancellation.

After the red-light test reaches standstill, **SIMULATE GREEN** becomes available. It releases the artificial hold, briefly yields openpilot longitudinal control while the existing Volvo resume-button/ACC-ack sequence runs, and restores openpilot longitudinal control after the car begins moving. If no movement is detected within 3 seconds, the standstill hold is restored.

## Procedure

1. While parked and offroad, open Developer settings and enable **Arm Volvo Brake Pulse Test**. Confirm the warning.
2. Begin the drive and engage sunnypilot normally, including setting the Volvo ACC speed.
3. On a phone connected to the comma's local network, open `http://COMMA_IP:8089`.
4. On a clear, empty, straight road, stabilize between 30 and 40 km/h with no lead shown. Keep your foot ready over the brake.
5. When the page says **READY**, choose one test:
   - Hold **BRAKE PULSE** for one second for the original 2.25-second diagnostic.
   - Hold **STOP AT RED** for two seconds to request braking through zero and a standstill hold.
6. At standstill, either:
   - Hold **SIMULATE GREEN** for one second to test no-lead departure toward the set speed, keeping your foot ready over the brake.
   - Press and hold the brake to take over the standstill.
7. Press the brake or disengage immediately if the response is unexpected.

The decisive post-drive comparison is requested `carControl.actuators.accel` versus Volvo `Brake_Info.BrakeCmd`, brake pressure, and measured `carState.aEgo`. A nonzero brake command/pressure correlated with the injected pulse supports the no-lead spoof hypothesis; a negative requested acceleration with zero `BrakeCmd` and pressure rejects it.
