# Real-Robot Tuning Logs

This directory stores one folder for every real-robot test. Templates and tools
are kept in Git; captured data under `runs/` is ignored by default.

## Create A Run

From `rm_test-dev`:

```bash
./tuning_logs/new_run.sh steer
```

This creates a directory similar to:

```text
tuning_logs/runs/2026-06-09_20-30-00_steer/
├── telemetry.csv
├── telemetry.raw
├── params.json
├── events.log
├── firmware.txt
└── notes.md
```

Put the exact test parameters in `params.json`. Record the test setup, support
frame, safety limits and observed behavior in `notes.md`. Do not overwrite an
old run when parameters change.

## Recommended Transfer Paths

### 1. SWD + SEGGER RTT

Use this first while the robot is suspended or fixed to a support frame. The
project already includes SEGGER RTT/SystemView and calls
`SEGGER_SYSVIEW_Conf()` during startup.

A J-Link can flash the firmware and read RTT through the same SWD connection.
Reading RTT does not require reflashing the MCU. Other probes can be used only
when their host software supports RTT or SWO reliably.

RTT is suitable for:

- Initial direction and sign checks
- State transition and fault event logs
- Low-to-medium-rate tuning telemetry
- Tests where a wired SWD connection cannot be pulled by the robot

Do not run unrestricted ground-motion tests with an SWD cable attached.

### 2. UART7 + Wireless Serial

Use this for real ground-motion tests. UART7 is already configured as
`921600 8N1`, with `PE7=RX` and `PE8=TX`, and has DMA support.

Connect a 3.3 V transparent wireless UART module to UART7. The PC receives the
stream through the paired USB receiver and stores it in the current run
directory. Use a dedicated telemetry task and DMA ring buffer; telemetry must
never block the 3 ms chassis control task.

The existing `USART_Vofa_Justfloat_Transmit()` packet contains only three
floats. It is useful for quick plots but is not sufficient for full tuning.
A versioned telemetry frame and PC decoder still need to be added before
`telemetry.csv` can be filled automatically.

### 3. USB CDC

USB CDC is already initialized and can provide a virtual serial port. It is
convenient on a support frame, but the USB cable has the same mechanical risk
as an SWD cable during movement.

## Sampling Plan

Fast frame, recommended 100-200 Hz:

- MCU timestamp and sequence number
- Chassis main state and steering sub-state
- Remote forward/steering input and emergency-stop flags
- Pitch, roll, yaw and three-axis gyro
- Estimated position/velocity and all setpoints
- Left/right `L0`, `theta`, `phi1`, `phi4`
- Wheel speed and final wheel torque command
- Four joint positions and final torque commands
- Saturation, offline, fall and dropped-frame flags

Slow frame, recommended 10-20 Hz:

- Bus voltage
- Motor current and temperature
- Task timing and telemetry queue usage

Use binary packets on the MCU and convert them to CSV on the PC. Avoid
formatting floating-point CSV strings inside the control task.

## Safety Rules

- Logging failure must only drop telemetry; it must never delay control.
- Parameter writes must be range checked.
- First tests should change parameters in RAM only.
- Saving parameters to flash is allowed only with motors disabled.
- Firmware updates are allowed only with motors disabled and the robot fixed.
- Keep a physical emergency stop independent from the telemetry link.
