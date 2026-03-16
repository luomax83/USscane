# USscane

Current baseline for the real hardware integration project.

Repository layout:

- `host_app/`: PC host application used for scan, debug, waveform upload, and installer packaging.
- `H743VGT6v2/`: STM32H743 target firmware for the current hardware platform.
- `debug_capture/`: debugger-assisted waveform capture helpers used during ADC bring-up.

Branch guidance:

- `main`: legacy snapshot history from the earlier F103/simulation stage.
- `archive/f103-sim-main-20260317`: preserved rollback point for the old repository state.
- `baseline/h743-hardware-ok-20260317`: validated H743 hardware baseline before motor integration.

Recommended workflow for the next stage:

1. Create the motor work from `baseline/h743-hardware-ok-20260317`.
2. Keep hardware bring-up changes isolated from motor-control experiments.
3. Merge into `main` only after motor integration is validated on hardware.
