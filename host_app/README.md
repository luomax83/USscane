# Ultrasonic Sonar Host Monitor

This host app is aligned with the binary sonar protocol and supports two transport modes:

- `SIMULATOR`: local loopback, no real device required
- `COMx`: real UART hardware

Recommended stack:

- `PySide6`: desktop UI
- `pyserial`: UART transport
- `pyqtgraph`: realtime scan and waveform plots

## Scope

- Scan mode
- Debug single-point mode
- Debug sweep mode
- Waveform upload and display
- Chinese / English UI switch
- Scan progress bar and ring counter
- Cartesian / polar scan view switch
- Optional 3D point cloud window
- Simulated anti-collision logic for axial stepping
- Local recording to `records/`

Reference documents:

- [protocol_v1_freeze.md](/D:/普中凤凰/cubeworkspace/docs/protocol_v1_freeze.md)
- [host_ui_spec.md](/D:/普中凤凰/cubeworkspace/docs/host_ui_spec.md)

## Layout

- [main.py](/D:/普中凤凰/cubeworkspace/host_app/main.py): main UI and transport worker
- [models.py](/D:/普中凤凰/cubeworkspace/host_app/sonar_host/models.py): enums and data models
- [protocol.py](/D:/普中凤凰/cubeworkspace/host_app/sonar_host/protocol.py): frame codec and parsers
- [simulator.py](/D:/普中凤凰/cubeworkspace/host_app/sonar_host/simulator.py): in-process loopback device

## Run

```powershell
cd D:\普中凤凰\cubeworkspace\host_app
.\.venv\Scripts\activate
python main.py
```

You can also double-click [start_host_app.bat](/D:/普中凤凰/cubeworkspace/start_host_app.bat) from the workspace root.

## Local Simulation

1. Start the UI.
2. Select `SIMULATOR` in the `Port` list.
3. Click `Connect`.
4. Use the `Scan` tab for `Configure Scan` and `Start Scan`.
5. Use the `Debug` tab for `Config Single`, `Move To Angle`, `Single Measure`, `Config Sweep`, and `Start Sweep`.
6. In the `Scan` tab you can enable `3D Point Cloud`, configure axial step and rings per position, and switch the plot between cartesian and polar views.

The app auto-requests device info after connect. Recording output is written under [records](/D:/普中凤凰/cubeworkspace/host_app/records) when `Start Record` is enabled.

## Real Hardware

- STM32 `PA9` -> USB-UART RX
- STM32 `PA10` -> USB-UART TX
- GND -> GND

Serial defaults:

- Baud rate: `115200`
- Data bits: `8`
- Stop bits: `1`
- Parity: `None`
