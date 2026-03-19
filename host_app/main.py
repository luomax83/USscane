from __future__ import annotations

import csv
import json
import math
import numpy as np
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Any

import pyqtgraph as pg
import serial
from PySide6 import QtCore, QtGui, QtWidgets
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from serial.tools import list_ports

from sonar_host.models import (
    CaptureType,
    CommandId,
    DebugConfig,
    DeviceStatus,
    DistanceResult,
    MediumType,
    ProbeType,
    ScanConfig,
    ScanPoint,
    WaveDataType,
)
from sonar_host.protocol import (
    Frame,
    FrameDecoder,
    decode_ack_payload,
    decode_nack_payload,
    encode_debug_config,
    encode_get_device_info,
    encode_move_to_angle,
    encode_scan_config,
    encode_start_debug_sweep,
    encode_start_scan,
    encode_start_single_measure,
    encode_stop,
    encode_zeroing,
    parse_debug_config_ack_detail,
    parse_debug_sweep_finish,
    parse_debug_sweep_point,
    parse_device_info_detail,
    parse_distance_result,
    parse_move_to_angle_ack_detail,
    parse_scan_config_ack_detail,
    parse_scan_finish,
    parse_scan_point,
    parse_status_event,
    parse_wave_chunk,
    parse_wave_finish,
    parse_zeroing_ack_detail,
)
from sonar_host.simulator import SimulatedDevice


APP_DIR = Path(__file__).resolve().parent
RECORD_ROOT = APP_DIR / "records"
SIMULATOR_PORT = "SIMULATOR"
ADC_REFERENCE_VOLTAGE = 3.3
ADC_FULL_SCALE = 65535.0
WAVE_SAMPLE_RATE_HZ = 3_000_000.0
MOTOR_ANGLE_RESOLUTION_DEG = 1.8
MOTOR_MAX_ANGLE_DEG = 360.0 - MOTOR_ANGLE_RESOLUTION_DEG
SIDE_SCAN_MIN_STEP_DEG = 1.8
SIDE_SCAN_DEFAULT_STEP_DEG = 5.4
SIDE_SCAN_MAX_STEP_DEG = 19.8
ANGLE_DISPLAY_DECIMALS = 3

EVENT_LABELS = {
    0x01: "Zeroing start",
    0x02: "Zeroing done",
    0x03: "Motion start",
    0x04: "Arrived",
    0x05: "Measure start",
    0x06: "Measure done",
    0x07: "Upload start",
    0x08: "Upload done",
    0xE0: "Timeout",
    0xE1: "Blocked",
    0xE2: "Acquire failed",
    0xE3: "Buffer overflow",
}

END_REASON_LABELS = {
    0x00: "Complete",
    0x01: "Stopped",
    0x02: "Motion fault",
    0x03: "Measure failed",
    0x04: "Timeout",
}

MOVE_DIR_LABELS = {
    0: "Hold",
    1: "CW",
    2: "CCW",
}

TEXTS = {
    "en": {
        "window_title": "Ultrasonic Sonar Host Console",
        "port": "Port",
        "baud": "Baud",
        "connect": "Connect",
        "disconnect": "Disconnect",
        "refresh": "Refresh",
        "get_info": "Get Info",
        "zeroing": "Zeroing",
        "stop": "Stop",
        "start_record": "Start Record",
        "stop_record": "Stop Record",
        "language": "Language",
        "scan_tab": "Scan",
        "debug_tab": "Debug",
        "scan_group": "Scan Parameters",
        "scan_summary": "Scan Summary",
        "step_angle": "Step Angle",
        "sound_speed": "Sound Speed",
        "medium": "Medium",
        "view_mode": "View Mode",
        "scan_view_cartesian": "Distance vs Angle",
        "scan_view_polar": "Polar",
        "expected_points": "Expected points: {value}",
        "scan_progress_text": "Progress: {done} / {total}",
        "configure_scan": "Configure Scan",
        "start_scan": "Start Scan",
        "save_scan_csv": "Save Scan CSV",
        "points_done": "Points Done",
        "valid_points": "Valid Points",
        "distance_range": "Distance Range",
        "session": "Session",
        "ring_count": "Ring Count",
        "axial_position": "Axial Position",
        "scan_3d_group": "3D Scan / Anti-collision",
        "enable_3d": "Enable 3D Point Cloud",
        "axial_step": "Axial Step",
        "rings_per_level": "Rings Per Position",
        "enable_collision": "Enable anti-collision",
        "collision_threshold": "Collision Threshold",
        "scan_depth": "Scan Depth",
        "front_distance": "Front Distance",
        "point_cloud_status": "3D Status",
        "debug_controls": "Debug Controls",
        "debug_state": "Debug State",
        "probe_type": "Probe Type",
        "depth_probe": "Depth Probe",
        "rotary_probe": "Side-scan Probe",
        "pulse_count": "Pulse Count",
        "sample_rate": "Sample Rate",
        "debug_sound_speed": "Wave Speed",
        "sweep_start": "Sweep Start",
        "sweep_end": "Sweep End",
        "sweep_step": "Sweep Step",
        "sample_count": "Sample Count",
        "capture_type": "Capture Type",
        "upload_distance": "Upload distance",
        "upload_wave": "Upload wave",
        "config_single": "Config Single",
        "move_to_angle": "Move To Angle",
        "single_measure": "Single Measure",
        "test_button": "Test",
        "config_sweep": "Config Sweep",
        "start_sweep": "Start Sweep",
        "save_wave_csv": "Save Wave CSV",
        "target": "Target: {value}",
        "direction": "Direction: {value}",
        "arrived": "Arrived: {value}",
        "distance": "Distance: {value}",
        "quality": "Quality: {value}",
        "wave": "Wave: {value}",
        "idle": "Idle",
        "disconnected": "Disconnected",
        "device_na": "Device: n/a",
        "status_idle": "Status: IDLE",
        "angle_fmt": "Angle: {value}",
        "mode_idle": "Mode: Idle",
        "event_none": "Event: none",
        "record_off": "Record: OFF",
        "no_active_scan": "Idle",
        "connected_loopback": "Connected: {port} @ {baud} (loopback)",
        "cartesian_plot_title": "Distance vs Angle",
        "polar_plot_title": "Polar Distance Map",
        "wave_plot_title": "Waveform",
        "debug_sweep_plot_title": "Debug Sweep Distance",
        "cloud_window_title": "3D Point Cloud",
        "lang_zh": "Chinese",
        "lang_en": "English",
    },
    "zh": {
        "window_title": "超声声纳上位机",
        "port": "端口",
        "baud": "波特率",
        "connect": "连接",
        "disconnect": "断开",
        "refresh": "刷新",
        "get_info": "读取信息",
        "zeroing": "回零",
        "stop": "停止",
        "start_record": "开始记录",
        "stop_record": "停止记录",
        "language": "语言",
        "scan_tab": "扫描",
        "debug_tab": "调试",
        "scan_group": "扫描参数",
        "scan_summary": "扫描统计",
        "step_angle": "步进角",
        "sound_speed": "声速",
        "medium": "介质",
        "view_mode": "视图模式",
        "scan_view_cartesian": "距离-角度",
        "scan_view_polar": "极坐标",
        "expected_points": "预计点数: {value}",
        "scan_progress_text": "进度: {done} / {total}",
        "configure_scan": "配置扫描",
        "start_scan": "开始扫描",
        "save_scan_csv": "保存扫描 CSV",
        "points_done": "已收点数",
        "valid_points": "有效点数",
        "distance_range": "距离范围",
        "session": "任务状态",
        "ring_count": "圈数",
        "axial_position": "轴向位置",
        "scan_3d_group": "3D 扫描 / 防撞",
        "enable_3d": "启用 3D 点云",
        "axial_step": "轴向步进",
        "rings_per_level": "每位置圈数",
        "enable_collision": "启用防撞",
        "collision_threshold": "防撞阈值",
        "sim_obstacle_distance": "仿真障碍距离",
        "front_distance": "前向距离",
        "point_cloud_status": "3D 状态",
        "debug_controls": "调试控制",
        "debug_state": "调试状态",
        "probe_type": "探头类型",
        "depth_probe": "测深度探头",
        "rotary_probe": "侧扫探头",
        "pulse_count": "脉冲数量",
        "sample_rate": "采样率",
        "debug_sound_speed": "波速",
        "sweep_start": "起始角",
        "sweep_end": "结束角",
        "sweep_step": "扫角步进",
        "sample_count": "采样点数",
        "capture_type": "采集类型",
        "upload_distance": "上传距离",
        "upload_wave": "上传波形",
        "config_single": "配置单点",
        "move_to_angle": "转到目标角",
        "single_measure": "单次测量",
        "test_button": "测试",
        "config_sweep": "配置扫角",
        "start_sweep": "开始扫角",
        "save_wave_csv": "保存波形 CSV",
        "target": "目标: {value}",
        "direction": "方向: {value}",
        "arrived": "到位: {value}",
        "distance": "距离: {value}",
        "quality": "质量: {value}",
        "wave": "波形: {value}",
        "idle": "空闲",
        "disconnected": "未连接",
        "device_na": "设备: n/a",
        "status_idle": "状态: IDLE",
        "angle_fmt": "角度: {value}",
        "mode_idle": "模式: 空闲",
        "event_none": "事件: 无",
        "record_off": "记录: 关闭",
        "no_active_scan": "空闲",
        "connected_loopback": "已连接: {port} @ {baud} (本地回环)",
        "cartesian_plot_title": "距离-角度",
        "polar_plot_title": "极坐标距离图",
        "wave_plot_title": "波形",
        "debug_sweep_plot_title": "调试扫角距离",
        "cloud_window_title": "3D 点云窗口",
        "lang_zh": "中文",
        "lang_en": "英文",
    },
}


class PointCloudWindow(QtWidgets.QWidget):
    def __init__(self, title: str) -> None:
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.setMinimumSize(420, 420)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.title_label)

        self.figure = Figure(figsize=(5, 4), facecolor="#101418")
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        layout.addWidget(self.canvas, stretch=1)
        self.axes = self.figure.add_subplot(111, projection="3d")
        self.figure.subplots_adjust(left=0.02, right=0.98, bottom=0.03, top=0.94)
        self._style_axes()
        self.update_points([], cylinder=None)

    def set_title(self, title: str) -> None:
        self.title_label.setText(title)

    def update_points(
        self,
        points: list[tuple[float, float, float]],
        *,
        cylinder: tuple[float, float, float, float, float] | None,
        mesh: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None,
        show_points: bool = True,
    ) -> None:
        self.axes.clear()
        self._style_axes()

        if not points and mesh is None:
            self._draw_axes_guides(center=(0.0, 0.0, 0.0), radius=600.0)
            self.canvas.draw_idle()
            return

        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        zs = [point[2] for point in points]
        if mesh is not None:
            mesh_x, mesh_y, mesh_z = mesh
            xs.extend(mesh_x.ravel().tolist())
            ys.extend(mesh_y.ravel().tolist())
            zs.extend(mesh_z.ravel().tolist())
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        z_min, z_max = min(zs), max(zs)
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        center_z = (z_min + z_max) / 2.0
        radius = max(x_max - x_min, y_max - y_min, z_max - z_min, 800.0) / 2.0
        radius *= 1.25

        if show_points and points:
            self.axes.scatter(
                [point[0] for point in points],
                [point[1] for point in points],
                [point[2] for point in points],
                c="#3da5d9",
                s=22,
                depthshade=True,
            )
        if mesh is not None:
            mesh_x, mesh_y, mesh_z = mesh
            self.axes.plot_surface(
                mesh_x,
                mesh_y,
                mesh_z,
                color="#46b8ff",
                alpha=0.88,
                edgecolor="#1d80c3",
                linewidth=0.35,
                antialiased=True,
                shade=True,
                rstride=1,
                cstride=1,
            )
        elif cylinder is not None:
            cyl_radius, cyl_center_x, cyl_center_y, cyl_z_min, cyl_z_max = cylinder
            theta = np.linspace(0.0, 2.0 * math.pi, 36)
            z_line = np.linspace(cyl_z_min, cyl_z_max, 16)
            theta_grid, z_grid = np.meshgrid(theta, z_line)
            x_grid = cyl_center_x + cyl_radius * np.cos(theta_grid)
            y_grid = cyl_center_y + cyl_radius * np.sin(theta_grid)
            self.axes.plot_surface(
                x_grid,
                y_grid,
                z_grid,
                color="#46b8ff",
                alpha=0.42,
                edgecolor="#1d80c3",
                linewidth=0.2,
                antialiased=True,
                shade=True,
            )
        self._draw_axes_guides(center=(center_x, center_y, center_z), radius=radius)
        self.axes.set_xlim(center_x - radius, center_x + radius)
        self.axes.set_ylim(center_y - radius, center_y + radius)
        self.axes.set_zlim(center_z - radius, center_z + radius)
        z_span = max(1.0, z_max - z_min)
        xy_span = max(1.0, radius * 2.0)
        self.axes.set_box_aspect((1.0, 1.0, max(0.55, min(1.2, z_span / xy_span * 3.0))))
        self.canvas.draw_idle()

    def _style_axes(self) -> None:
        self.axes.set_facecolor("#101418")
        self.axes.set_xlabel("X", color="#dbeafe")
        self.axes.set_ylabel("Y", color="#dbeafe")
        self.axes.set_zlabel("Z", color="#dbeafe")
        self.axes.tick_params(colors="#cbd5e1", labelsize=8)
        self.axes.grid(True, color="#475569", alpha=0.35)
        self.axes.view_init(elev=20, azim=35)
        self.axes.set_box_aspect((1.0, 1.0, 0.8))
        self.axes.xaxis.set_pane_color((0.07, 0.09, 0.12, 1.0))
        self.axes.yaxis.set_pane_color((0.07, 0.09, 0.12, 1.0))
        self.axes.zaxis.set_pane_color((0.07, 0.09, 0.12, 1.0))

    def _draw_axes_guides(self, *, center: tuple[float, float, float], radius: float) -> None:
        cx, cy, cz = center
        self.axes.plot([cx - radius, cx + radius], [cy, cy], [cz, cz], color="#ef4444", linewidth=1.2)
        self.axes.plot([cx, cx], [cy - radius, cy + radius], [cz, cz], color="#22c55e", linewidth=1.2)
        self.axes.plot([cx, cx], [cy, cy], [cz - radius, cz + radius], color="#3b82f6", linewidth=1.2)


class SquareViewport(QtWidgets.QWidget):
    def __init__(self, child: QtWidgets.QWidget) -> None:
        super().__init__()
        self.child = child
        self.child.setParent(self)
        self.child.show()
        self.setMinimumSize(420, 420)

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # type: ignore[name-defined]
        side = min(self.width(), self.height())
        x_offset = (self.width() - side) // 2
        y_offset = (self.height() - side) // 2
        self.child.setGeometry(x_offset, y_offset, side, side)
        super().resizeEvent(event)


class DistanceAxisItem(pg.AxisItem):
    def __init__(self, *, parent: "MainWindow") -> None:
        super().__init__(orientation="top")
        self._parent = parent

    def tickStrings(self, values: list[float], scale: float, spacing: float) -> list[str]:
        wave_speed = self._parent.debug_sound_speed_spin.value() if hasattr(self._parent, "debug_sound_speed_spin") else 1500
        factor = float(wave_speed) / 2000.0
        labels: list[str] = []
        for value in values:
            distance_mm = value * factor
            labels.append(f"{distance_mm:.2f}")
        return labels


class SessionRecorder:
    def __init__(self, root_dir: Path) -> None:
        self.root_dir = root_dir
        self.session_dir: Path | None = None
        self._event_handle: Any | None = None

    @property
    def active(self) -> bool:
        return self.session_dir is not None

    def start(self) -> Path:
        if self.session_dir is not None:
            return self.session_dir

        stamp = time.strftime("%Y%m%d_%H%M%S")
        session_dir = self.root_dir / stamp
        session_dir.mkdir(parents=True, exist_ok=True)
        self._event_handle = session_dir.joinpath("events.jsonl").open("a", encoding="utf-8")
        self.session_dir = session_dir
        self.log_event("recording_started", session_dir=str(session_dir))
        return session_dir

    def stop(self) -> Path | None:
        if self.session_dir is None:
            return None

        session_dir = self.session_dir
        self.log_event("recording_stopped")
        if self._event_handle is not None:
            self._event_handle.close()
        self._event_handle = None
        self.session_dir = None
        return session_dir

    def log_packet(self, *, direction: str, label: str, frame: Frame) -> None:
        self.log_event(
            "packet",
            direction=direction,
            label=label,
            command=frame.command,
            sequence=frame.sequence,
            payload_hex=frame.payload.hex(" "),
            frame_hex=frame.encode().hex(" "),
        )

    def log_event(self, event_type: str, **payload: Any) -> None:
        if self._event_handle is None:
            return

        record = {
            "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
            "event_type": event_type,
            **payload,
        }
        self._event_handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._event_handle.flush()

    def append_scan_point(self, point: ScanPoint, *, channel: str) -> None:
        self._append_csv(
            f"{channel}_points.csv",
            ["point_index", "angle_deg", "distance_mm", "quality", "valid", "has_wave", "timestamp_ms"],
            [
                point.point_index,
                f"{point.angle / 100.0:.2f}",
                point.distance_mm,
                point.quality,
                int(point.valid),
                int(point.has_wave),
                point.timestamp_ms,
            ],
        )

    def append_distance_result(self, result: DistanceResult) -> None:
        self._append_csv(
            "distance_results.csv",
            ["angle_deg", "distance_mm", "quality", "valid"],
            [f"{result.angle / 100.0:.2f}", result.distance_mm, result.quality, int(result.valid)],
        )

    def save_waveform(self, *, angle: int, data_type: WaveDataType, raw_bytes: bytes, sample_count: int, sound_speed_m_s: int, sample_rate_hz: float) -> None:
        if self.session_dir is None:
            return

        stamp = int(time.time() * 1000)
        bin_path = self.session_dir / f"wave_{angle:05d}_{stamp}.bin"
        meta_path = self.session_dir / f"wave_{angle:05d}_{stamp}.json"
        csv_path = self.session_dir / f"wave_{angle:05d}_{stamp}.csv"
        bin_path.write_bytes(raw_bytes)
        meta_path.write_text(
            json.dumps(
                {
                    "angle_deg": angle / 100.0,
                    "data_type": data_type.name,
                    "sample_count": sample_count,
                    "sound_speed_m_s": sound_speed_m_s,
                    "sample_rate_hz": sample_rate_hz,
                    "bin_file": bin_path.name,
                    "csv_file": csv_path.name,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        samples: list[int]
        if data_type in (WaveDataType.U16_RAW, WaveDataType.U16_ENVELOPE):
            count = min(sample_count, len(raw_bytes) // 2)
            samples = list(struct.unpack(f"<{count}H", raw_bytes[: count * 2]))
            with csv_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(["sample_index", "time_us", "distance_mm", "raw_code", "voltage_v"])
                for index, value in enumerate(samples):
                    time_us = (index / sample_rate_hz) * 1_000_000.0
                    distance_mm = time_us * float(sound_speed_m_s) / 2000.0
                    voltage_v = float(value) * ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE
                    writer.writerow([index, f"{time_us:.6f}", f"{distance_mm:.6f}", value, f"{voltage_v:.6f}"])
        else:
            samples = list(raw_bytes[:sample_count])
            with csv_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(["sample_index", "time_us", "distance_mm", "value"])
                for index, value in enumerate(samples):
                    time_us = (index / sample_rate_hz) * 1_000_000.0
                    distance_mm = time_us * float(sound_speed_m_s) / 2000.0
                    writer.writerow([index, f"{time_us:.6f}", f"{distance_mm:.6f}", value])

        self._append_csv(
            "wave_index.csv",
            ["angle_deg", "data_type", "sample_count", "sound_speed_m_s", "sample_rate_hz", "bin_file", "json_file", "csv_file"],
            [f"{angle / 100.0:.2f}", data_type.name, sample_count, sound_speed_m_s, f"{sample_rate_hz:.3f}", bin_path.name, meta_path.name, csv_path.name],
        )

    def _append_csv(self, filename: str, header: list[str], row: list[Any]) -> None:
        if self.session_dir is None:
            return

        path = self.session_dir / filename
        write_header = not path.exists()
        with path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            if write_header:
                writer.writerow(header)
            writer.writerow(row)


class TransportWorker(QtCore.QThread):
    frame_received = QtCore.Signal(object)
    status_changed = QtCore.Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._port_name = ""
        self._baud_rate = 115200
        self._stop_event = threading.Event()
        self._serial_lock = threading.Lock()
        self._serial: serial.Serial | None = None
        self._outgoing: list[bytes] = []

    def configure(self, port_name: str, baud_rate: int) -> None:
        self._port_name = port_name
        self._baud_rate = baud_rate

    def run(self) -> None:
        self._stop_event.clear()
        if self._port_name == SIMULATOR_PORT:
            self._run_simulator()
            return
        self._run_serial()

    def _run_serial(self) -> None:
        decoder = FrameDecoder()
        try:
            serial_port = serial.Serial()
            serial_port.port = self._port_name
            serial_port.baudrate = self._baud_rate
            serial_port.timeout = 0.05
            serial_port.dsrdtr = False
            serial_port.rtscts = False
            serial_port.dtr = False
            serial_port.rts = False
            serial_port.open()
            self._serial = serial_port
            self.status_changed.emit(f"Connected: {self._port_name} @ {self._baud_rate} (DTR/RTS disabled)")
        except serial.SerialException as exc:
            self.status_changed.emit(f"Open failed: {exc}")
            return

        try:
            while not self._stop_event.is_set():
                self._flush_outgoing()
                try:
                    raw = self._serial.read(256) if self._serial is not None else b""
                except serial.SerialException as exc:
                    self.status_changed.emit(f"Serial error: {exc}")
                    break

                if not raw:
                    continue

                for frame in decoder.feed(raw):
                    self.frame_received.emit(frame)
        finally:
            with self._serial_lock:
                if self._serial is not None and self._serial.is_open:
                    self._serial.close()
                self._serial = None
            self.status_changed.emit("Disconnected")

    def _run_simulator(self) -> None:
        device = SimulatedDevice()
        decoder = FrameDecoder()
        self.status_changed.emit(f"Connected: {SIMULATOR_PORT} @ {self._baud_rate} (loopback)")
        try:
            while not self._stop_event.is_set():
                packet = self._pop_outgoing()
                if packet is None:
                    self.msleep(10)
                    continue

                try:
                    responses = device.handle_bytes(packet)
                except Exception as exc:
                    self.status_changed.emit(f"Simulator error: {exc}")
                    break

                for response in responses:
                    if self._stop_event.is_set():
                        break
                    self.msleep(12)
                    for frame in decoder.feed(response):
                        self.frame_received.emit(frame)
        finally:
            self.status_changed.emit("Disconnected")

    def _pop_outgoing(self) -> bytes | None:
        with self._serial_lock:
            if not self._outgoing:
                return None
            return self._outgoing.pop(0)

    def _flush_outgoing(self) -> None:
        while True:
            with self._serial_lock:
                if not self._outgoing:
                    return
                packet = self._outgoing.pop(0)
                serial_ref = self._serial

            if serial_ref is None or not serial_ref.is_open:
                self.status_changed.emit("Send skipped: not connected")
                return

            try:
                serial_ref.write(packet)
                serial_ref.flush()
            except serial.SerialException as exc:
                self.status_changed.emit(f"Send failed: {exc}")
                return

    def send_packet(self, packet: bytes) -> None:
        with self._serial_lock:
            self._outgoing.append(packet)

    def stop(self) -> None:
        self._stop_event.set()
        self.wait(1500)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.language = "zh"
        self.setWindowTitle(self._tr("window_title"))
        self.resize(1220, 760)

        self.worker: TransportWorker | None = None
        self.recorder = SessionRecorder(RECORD_ROOT)
        self.point_cloud_window: PointCloudWindow | None = None

        self.sequence = 0
        self.device_status = DeviceStatus.IDLE
        self.current_angle = 0
        self.device_serial = ""
        self.current_mode_name = "Idle"
        self.last_point_count = 0
        self.scan_started_at: float | None = None
        self.debug_started_at: float | None = None
        self.current_scan_points: list[ScanPoint] = []
        self.current_debug_points: list[ScanPoint] = []
        self.current_wave_bytes = b""
        self.current_wave_angle = 0
        self.current_wave_data_type = WaveDataType.U8_RAW
        self.current_wave_samples: list[int] = []
        self.current_wave_display_samples: list[float] = []
        self.wave_capture_history: list[dict[str, Any]] = []
        self.wave_streams: dict[int, dict[str, Any]] = {}
        self.runtime_sample_rate_hz = WAVE_SAMPLE_RATE_HZ
        self.measured_sample_rate_hz = 0.0
        self.debug_probe_type = ProbeType.DEPTH_PROBE
        self.lap_count = 0
        self.rings_at_position = 0
        self.axial_position_mm = 0.0
        self.front_distance_mm = 0.0
        self.point_cloud_points: list[tuple[float, float, float]] = []
        self.point_cloud_layers: list[list[tuple[float, float, float]]] = []
        self.current_ring_points_3d: list[tuple[float, float, float]] = []
        self.cylinder_fit: tuple[float, float, float, float, float] | None = None
        self.surface_mesh: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
        self.mesh_render_only = False
        self.scan_point_list_index: dict[int, int] = {}
        self.scan_point_accumulators: dict[int, dict[str, float | int]] = {}
        self.current_ring_received = 0
        self.auto_scan_active = False
        self.collision_risk_active = False
        self.pending_cylinder_fit = False

        self.scan_visual_timer = QtCore.QTimer(self)
        self.scan_visual_timer.setSingleShot(True)
        self.scan_visual_timer.setInterval(33)
        self.scan_visual_timer.timeout.connect(self._apply_scan_visuals)

        self.point_cloud_timer = QtCore.QTimer(self)
        self.point_cloud_timer.setSingleShot(True)
        self.point_cloud_timer.setInterval(80)
        self.point_cloud_timer.timeout.connect(self._apply_point_cloud_update)

        self._build_ui()
        self._refresh_ports()
        self._set_transport_connected(False)

        self.port_refresh_timer = QtCore.QTimer(self)
        self.port_refresh_timer.timeout.connect(self._refresh_ports)
        self.port_refresh_timer.start(2000)

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)

        top_bar = QtWidgets.QGridLayout()
        layout.addLayout(top_bar)

        self.port_title = QtWidgets.QLabel()
        self.baud_title = QtWidgets.QLabel()
        self.language_title = QtWidgets.QLabel()
        self.port_combo = QtWidgets.QComboBox()
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["115200", "230400", "460800"])
        self.baud_combo.setCurrentText("115200")
        self.language_combo = QtWidgets.QComboBox()
        self.language_combo.addItem("")
        self.language_combo.addItem("")
        self.language_combo.setCurrentIndex(0)

        self.connect_button = QtWidgets.QPushButton("Connect")
        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.refresh_button = QtWidgets.QPushButton("Refresh")
        self.info_button = QtWidgets.QPushButton("Get Info")
        self.zero_button = QtWidgets.QPushButton("Zeroing")
        self.stop_button = QtWidgets.QPushButton("Stop")
        for button in (
            self.connect_button,
            self.disconnect_button,
            self.refresh_button,
            self.info_button,
            self.zero_button,
            self.stop_button,
        ):
            button.setMinimumWidth(88)
            button.setMaximumWidth(118)

        self.connection_label = QtWidgets.QLabel("Disconnected")
        self.device_label = QtWidgets.QLabel("Device: n/a")
        self.status_label = QtWidgets.QLabel("Status: IDLE")
        self.angle_label = QtWidgets.QLabel("Angle: 0.00 deg")
        self.mode_label = QtWidgets.QLabel("Mode: Idle")
        self.event_label = QtWidgets.QLabel("Event: none")
        top_bar.addWidget(self.port_title, 0, 0)
        top_bar.addWidget(self.port_combo, 0, 1)
        top_bar.addWidget(self.baud_title, 0, 2)
        top_bar.addWidget(self.baud_combo, 0, 3)
        top_bar.addWidget(self.connect_button, 0, 4)
        top_bar.addWidget(self.disconnect_button, 0, 5)
        top_bar.addWidget(self.refresh_button, 0, 6)
        top_bar.addWidget(self.info_button, 0, 7)
        top_bar.addWidget(self.zero_button, 0, 8)
        top_bar.addWidget(self.stop_button, 0, 9)
        top_bar.addWidget(self.language_title, 0, 10)
        top_bar.addWidget(self.language_combo, 0, 11)

        top_bar.addWidget(self.connection_label, 1, 0, 1, 2)
        top_bar.addWidget(self.device_label, 1, 2, 1, 2)
        top_bar.addWidget(self.status_label, 1, 4, 1, 2)
        top_bar.addWidget(self.angle_label, 1, 6, 1, 2)
        top_bar.addWidget(self.mode_label, 1, 8, 1, 1)
        top_bar.addWidget(self.event_label, 1, 9, 1, 3)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        layout.addWidget(splitter, stretch=1)

        self.tabs = QtWidgets.QTabWidget()
        splitter.addWidget(self.tabs)

        self.scan_tab = self._wrap_scroll_area(self._build_scan_tab())
        self.debug_tab = self._wrap_scroll_area(self._build_debug_tab())
        self.tabs.addTab(self.scan_tab, "Scan")
        self.tabs.addTab(self.debug_tab, "Debug")

        self.log_output = QtWidgets.QPlainTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setMaximumBlockCount(1200)
        splitter.addWidget(self.log_output)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)

        self.connect_button.clicked.connect(self._connect_transport)
        self.disconnect_button.clicked.connect(self._disconnect_transport)
        self.refresh_button.clicked.connect(self._refresh_ports)
        self.info_button.clicked.connect(self._request_device_info)
        self.zero_button.clicked.connect(self._request_zeroing)
        self.stop_button.clicked.connect(self._request_stop)
        self.tabs.currentChanged.connect(self._handle_tab_changed)
        self.language_combo.currentIndexChanged.connect(self._change_language)
        self._retranslate_ui()

    def _wrap_scroll_area(self, widget: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setWidget(widget)
        return scroll

    def _build_scan_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(tab)

        self.scan_param_group = QtWidgets.QGroupBox()
        self.scan_param_group.setMaximumWidth(330)
        param_form = QtWidgets.QFormLayout(self.scan_param_group)

        self.scan_step_title = QtWidgets.QLabel()
        self.scan_step_spin = QtWidgets.QDoubleSpinBox()
        self.scan_step_spin.setDecimals(ANGLE_DISPLAY_DECIMALS)
        self.scan_step_spin.setRange(SIDE_SCAN_MIN_STEP_DEG, SIDE_SCAN_MAX_STEP_DEG)
        self.scan_step_spin.setValue(self._normalize_motor_step_degrees(SIDE_SCAN_DEFAULT_STEP_DEG))
        self.scan_step_spin.setSuffix(" deg")

        self.sound_speed_title = QtWidgets.QLabel()
        self.sound_speed_spin = QtWidgets.QSpinBox()
        self.sound_speed_spin.setRange(100, 3000)
        self.sound_speed_spin.setValue(1500)
        self.sound_speed_spin.setSuffix(" m/s")

        self.medium_title = QtWidgets.QLabel()
        self.medium_combo = QtWidgets.QComboBox()
        self.medium_combo.addItem("Air", MediumType.AIR)
        self.medium_combo.addItem("Fresh Water", MediumType.FRESH_WATER)
        self.medium_combo.addItem("Sea Water", MediumType.SEA_WATER)
        self.medium_combo.addItem("Custom", MediumType.CUSTOM)

        self.expected_points_label = QtWidgets.QLabel()
        self.scan_progress_label = QtWidgets.QLabel()
        self.scan_progress_bar = QtWidgets.QProgressBar()
        self.scan_progress_bar.setRange(0, 100)
        self.scan_progress_bar.setValue(0)
        self.configure_scan_button = QtWidgets.QPushButton()
        self.start_scan_button = QtWidgets.QPushButton()
        self.save_scan_button = QtWidgets.QPushButton()

        param_form.addRow(self.scan_step_title, self.scan_step_spin)
        param_form.addRow(self.sound_speed_title, self.sound_speed_spin)
        param_form.addRow(self.medium_title, self.medium_combo)
        param_form.addRow(self.expected_points_label)
        param_form.addRow(self.scan_progress_label)
        param_form.addRow(self.scan_progress_bar)
        param_form.addRow(self.configure_scan_button)
        param_form.addRow(self.start_scan_button)
        self.scan_stats_group = QtWidgets.QGroupBox()
        self.scan_stats_group.setMaximumWidth(330)
        stats_form = QtWidgets.QFormLayout(self.scan_stats_group)
        self.points_done_label = QtWidgets.QLabel("0")
        self.valid_points_label = QtWidgets.QLabel("0")
        self.distance_range_label = QtWidgets.QLabel("n/a")
        self.scan_session_label = QtWidgets.QLabel()
        self.ring_count_label = QtWidgets.QLabel("0")
        self.axial_position_label = QtWidgets.QLabel("0 mm")
        self.points_done_title = QtWidgets.QLabel()
        self.valid_points_title = QtWidgets.QLabel()
        self.distance_range_title = QtWidgets.QLabel()
        self.session_title = QtWidgets.QLabel()
        self.ring_count_title = QtWidgets.QLabel()
        self.axial_position_title = QtWidgets.QLabel()
        stats_form.addRow(self.points_done_title, self.points_done_label)
        stats_form.addRow(self.valid_points_title, self.valid_points_label)
        stats_form.addRow(self.distance_range_title, self.distance_range_label)
        stats_form.addRow(self.session_title, self.scan_session_label)
        stats_form.addRow(self.ring_count_title, self.ring_count_label)
        stats_form.addRow(self.axial_position_title, self.axial_position_label)

        self.polar_plot = pg.PlotWidget()
        self.polar_plot.showGrid(x=False, y=False)
        self.polar_plot.setAspectLocked(True)
        self.polar_plot.disableAutoRange()
        self.polar_plot.setMouseEnabled(x=False, y=False)
        self.polar_plot.setMenuEnabled(False)
        self.polar_plot.hideAxis("left")
        self.polar_plot.hideAxis("bottom")
        self.polar_plot.hideButtons()
        self.polar_plot.getPlotItem().layout.setContentsMargins(0, 0, 0, 0)
        self.polar_curve = self.polar_plot.plot(
            pen=pg.mkPen("#ae2012", width=2),
            symbol=None,
        )
        self.polar_points = self.polar_plot.plot(
            pen=None,
            symbol="o",
            symbolSize=6,
            symbolBrush="#ee9b00",
            symbolPen=pg.mkPen("#ca6702", width=1),
        )
        self.polar_sweep_line = self.polar_plot.plot(pen=pg.mkPen("#0a9396", width=2))
        self.polar_cross_h = self.polar_plot.plot(pen=pg.mkPen("#94a1b2", width=1))
        self.polar_cross_v = self.polar_plot.plot(pen=pg.mkPen("#94a1b2", width=1))
        self.polar_range_circle = self.polar_plot.plot(pen=pg.mkPen("#cbd5e1", width=1))
        self.polar_risk_indicator = self.polar_plot.plot(
            pen=None,
            symbol="o",
            symbolSize=18,
            symbolBrush="#22c55e",
            symbolPen=pg.mkPen("#e2e8f0", width=1),
        )
        self.polar_plot.setBackground("w")
        self.polar_viewport = SquareViewport(self.polar_plot)
        self.point_cloud_window = PointCloudWindow(self._tr("cloud_window_title"))
        self.point_cloud_window.setMinimumWidth(360)
        self.point_cloud_window.hide()

        self.scan_3d_group = QtWidgets.QGroupBox()
        self.scan_3d_group.setMaximumWidth(330)
        scan_3d_form = QtWidgets.QFormLayout(self.scan_3d_group)
        self.enable_3d_check = QtWidgets.QCheckBox()
        self.axial_step_title = QtWidgets.QLabel()
        self.axial_step_spin = QtWidgets.QDoubleSpinBox()
        self.axial_step_spin.setRange(1.0, 2000.0)
        self.axial_step_spin.setValue(200.0)
        self.axial_step_spin.setSuffix(" mm")
        self.rings_per_level_title = QtWidgets.QLabel()
        self.rings_per_level_spin = QtWidgets.QSpinBox()
        self.rings_per_level_spin.setRange(1, 100)
        self.rings_per_level_spin.setValue(1)
        self.enable_collision_check = QtWidgets.QCheckBox()
        self.enable_collision_check.setChecked(True)
        self.collision_threshold_title = QtWidgets.QLabel()
        self.collision_threshold_spin = QtWidgets.QDoubleSpinBox()
        self.collision_threshold_spin.setRange(50.0, 5000.0)
        self.collision_threshold_spin.setValue(450.0)
        self.collision_threshold_spin.setSuffix(" mm")
        self.scan_depth_title = QtWidgets.QLabel()
        self.scan_depth_spin = QtWidgets.QDoubleSpinBox()
        self.scan_depth_spin.setRange(100.0, 10000.0)
        self.scan_depth_spin.setValue(800.0)
        self.scan_depth_spin.setSuffix(" mm")
        self.front_distance_title = QtWidgets.QLabel()
        self.front_distance_value = QtWidgets.QLabel("n/a")
        self.point_cloud_status_title = QtWidgets.QLabel()
        self.point_cloud_status_value = QtWidgets.QLabel("OFF")

        scan_3d_form.addRow(self.enable_3d_check)
        scan_3d_form.addRow(self.axial_step_title, self.axial_step_spin)
        scan_3d_form.addRow(self.rings_per_level_title, self.rings_per_level_spin)
        scan_3d_form.addRow(self.enable_collision_check)
        scan_3d_form.addRow(self.collision_threshold_title, self.collision_threshold_spin)
        scan_3d_form.addRow(self.scan_depth_title, self.scan_depth_spin)
        scan_3d_form.addRow(self.front_distance_title, self.front_distance_value)
        scan_3d_form.addRow(self.point_cloud_status_title, self.point_cloud_status_value)

        self.scan_table = QtWidgets.QTableWidget(0, 7)
        self.scan_table.setHorizontalHeaderLabels(
            ["Index", "Angle (deg)", "Distance (mm)", "Z (mm)", "Quality", "Valid", "t (ms)"]
        )
        self.scan_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.scan_table.setColumnHidden(3, True)
        self.save_scan_button.setMinimumWidth(140)

        self.scan_table_panel = QtWidgets.QWidget()
        table_layout = QtWidgets.QVBoxLayout(self.scan_table_panel)
        table_layout.setContentsMargins(0, 0, 0, 0)
        table_layout.setSpacing(6)
        table_toolbar = QtWidgets.QHBoxLayout()
        table_toolbar.setContentsMargins(0, 0, 0, 0)
        table_toolbar.addStretch(1)
        table_toolbar.addWidget(self.save_scan_button)
        table_layout.addLayout(table_toolbar)
        table_layout.addWidget(self.scan_table, stretch=1)

        self.scan_right_panel = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(self.scan_right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)
        right_layout.addWidget(self.scan_stats_group)
        right_layout.addWidget(self.scan_3d_group)
        right_layout.addStretch(1)

        self.scan_plot_panel = QtWidgets.QWidget()
        plot_layout = QtWidgets.QHBoxLayout(self.scan_plot_panel)
        plot_layout.setContentsMargins(0, 0, 0, 0)
        plot_layout.setSpacing(8)
        plot_layout.addWidget(self.polar_viewport, stretch=1)
        plot_layout.addWidget(self.point_cloud_window, stretch=1)

        layout.addWidget(self.scan_param_group, 0, 0)
        layout.addWidget(self.scan_plot_panel, 0, 1)
        layout.addWidget(self.scan_right_panel, 0, 2)
        layout.addWidget(self.scan_table_panel, 1, 0, 1, 3)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 0)
        layout.setRowStretch(1, 1)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(8)

        self.configure_scan_button.clicked.connect(self._configure_scan)
        self.start_scan_button.clicked.connect(self._start_scan)
        self.save_scan_button.clicked.connect(self._save_scan_csv)
        self.enable_3d_check.toggled.connect(self._toggle_point_cloud_window)
        self.scan_step_spin.valueChanged.connect(self._normalize_scan_step_input)
        return tab

    def _build_debug_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(tab)

        self.debug_control_group = QtWidgets.QGroupBox()
        self.debug_control_group.setMaximumWidth(330)
        control_form = QtWidgets.QFormLayout(self.debug_control_group)

        self.probe_type_title = QtWidgets.QLabel()
        self.probe_type_tabs = QtWidgets.QTabBar()
        self.probe_type_tabs.addTab("")
        self.probe_type_tabs.addTab("")

        self.debug_start_title = QtWidgets.QLabel()
        self.debug_start_spin = QtWidgets.QDoubleSpinBox()
        self.debug_start_spin.setDecimals(ANGLE_DISPLAY_DECIMALS)
        self.debug_start_spin.setRange(0.0, MOTOR_MAX_ANGLE_DEG)
        self.debug_start_spin.setValue(0.0)
        self.debug_start_spin.setSuffix(" deg")

        self.debug_end_title = QtWidgets.QLabel()
        self.debug_end_spin = QtWidgets.QDoubleSpinBox()
        self.debug_end_spin.setDecimals(ANGLE_DISPLAY_DECIMALS)
        self.debug_end_spin.setRange(0.0, MOTOR_MAX_ANGLE_DEG)
        self.debug_end_spin.setValue(90.0)
        self.debug_end_spin.setSuffix(" deg")

        self.debug_step_title = QtWidgets.QLabel()
        self.debug_step_spin = QtWidgets.QDoubleSpinBox()
        self.debug_step_spin.setDecimals(ANGLE_DISPLAY_DECIMALS)
        self.debug_step_spin.setRange(0.0, SIDE_SCAN_MAX_STEP_DEG)
        self.debug_step_spin.setValue(self._normalize_motor_step_degrees(SIDE_SCAN_DEFAULT_STEP_DEG))
        self.debug_step_spin.setSuffix(" deg")

        self.debug_sound_speed_title = QtWidgets.QLabel()
        self.debug_sound_speed_spin = QtWidgets.QSpinBox()
        self.debug_sound_speed_spin.setRange(100, 3000)
        self.debug_sound_speed_spin.setValue(1500)
        self.debug_sound_speed_spin.setSuffix(" m/s")

        self.sample_count_title = QtWidgets.QLabel()
        self.sample_count_spin = QtWidgets.QSpinBox()
        self.sample_count_spin.setRange(32, 7500)
        self.sample_count_spin.setValue(7500)

        self.capture_type_title = QtWidgets.QLabel()
        self.capture_type_combo = QtWidgets.QComboBox()
        self.capture_type_combo.addItem("Raw Wave", CaptureType.RAW_WAVE)
        self.capture_type_combo.addItem("Envelope", CaptureType.ENVELOPE)

        self.pulse_count_title = QtWidgets.QLabel()
        self.pulse_count_spin = QtWidgets.QSpinBox()
        self.pulse_count_spin.setRange(1, 255)
        self.pulse_count_spin.setValue(6)

        self.upload_wave_check = QtWidgets.QCheckBox("Upload wave")
        self.upload_wave_check.setChecked(True)

        self.single_measure_button = QtWidgets.QPushButton()
        self.start_debug_sweep_button = QtWidgets.QPushButton()
        self.save_wave_button = QtWidgets.QPushButton()

        control_form.addRow(self.probe_type_title, self.probe_type_tabs)
        control_form.addRow(self.debug_start_title, self.debug_start_spin)
        control_form.addRow(self.debug_end_title, self.debug_end_spin)
        control_form.addRow(self.debug_step_title, self.debug_step_spin)
        control_form.addRow(self.debug_sound_speed_title, self.debug_sound_speed_spin)
        control_form.addRow(self.sample_count_title, self.sample_count_spin)
        control_form.addRow(self.capture_type_title, self.capture_type_combo)
        control_form.addRow(self.pulse_count_title, self.pulse_count_spin)
        control_form.addRow(self.upload_wave_check)
        control_form.addRow(self.single_measure_button)
        control_form.addRow(self.start_debug_sweep_button)
        control_form.addRow(self.save_wave_button)

        self.debug_result_group = QtWidgets.QGroupBox()
        self.debug_result_group.setMaximumWidth(330)
        result_form = QtWidgets.QFormLayout(self.debug_result_group)
        self.debug_target_label = QtWidgets.QLabel("Target: n/a")
        self.debug_move_label = QtWidgets.QLabel("Direction: n/a")
        self.debug_arrived_label = QtWidgets.QLabel("Arrived: n/a")
        self.debug_distance_label = QtWidgets.QLabel("Distance: n/a")
        self.debug_quality_label = QtWidgets.QLabel("Quality: n/a")
        self.debug_sample_rate_label = QtWidgets.QLabel("Sample Rate: n/a")
        self.wave_info_label = QtWidgets.QLabel("Wave: n/a")
        self.debug_session_label = QtWidgets.QLabel("Idle")
        result_form.addRow(self.debug_target_label)
        result_form.addRow(self.debug_move_label)
        result_form.addRow(self.debug_arrived_label)
        result_form.addRow(self.debug_distance_label)
        result_form.addRow(self.debug_quality_label)
        result_form.addRow(self.debug_sample_rate_label)
        result_form.addRow(self.wave_info_label)
        self.debug_session_title = QtWidgets.QLabel()
        result_form.addRow(self.debug_session_title, self.debug_session_label)

        self.wave_distance_axis = DistanceAxisItem(parent=self)
        self.wave_plot = pg.PlotWidget(axisItems={"top": self.wave_distance_axis})
        self.wave_plot.showGrid(x=True, y=True, alpha=0.25)
        self.wave_plot.setLabel("left", "Amplitude")
        self.wave_plot.setLabel("bottom", "Time (us)")
        self.wave_plot.showAxis("top")
        self.wave_plot.setLabel("top", "Distance (mm)")
        self.wave_curve = self.wave_plot.plot(pen=pg.mkPen("#9b2226", width=2))

        self.debug_sweep_plot = pg.PlotWidget()
        self.debug_sweep_plot.showGrid(x=True, y=True, alpha=0.25)
        self.debug_sweep_plot.setLabel("left", "Distance (mm)")
        self.debug_sweep_plot.setLabel("bottom", "Angle (deg)")
        self.debug_sweep_curve = self.debug_sweep_plot.plot(
            pen=pg.mkPen("#0a9396", width=2),
            symbol="o",
            symbolSize=5,
            symbolBrush="#ca6702",
        )

        self.debug_table = QtWidgets.QTableWidget(0, 6)
        self.debug_table.setHorizontalHeaderLabels(
            ["Index", "Angle (deg)", "Distance (mm)", "Quality", "Valid", "Wave"]
        )
        self.debug_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)

        layout.addWidget(self.debug_control_group, 0, 0, 2, 1)
        layout.addWidget(self.debug_result_group, 2, 0)
        layout.addWidget(self.wave_plot, 0, 1)
        layout.addWidget(self.debug_sweep_plot, 1, 1)
        layout.addWidget(self.debug_table, 2, 1)
        layout.setColumnStretch(1, 1)
        layout.setRowStretch(2, 1)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(8)

        self.single_measure_button.clicked.connect(self._start_single_measure)
        self.start_debug_sweep_button.clicked.connect(self._start_debug_sweep)
        self.save_wave_button.clicked.connect(self._save_wave_csv)
        self.probe_type_tabs.currentChanged.connect(self._handle_debug_probe_change)
        self.debug_sound_speed_spin.valueChanged.connect(self._refresh_wave_plot_axes)
        self.debug_start_spin.valueChanged.connect(self._normalize_debug_angle_inputs)
        self.debug_end_spin.valueChanged.connect(self._normalize_debug_angle_inputs)
        self.debug_step_spin.valueChanged.connect(self._normalize_debug_angle_inputs)
        self._update_debug_probe_mode_ui()
        return tab

    def _tr(self, key: str, **kwargs: Any) -> str:
        value = TEXTS[self.language].get(key, TEXTS["en"].get(key, key))
        if kwargs:
            return value.format(**kwargs)
        return value

    def _change_language(self) -> None:
        self.language = "zh" if self.language_combo.currentIndex() == 0 else "en"
        self._retranslate_ui()

    def _retranslate_ui(self) -> None:
        self.setWindowTitle(self._tr("window_title"))
        self.port_title.setText(self._tr("port"))
        self.baud_title.setText(self._tr("baud"))
        self.language_title.setText(self._tr("language"))
        self.language_combo.blockSignals(True)
        self.language_combo.setItemText(0, self._tr("lang_zh"))
        self.language_combo.setItemText(1, self._tr("lang_en"))
        self.language_combo.setCurrentIndex(0 if self.language == "zh" else 1)
        self.language_combo.blockSignals(False)

        self.connect_button.setText(self._tr("connect"))
        self.disconnect_button.setText(self._tr("disconnect"))
        self.refresh_button.setText(self._tr("refresh"))
        self.info_button.setText(self._tr("get_info"))
        self.zero_button.setText(self._tr("zeroing"))
        self.stop_button.setText(self._tr("stop"))
        self.tabs.setTabText(0, self._tr("scan_tab"))
        self.tabs.setTabText(1, self._tr("debug_tab"))

        self.scan_param_group.setTitle(self._tr("scan_group"))
        self.scan_stats_group.setTitle(self._tr("scan_summary"))
        self.scan_3d_group.setTitle(self._tr("scan_3d_group"))
        self.scan_step_title.setText(self._tr("step_angle"))
        self.sound_speed_title.setText(self._tr("sound_speed"))
        self.medium_title.setText(self._tr("medium"))
        self.configure_scan_button.setText(self._tr("configure_scan"))
        self.start_scan_button.setText(self._tr("start_scan"))
        self.save_scan_button.setText("Save Table CSV" if self.language == "en" else "保存表格数据")
        self.points_done_title.setText(self._tr("points_done"))
        self.valid_points_title.setText(self._tr("valid_points"))
        self.distance_range_title.setText(self._tr("distance_range"))
        self.session_title.setText(self._tr("session"))
        self.ring_count_title.setText(self._tr("ring_count"))
        self.axial_position_title.setText(self._tr("axial_position"))
        self.enable_3d_check.setText(self._tr("enable_3d"))
        self.axial_step_title.setText(self._tr("axial_step"))
        self.rings_per_level_title.setText(self._tr("rings_per_level"))
        self.enable_collision_check.setText(self._tr("enable_collision"))
        self.collision_threshold_title.setText(self._tr("collision_threshold"))
        self.scan_depth_title.setText(self._tr("scan_depth"))
        self.front_distance_title.setText(self._tr("front_distance"))
        self.point_cloud_status_title.setText(self._tr("point_cloud_status"))

        self.debug_control_group.setTitle(self._tr("debug_controls"))
        self.debug_result_group.setTitle(self._tr("debug_state"))
        self.probe_type_title.setText(self._tr("probe_type"))
        self.probe_type_tabs.setTabText(0, self._tr("depth_probe"))
        self.probe_type_tabs.setTabText(1, self._tr("rotary_probe"))
        self.debug_start_title.setText(self._tr("sweep_start"))
        self.debug_end_title.setText(self._tr("sweep_end"))
        self.debug_step_title.setText(self._tr("sweep_step"))
        self.debug_sound_speed_title.setText(self._tr("debug_sound_speed"))
        self.sample_count_title.setText(self._tr("sample_count"))
        self.capture_type_title.setText(self._tr("capture_type"))
        self.pulse_count_title.setText(self._tr("pulse_count"))
        self.upload_wave_check.setText(self._tr("upload_wave"))
        self.single_measure_button.setText(
            self._tr("test_button") if self.debug_probe_type == ProbeType.DEPTH_PROBE else self._tr("single_measure")
        )
        self.start_debug_sweep_button.setText(self._tr("start_sweep"))
        self.save_wave_button.setText(self._tr("save_wave_csv"))
        self.debug_session_title.setText(self._tr("session"))

        if self.worker is None or not self.worker.isRunning():
            self.connection_label.setText(self._tr("disconnected"))
        if self.device_serial:
            self.device_label.setText(self.device_label.text())
        else:
            self.device_label.setText(self._tr("device_na"))
        self.status_label.setText(f"{self._tr('status_idle').split(':')[0]}: {self.device_status.name}")
        self.angle_label.setText(self._tr("angle_fmt", value=self._format_angle_value(self.current_angle)))
        self.event_label.setText(self._tr("event_none") if self.event_label.text().endswith("none") or self.event_label.text().endswith("无") else self.event_label.text())
        self.debug_target_label.setText(self._tr("target", value=self._suffix_after_colon(self.debug_target_label.text())))
        self.debug_move_label.setText(self._tr("direction", value=self._suffix_after_colon(self.debug_move_label.text())))
        self.debug_arrived_label.setText(self._tr("arrived", value=self._suffix_after_colon(self.debug_arrived_label.text())))
        self.debug_distance_label.setText(self._tr("distance", value=self._suffix_after_colon(self.debug_distance_label.text())))
        self.debug_quality_label.setText(self._tr("quality", value=self._suffix_after_colon(self.debug_quality_label.text())))
        self._update_debug_sample_rate_label()
        self.wave_info_label.setText(self._tr("wave", value=self._suffix_after_colon(self.wave_info_label.text())))

        self.wave_plot.setTitle(self._tr("wave_plot_title"))
        self.wave_plot.setLabel("bottom", "Time (us)")
        self.wave_plot.setLabel("top", "Distance (mm)")
        self.debug_sweep_plot.setTitle(self._tr("debug_sweep_plot_title"))
        if self.point_cloud_window is not None:
            self.point_cloud_window.set_title(self._tr("cloud_window_title"))

        self._update_expected_points_label()
        self._update_progress_widgets()
        self._update_ring_and_axial_labels()
        self._update_front_distance_label()
        self._update_point_cloud_status_label()
        self._refresh_wave_plot_axes()

    def _handle_tab_changed(self, index: int) -> None:
        self._sync_mode_label()
        if index == self.tabs.indexOf(self.debug_tab):
            self._update_debug_probe_mode_ui()
            self._notify_debug_mode_switch()

    def _handle_debug_probe_change(self, index: int) -> None:
        self.debug_probe_type = ProbeType.DEPTH_PROBE if index == 0 else ProbeType.ROTARY_DEBUG
        self._update_debug_probe_mode_ui()
        if self.tabs.currentWidget() == self.debug_tab:
            self._notify_debug_mode_switch()

    def _normalize_debug_angle_inputs(self) -> None:
        if self.debug_probe_type == ProbeType.DEPTH_PROBE:
            return
        start_value = self._normalize_motor_angle_degrees(self.debug_start_spin.value())
        end_value = self._normalize_motor_angle_degrees(self.debug_end_spin.value())
        step_value = self._normalize_motor_step_degrees(self.debug_step_spin.value())

        if end_value < start_value:
            end_value = start_value

        self._set_spinbox_value_if_needed(self.debug_start_spin, start_value)
        self._set_spinbox_value_if_needed(self.debug_end_spin, end_value)
        self._set_spinbox_value_if_needed(self.debug_step_spin, step_value)

    def _update_debug_probe_mode_ui(self) -> None:
        depth_mode = self.debug_probe_type == ProbeType.DEPTH_PROBE
        angle_widgets = (self.debug_start_spin, self.debug_end_spin, self.debug_step_spin)
        for widget in angle_widgets:
            widget.setEnabled(not depth_mode)
        connected = self.worker is not None and self.worker.isRunning()
        self.start_debug_sweep_button.setEnabled(not depth_mode and connected)
        if depth_mode:
            self.debug_start_spin.setValue(0.0)
            self.debug_end_spin.setValue(0.0)
            self.debug_step_spin.setValue(0.0)
        else:
            if self.debug_step_spin.value() <= 0.0:
                self.debug_step_spin.setValue(self._normalize_motor_step_degrees(SIDE_SCAN_DEFAULT_STEP_DEG))
            self._normalize_debug_angle_inputs()
        self.sample_count_spin.setValue(7500)
        self.sample_count_spin.setEnabled(False)
        self.capture_type_combo.setCurrentIndex(0)
        self.capture_type_combo.setEnabled(False)
        self.single_measure_button.setText(
            self._tr("test_button") if depth_mode else self._tr("single_measure")
        )

    def _notify_debug_mode_switch(self) -> None:
        if self.worker is None or not self.worker.isRunning():
            return
        self._configure_debug_single()

    def _toggle_point_cloud_window(self, enabled: bool) -> None:
        if self.point_cloud_window is not None:
            self.point_cloud_window.setVisible(enabled)
        self.scan_table.setColumnHidden(3, not enabled)
        if enabled:
            self._update_point_cloud_window()
        self._update_point_cloud_status_label()

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        ports = [SIMULATOR_PORT] + [port.device for port in list_ports.comports()]
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)
        else:
            self.port_combo.setCurrentText(SIMULATOR_PORT)
        self.port_combo.blockSignals(False)

    def _connect_transport(self) -> None:
        if self.worker is not None and self.worker.isRunning():
            return

        port_name = self.port_combo.currentText().strip()
        if not port_name:
            self._append_log("No transport selected.")
            return

        self.worker = TransportWorker()
        self.worker.configure(port_name, int(self.baud_combo.currentText()))
        self.worker.frame_received.connect(self._handle_frame)
        self.worker.status_changed.connect(self._handle_status)
        self.worker.finished.connect(self._worker_finished)
        self.worker.start()

        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)

    def _disconnect_transport(self) -> None:
        if self.worker is not None:
            self.worker.stop()

    def _worker_finished(self) -> None:
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.worker = None

    def _request_device_info(self) -> None:
        self._send_packet(encode_get_device_info(self._next_sequence()), "GET_DEVICE_INFO")

    def _request_zeroing(self) -> None:
        self._send_packet(encode_zeroing(self._next_sequence()), "ZEROING")

    def _request_stop(self) -> None:
        self.auto_scan_active = False
        self._finalize_point_cloud_render(include_preview=True)
        self._send_packet(encode_stop(self._next_sequence()), "STOP")

    def _configure_scan(self) -> None:
        self._normalize_scan_step_input()
        config = ScanConfig(
            scan_step=int(round(self.scan_step_spin.value() * 100.0)),
            sound_speed=self.sound_speed_spin.value(),
            medium=MediumType(self.medium_combo.currentData()),
            flags=0,
        )
        self.last_point_count = self._estimated_scan_points()
        self._update_expected_points_label()
        self._update_progress_widgets()
        self._send_packet(encode_scan_config(self._next_sequence(), config), "CONFIG_SCAN")

    def _start_scan(self) -> None:
        if not self._front_clearance_ok(check_for_advance=False):
            self.auto_scan_active = False
            return

        starting_new_session = not self.auto_scan_active
        if starting_new_session:
            self._reset_scan_table()

        continuing_same_position = (
            self.enable_3d_check.isChecked()
            and self.auto_scan_active
            and self.rings_at_position > 0
            and self.rings_at_position < self.rings_per_level_spin.value()
        )

        if self.enable_3d_check.isChecked() and not self.auto_scan_active:
            self.axial_position_mm = 0.0
            self.lap_count = 0
            self.rings_at_position = 0
            self.point_cloud_points.clear()
            self.point_cloud_layers.clear()
            self.cylinder_fit = None
            self.surface_mesh = None
            self.mesh_render_only = False
            self.pending_cylinder_fit = False
            self._update_ring_and_axial_labels()
            self._new_scan_position(reset_point_cloud=False)
        elif not continuing_same_position:
            self._new_scan_position(reset_point_cloud=False)

        self.current_ring_points_3d = []
        self.mesh_render_only = False
        self.current_ring_received = 0
        self.scan_started_at = time.time()
        if not continuing_same_position:
            self._reset_scan_views()
        self._set_mode("Scan")
        self.auto_scan_active = self.enable_3d_check.isChecked()
        self._send_packet(encode_start_scan(self._next_sequence()), "START_SCAN")

    def _configure_debug_single(self) -> None:
        depth_mode = self.debug_probe_type == ProbeType.DEPTH_PROBE
        if not depth_mode:
            self._normalize_debug_angle_inputs()
        angle = 0 if depth_mode else int(round(self.debug_start_spin.value() * 100.0))
        flags = 0x05
        if self.upload_wave_check.isChecked():
            flags |= 0x08

        config = DebugConfig(
            start_angle=angle,
            end_angle=angle,
            step_angle=0,
            capture_type=CaptureType.RAW_WAVE,
            probe_type=self.debug_probe_type,
            sample_count=7500,
            pulse_count=self.pulse_count_spin.value(),
            flags=flags,
        )
        self.debug_target_label.setText(self._tr("target", value=self._format_angle(angle)))
        self._send_packet(encode_debug_config(self._next_sequence(), config), "CONFIG_DEBUG(single)")

    def _configure_debug_sweep(self) -> None:
        if self.debug_probe_type == ProbeType.DEPTH_PROBE:
            self._append_log("Depth probe mode does not support sweep config.")
            return
        self._normalize_debug_angle_inputs()
        start_angle = int(round(self.debug_start_spin.value() * 100.0))
        end_angle = int(round(self.debug_end_spin.value() * 100.0))
        step_angle = int(round(self.debug_step_spin.value() * 100.0))
        if end_angle < start_angle:
            self._append_log("Sweep config rejected: end angle must be >= start angle.")
            return
        if (end_angle > start_angle) and (step_angle == 0):
            step_angle = int(round(SIDE_SCAN_MIN_STEP_DEG * 100.0))
            self.debug_step_spin.blockSignals(True)
            self.debug_step_spin.setValue(SIDE_SCAN_MIN_STEP_DEG)
            self.debug_step_spin.blockSignals(False)
        flags = 0x06
        if self.upload_wave_check.isChecked():
            flags |= 0x08

        config = DebugConfig(
            start_angle=start_angle,
            end_angle=end_angle,
            step_angle=step_angle,
            capture_type=CaptureType.RAW_WAVE,
            probe_type=self.debug_probe_type,
            sample_count=7500,
            pulse_count=self.pulse_count_spin.value(),
            flags=flags,
        )
        self._send_packet(encode_debug_config(self._next_sequence(), config), "CONFIG_DEBUG(sweep)")

    def _move_to_angle(self) -> None:
        if self.debug_probe_type == ProbeType.DEPTH_PROBE:
            self._append_log("Depth probe mode keeps the motor fixed.")
            return
        self._normalize_debug_angle_inputs()
        angle = int(round(self.debug_start_spin.value() * 100.0))
        self.debug_target_label.setText(self._tr("target", value=self._format_angle(angle)))
        self._send_packet(encode_move_to_angle(self._next_sequence(), angle), "MOVE_TO_ANGLE")

    def _start_single_measure(self) -> None:
        self.debug_started_at = time.time()
        self.measured_sample_rate_hz = 0.0
        self.current_wave_bytes = b""
        self.current_wave_samples.clear()
        self.current_wave_display_samples.clear()
        self.wave_capture_history.clear()
        self.wave_streams.clear()
        self.wave_curve.setData([], [])
        self._set_mode("Debug Single")
        self.debug_session_label.setText("Single measure running.")
        self._configure_debug_single()
        QtCore.QTimer.singleShot(
            120,
            lambda: self._send_packet(encode_start_single_measure(self._next_sequence()), "START_SINGLE_MEASURE"),
        )

    def _start_debug_sweep(self) -> None:
        if self.debug_probe_type == ProbeType.DEPTH_PROBE:
            self._append_log("Depth probe mode does not support debug sweep.")
            return
        self.current_debug_points.clear()
        self.debug_started_at = time.time()
        self.measured_sample_rate_hz = 0.0
        self.current_wave_bytes = b""
        self.current_wave_samples.clear()
        self.current_wave_display_samples.clear()
        self.wave_capture_history.clear()
        self.wave_streams.clear()
        self.wave_curve.setData([], [])
        self._reset_debug_views()
        self._set_mode("Debug Sweep")
        self.debug_session_label.setText("Debug sweep running.")
        self._configure_debug_sweep()
        QtCore.QTimer.singleShot(
            120,
            lambda: self._send_packet(encode_start_debug_sweep(self._next_sequence()), "START_DEBUG_SWEEP"),
        )

    def _save_scan_csv(self) -> None:
        if self.scan_table.rowCount() == 0:
            self._append_log("Save skipped: no table data available.")
            return

        suggested = self._default_scan_export_name()
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Table Data" if self.language == "en" else "保存表格数据",
            str(APP_DIR / suggested),
            "CSV Files (*.csv)",
        )
        if not path:
            return

        if not path.lower().endswith(".csv"):
            path += ".csv"

        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    self.scan_table.horizontalHeaderItem(column).text()
                    if self.scan_table.horizontalHeaderItem(column) is not None
                    else f"col_{column}"
                    for column in range(self.scan_table.columnCount())
                ]
            )
            for row in range(self.scan_table.rowCount()):
                writer.writerow(
                    [
                        self.scan_table.item(row, column).text()
                        if self.scan_table.item(row, column) is not None
                        else ""
                        for column in range(self.scan_table.columnCount())
                    ]
                )
        self._append_log(f"Saved scan CSV: {path}")

    def _default_scan_export_name(self) -> str:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        point_count = self.scan_table.rowCount()
        step_text = self._compact_angle_token(self.scan_step_spin.value())
        mode_tag = "3d" if self.enable_3d_check.isChecked() else "2d"
        return f"scan_{stamp}_p{point_count}_a{step_text}_{mode_tag}.csv"

    def _compact_angle_token(self, value: float) -> str:
        text = f"{value:.3f}".rstrip("0").rstrip(".")
        return text.replace(".", "p")

    def _save_wave_csv(self) -> None:
        if not self.current_wave_samples and not self.wave_capture_history:
            self._append_log("Save skipped: no waveform available.")
            return

        if len(self.wave_capture_history) > 1:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            default_dir = APP_DIR / f"wave_sweep_{stamp}"
            target_dir = QtWidgets.QFileDialog.getExistingDirectory(
                self,
                "Save Sweep Waveforms",
                str(default_dir.parent),
            )
            if not target_dir:
                return
            output_dir = Path(target_dir) / default_dir.name
            output_dir.mkdir(parents=True, exist_ok=True)

            index_path = output_dir / "wave_index.csv"
            with index_path.open("w", newline="", encoding="utf-8") as index_handle:
                index_writer = csv.writer(index_handle)
                index_writer.writerow(["wave_index", "angle_deg", "data_type", "sample_count", "csv_file"])
                for wave_index, wave in enumerate(self.wave_capture_history):
                    filename = f"wave_{wave_index:03d}_{wave['angle']:05d}.csv"
                    csv_path = output_dir / filename
                    self._write_wave_csv(
                        csv_path,
                        wave["data_type"],
                        wave["samples"],
                        float(wave.get("sample_rate_hz", self._effective_wave_sample_rate_hz())),
                    )
                    index_writer.writerow(
                        [
                            wave_index,
                            f"{wave['angle'] / 100.0:.2f}",
                            wave["data_type"].name,
                            len(wave["samples"]),
                            filename,
                        ]
                    )

            self._append_log(f"Saved {len(self.wave_capture_history)} sweep waveforms: {output_dir}")
            return

        suggested = f"wave_{self.current_wave_angle:05d}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Waveform",
            str(APP_DIR / suggested),
            "CSV Files (*.csv)",
        )
        if not path:
            return

        self._write_wave_csv(Path(path), self.current_wave_data_type, self.current_wave_samples, self._effective_wave_sample_rate_hz())
        self._append_log(f"Saved waveform CSV: {path}")

    def _write_wave_csv(self, path: Path, data_type: WaveDataType, samples: list[int], sample_rate_hz: float) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            if data_type in (WaveDataType.U16_RAW, WaveDataType.U16_ENVELOPE):
                writer.writerow(["sample_index", "time_us", "distance_mm", "raw_code", "voltage_v"])
                for index, value in enumerate(samples):
                    time_us = (float(index) / sample_rate_hz) * 1_000_000.0
                    writer.writerow(
                        [
                            index,
                            f"{time_us:.6f}",
                            f"{self._time_us_to_distance_mm(time_us):.6f}",
                            value,
                            f"{(float(value) * ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE):.6f}",
                        ]
                    )
            else:
                writer.writerow(["sample_index", "time_us", "distance_mm", "value"])
                for index, value in enumerate(samples):
                    time_us = (float(index) / sample_rate_hz) * 1_000_000.0
                    writer.writerow([index, f"{time_us:.6f}", f"{self._time_us_to_distance_mm(time_us):.6f}", value])

    def _send_packet(self, packet: bytes, label: str) -> None:
        if self.worker is None or not self.worker.isRunning():
            self._append_log(f"TX skipped: {label}")
            return

        frame = self._decode_frame_bytes(packet)
        self._append_log(f"TX {label}: {packet.hex(' ')}")
        if frame is not None:
            self.recorder.log_packet(direction="tx", label=label, frame=frame)
        self.worker.send_packet(packet)

    def _handle_status(self, message: str) -> None:
        if message.startswith(f"Connected: {SIMULATOR_PORT}"):
            display_message = self._tr("connected_loopback", port=SIMULATOR_PORT, baud=self.baud_combo.currentText())
        elif message == "Disconnected":
            display_message = self._tr("disconnected")
        else:
            display_message = message
        self.connection_label.setText(display_message)
        self._append_log(f"[transport] {message}")

        if message.startswith("Connected:"):
            self._set_transport_connected(True)
            QtCore.QTimer.singleShot(150, self._request_device_info)
            if self.tabs.currentWidget() == self.debug_tab:
                QtCore.QTimer.singleShot(260, self._notify_debug_mode_switch)
            return

        if message == "Disconnected":
            self._set_transport_connected(False)

    def _handle_frame(self, frame: Frame) -> None:
        packet = frame.encode()
        try:
            command = CommandId(frame.command)
        except ValueError:
            self._append_log(f"RX UNKNOWN({frame.command:#x}): {frame.payload.hex(' ')}")
            return

        if not self._is_high_rate_command(command):
            self._append_log(f"RX {command.name}: {packet.hex(' ')}")
        self.recorder.log_packet(direction="rx", label=command.name, frame=frame)

        if command == CommandId.ACK:
            self._handle_ack(frame)
        elif command == CommandId.NACK:
            self._handle_nack(frame)
        elif command == CommandId.SCAN_POINT:
            self._handle_scan_point(frame)
        elif command == CommandId.SCAN_FINISH:
            self._handle_scan_finish(frame)
        elif command == CommandId.STATUS_EVENT:
            self._handle_status_event(frame)
        elif command == CommandId.DISTANCE_RESULT:
            self._handle_distance_result(frame)
        elif command == CommandId.WAVE_CHUNK:
            self._handle_wave_chunk(frame)
        elif command == CommandId.WAVE_FINISH:
            self._handle_wave_finish(frame)
        elif command == CommandId.DEBUG_SWEEP_POINT:
            self._handle_debug_sweep_point(frame)
        elif command == CommandId.DEBUG_SWEEP_FINISH:
            self._handle_debug_sweep_finish(frame)

    def _handle_ack(self, frame: Frame) -> None:
        ack = decode_ack_payload(frame.payload)
        ref_cmd = CommandId(ack["ref_cmd"])
        status = DeviceStatus(ack["status"])
        detail = ack["detail"]
        self._set_device_status(status)

        if ref_cmd == CommandId.GET_DEVICE_INFO:
            info = parse_device_info_detail(detail)
            self.device_serial = str(info["serial_number"] or "n/a")
            self.device_label.setText(
                f"Device: SN {self.device_serial} / FW {info['fw_major']}.{info['fw_minor']} / HW {info['hw_major']}.{info['hw_minor']}"
            )
            self._append_log(
                f"Device info serial={self.device_serial} max_baud={info['max_baud']} capabilities=0x{info['capabilities']:08X}"
            )
            self.recorder.log_event("device_info", **info)
            return

        if ref_cmd == CommandId.ZEROING:
            zeroing = parse_zeroing_ack_detail(detail)
            self._set_current_angle(zeroing["current_angle"])
            self._set_last_event("Zeroing completed")
            self.scan_session_label.setText(self._tr("idle"))
            self.debug_session_label.setText(self._tr("idle"))
            return

        if ref_cmd == CommandId.CONFIG_SCAN:
            scan_info = parse_scan_config_ack_detail(detail)
            self.last_point_count = scan_info["point_count"]
            self._set_current_angle(scan_info["current_angle"])
            self._update_expected_points_label()
            self._update_progress_widgets()
            self.scan_session_label.setText("Configured")
            self._set_last_event("Scan config applied")
            return

        if ref_cmd == CommandId.START_SCAN:
            self.scan_session_label.setText("Scan accepted.")
            self._set_mode("Scan")
            return

        if ref_cmd == CommandId.CONFIG_DEBUG:
            debug_info = parse_debug_config_ack_detail(detail)
            sample_rate_hz = int(debug_info.get("sample_rate_hz", WAVE_SAMPLE_RATE_HZ))
            if sample_rate_hz > 0:
                self.runtime_sample_rate_hz = float(sample_rate_hz)
                self.measured_sample_rate_hz = 0.0
                self._refresh_wave_plot_axes()
            self._update_debug_sample_rate_label()
            self.debug_session_label.setText(
                f"Debug config accepted={debug_info['accepted']} normalized={debug_info['normalized']}"
            )
            self._set_last_event("Debug config applied")
            return

        if ref_cmd == CommandId.MOVE_TO_ANGLE:
            move = parse_move_to_angle_ack_detail(detail)
            self._set_current_angle(move["current_angle"])
            self.debug_target_label.setText(self._tr("target", value=self._format_angle(move["target_angle"])))
            self.debug_move_label.setText(self._tr("direction", value=MOVE_DIR_LABELS.get(move["move_dir"], "n/a")))
            self.debug_arrived_label.setText(self._tr("arrived", value="YES" if move["arrived"] else "NO"))
            self.debug_session_label.setText("Target angle reached.")
            self._set_last_event("Move acknowledged")
            return

        if ref_cmd == CommandId.START_SINGLE_MEASURE:
            self._set_mode("Debug Single")
            self.debug_session_label.setText("Single measure accepted.")
            return

        if ref_cmd == CommandId.START_DEBUG_SWEEP:
            self._set_mode("Debug Sweep")
            self.debug_session_label.setText("Debug sweep accepted.")
            return

        if ref_cmd == CommandId.STOP:
            self.scan_session_label.setText("Stop acknowledged.")
            self.debug_session_label.setText("Stop acknowledged.")
            self._set_last_event("Stop acknowledged")

    def _handle_nack(self, frame: Frame) -> None:
        nack = decode_nack_payload(frame.payload)
        self._set_device_status(DeviceStatus(nack["status"]))
        try:
            ref_name = CommandId(nack["ref_cmd"]).name
        except ValueError:
            ref_name = f"0x{nack['ref_cmd']:02X}"
        self._set_last_event(f"NACK {ref_name} result={nack['result']} error={nack['error_code']}")

    def _handle_scan_point(self, frame: Frame) -> None:
        point = parse_scan_point(frame.payload)
        point.timestamp_ms = self._elapsed_ms(self.scan_started_at)
        averaged_point = self._accumulate_scan_point(point)
        self.current_ring_received += 1
        self._set_current_angle(point.angle)
        self._update_progress_widgets()
        self._append_scan_point(averaged_point, z_value=self.axial_position_mm)
        self._schedule_scan_visual_refresh()
        if self.enable_3d_check.isChecked():
            self._schedule_point_cloud_refresh()
        self.recorder.append_scan_point(averaged_point, channel="scan")

    def _handle_scan_finish(self, frame: Frame) -> None:
        finish = parse_scan_finish(frame.payload)
        reason = END_REASON_LABELS.get(finish["end_reason"], f"0x{finish['end_reason']:02X}")
        self.lap_count += 1
        self.rings_at_position += 1
        if self.enable_3d_check.isChecked() and self.rings_at_position >= self.rings_per_level_spin.value():
            position_points = self._current_position_points_3d()
            if position_points:
                self.point_cloud_layers.append(position_points)
                self.point_cloud_points = [point for layer in self.point_cloud_layers for point in layer]
        self.current_ring_points_3d = []
        self._set_current_angle(0)
        self._schedule_point_cloud_refresh()
        self._update_ring_and_axial_labels()
        self.scan_session_label.setText(
            f"Scan finished: points={finish['total_points']} reason={reason} dropped={finish['dropped']}"
        )
        self._set_last_event(f"Scan finished ({reason})")
        self._set_device_status(DeviceStatus.READY)

        if self.enable_3d_check.isChecked() and self.auto_scan_active:
            self._advance_3d_position_if_needed()
            if self.auto_scan_active:
                QtCore.QTimer.singleShot(120, self._start_scan)

    def _handle_status_event(self, frame: Frame) -> None:
        event = parse_status_event(frame.payload)
        self._set_current_angle(event["current_angle"])
        self._set_device_status(DeviceStatus(event["status"]))
        label = EVENT_LABELS.get(event["event"], f"Event 0x{event['event']:02X}")
        self._set_last_event(f"{label} / detail={event['detail']}")
        self.recorder.log_event(
            "status_event",
            event=event["event"],
            status=event["status"],
            current_angle=event["current_angle"],
            detail=event["detail"],
        )

    def _handle_distance_result(self, frame: Frame) -> None:
        result = parse_distance_result(frame.payload)
        self.debug_distance_label.setText(self._tr("distance", value=f"{result.distance_mm} mm"))
        self.debug_quality_label.setText(self._tr("quality", value=f"{result.quality} / valid={int(result.valid)}"))
        self._set_current_angle(result.angle)
        self.debug_session_label.setText("Distance result received.")
        self.recorder.append_distance_result(result)

    def _handle_wave_chunk(self, frame: Frame) -> None:
        chunk = parse_wave_chunk(frame.payload)
        stream = self.wave_streams.setdefault(
            frame.sequence,
            {
                "angle": chunk.angle,
                "data_type": chunk.data_type,
                "total_frames": chunk.total_frames,
                "chunks": {},
            },
        )
        stream["angle"] = chunk.angle
        stream["data_type"] = chunk.data_type
        stream["total_frames"] = chunk.total_frames
        stream["chunks"][chunk.frame_id] = chunk.samples
        self.wave_info_label.setText(
            self._tr(
                "wave",
                value=f"{len(stream['chunks'])}/{chunk.total_frames} frames @ {self._format_angle(chunk.angle)}",
            )
        )

    def _handle_wave_finish(self, frame: Frame) -> None:
        finish = parse_wave_finish(frame.payload)
        stream = self.wave_streams.pop(frame.sequence, None)
        if stream is None:
            self._append_log("Wave finish received without matching chunks.")
            return

        measured_sample_rate_hz = int(finish.get("sample_rate_hz", 0))
        if measured_sample_rate_hz > 0:
            self.measured_sample_rate_hz = float(measured_sample_rate_hz)
            self._update_debug_sample_rate_label()

        total_frames = int(stream["total_frames"])
        chunks = stream["chunks"]
        raw_bytes = b"".join(chunks[index] for index in range(total_frames) if index in chunks)
        data_type = stream["data_type"]
        bytes_per_sample = 2 if data_type in (WaveDataType.U16_RAW, WaveDataType.U16_ENVELOPE) else 1
        raw_bytes = raw_bytes[: finish["total_samples"] * bytes_per_sample]

        self.current_wave_angle = finish["angle"]
        self.current_wave_data_type = data_type
        self.current_wave_bytes = raw_bytes
        self.current_wave_samples = self._decode_wave_samples(data_type, raw_bytes)
        self.current_wave_display_samples = self._wave_samples_to_display(data_type, self.current_wave_samples)
        self.wave_capture_history.append(
            {
                "angle": finish["angle"],
                "data_type": data_type,
                "samples": list(self.current_wave_samples),
                "raw_bytes": raw_bytes,
                "sample_rate_hz": self._effective_wave_sample_rate_hz(),
            }
        )
        x_values = [self._sample_index_to_time_us(index) for index in range(len(self.current_wave_display_samples))]
        self.wave_curve.setData(x_values, self.current_wave_display_samples)
        self.wave_plot.setLabel("left", self._wave_axis_label(data_type))
        self._refresh_wave_plot_axes()
        self.wave_info_label.setText(
            self._tr(
                "wave",
                value=f"{len(self.current_wave_samples)} samples / {data_type.name} @ {self._format_angle(finish['angle'])}",
            )
        )
        self.debug_session_label.setText("Wave upload completed.")
        self.recorder.save_waveform(
            angle=finish["angle"],
            data_type=data_type,
            raw_bytes=raw_bytes,
            sample_count=finish["total_samples"],
            sound_speed_m_s=self.debug_sound_speed_spin.value(),
            sample_rate_hz=self._effective_wave_sample_rate_hz(),
        )

    def _handle_debug_sweep_point(self, frame: Frame) -> None:
        point = parse_debug_sweep_point(frame.payload)
        point.timestamp_ms = self._elapsed_ms(self.debug_started_at)
        self.current_debug_points.append(point)
        self._set_current_angle(point.angle)
        self._append_debug_point(point)
        self._refresh_debug_plot()
        self.debug_session_label.setText(f"Debug sweep points: {len(self.current_debug_points)}")
        self.recorder.append_scan_point(point, channel="debug_sweep")

    def _handle_debug_sweep_finish(self, frame: Frame) -> None:
        finish = parse_debug_sweep_finish(frame.payload)
        reason = END_REASON_LABELS.get(finish["end_reason"], f"0x{finish['end_reason']:02X}")
        self.debug_session_label.setText(
            f"Debug sweep finished: points={finish['total_points']} reason={reason}"
        )
        self._set_last_event(f"Debug sweep finished ({reason})")
        self._set_device_status(DeviceStatus.READY)

    def _new_scan_position(self, *, reset_point_cloud: bool) -> None:
        self.current_scan_points.clear()
        self.scan_point_list_index.clear()
        self.scan_point_accumulators.clear()
        self.current_ring_points_3d.clear()
        self.current_ring_received = 0
        if reset_point_cloud:
            self.point_cloud_points.clear()
            self.point_cloud_layers.clear()
            self.cylinder_fit = None
            self.surface_mesh = None
            self.mesh_render_only = False
        self.pending_cylinder_fit = False

    def _accumulate_scan_point(self, point: ScanPoint) -> ScanPoint:
        entry = self.scan_point_accumulators.get(point.point_index)
        if entry is None:
            entry = {
                "distance_sum": float(point.distance_mm),
                "quality_sum": float(point.quality),
                "count": 1,
                "angle": point.angle,
                "valid_count": 1 if point.valid else 0,
                "timestamp_ms": point.timestamp_ms,
            }
            self.scan_point_accumulators[point.point_index] = entry
            averaged = ScanPoint(
                point_index=point.point_index,
                angle=point.angle,
                distance_mm=point.distance_mm,
                quality=point.quality,
                valid=point.valid,
                timestamp_ms=point.timestamp_ms,
            )
            self.current_scan_points.append(averaged)
            self.scan_point_list_index[point.point_index] = len(self.current_scan_points) - 1
            return averaged

        entry["distance_sum"] = float(entry["distance_sum"]) + point.distance_mm
        entry["quality_sum"] = float(entry["quality_sum"]) + point.quality
        entry["count"] = int(entry["count"]) + 1
        entry["valid_count"] = int(entry["valid_count"]) + (1 if point.valid else 0)
        entry["timestamp_ms"] = point.timestamp_ms

        row = self.scan_point_list_index[point.point_index]
        averaged_point = self.current_scan_points[row]
        count = int(entry["count"])
        averaged_point.distance_mm = int(round(float(entry["distance_sum"]) / count))
        averaged_point.quality = int(round(float(entry["quality_sum"]) / count))
        averaged_point.valid = int(entry["valid_count"]) > 0
        averaged_point.timestamp_ms = point.timestamp_ms
        return averaged_point

    def _append_scan_point(self, point: ScanPoint, *, z_value: float) -> None:
        row = self.scan_table.rowCount()
        self.scan_table.insertRow(row)
        values = (
            str(point.point_index),
            f"{point.angle / 100.0:.2f}",
            str(point.distance_mm),
            f"{z_value:.1f}",
            str(point.quality),
            "YES" if point.valid else "NO",
            str(point.timestamp_ms),
        )
        for column, value in enumerate(values):
            self.scan_table.setItem(row, column, QtWidgets.QTableWidgetItem(value))
        if row % 24 == 0:
            self.scan_table.scrollToBottom()

    def _append_debug_point(self, point: ScanPoint) -> None:
        row = self.debug_table.rowCount()
        self.debug_table.insertRow(row)
        self.debug_table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(point.point_index)))
        self.debug_table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{point.angle / 100.0:.2f}"))
        self.debug_table.setItem(row, 2, QtWidgets.QTableWidgetItem(str(point.distance_mm)))
        self.debug_table.setItem(row, 3, QtWidgets.QTableWidgetItem(str(point.quality)))
        self.debug_table.setItem(row, 4, QtWidgets.QTableWidgetItem("YES" if point.valid else "NO"))
        self.debug_table.setItem(row, 5, QtWidgets.QTableWidgetItem("YES" if point.has_wave else "NO"))
        self.debug_table.scrollToBottom()

    def _schedule_scan_visual_refresh(self) -> None:
        if not self.scan_visual_timer.isActive():
            self.scan_visual_timer.start()

    def _schedule_point_cloud_refresh(self) -> None:
        if not self.point_cloud_timer.isActive():
            self.point_cloud_timer.start()

    def _apply_scan_visuals(self) -> None:
        self._refresh_scan_plot()
        self._refresh_scan_summary()

    def _apply_point_cloud_update(self) -> None:
        self._update_point_cloud_window()

    def _current_position_points_3d(self) -> list[tuple[float, float, float]]:
        ordered_points = sorted(self.current_scan_points, key=lambda point: point.point_index)
        return [self._scan_point_to_3d(point) for point in ordered_points if point.valid]

    def _build_surface_mesh(self, *, include_preview: bool) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        layers = [list(layer) for layer in self.point_cloud_layers if len(layer) >= 3]
        if include_preview:
            preview = self._current_position_points_3d()
            if preview:
                should_append_preview = True
                if layers:
                    last_layer = layers[-1]
                    if len(last_layer) == len(preview) and abs(last_layer[0][2] - preview[0][2]) < 0.5:
                        should_append_preview = False
                if should_append_preview:
                    layers.append(preview)
        if not layers:
            preview = self._current_position_points_3d()
            if len(preview) >= 3:
                layers = [preview]
        if not layers:
            return None

        min_count = min(len(layer) for layer in layers)
        if min_count < 3:
            return None

        normalized_layers: list[list[tuple[float, float, float]]] = []
        for layer in layers:
            ordered = sorted(
                layer,
                key=lambda point: (math.atan2(point[1], point[0]) + 2.0 * math.pi) % (2.0 * math.pi),
            )
            normalized_layers.append(ordered[:min_count] + [ordered[0]])

        if len(normalized_layers) == 1:
            top_layer = normalized_layers[0]
            thickness = max(40.0, min(float(self.axial_step_spin.value()) * 0.45, 180.0))
            bottom_layer = [(x_value, y_value, z_value - thickness) for x_value, y_value, z_value in top_layer]
            normalized_layers = [top_layer, bottom_layer]

        x_grid = np.array([[point[0] for point in layer] for layer in normalized_layers], dtype=float)
        y_grid = np.array([[point[1] for point in layer] for layer in normalized_layers], dtype=float)
        z_grid = np.array([[point[2] for point in layer] for layer in normalized_layers], dtype=float)
        return (x_grid, y_grid, z_grid)

    def _finalize_point_cloud_render(self, *, include_preview: bool) -> None:
        if not self.enable_3d_check.isChecked():
            return
        self.surface_mesh = self._build_surface_mesh(include_preview=include_preview)
        self.mesh_render_only = self.surface_mesh is not None
        self.pending_cylinder_fit = False
        self._schedule_point_cloud_refresh()

    def _fit_cylinder(self, points: list[tuple[float, float, float]]) -> tuple[float, float, float, float, float] | None:
        if len(points) < 8:
            return None
        radii = [math.hypot(x_value, y_value) for x_value, y_value, _ in points]
        if not radii:
            return None
        z_values = [point[2] for point in points]
        z_min = min(z_values)
        z_max = max(z_values)
        if z_max - z_min < 1.0:
            z_min -= max(1.0, float(self.axial_step_spin.value()))
        return (float(sum(radii) / len(radii)), 0.0, 0.0, float(z_min), float(z_max))

    def _refresh_scan_plot(self) -> None:
        ordered_points = sorted(self.current_scan_points, key=lambda point: point.point_index)
        polar_x = [point.distance_mm * math.cos(math.radians(point.angle / 100.0)) for point in ordered_points]
        polar_y = [point.distance_mm * math.sin(math.radians(point.angle / 100.0)) for point in ordered_points]
        self.polar_points.setData(polar_x, polar_y)
        if len(polar_x) > 1:
            polar_x.append(polar_x[0])
            polar_y.append(polar_y[0])
        self.polar_curve.setData(polar_x, polar_y)
        self._update_polar_frame()

    def _refresh_debug_plot(self) -> None:
        x_values = [point.angle / 100.0 for point in self.current_debug_points]
        y_values = [point.distance_mm for point in self.current_debug_points]
        self.debug_sweep_curve.setData(x_values, y_values)

    def _refresh_scan_summary(self) -> None:
        self.points_done_label.setText(str(len(self.current_scan_points)))
        valid_count = sum(1 for point in self.current_scan_points if point.valid)
        self.valid_points_label.setText(str(valid_count))
        if not self.current_scan_points:
            self.distance_range_label.setText("n/a")
            return
        distances = [point.distance_mm for point in self.current_scan_points]
        self.distance_range_label.setText(f"{min(distances)} .. {max(distances)}")

    def _reset_scan_views(self) -> None:
        self.polar_curve.setData([], [])
        self.polar_points.setData([], [])
        self.polar_sweep_line.setData([], [])
        self.points_done_label.setText("0")
        self.valid_points_label.setText("0")
        self.distance_range_label.setText("n/a")
        self._update_progress_widgets()
        self.scan_session_label.setText("Scan started.")
        self._update_polar_frame()
        self._schedule_point_cloud_refresh()

    def _reset_scan_table(self) -> None:
        self.scan_table.setRowCount(0)

    def _reset_debug_views(self) -> None:
        self.debug_table.setRowCount(0)
        self.debug_sweep_curve.setData([], [])
        self._update_debug_sample_rate_label()

    def _set_device_status(self, status: DeviceStatus) -> None:
        self.device_status = status
        self.status_label.setText(f"Status: {status.name}")

    def _set_current_angle(self, angle: int) -> None:
        self.current_angle = angle
        self.angle_label.setText(self._tr("angle_fmt", value=self._format_angle(angle)))
        self._update_polar_frame()

    def _set_mode(self, mode_name: str) -> None:
        self.current_mode_name = mode_name
        mode_prefix = self._tr("mode_idle").split(":", 1)[0]
        self.mode_label.setText(f"{mode_prefix}: {mode_name}")

    def _sync_mode_label(self) -> None:
        if self.current_mode_name == "Idle":
            mode_prefix = self._tr("mode_idle").split(":", 1)[0]
            self.mode_label.setText(f"{mode_prefix}: {self.tabs.tabText(self.tabs.currentIndex())}")

    def _set_last_event(self, text: str) -> None:
        event_prefix = self._tr("event_none").split(":", 1)[0]
        self.event_label.setText(f"{event_prefix}: {text}")
        self._append_log(text)

    def _effective_wave_sample_rate_hz(self) -> float:
        if self.measured_sample_rate_hz > 0.0:
            return self.measured_sample_rate_hz
        if self.runtime_sample_rate_hz > 0.0:
            return self.runtime_sample_rate_hz
        return WAVE_SAMPLE_RATE_HZ

    def _update_debug_sample_rate_label(self) -> None:
        if self.measured_sample_rate_hz > 0.0:
            text = (
                f"{self._tr('sample_rate')}: "
                f"cfg {self.runtime_sample_rate_hz / 1_000_000.0:.6f} MHz / "
                f"meas {self.measured_sample_rate_hz / 1_000_000.0:.6f} MHz"
            )
        else:
            text = f"{self._tr('sample_rate')}: {self.runtime_sample_rate_hz / 1_000_000.0:.6f} MHz"
        self.debug_sample_rate_label.setText(text)

    def _set_transport_connected(self, connected: bool) -> None:
        self.info_button.setEnabled(connected)
        self.zero_button.setEnabled(connected)
        self.stop_button.setEnabled(connected)
        self.configure_scan_button.setEnabled(connected)
        self.start_scan_button.setEnabled(connected)
        self.single_measure_button.setEnabled(connected)
        self.start_debug_sweep_button.setEnabled(connected and self.debug_probe_type == ProbeType.ROTARY_DEBUG)
        self._update_debug_probe_mode_ui()
        if not connected:
            self.connection_label.setText(self._tr("disconnected"))

    def _estimated_scan_points(self) -> int:
        step = max(1, int(round(self.scan_step_spin.value() * 100.0)))
        return max(1, (36000 + step - 1) // step)

    def _normalize_motor_angle_degrees(self, value: float) -> float:
        value = min(max(value, 0.0), MOTOR_MAX_ANGLE_DEG)
        steps = round(value / MOTOR_ANGLE_RESOLUTION_DEG)
        return min(max(steps * MOTOR_ANGLE_RESOLUTION_DEG, 0.0), MOTOR_MAX_ANGLE_DEG)

    def _normalize_motor_step_degrees(self, value: float) -> float:
        value = min(max(value, SIDE_SCAN_MIN_STEP_DEG), SIDE_SCAN_MAX_STEP_DEG)
        steps = round(value / MOTOR_ANGLE_RESOLUTION_DEG)
        normalized = steps * MOTOR_ANGLE_RESOLUTION_DEG
        return min(max(normalized, SIDE_SCAN_MIN_STEP_DEG), SIDE_SCAN_MAX_STEP_DEG)

    def _set_spinbox_value_if_needed(self, widget: QtWidgets.QDoubleSpinBox, value: float) -> None:
        if abs(widget.value() - value) < 0.0005:
            return
        widget.blockSignals(True)
        widget.setValue(value)
        widget.blockSignals(False)

    def _normalize_scan_step_input(self) -> None:
        normalized = self._normalize_motor_step_degrees(self.scan_step_spin.value())
        self._set_spinbox_value_if_needed(self.scan_step_spin, normalized)
        self.last_point_count = self._estimated_scan_points()
        self._update_expected_points_label()
        self._update_progress_widgets()

    def _update_expected_points_label(self) -> None:
        value = self.last_point_count if self.last_point_count else "n/a"
        self.expected_points_label.setText(self._tr("expected_points", value=value))

    def _update_progress_widgets(self) -> None:
        total = self.last_point_count
        done = self.current_ring_received if self.current_ring_received > 0 else len(self.current_scan_points)
        self.scan_progress_label.setText(self._tr("scan_progress_text", done=done, total=total))
        if total > 0:
            self.scan_progress_bar.setRange(0, total)
            self.scan_progress_bar.setValue(min(done, total))
        else:
            self.scan_progress_bar.setRange(0, 1)
            self.scan_progress_bar.setValue(0)

    def _update_ring_and_axial_labels(self) -> None:
        self.ring_count_label.setText(str(self.lap_count))
        self.axial_position_label.setText(f"{self.axial_position_mm:.1f} mm")

    def _update_front_distance_label(self) -> None:
        if self.front_distance_mm <= 0:
            self.front_distance_value.setText("n/a")
        else:
            self.front_distance_value.setText(f"{self.front_distance_mm:.1f} mm")

    def _update_point_cloud_status_label(self) -> None:
        if self.enable_3d_check.isChecked():
            state = f"ON / points={len(self._active_point_cloud_points())} / z={self.axial_position_mm:.1f} mm / rings={self.rings_at_position}"
        else:
            state = "OFF"
        self.point_cloud_status_value.setText(state)

    def _polar_display_radius(self) -> float:
        if self.current_scan_points:
            radius = max(point.distance_mm for point in self.current_scan_points)
        else:
            radius = 1000.0
        radius = max(500.0, radius * 1.12)
        return float(radius)

    def _update_polar_frame(self) -> None:
        radius = self._polar_display_radius()
        self.polar_plot.setXRange(-radius, radius, padding=0.0)
        self.polar_plot.setYRange(-radius, radius, padding=0.0)

        self.polar_cross_h.setData([-radius, radius], [0.0, 0.0])
        self.polar_cross_v.setData([0.0, 0.0], [-radius, radius])

        circle_points = 96
        circle_x: list[float] = []
        circle_y: list[float] = []
        for index in range(circle_points + 1):
            theta = (2.0 * math.pi * index) / circle_points
            circle_x.append(radius * math.cos(theta))
            circle_y.append(radius * math.sin(theta))
        self.polar_range_circle.setData(circle_x, circle_y)

        sweep_theta = math.radians(self.current_angle / 100.0)
        sweep_x = radius * math.cos(sweep_theta)
        sweep_y = radius * math.sin(sweep_theta)
        self.polar_sweep_line.setData([0.0, sweep_x], [0.0, sweep_y])
        indicator_x = -radius * 0.82
        indicator_y = -radius * 0.82
        indicator_color = "#ef4444" if self.collision_risk_active else "#22c55e"
        self.polar_risk_indicator.setData(
            [indicator_x],
            [indicator_y],
            symbolBrush=indicator_color,
            symbolPen=pg.mkPen("#e2e8f0", width=1),
            symbolSize=18,
        )

    def _suffix_after_colon(self, text: str) -> str:
        if ":" not in text:
            return text
        return text.split(":", 1)[1].strip() or "n/a"

    def _format_angle_value(self, angle: int) -> str:
        return f"{angle / 100.0:.3f} deg"

    def _simulate_front_distance(self, next_axial: bool = False) -> float:
        axial_position = self.axial_position_mm - (self.axial_step_spin.value() if next_axial else 0.0)
        depth = abs(axial_position)
        remaining_scan_depth = max(0.0, self.scan_depth_spin.value() - depth)
        distance = self.collision_threshold_spin.value() + remaining_scan_depth
        self.front_distance_mm = distance
        self.collision_risk_active = distance <= self.collision_threshold_spin.value()
        self._update_front_distance_label()
        self._update_polar_frame()
        return distance

    def _front_clearance_ok(self, *, check_for_advance: bool) -> bool:
        if check_for_advance and abs(self.axial_position_mm - self.axial_step_spin.value()) > self.scan_depth_spin.value():
            self.auto_scan_active = False
            self._set_last_event("Target scan depth reached")
            self.scan_session_label.setText("Target scan depth reached.")
            self._finalize_point_cloud_render(include_preview=False)
            self._update_point_cloud_status_label()
            return False
        distance = self._simulate_front_distance(next_axial=check_for_advance)
        if self.enable_collision_check.isChecked() and distance <= self.collision_threshold_spin.value():
            self.auto_scan_active = False
            self._set_last_event(f"Collision risk: {distance:.1f} mm")
            self.scan_session_label.setText("Blocked by front sensor.")
            self._finalize_point_cloud_render(include_preview=False)
            self._update_point_cloud_status_label()
            return False
        return True

    def _advance_3d_position_if_needed(self) -> None:
        if self.rings_at_position < self.rings_per_level_spin.value():
            self._update_point_cloud_status_label()
            return
        if not self._front_clearance_ok(check_for_advance=True):
            return
        self.axial_position_mm -= self.axial_step_spin.value()
        self.rings_at_position = 0
        self._update_ring_and_axial_labels()
        self._update_point_cloud_status_label()

    def _scan_point_to_3d(self, point: ScanPoint) -> tuple[float, float, float]:
        angle_rad = math.radians(point.angle / 100.0)
        radius = float(point.distance_mm)
        x_value = radius * math.cos(angle_rad)
        y_value = radius * math.sin(angle_rad)
        return (x_value, y_value, float(self.axial_position_mm))

    def _active_point_cloud_points(self) -> list[tuple[float, float, float]]:
        position_preview = self._current_position_points_3d() if self.current_scan_points else []
        if position_preview:
            return [*self.point_cloud_points, *position_preview]
        return self.point_cloud_points

    def _update_point_cloud_window(self) -> None:
        if self.point_cloud_window is not None and self.enable_3d_check.isChecked():
            points = self._active_point_cloud_points()
            if self.mesh_render_only and self.surface_mesh is not None:
                self.point_cloud_window.update_points(
                    points,
                    cylinder=None,
                    mesh=self.surface_mesh,
                    show_points=False,
                )
            else:
                self.point_cloud_window.update_points(
                    points,
                    cylinder=None,
                    mesh=None,
                    show_points=True,
                )
        self._update_point_cloud_status_label()

    def _is_high_rate_command(self, command: CommandId) -> bool:
        return command in {CommandId.SCAN_POINT, CommandId.DEBUG_SWEEP_POINT, CommandId.WAVE_CHUNK}

    def _decode_wave_samples(self, data_type: WaveDataType, raw_bytes: bytes) -> list[int]:
        if data_type == WaveDataType.U8_RAW:
            return list(raw_bytes)
        if len(raw_bytes) < 2:
            return []
        count = len(raw_bytes) // 2
        return list(struct.unpack(f"<{count}H", raw_bytes[: count * 2]))

    def _wave_samples_to_display(self, data_type: WaveDataType, samples: list[int]) -> list[float]:
        if data_type in (WaveDataType.U16_RAW, WaveDataType.U16_ENVELOPE):
            return [(float(value) * ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE) for value in samples]
        return [float(value) for value in samples]

    def _wave_axis_label(self, data_type: WaveDataType) -> str:
        if data_type in (WaveDataType.U16_RAW, WaveDataType.U16_ENVELOPE):
            return "Voltage (V)"
        return "Amplitude"

    def _sample_index_to_time_us(self, index: int) -> float:
        return (float(index) / self._effective_wave_sample_rate_hz()) * 1_000_000.0

    def _time_us_to_distance_mm(self, time_us: float) -> float:
        return time_us * float(self.debug_sound_speed_spin.value()) / 2000.0

    def _refresh_wave_plot_axes(self) -> None:
        self.wave_plot.getPlotItem().getAxis("top").update()

    def _decode_frame_bytes(self, packet: bytes) -> Frame | None:
        decoder = FrameDecoder()
        frames = decoder.feed(packet)
        return frames[0] if frames else None

    def _elapsed_ms(self, started_at: float | None) -> int:
        if started_at is None:
            return 0
        return int((time.time() - started_at) * 1000.0)

    def _append_log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log_output.appendPlainText(f"{stamp} {message}")

    def _format_angle(self, angle: int) -> str:
        return self._format_angle_value(angle)

    def _next_sequence(self) -> int:
        self.sequence = (self.sequence + 1) & 0xFFFF
        return self.sequence

    def closeEvent(self, event: QtGui.QCloseEvent | None) -> None:  # type: ignore[name-defined]
        self._disconnect_transport()
        if self.recorder.active:
            self.recorder.stop()
        super().closeEvent(event)


def main() -> int:
    pg.setConfigOptions(antialias=True, background="w", foreground="k")
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
