from __future__ import annotations

import sys
from dataclasses import dataclass
from typing import Iterable

import serial
from serial.tools import list_ports
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


STATUS_TEXT = {
    0x0000: "????????",
    0x0001: "????",
    0x0002: "???",
    0x0003: "????",
    0x0004: "????",
}

EXCEPTION_TEXT = {
    0x0001: "?????",
    0x0002: "????",
    0x0003: "????",
}

REG_NAME = {
    0x0000: "??",
    0x0001: "??????",
    0x0002: "??????",
    0x001F: "????",
    0x0020: "??????",
    0x0021: "??????",
    0x0022: "??????",
    0x00D5: "????",
    0x00E3: "????",
}


@dataclass
class RequestContext:
    label: str
    func: int
    register: int
    count: int = 0


def modbus_crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def normalize_hex_tokens(text: str) -> list[int]:
    compact = text.replace(",", " ").replace("0x", " ").replace("0X", " ")
    tokens = [token for token in compact.split() if token]
    if not tokens:
        raise ValueError("?????????,??:01 03 00 01 00 02")
    values: list[int] = []
    for token in tokens:
        value = int(token, 16)
        if not 0 <= value <= 0xFF:
            raise ValueError(f"????: {token}")
        values.append(value)
    return values


def bytes_to_hex(data: Iterable[int]) -> str:
    return " ".join(f"{value:02X}" for value in data)


def u16_to_bytes(value: int) -> bytes:
    return bytes(((value >> 8) & 0xFF, value & 0xFF))


def read_holding_registers(slave: int, register: int, count: int) -> bytes:
    payload = bytes((slave, 0x03)) + u16_to_bytes(register) + u16_to_bytes(count)
    return payload + modbus_crc16(payload)


def write_single_register(slave: int, register: int, value: int) -> bytes:
    payload = bytes((slave, 0x06)) + u16_to_bytes(register) + u16_to_bytes(value)
    return payload + modbus_crc16(payload)


def write_multiple_registers(slave: int, register: int, values: list[int]) -> bytes:
    payload = (
        bytes((slave, 0x10))
        + u16_to_bytes(register)
        + u16_to_bytes(len(values))
        + bytes((len(values) * 2,))
    )
    for value in values:
        payload += u16_to_bytes(value)
    return payload + modbus_crc16(payload)


def decode_u16(data: bytes) -> int:
    return (data[0] << 8) | data[1]


def decode_s32_from_u16_pair(high_word: int, low_word: int) -> int:
    value = ((high_word & 0xFFFF) << 16) | (low_word & 0xFFFF)
    if value & 0x80000000:
        value -= 0x100000000
    return value


def verify_crc(frame: bytes) -> bool:
    return len(frame) >= 4 and modbus_crc16(frame[:-2]) == frame[-2:]


class MotorPassthroughWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("US360 ??????")
        self.resize(1100, 780)

        self._serial: serial.Serial | None = None
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(40)
        self._poll_timer.timeout.connect(self._poll_serial)
        self._last_request: RequestContext | None = None

        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("????")
        self.connect_button = QPushButton("??")
        self.port_status_label = QLabel("???")

        self.slave_spin = QSpinBox()
        self.slave_spin.setRange(1, 254)
        self.slave_spin.setValue(1)

        self.status_value_label = QLabel("-")
        self.steps_value_label = QLabel("-")
        self.single_turn_value_label = QLabel("-")

        self.target_steps_spin = QSpinBox()
        self.target_steps_spin.setRange(0, 65535)
        self.target_steps_spin.setValue(0)

        self.target_speed_spin = QSpinBox()
        self.target_speed_spin.setRange(1, 20000)
        self.target_speed_spin.setValue(5000)

        self.single_turn_mode_combo = QComboBox()
        self.single_turn_mode_combo.addItem("0 - ????", 0)
        self.single_turn_mode_combo.addItem("1 - ?????", 1)

        self.home_speed_spin = QSpinBox()
        self.home_speed_spin.setRange(1, 20000)
        self.home_speed_spin.setValue(100)

        self.hex_input = QLineEdit("01 03 00 00 00 01")
        self.append_crc_checkbox = QCheckBox("????????? CRC")
        self.append_crc_checkbox.setChecked(True)
        self.send_button = QPushButton("?? HEX")

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)

        self._build_ui()
        self._bind_signals()
        self._refresh_ports()

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        top_row = QHBoxLayout()
        top_row.addWidget(QLabel("??"))
        top_row.addWidget(self.port_combo, 1)
        top_row.addWidget(self.refresh_button)
        top_row.addWidget(self.connect_button)
        top_row.addWidget(QLabel("????"))
        top_row.addWidget(self.slave_spin)
        top_row.addWidget(self.port_status_label, 1)
        root.addLayout(top_row)

        root.addWidget(self._build_protocol_group())
        root.addWidget(self._build_single_turn_group())
        root.addWidget(self._build_manual_group())
        root.addWidget(self.log_view, 1)

    def _build_protocol_group(self) -> QWidget:
        group = QGroupBox("??????")
        layout = QGridLayout(group)

        read_status_button = QPushButton("??? 00H")
        read_steps_button = QPushButton("????? 01H~02H")
        read_status_steps_button = QPushButton("???+?? 00H~02H")
        read_single_turn_button = QPushButton("??????? 20H~22H")
        read_steps_per_turn_button = QPushButton("??? E3H")
        stop_button = QPushButton("?? 04H")
        home_button = QPushButton("?? 1FH")
        set_zero_button = QPushButton("?? D5H")

        read_status_button.clicked.connect(self._send_read_status)
        read_steps_button.clicked.connect(self._send_read_actual_steps)
        read_status_steps_button.clicked.connect(self._send_read_status_and_steps)
        read_single_turn_button.clicked.connect(self._send_read_single_turn_zone)
        read_steps_per_turn_button.clicked.connect(self._send_read_subdivision)
        stop_button.clicked.connect(self._send_stop)
        home_button.clicked.connect(self._send_home)
        set_zero_button.clicked.connect(self._send_set_zero)

        buttons = [
            read_status_button,
            read_steps_button,
            read_status_steps_button,
            read_single_turn_button,
            read_steps_per_turn_button,
            stop_button,
            home_button,
            set_zero_button,
        ]
        for idx, button in enumerate(buttons):
            layout.addWidget(button, idx // 4, idx % 4)

        layout.addWidget(QLabel("????"), 2, 0)
        layout.addWidget(self.status_value_label, 2, 1)
        layout.addWidget(QLabel("????"), 2, 2)
        layout.addWidget(self.steps_value_label, 2, 3)
        layout.addWidget(QLabel("20H~22H"), 3, 0)
        layout.addWidget(self.single_turn_value_label, 3, 1, 1, 3)

        hint = QLabel(
            "??? motor.docx ??:00H ???01H~02H ?????20H~22H ???????1FH ???D5H ???04H ???"
        )
        layout.addWidget(hint, 4, 0, 1, 4)
        return group

    def _build_single_turn_group(self) -> QWidget:
        group = QGroupBox("???????? 20H~22H")
        layout = QGridLayout(group)

        write_button = QPushButton("??????")
        write_button.clicked.connect(self._send_write_single_turn_zone)

        layout.addWidget(QLabel("???? 20H"), 0, 0)
        layout.addWidget(self.target_steps_spin, 0, 1)
        layout.addWidget(QLabel("???? 21H"), 0, 2)
        layout.addWidget(self.target_speed_spin, 0, 3)
        layout.addWidget(QLabel("?? 22H"), 1, 0)
        layout.addWidget(self.single_turn_mode_combo, 1, 1)
        layout.addWidget(QLabel("???? 1FH"), 1, 2)
        layout.addWidget(self.home_speed_spin, 1, 3)
        layout.addWidget(write_button, 2, 0, 1, 2)

        summary = QLabel(
            "?????:01 10 00 20 00 03 06 [????] [????] [??] CRC??? 0 ???,1 ?????"
        )
        layout.addWidget(summary, 3, 0, 1, 4)
        return group

    def _build_manual_group(self) -> QWidget:
        group = QGroupBox("?? HEX ??")
        layout = QVBoxLayout(group)

        row = QHBoxLayout()
        row.addWidget(QLabel("HEX"))
        row.addWidget(self.hex_input, 1)
        row.addWidget(self.append_crc_checkbox)
        row.addWidget(self.send_button)
        layout.addLayout(row)

        hint = QLabel("????????? CRC,???????????:01 03 00 00 00 01")
        layout.addWidget(hint)
        return group

    def _bind_signals(self) -> None:
        self.refresh_button.clicked.connect(self._refresh_ports)
        self.connect_button.clicked.connect(self._toggle_connection)
        self.send_button.clicked.connect(self._send_manual_hex)

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = [port.device for port in list_ports.comports()]
        self.port_combo.addItems(ports)
        if current:
            index = self.port_combo.findText(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def _toggle_connection(self) -> None:
        if self._serial and self._serial.is_open:
            self._disconnect_serial()
        else:
            self._connect_serial()

    def _connect_serial(self) -> None:
        port_name = self.port_combo.currentText().strip()
        if not port_name:
            QMessageBox.warning(self, "??", "???????")
            return
        try:
            self._serial = serial.Serial(port_name, 115200, timeout=0)
        except Exception as exc:
            QMessageBox.critical(self, "????", str(exc))
            return

        self.port_status_label.setText(f"??? {port_name} @115200")
        self.connect_button.setText("??")
        self._poll_timer.start()
        self._append_log(f"[OPEN] {port_name}")

    def _disconnect_serial(self) -> None:
        self._poll_timer.stop()
        if self._serial:
            try:
                name = self._serial.port
                self._serial.close()
                self._append_log(f"[CLOSE] {name}")
            except Exception:
                pass
        self._serial = None
        self.port_status_label.setText("???")
        self.connect_button.setText("??")

    def _require_serial(self) -> serial.Serial | None:
        if not (self._serial and self._serial.is_open):
            QMessageBox.warning(self, "??", "???????")
            return None
        return self._serial

    def _send_frame(self, frame: bytes, context: RequestContext) -> None:
        port = self._require_serial()
        if port is None:
            return
        try:
            port.write(frame)
            port.flush()
        except Exception as exc:
            QMessageBox.critical(self, "????", str(exc))
            self._disconnect_serial()
            return

        self._last_request = context
        self.hex_input.setText(bytes_to_hex(frame))
        self._append_log(f"[TX] {context.label}: {bytes_to_hex(frame)}")

    def _send_read_status(self) -> None:
        slave = self.slave_spin.value()
        self._send_frame(read_holding_registers(slave, 0x0000, 1), RequestContext("???", 0x03, 0x0000, 1))

    def _send_read_actual_steps(self) -> None:
        slave = self.slave_spin.value()
        self._send_frame(read_holding_registers(slave, 0x0001, 2), RequestContext("?????", 0x03, 0x0001, 2))

    def _send_read_status_and_steps(self) -> None:
        slave = self.slave_spin.value()
        self._send_frame(read_holding_registers(slave, 0x0000, 3), RequestContext("???+??", 0x03, 0x0000, 3))

    def _send_read_single_turn_zone(self) -> None:
        slave = self.slave_spin.value()
        self._send_frame(read_holding_registers(slave, 0x0020, 3), RequestContext("???????", 0x03, 0x0020, 3))

    def _send_read_subdivision(self) -> None:
        slave = self.slave_spin.value()
        self._send_frame(read_holding_registers(slave, 0x00E3, 1), RequestContext("???", 0x03, 0x00E3, 1))

    def _send_stop(self) -> None:
        slave = self.slave_spin.value()
        frame = write_multiple_registers(slave, 0x0004, [0x0000])
        self._send_frame(frame, RequestContext("??", 0x10, 0x0004, 1))

    def _send_home(self) -> None:
        slave = self.slave_spin.value()
        speed = self.home_speed_spin.value()
        self._send_frame(write_single_register(slave, 0x001F, speed), RequestContext("??", 0x06, 0x001F, 1))

    def _send_set_zero(self) -> None:
        slave = self.slave_spin.value()
        self._send_frame(write_single_register(slave, 0x00D5, 0x0000), RequestContext("??", 0x06, 0x00D5, 1))

    def _send_write_single_turn_zone(self) -> None:
        slave = self.slave_spin.value()
        target_steps = self.target_steps_spin.value()
        target_speed = self.target_speed_spin.value()
        mode = int(self.single_turn_mode_combo.currentData())
        frame = write_multiple_registers(slave, 0x0020, [target_steps, target_speed, mode])
        self._send_frame(frame, RequestContext("???????", 0x10, 0x0020, 3))

    def _send_manual_hex(self) -> None:
        port = self._require_serial()
        if port is None:
            return

        try:
            values = normalize_hex_tokens(self.hex_input.text())
        except ValueError as exc:
            QMessageBox.warning(self, "HEX ????", str(exc))
            return

        frame = bytes(values)
        if self.append_crc_checkbox.isChecked():
            frame += modbus_crc16(frame)

        try:
            port.write(frame)
            port.flush()
        except Exception as exc:
            QMessageBox.critical(self, "????", str(exc))
            self._disconnect_serial()
            return

        self._last_request = None
        self._append_log(f"[TX] ????: {bytes_to_hex(frame)}")

    def _poll_serial(self) -> None:
        if not (self._serial and self._serial.is_open):
            return

        try:
            waiting = self._serial.in_waiting
            if waiting <= 0:
                return
            data = self._serial.read(waiting)
        except Exception as exc:
            self._append_log(f"[ERR] {exc}")
            self._disconnect_serial()
            return

        if data:
            self._append_log(f"[RX] {bytes_to_hex(data)}")
            self._decode_response(data)

    def _decode_response(self, frame: bytes) -> None:
        if len(frame) < 5:
            self._append_log("[INFO] ??????,?????")
            return

        if not verify_crc(frame):
            self._append_log("[WARN] CRC ?????")
            return

        slave = frame[0]
        func = frame[1]

        if func & 0x80:
            error_code = decode_u16(b"\x00" + bytes((frame[2],)))
            error_text = EXCEPTION_TEXT.get(error_code, "????")
            self._append_log(f"[PARSE] ?? {slave:02X} ????: {error_code:04X} {error_text}")
            return

        if self._last_request is None:
            self._append_log("[PARSE] ???????,??? CRC?")
            return

        context = self._last_request
        if func == 0x03:
            self._decode_read_response(context, frame)
        elif func == 0x06:
            register = decode_u16(frame[2:4])
            value = decode_u16(frame[4:6])
            self._append_log(
                f"[PARSE] ???????: {REG_NAME.get(register, f'{register:04X}H')} = {value} (0x{value:04X})"
            )
        elif func == 0x10:
            register = decode_u16(frame[2:4])
            count = decode_u16(frame[4:6])
            self._append_log(
                f"[PARSE] ???????: ?? {REG_NAME.get(register, f'{register:04X}H')},?? {count}"
            )
        else:
            self._append_log(f"[PARSE] ??????: 0x{func:02X}")

    def _decode_read_response(self, context: RequestContext, frame: bytes) -> None:
        byte_count = frame[2]
        data = frame[3 : 3 + byte_count]
        words = [decode_u16(data[i : i + 2]) for i in range(0, len(data), 2)]

        if context.register == 0x0000 and context.count == 1 and len(words) >= 1:
            status = words[0]
            self.status_value_label.setText(f"0x{status:04X} {STATUS_TEXT.get(status, '????')}")
            self._append_log(f"[PARSE] ????: 0x{status:04X} {STATUS_TEXT.get(status, '????')}")
            return

        if context.register == 0x0001 and context.count == 2 and len(words) >= 2:
            steps = decode_s32_from_u16_pair(words[0], words[1])
            self.steps_value_label.setText(f"{steps}")
            self._append_log(f"[PARSE] ????: {steps}")
            return

        if context.register == 0x0000 and context.count == 3 and len(words) >= 3:
            status = words[0]
            steps = decode_s32_from_u16_pair(words[1], words[2])
            self.status_value_label.setText(f"0x{status:04X} {STATUS_TEXT.get(status, '????')}")
            self.steps_value_label.setText(f"{steps}")
            self._append_log(
                f"[PARSE] ??+??: 0x{status:04X} {STATUS_TEXT.get(status, '????')},???? {steps}"
            )
            return

        if context.register == 0x0020 and context.count == 3 and len(words) >= 3:
            target_steps, target_speed, mode = words[:3]
            mode_text = "????" if mode == 0 else "?????" if mode == 1 else f"??? {mode}"
            self.single_turn_value_label.setText(
                f"????={target_steps},????={target_speed},??={mode_text}"
            )
            self._append_log(
                f"[PARSE] ??????: 20H={target_steps},21H={target_speed},22H={mode}({mode_text})"
            )
            return

        if context.register == 0x00E3 and context.count == 1 and len(words) >= 1:
            self._append_log(f"[PARSE] ?? E3H = {words[0]}")
            return

        self._append_log(
            f"[PARSE] ??????: ?? 0x{context.register:04X} ?? {context.count} ?? {words}"
        )

    def _append_log(self, text: str) -> None:
        self.log_view.appendPlainText(text)


def main() -> int:
    app = QApplication(sys.argv)
    window = MotorPassthroughWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
