from __future__ import annotations

import math
from dataclasses import dataclass, field
from struct import pack, unpack_from

from .models import CaptureType, CommandId, DeviceStatus, ResultCode, WaveDataType
from .protocol import Frame, FrameDecoder, chunk_samples


CAPABILITY_SCAN = 1 << 0
CAPABILITY_SINGLE_DEBUG = 1 << 1
CAPABILITY_SWEEP_DEBUG = 1 << 2
CAPABILITY_RAW_WAVE = 1 << 3
CAPABILITY_ENVELOPE = 1 << 4
CAPABILITY_RECORD = 1 << 5

EVENT_ZEROING_START = 0x01
EVENT_ZEROING_DONE = 0x02
EVENT_MOVE_START = 0x03
EVENT_MOVE_ARRIVED = 0x04
EVENT_MEASURE_START = 0x05
EVENT_MEASURE_DONE = 0x06
EVENT_UPLOAD_START = 0x07
EVENT_UPLOAD_DONE = 0x08


@dataclass(slots=True)
class SimulatedDevice:
    serial_number: str = "123456789"
    current_angle: int = 0
    status: DeviceStatus = DeviceStatus.IDLE
    scan_step: int = 100
    scan_cycle: int = 0
    sound_speed: int = 1500
    medium: int = 0
    debug_start_angle: int = 0
    debug_end_angle: int = 0
    debug_step_angle: int = 0
    debug_sample_count: int = 256
    debug_capture_type: CaptureType = CaptureType.RAW_WAVE
    debug_flags: int = 0
    history: list[str] = field(default_factory=list)
    _decoder: FrameDecoder = field(default_factory=FrameDecoder)

    def handle_bytes(self, chunk: bytes) -> list[bytes]:
        responses: list[bytes] = []
        for frame in self._decoder.feed(chunk):
            responses.extend(self.handle_frame(frame))
        return responses

    def handle_frame(self, frame: Frame) -> list[bytes]:
        command = CommandId(frame.command)
        self.history.append(f"rx:{command.name}")

        if command == CommandId.GET_DEVICE_INFO:
            detail = (
                pack(
                    "<BBBBII",
                    1,
                    1,
                    1,
                    0,
                    460800,
                    CAPABILITY_SCAN
                    | CAPABILITY_SINGLE_DEBUG
                    | CAPABILITY_SWEEP_DEBUG
                    | CAPABILITY_RAW_WAVE
                    | CAPABILITY_ENVELOPE
                    | CAPABILITY_RECORD,
                )
                + self.serial_number.encode("ascii")
                + b"\x00"
            )
            return [self._ack(frame.sequence, command, detail)]

        if command == CommandId.ZEROING:
            self.status = DeviceStatus.ZEROING
            packets = [self._status_event(EVENT_ZEROING_START)]
            self.current_angle = 0
            self.status = DeviceStatus.READY
            packets.append(self._status_event(EVENT_ZEROING_DONE))
            packets.append(self._ack(frame.sequence, command, pack("<BH", 1, self.current_angle)))
            return packets

        if command == CommandId.CONFIG_SCAN:
            self.scan_step, self.sound_speed, self.medium, _flags = _parse_scan_payload(frame.payload)
            point_count = _calc_point_count(self.scan_step)
            self.status = DeviceStatus.ZEROING
            packets = [self._status_event(EVENT_ZEROING_START)]
            self.current_angle = 0
            self.status = DeviceStatus.READY
            packets.append(self._status_event(EVENT_ZEROING_DONE))
            packets.append(self._ack(frame.sequence, command, pack("<BHH", 1, self.current_angle, point_count)))
            return packets

        if command == CommandId.START_SCAN:
            self.status = DeviceStatus.SCANNING
            self.scan_cycle += 1
            packets = [
                self._ack(frame.sequence, command, pack("<B", 1)),
                self._status_event(EVENT_MOVE_START),
            ]
            point_count = _calc_point_count(self.scan_step)
            phase_shift = (self.scan_cycle - 1) * 0.08
            for point_index in range(point_count):
                self.current_angle = min(point_index * self.scan_step, 35999)
                distance = _distance_for_angle(
                    self.current_angle,
                    base=1200,
                    span=900,
                    phase_shift=phase_shift,
                )
                quality = 80 + (point_index % 20)
                packets.append(
                    Frame(
                        CommandId.SCAN_POINT,
                        frame.sequence,
                        pack("<HHIHBB", point_index, self.current_angle, distance, quality, 1, 0),
                    ).encode()
                )
            packets.append(Frame(CommandId.SCAN_FINISH, frame.sequence, pack("<HBH", point_count, 0, 0)).encode())
            self.status = DeviceStatus.READY
            packets.append(self._status_event(EVENT_MEASURE_DONE))
            return packets

        if command == CommandId.CONFIG_DEBUG:
            (
                self.debug_start_angle,
                self.debug_end_angle,
                self.debug_step_angle,
                capture_type,
                self.debug_sample_count,
                self.debug_flags,
            ) = _parse_debug_payload(frame.payload)
            self.debug_capture_type = CaptureType(capture_type)
            return [self._ack(frame.sequence, command, pack("<BB", 1, 1))]

        if command == CommandId.MOVE_TO_ANGLE:
            target_angle = int.from_bytes(frame.payload[:2], "little") if len(frame.payload) >= 2 else 0
            move_dir = _compute_move_dir(self.current_angle, target_angle)
            self.status = DeviceStatus.MOVING
            packets = [self._status_event(EVENT_MOVE_START, target_angle)]
            self.current_angle = target_angle
            self.status = DeviceStatus.READY
            packets.append(self._status_event(EVENT_MOVE_ARRIVED, target_angle))
            packets.append(
                self._ack(
                    frame.sequence,
                    command,
                    pack("<HHBB", self.current_angle, target_angle, move_dir, 1),
                )
            )
            return packets

        if command == CommandId.START_SINGLE_MEASURE:
            self.status = DeviceStatus.MEASURING
            packets = [
                self._ack(frame.sequence, command, pack("<B", 1)),
                self._status_event(EVENT_MEASURE_START),
            ]
            if self.debug_flags & 0x08:
                self.status = DeviceStatus.UPLOADING
                packets.append(self._status_event(EVENT_UPLOAD_START))
                wave_type, samples = _build_wave_payload(
                    self.current_angle,
                    self.debug_capture_type,
                    self.debug_sample_count,
                )
                packets.extend(
                    chunk_samples(
                        sequence=frame.sequence,
                        angle=self.current_angle,
                        data_type=int(wave_type),
                        samples=samples,
                        chunk_size=240,
                    )
                )
                packets.append(
                    Frame(
                        CommandId.WAVE_FINISH,
                        frame.sequence,
                        pack("<HHB", self.current_angle, self.debug_sample_count, 0),
                    ).encode()
                )
                packets.append(self._status_event(EVENT_UPLOAD_DONE))

            if self.debug_flags & 0x04:
                packets.append(
                    Frame(
                        CommandId.DISTANCE_RESULT,
                        frame.sequence,
                        pack(
                            "<HIHB",
                            self.current_angle,
                            _distance_for_angle(self.current_angle, base=950, span=500),
                            96,
                            1,
                        ),
                    ).encode()
                )

            self.status = DeviceStatus.READY
            packets.append(self._status_event(EVENT_MEASURE_DONE))
            return packets

        if command == CommandId.START_DEBUG_SWEEP:
            self.status = DeviceStatus.DEBUG_SCANNING
            packets = [
                self._ack(frame.sequence, command, pack("<B", 1)),
                self._status_event(EVENT_MOVE_START, self.debug_start_angle),
            ]
            angles = _build_debug_angles(
                self.debug_start_angle,
                self.debug_end_angle,
                self.debug_step_angle,
            )
            for point_index, angle in enumerate(angles):
                self.current_angle = angle
                distance = _distance_for_angle(angle, base=780, span=720)
                quality = 70 + (point_index % 25)
                has_wave = 1 if (self.debug_flags & 0x08) else 0
                packets.append(
                    Frame(
                        CommandId.DEBUG_SWEEP_POINT,
                        frame.sequence,
                        pack("<HHIHBB", point_index, angle, distance, quality, 1, has_wave),
                    ).encode()
                )
            packets.append(
                Frame(
                    CommandId.DEBUG_SWEEP_FINISH,
                    frame.sequence,
                    pack("<HB", len(angles), 0),
                ).encode()
            )
            self.status = DeviceStatus.READY
            packets.append(self._status_event(EVENT_MOVE_ARRIVED, self.current_angle))
            return packets

        if command == CommandId.STOP:
            self.status = DeviceStatus.READY
            return [self._ack(frame.sequence, command, pack("<B", 1))]

        return [self._nack(frame.sequence, command, ResultCode.UNSUPPORTED)]

    def _ack(self, sequence: int, ref_cmd: CommandId, detail: bytes = b"") -> bytes:
        payload = pack("<BBBB", ResultCode.OK, int(self.status), int(ref_cmd), len(detail)) + detail
        return Frame(CommandId.ACK, sequence, payload).encode()

    def _nack(self, sequence: int, ref_cmd: CommandId, result: ResultCode) -> bytes:
        payload = pack("<BBBB", int(result), int(self.status), int(ref_cmd), int(result))
        return Frame(CommandId.NACK, sequence, payload).encode()

    def _status_event(self, event: int, detail: int = 0) -> bytes:
        payload = pack("<BBHH", event, int(self.status), self.current_angle, detail & 0xFFFF)
        return Frame(CommandId.STATUS_EVENT, 0, payload).encode()


def _parse_scan_payload(payload: bytes) -> tuple[int, int, int, int]:
    if len(payload) < 6:
        return 100, 1500, 0, 0
    return unpack_from("<HHBB", payload, 0)


def _parse_debug_payload(payload: bytes) -> tuple[int, int, int, int, int, int]:
    if len(payload) < 9:
        return 0, 0, 0, int(CaptureType.RAW_WAVE), 256, 0x0D
    return unpack_from("<HHHBHB", payload, 0)


def _calc_point_count(scan_step: int) -> int:
    step = max(1, scan_step)
    return max(1, (36000 + step - 1) // step)


def _build_debug_angles(start_angle: int, end_angle: int, step_angle: int) -> list[int]:
    if start_angle == end_angle or step_angle <= 0:
        return [start_angle]
    step = max(1, step_angle)
    if end_angle >= start_angle:
        return list(range(start_angle, end_angle + 1, step))
    return list(range(start_angle, end_angle - 1, -step))


def _distance_for_angle(angle: int, *, base: int, span: int, phase_shift: float = 0.0) -> int:
    radians = math.radians(angle / 100.0) + phase_shift
    ripple = (
        0.58 * math.cos(radians - 0.30)
        + 0.27 * math.cos(2.0 * radians + 0.85)
        + 0.15 * math.sin(3.0 * radians - 0.55)
    )
    return max(100, int(base + ripple * span))


def _compute_move_dir(current_angle: int, target_angle: int) -> int:
    if current_angle == target_angle:
        return 0
    clockwise = (target_angle - current_angle) % 36000
    counter_clockwise = (current_angle - target_angle) % 36000
    return 1 if clockwise <= counter_clockwise else 2


def _build_wave_payload(angle: int, capture_type: CaptureType, sample_count: int) -> tuple[WaveDataType, bytes]:
    sample_count = max(32, sample_count)
    if capture_type == CaptureType.ENVELOPE:
        samples = bytearray()
        for index in range(sample_count):
            radians = (index / sample_count) * math.pi * 4.0
            value = int(1200 + math.sin(radians + angle / 3000.0) * 500)
            samples.extend(pack("<H", max(0, value)))
        return WaveDataType.U16_ENVELOPE, bytes(samples)

    samples = bytearray()
    for index in range(sample_count):
        radians = (index / sample_count) * math.pi * 6.0
        value = int(128 + math.sin(radians + angle / 2000.0) * 90)
        samples.append(max(0, min(255, value)))
    return WaveDataType.U8_RAW, bytes(samples)
