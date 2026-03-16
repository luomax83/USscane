from __future__ import annotations

from dataclasses import dataclass
from struct import pack, unpack_from
from typing import Iterable

from .models import CommandId, DebugConfig, DistanceResult, ScanConfig, ScanPoint, WaveChunk, WaveDataType


SOF = b"\x55\xAA"
VERSION = 0x01
BODY_HEADER_SIZE = 6
CRC_SIZE = 2
MAX_PAYLOAD = 1024


def crc16_ibm(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


@dataclass(slots=True)
class Frame:
    command: int
    sequence: int
    payload: bytes = b""
    version: int = VERSION

    def encode(self) -> bytes:
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError("payload too large")

        body = pack("<BBHH", self.version, self.command, self.sequence, len(self.payload)) + self.payload
        crc = crc16_ibm(body)
        return SOF + body + pack("<H", crc)


class FrameDecoder:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> list[Frame]:
        self._buffer.extend(chunk)
        frames: list[Frame] = []

        while True:
            if len(self._buffer) < 2 + BODY_HEADER_SIZE + CRC_SIZE:
                return frames

            sof_index = self._buffer.find(SOF)
            if sof_index < 0:
                self._buffer.clear()
                return frames

            if sof_index > 0:
                del self._buffer[:sof_index]

            if len(self._buffer) < 2 + BODY_HEADER_SIZE + CRC_SIZE:
                return frames

            version, command, sequence, payload_length = unpack_from("<BBHH", self._buffer, 2)
            frame_size = 2 + BODY_HEADER_SIZE + payload_length + CRC_SIZE
            if payload_length > MAX_PAYLOAD:
                del self._buffer[:2]
                continue

            if len(self._buffer) < frame_size:
                return frames

            body = bytes(self._buffer[2 : 2 + BODY_HEADER_SIZE + payload_length])
            expected_crc = unpack_from("<H", self._buffer, 2 + BODY_HEADER_SIZE + payload_length)[0]
            actual_crc = crc16_ibm(body)
            if expected_crc != actual_crc:
                del self._buffer[:2]
                continue

            payload_start = 2 + BODY_HEADER_SIZE
            payload_end = payload_start + payload_length
            payload = bytes(self._buffer[payload_start:payload_end])
            frames.append(Frame(command=command, sequence=sequence, payload=payload, version=version))
            del self._buffer[:frame_size]


def encode_get_device_info(sequence: int) -> bytes:
    return Frame(CommandId.GET_DEVICE_INFO, sequence).encode()


def encode_zeroing(sequence: int) -> bytes:
    return Frame(CommandId.ZEROING, sequence).encode()


def encode_stop(sequence: int) -> bytes:
    return Frame(CommandId.STOP, sequence).encode()


def encode_start_scan(sequence: int) -> bytes:
    return Frame(CommandId.START_SCAN, sequence, pack("<B", 0x01)).encode()


def encode_start_single_measure(sequence: int) -> bytes:
    return Frame(CommandId.START_SINGLE_MEASURE, sequence, pack("<B", 0x01)).encode()


def encode_start_debug_sweep(sequence: int) -> bytes:
    return Frame(CommandId.START_DEBUG_SWEEP, sequence).encode()


def encode_move_to_angle(sequence: int, angle: int) -> bytes:
    return Frame(CommandId.MOVE_TO_ANGLE, sequence, pack("<H", angle)).encode()


def encode_scan_config(sequence: int, config: ScanConfig) -> bytes:
    payload = pack(
        "<HHBB",
        config.scan_step,
        config.sound_speed,
        int(config.medium),
        config.flags,
    )
    return Frame(CommandId.CONFIG_SCAN, sequence, payload).encode()


def encode_debug_config(sequence: int, config: DebugConfig) -> bytes:
    payload = pack(
        "<HHHBBHBB",
        config.start_angle,
        config.end_angle,
        config.step_angle,
        int(config.capture_type),
        int(config.probe_type),
        config.sample_count,
        config.pulse_count,
        config.flags,
    )
    return Frame(CommandId.CONFIG_DEBUG, sequence, payload).encode()


def chunk_samples(
    *,
    sequence: int,
    angle: int,
    data_type: int,
    samples: bytes,
    chunk_size: int = 240,
) -> Iterable[bytes]:
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")

    total_frames = (len(samples) + chunk_size - 1) // chunk_size
    for frame_id in range(total_frames):
        start = frame_id * chunk_size
        end = min(start + chunk_size, len(samples))
        chunk = samples[start:end]
        payload = pack("<HHHHHB", frame_id, total_frames, start, len(chunk), angle, data_type) + chunk
        yield Frame(CommandId.WAVE_CHUNK, sequence, payload).encode()


def decode_ack_payload(payload: bytes) -> dict[str, int | bytes]:
    if len(payload) < 4:
        raise ValueError("ACK payload too short")
    result, status, ref_cmd, detail_len = unpack_from("<BBBB", payload, 0)
    detail = payload[4 : 4 + detail_len]
    return {
        "result": result,
        "status": status,
        "ref_cmd": ref_cmd,
        "detail_len": detail_len,
        "detail": detail,
    }


def decode_nack_payload(payload: bytes) -> dict[str, int]:
    if len(payload) < 4:
        raise ValueError("NACK payload too short")
    result, status, ref_cmd, error_code = unpack_from("<BBBB", payload, 0)
    return {
        "result": result,
        "status": status,
        "ref_cmd": ref_cmd,
        "error_code": error_code,
    }


def parse_device_info_detail(detail: bytes) -> dict[str, int | str]:
    if len(detail) < 12:
        raise ValueError("device info detail too short")
    fw_major, fw_minor, hw_major, hw_minor, max_baud, capabilities = unpack_from("<BBBBII", detail, 0)
    serial_number = ""
    if len(detail) > 12:
        serial_number = detail[12:].split(b"\x00", 1)[0].decode("ascii", errors="ignore")
    return {
        "fw_major": fw_major,
        "fw_minor": fw_minor,
        "hw_major": hw_major,
        "hw_minor": hw_minor,
        "max_baud": max_baud,
        "capabilities": capabilities,
        "serial_number": serial_number,
    }


def parse_scan_config_ack_detail(detail: bytes) -> dict[str, int]:
    if len(detail) < 5:
        raise ValueError("scan config ack detail too short")
    zero_done, current_angle, point_count = unpack_from("<BHH", detail, 0)
    return {
        "zero_done": zero_done,
        "current_angle": current_angle,
        "point_count": point_count,
    }


def parse_zeroing_ack_detail(detail: bytes) -> dict[str, int]:
    if len(detail) < 3:
        raise ValueError("zeroing ack detail too short")
    zero_done, current_angle = unpack_from("<BH", detail, 0)
    return {
        "zero_done": zero_done,
        "current_angle": current_angle,
    }


def parse_debug_config_ack_detail(detail: bytes) -> dict[str, int]:
    if len(detail) < 2:
        raise ValueError("debug config ack detail too short")
    accepted, normalized = unpack_from("<BB", detail, 0)
    return {
        "accepted": accepted,
        "normalized": normalized,
    }


def parse_move_to_angle_ack_detail(detail: bytes) -> dict[str, int]:
    if len(detail) < 6:
        raise ValueError("move-to-angle ack detail too short")
    current_angle, target_angle, move_dir, arrived = unpack_from("<HHBB", detail, 0)
    return {
        "current_angle": current_angle,
        "target_angle": target_angle,
        "move_dir": move_dir,
        "arrived": arrived,
    }


def parse_scan_point(payload: bytes) -> ScanPoint:
    if len(payload) >= 12:
        point_index, angle, distance_mm, quality, valid, _reserved = unpack_from("<HHIHBB", payload, 0)
        return ScanPoint(
            point_index=point_index,
            angle=angle,
            distance_mm=distance_mm,
            quality=quality,
            valid=bool(valid),
        )
    if len(payload) >= 6:
        angle, distance_mm = unpack_from("<HI", payload, 0)
        return ScanPoint(
            angle=angle,
            distance_mm=distance_mm,
        )
    raise ValueError("scan point payload too short")


def parse_scan_finish(payload: bytes) -> dict[str, int]:
    if len(payload) < 5:
        raise ValueError("scan finish payload too short")
    total_points, end_reason, dropped = unpack_from("<HBH", payload, 0)
    return {
        "total_points": total_points,
        "end_reason": end_reason,
        "dropped": dropped,
    }


def parse_wave_chunk(payload: bytes) -> WaveChunk:
    if len(payload) < 11:
        raise ValueError("wave chunk payload too short")
    frame_id, total_frames, sample_offset, sample_count, angle, data_type = unpack_from("<HHHHHB", payload, 0)
    samples = payload[11 : 11 + sample_count]
    return WaveChunk(
        frame_id=frame_id,
        total_frames=total_frames,
        sample_offset=sample_offset,
        sample_count=sample_count,
        angle=angle,
        data_type=WaveDataType(data_type),
        samples=samples,
    )


def parse_wave_finish(payload: bytes) -> dict[str, int]:
    if len(payload) < 5:
        raise ValueError("wave finish payload too short")
    angle, total_samples, result = unpack_from("<HHB", payload, 0)
    return {
        "angle": angle,
        "total_samples": total_samples,
        "result": result,
    }


def parse_distance_result(payload: bytes) -> DistanceResult:
    if len(payload) < 9:
        raise ValueError("distance result payload too short")
    angle, distance_mm, quality, valid = unpack_from("<HIHB", payload, 0)
    return DistanceResult(
        angle=angle,
        distance_mm=distance_mm,
        quality=quality,
        valid=bool(valid),
    )


def parse_debug_sweep_point(payload: bytes) -> ScanPoint:
    if len(payload) < 12:
        raise ValueError("debug sweep point payload too short")
    point_index, angle, distance_mm, quality, valid, has_wave = unpack_from("<HHIHBB", payload, 0)
    return ScanPoint(
        point_index=point_index,
        angle=angle,
        distance_mm=distance_mm,
        quality=quality,
        valid=bool(valid),
        has_wave=bool(has_wave),
    )


def parse_debug_sweep_finish(payload: bytes) -> dict[str, int]:
    if len(payload) < 3:
        raise ValueError("debug sweep finish payload too short")
    total_points, end_reason = unpack_from("<HB", payload, 0)
    return {
        "total_points": total_points,
        "end_reason": end_reason,
    }


def parse_status_event(payload: bytes) -> dict[str, int]:
    if len(payload) < 6:
        raise ValueError("status event payload too short")
    event, status, current_angle, detail = unpack_from("<BBHH", payload, 0)
    return {
        "event": event,
        "status": status,
        "current_angle": current_angle,
        "detail": detail,
    }
