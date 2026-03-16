from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class ResultCode(IntEnum):
    OK = 0x00
    FAIL = 0x01
    BUSY = 0x02
    INVALID_PARAM = 0x03
    INVALID_STATE = 0x04
    CRC_ERROR = 0x05
    UNSUPPORTED = 0x06
    TIMEOUT = 0x07
    HARDWARE_ERROR = 0x08


class DeviceStatus(IntEnum):
    IDLE = 0x00
    ZEROING = 0x01
    MOVING = 0x02
    READY = 0x03
    MEASURING = 0x04
    SCANNING = 0x05
    DEBUG_SCANNING = 0x06
    UPLOADING = 0x07
    STOPPING = 0x08
    ERROR = 0x09


class CommandId(IntEnum):
    GET_DEVICE_INFO = 0x01
    ZEROING = 0x02
    STOP = 0x03
    ACK = 0x04
    NACK = 0x05
    CONFIG_SCAN = 0x10
    START_SCAN = 0x11
    SCAN_POINT = 0x12
    SCAN_FINISH = 0x13
    CONFIG_DEBUG = 0x20
    MOVE_TO_ANGLE = 0x21
    START_SINGLE_MEASURE = 0x22
    WAVE_CHUNK = 0x23
    WAVE_FINISH = 0x24
    DISTANCE_RESULT = 0x25
    START_DEBUG_SWEEP = 0x26
    DEBUG_SWEEP_POINT = 0x27
    DEBUG_SWEEP_FINISH = 0x28
    STATUS_EVENT = 0x7F


class MediumType(IntEnum):
    AIR = 0x00
    FRESH_WATER = 0x01
    SEA_WATER = 0x02
    CUSTOM = 0x03


class CaptureType(IntEnum):
    RAW_WAVE = 0x00
    ENVELOPE = 0x01


class ProbeType(IntEnum):
    ROTARY_DEBUG = 0x00
    DEPTH_PROBE = 0x01


class WaveDataType(IntEnum):
    U8_RAW = 0x00
    U16_RAW = 0x01
    U16_ENVELOPE = 0x02


@dataclass(slots=True)
class ScanConfig:
    scan_step: int
    sound_speed: int
    medium: MediumType
    flags: int = 0


@dataclass(slots=True)
class DebugConfig:
    start_angle: int
    end_angle: int
    step_angle: int
    capture_type: CaptureType
    sample_count: int
    flags: int
    probe_type: ProbeType = ProbeType.ROTARY_DEBUG
    pulse_count: int = 6


@dataclass(slots=True)
class ScanPoint:
    point_index: int = 0
    angle: int = 0
    distance_mm: int = 0
    quality: int = 0
    valid: bool = True
    has_wave: bool = False
    timestamp_ms: int = 0


@dataclass(slots=True)
class DistanceResult:
    angle: int
    distance_mm: int
    quality: int
    valid: bool


@dataclass(slots=True)
class WaveChunk:
    frame_id: int
    total_frames: int
    sample_offset: int
    sample_count: int
    angle: int
    data_type: WaveDataType
    samples: bytes
