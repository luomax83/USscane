from __future__ import annotations

import argparse
import subprocess
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


PROBE_UID = "000000800671ff525154887767033308a5a5a5a597969908"
TARGET = "stm32h743xx"
FREQUENCY = "100k"

ADDR_CAPTURE_RESULT = 0x2000007C
ADDR_CAPTURE_DONE = 0x20000080
ADDR_CAPTURE_REQUEST = 0x20000084
ADDR_SAMPLE_BUFFER = 0x30000000
SAMPLE_COUNT = 7500
SAMPLE_BYTES = SAMPLE_COUNT * 2


def run_pyocd(*commands: str, workdir: Path) -> str:
    args = [
        "pyocd",
        "commander",
        "-u",
        PROBE_UID,
        "-t",
        TARGET,
        "-f",
        FREQUENCY,
        "-M",
        "attach",
    ]
    for command in commands:
        args.extend(["-c", command])
    completed = subprocess.run(
        args,
        cwd=workdir,
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout


def read_word(address: int, *, workdir: Path) -> int:
    output = run_pyocd(f"read32 0x{address:08x}", workdir=workdir)
    for line in output.splitlines():
        line = line.strip()
        if line.startswith(f"{address:08x}:"):
            return int(line.split()[1], 16)
    raise RuntimeError(f"failed to parse read32 output for 0x{address:08x}")


def trigger_capture(*, workdir: Path, timeout_s: float) -> int:
    run_pyocd(
        f"ww 0x{ADDR_CAPTURE_RESULT:08x} 0",
        f"ww 0x{ADDR_CAPTURE_DONE:08x} 0",
        f"ww 0x{ADDR_CAPTURE_REQUEST:08x} 1",
        workdir=workdir,
    )

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        done = read_word(ADDR_CAPTURE_DONE, workdir=workdir)
        if done != 0:
            return read_word(ADDR_CAPTURE_RESULT, workdir=workdir)
        time.sleep(0.1)
    raise TimeoutError("capture timeout")


def save_memory(output_bin: Path, *, workdir: Path) -> None:
    run_pyocd(
        f"savemem 0x{ADDR_SAMPLE_BUFFER:08x} {SAMPLE_BYTES} {output_bin.as_posix()}",
        workdir=workdir,
    )


def render_plot(bin_path: Path, png_path: Path) -> None:
    samples = np.fromfile(bin_path, dtype="<u2")
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), dpi=140)
    axes[0].plot(samples, linewidth=0.8, color="#0f766e")
    axes[0].set_title("ADC2 DMA Capture (7500 samples)")
    axes[0].set_xlabel("Sample Index")
    axes[0].set_ylabel("ADC Code")
    axes[0].grid(True, alpha=0.25)

    head = samples[:600] if samples.size >= 600 else samples
    axes[1].plot(head, linewidth=1.0, color="#b45309")
    axes[1].set_title("First 600 Samples")
    axes[1].set_xlabel("Sample Index")
    axes[1].set_ylabel("ADC Code")
    axes[1].grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(png_path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent,
    )
    args = parser.parse_args()

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    bin_path = output_dir / "depth_7500_u16.bin"
    png_path = output_dir / "depth_7500_u16.png"

    result = trigger_capture(workdir=output_dir.parent, timeout_s=args.timeout)
    if result != 1:
      raise RuntimeError(f"capture failed, result={result}")

    save_memory(bin_path, workdir=output_dir.parent)
    render_plot(bin_path, png_path)

    samples = np.fromfile(bin_path, dtype="<u2")
    print(f"sample_count={samples.size}")
    print(f"min={int(samples.min())} max={int(samples.max())} mean={float(samples.mean()):.3f}")
    print(f"bin={bin_path}")
    print(f"png={png_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
