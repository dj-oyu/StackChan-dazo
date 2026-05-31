#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "pyserial>=3.5",
# ]
# ///
"""Unified CAMRAW tool: receive / extract / render a raw camera frame.

`self.camera.dump_raw_frame` prints one *un-normalized* sensor buffer to the
serial log as CAMRAW Base64 chunks. This tool reassembles that dump and renders
it.

IMPORTANT: the `format=0x...` FourCC in the dump is NOT trusted. The driver is
known to deliver e.g. YUV422P as packed YUYV (see docs/camera-notes.md), and the
dump is the *raw* buffer taken before any normalization. So `render` always
emits every candidate interpretation (RGB565 be/le + the four YUV422 orderings)
and you pick the one that looks right by eye. The reported FourCC is printed only
as a hint.

Subcommands:
  receive   listen on a serial port, capture a dump live           -> camera.raw
  extract   pull a dump out of an existing monitor log             -> camera.raw
  render    decode a raw frame into every candidate .ppm           -> out dir

Examples:
  uv run firmware/tools/camraw.py receive -p COM3 -o camera.raw
  uv run firmware/tools/camraw.py extract camraw.log -o camera.raw
  uv run firmware/tools/camraw.py render camera.raw --width 320 --height 240 --out-dir camera_renders
"""
import argparse
import base64
import binascii
import re
import sys
from pathlib import Path

# --- CAMRAW wire format -----------------------------------------------------

BEGIN_RE = re.compile(
    r"CAMRAW:BEGIN width=(\d+) height=(\d+) format=0x([0-9a-fA-F]+) len=(\d+) chunk=(\d+) chunks=(\d+)"
)
DATA_RE = re.compile(r"CAMRAW:DATA index=(\d+) offset=(\d+) raw=(\d+) b64=([A-Za-z0-9+/=]+)")
END_RE = re.compile(r"CAMRAW:END len=(\d+)")


def fourcc_str(format_code: int) -> str:
    """Decode a V4L2 FourCC into its 4 ASCII chars (hint only, not trusted)."""
    chars = [(format_code >> shift) & 0xFF for shift in (0, 8, 16, 24)]
    return "".join(chr(c) if 32 <= c < 127 else "?" for c in chars)


def parse_begin(match: re.Match) -> dict:
    return {
        "width": int(match.group(1)),
        "height": int(match.group(2)),
        "format": int(match.group(3), 16),
        "len": int(match.group(4)),
        "chunk": int(match.group(5)),
        "chunks": int(match.group(6)),
    }


def add_data(chunks: dict[int, tuple[int, bytes]], match: re.Match) -> None:
    index = int(match.group(1))
    offset = int(match.group(2))
    raw_len = int(match.group(3))
    decoded = base64.b64decode(match.group(4), validate=True)
    if len(decoded) != raw_len:
        raise ValueError(f"chunk {index} length mismatch: expected {raw_len}, got {len(decoded)}")
    chunks[index] = (offset, decoded)


def reassemble(meta: dict, chunks: dict[int, tuple[int, bytes]]) -> bytes:
    if len(chunks) != meta["chunks"]:
        missing = sorted(set(range(meta["chunks"])) - set(chunks))
        raise SystemExit(f"missing chunks: {missing[:20]}")
    frame = bytearray(meta["len"])
    for index in range(meta["chunks"]):
        offset, data = chunks[index]
        frame[offset : offset + len(data)] = data
    return bytes(frame)


def describe(meta: dict) -> str:
    return (
        f"width={meta['width']} height={meta['height']} "
        f"format=0x{meta['format']:08x} ('{fourcc_str(meta['format'])}', untrusted)"
    )


# --- pixel decoding (every candidate interpretation, none trusted) ----------

def clamp(value: int) -> int:
    return max(0, min(255, value))


def rgb565_to_rgb(value: int) -> tuple[int, int, int]:
    r = (value >> 11) & 0x1F
    g = (value >> 5) & 0x3F
    b = value & 0x1F
    return (r * 255 // 31, g * 255 // 63, b * 255 // 31)


def yuv_to_rgb(y: int, u: int, v: int) -> tuple[int, int, int]:
    c = y - 16
    d = u - 128
    e = v - 128
    return (
        clamp((298 * c + 409 * e + 128) >> 8),
        clamp((298 * c - 100 * d - 208 * e + 128) >> 8),
        clamp((298 * c + 516 * d + 128) >> 8),
    )


def render_rgb565(data: bytes, endian: str) -> bytes:
    out = bytearray()
    for i in range(0, len(data) - 1, 2):
        value = (data[i] << 8) | data[i + 1] if endian == "be" else data[i] | (data[i + 1] << 8)
        out.extend(rgb565_to_rgb(value))
    return bytes(out)


def render_yuv422(data: bytes, order: str) -> bytes:
    out = bytearray()
    for i in range(0, len(data) - 3, 4):
        a, b, c, d = data[i : i + 4]
        if order == "yuyv":
            y0, u, y1, v = a, b, c, d
        elif order == "yvyu":
            y0, v, y1, u = a, b, c, d
        elif order == "uyvy":
            u, y0, v, y1 = a, b, c, d
        elif order == "vyuy":
            v, y0, u, y1 = a, b, c, d
        else:
            raise ValueError(order)
        out.extend(yuv_to_rgb(y0, u, v))
        out.extend(yuv_to_rgb(y1, u, v))
    return bytes(out)


INTERPRETATIONS = {
    "rgb565_be": lambda d: render_rgb565(d, "be"),
    "rgb565_le": lambda d: render_rgb565(d, "le"),
    "yuyv": lambda d: render_yuv422(d, "yuyv"),
    "yvyu": lambda d: render_yuv422(d, "yvyu"),
    "uyvy": lambda d: render_yuv422(d, "uyvy"),
    "vyuy": lambda d: render_yuv422(d, "vyuy"),
}


def write_ppm(path: Path, width: int, height: int, rgb: bytes) -> None:
    expected = width * height * 3
    if len(rgb) < expected:
        raise ValueError(f"{path}: not enough pixels: expected {expected}, got {len(rgb)}")
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + rgb[:expected])


def render_all(raw: Path, width: int, height: int, out_dir: Path) -> None:
    data = raw.read_bytes()
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, render in INTERPRETATIONS.items():
        path = out_dir / f"{raw.stem}_{name}.ppm"
        write_ppm(path, width, height, render(data))
        print(path)


# --- subcommands ------------------------------------------------------------

def cmd_receive(args: argparse.Namespace) -> int:
    import serial  # lazy: only the receive path needs pyserial

    meta = None
    chunks: dict[int, tuple[int, bytes]] = {}

    print(
        f"listening on {args.port} at {args.baud}. Trigger self.camera.dump_raw_frame now.",
        file=sys.stderr,
    )
    with serial.Serial(args.port, args.baud, timeout=1) as ser, args.log.open("w", encoding="utf-8") as log:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").rstrip()
            print(line)
            log.write(line + "\n")
            log.flush()

            # A fresh BEGIN can arrive at any time: the device may reboot mid-dump
            # (e.g. watchdog) and re-trigger. Treat it as a restart, not an error.
            match = BEGIN_RE.search(line)
            if match:
                if meta is not None and len(chunks) < meta["chunks"]:
                    print(
                        f"restart: new CAMRAW:BEGIN after only {len(chunks)}/{meta['chunks']} chunks "
                        "(device rebooted mid-dump?)",
                        file=sys.stderr,
                    )
                meta = parse_begin(match)
                chunks = {}
                print(f"receiving {meta['len']} bytes in {meta['chunks']} chunks ({describe(meta)})", file=sys.stderr)
                continue
            if meta is None:
                continue

            match = DATA_RE.search(line)
            if match:
                try:
                    add_data(chunks, match)
                except (ValueError, binascii.Error) as exc:
                    # Serial lines get interleaved with other log output (a stray WDT/log
                    # message can splice into a b64 payload); skip the corrupt chunk.
                    print(f"skip corrupt chunk: {exc}", file=sys.stderr)
                    continue
                index = int(match.group(1))
                if index % 25 == 0:
                    print(f"received chunk {index + 1}/{meta['chunks']}", file=sys.stderr)
                continue

            match = END_RE.search(line)
            if match:
                ended_len = int(match.group(1))
                if ended_len != meta["len"]:
                    raise ValueError(f"end length mismatch: expected {meta['len']}, got {ended_len}")
                break

    if meta is None:
        raise SystemExit("CAMRAW:BEGIN not received")
    frame = reassemble(meta, chunks)
    args.output.write_bytes(frame)
    print(f"wrote {args.output} ({len(frame)} bytes), {describe(meta)}", file=sys.stderr)
    _maybe_render(args, meta)
    return 0


def cmd_extract(args: argparse.Namespace) -> int:
    meta = None
    chunks: dict[int, tuple[int, bytes]] = {}
    ended_len = None

    for line in args.log.read_text(errors="ignore").splitlines():
        if meta is None:
            match = BEGIN_RE.search(line)
            if match:
                meta = parse_begin(match)
            continue
        match = DATA_RE.search(line)
        if match:
            add_data(chunks, match)
            continue
        match = END_RE.search(line)
        if match:
            ended_len = int(match.group(1))

    if meta is None:
        raise SystemExit("CAMRAW:BEGIN not found")
    if ended_len != meta["len"]:
        raise SystemExit(f"CAMRAW:END missing or length mismatch: expected {meta['len']}, got {ended_len}")
    frame = reassemble(meta, chunks)
    args.output.write_bytes(frame)
    print(f"wrote {args.output} ({len(frame)} bytes), {describe(meta)}")
    _maybe_render(args, meta)
    return 0


def cmd_render(args: argparse.Namespace) -> int:
    render_all(args.raw, args.width, args.height, args.out_dir)
    return 0


def _maybe_render(args: argparse.Namespace, meta: dict) -> None:
    """receive/extract know width/height, so optionally render right away."""
    if not args.render_dir:
        return
    render_all(args.output, meta["width"], meta["height"], args.render_dir)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p_recv = sub.add_parser("receive", help="capture a dump live from a serial port")
    p_recv.add_argument("-p", "--port", default="COM3", help="serial port, e.g. COM3")
    p_recv.add_argument("-b", "--baud", type=int, default=115200)
    p_recv.add_argument("-o", "--output", type=Path, default=Path("camera.raw"))
    p_recv.add_argument("--log", type=Path, default=Path("camraw.log"), help="raw serial text log")
    p_recv.add_argument("--render-dir", type=Path, default=None, help="also render all interpretations here")
    p_recv.set_defaults(func=cmd_receive)

    p_ext = sub.add_parser("extract", help="pull a dump out of an existing monitor log")
    p_ext.add_argument("log", type=Path, help="monitor log containing CAMRAW lines")
    p_ext.add_argument("-o", "--output", type=Path, default=Path("camera.raw"))
    p_ext.add_argument("--render-dir", type=Path, default=None, help="also render all interpretations here")
    p_ext.set_defaults(func=cmd_extract)

    p_ren = sub.add_parser("render", help="decode a raw frame into every candidate .ppm")
    p_ren.add_argument("raw", type=Path)
    p_ren.add_argument("--width", type=int, required=True)
    p_ren.add_argument("--height", type=int, required=True)
    p_ren.add_argument("--out-dir", type=Path, default=Path("camera_raw_renders"))
    p_ren.set_defaults(func=cmd_render)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
