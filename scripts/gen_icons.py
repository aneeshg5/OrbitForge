#!/usr/bin/env python3
import math
import struct
import zlib

BG = (10, 10, 15, 255)
PLANET = (40, 70, 110, 255)
ORBIT = (56, 189, 248, 255)
SAT = (251, 146, 60, 255)


def make_icon(size: int) -> list[list[tuple[int, int, int, int]]]:
    px = [[BG for _ in range(size)] for _ in range(size)]
    cx = cy = size / 2
    planet_r = size * 0.22
    orbit_a = size * 0.40
    orbit_b = size * 0.20
    ring_w = max(1.5, size * 0.012)

    for y in range(size):
        for x in range(size):
            dx, dy = x - cx, y - cy
            if dx * dx + dy * dy <= planet_r * planet_r:
                px[y][x] = PLANET
                continue
            t = (dx * dx) / (orbit_a * orbit_a) + (dy * dy) / (orbit_b * orbit_b)
            edge = abs(math.sqrt(t) - 1.0) * min(orbit_a, orbit_b)
            if edge <= ring_w:
                px[y][x] = ORBIT

    theta = math.radians(40)
    sx = cx + orbit_a * math.cos(theta)
    sy = cy + orbit_b * math.sin(theta)
    sat_r = size * 0.035
    for y in range(size):
        for x in range(size):
            dx, dy = x - sx, y - sy
            if dx * dx + dy * dy <= sat_r * sat_r:
                px[y][x] = SAT
    return px


def write_png(path: str, px: list[list[tuple[int, int, int, int]]]) -> None:
    size = len(px)
    raw = bytearray()
    for row in px:
        raw.append(0)
        for (r, g, b, a) in row:
            raw.extend((r, g, b, a))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", size, size, 8, 6, 0, 0, 0)
    idat = zlib.compress(bytes(raw), 9)
    with open(path, "wb") as f:
        f.write(sig)
        f.write(chunk(b"IHDR", ihdr))
        f.write(chunk(b"IDAT", idat))
        f.write(chunk(b"IEND", b""))


if __name__ == "__main__":
    import os

    out_dir = os.path.join(os.path.dirname(__file__), "..", "web", "public")
    for size in (192, 512):
        path = os.path.join(out_dir, f"icon-{size}.png")
        write_png(path, make_icon(size))
        print(f"wrote {path}")
