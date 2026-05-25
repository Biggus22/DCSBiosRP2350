#!/usr/bin/env python3
"""Convert a JPEG image to an RGB565 C header for ST7789 display."""

import sys
from PIL import Image


def rgb888_to_rgb565_be(r, g, b):
    r5 = r >> 3
    g6 = g >> 2
    b5 = b >> 3
    val = (r5 << 11) | (g6 << 5) | b5
    return (val >> 8, val & 0xFF)


def convert_jpg_to_header(input_path, output_path, crop_width=None):
    img = Image.open(input_path)
    if img.mode != 'RGB':
        img = img.convert('RGB')

    orig_w, orig_h = img.size

    if crop_width is not None and crop_width < orig_w:
        x_offset = (orig_w - crop_width) // 2
        img = img.crop((x_offset, 0, x_offset + crop_width, orig_h))

    w, h = img.size
    pixels = list(img.getdata())

    lines = []
    lines.append(f"// Auto-generated from {input_path}")
    lines.append(f"// Original: {orig_w}x{orig_h}, Output: {w}x{h}")
    lines.append(f"// RGB565 big-endian byte order")
    lines.append("#ifndef GEAR_FLAG_H")
    lines.append("#define GEAR_FLAG_H")
    lines.append("")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append(f"#define GEAR_FLAG_WIDTH  {w}")
    lines.append(f"#define GEAR_FLAG_HEIGHT {h}")
    lines.append("#define GEAR_FLAG_UP_END    223")
    lines.append("#define GEAR_FLAG_TRANS_END 714")
    lines.append(f"// DOWN section: rows 715 to {h-1}")
    lines.append("")
    lines.append(f"static const uint8_t gearFlagBitmap[{w * h * 2}] = {{")

    col = 0
    buf = " "
    for y in range(h):
        for x in range(w):
            idx = y * w + x
            r, g, b = pixels[idx]
            hi, lo = rgb888_to_rgb565_be(r, g, b)
            buf += f" 0x{hi:02x}, 0x{lo:02x},"
            col += 1
            if col >= 8:
                lines.append(buf)
                buf = " "
                col = 0
    if col > 0:
        lines.append(buf.rstrip(","))
    else:
        # remove trailing comma from last line
        if lines[-1].endswith(","):
            lines[-1] = lines[-1][:-1]

    lines.append("};")
    lines.append("")
    lines.append("#endif // GEAR_FLAG_H")

    with open(output_path, "w") as f:
        f.write("\n".join(lines))
        f.write("\n")

    file_size = w * h * 2
    print(f"Written: {output_path}")
    print(f"Array size: {w}x{h}x2 = {file_size} bytes ({file_size/1024:.1f} KB)")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python jpg_to_rgb565.py <input.jpg> <output.h> [crop_width]")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2]
    crop_width = int(sys.argv[3]) if len(sys.argv) > 3 else None

    convert_jpg_to_header(input_path, output_path, crop_width)
