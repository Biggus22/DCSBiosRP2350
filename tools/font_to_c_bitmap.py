#!/usr/bin/env python3
"""
Render selected glyphs from a TTF/OTF into a compact C header file.

Usage:
  python tools/font_to_c_bitmap.py \
    --font /path/to/MS33558.ttf \
    --size 48 \
    --chars "0123456789." \
    --out src/internal/MS33558_48.h

The generated header will define:
  - MS33558_48_glyph_count
  - MS33558_48_glyph_widths[] (per-glyph width)
  - MS33558_48_height
  - MS33558_48_bitmap[] (packed rows, one bit per pixel, MSB-first)
  - MS33558_48_first_char (start index of chars supplied)

This is a simple, portable format intended for embedded use.
"""
import argparse
from PIL import Image, ImageFont, ImageDraw
import math

def render_glyphs(ttf_path, size, chars):
    font = ImageFont.truetype(ttf_path, size)
    glyphs = []
    max_h = 0
    for ch in chars:
        mask = font.getmask(ch, mode='1')
        w, h = mask.size
        img = Image.new('1', (w, h), 0)
        img.putdata(list(mask))
        glyphs.append((ch, img))
        max_h = max(max_h, h)
    return glyphs, max_h

def pack_bitmap(glyphs, height):
    widths = []
    packed = []
    for ch, img in glyphs:
        w, h = img.size
        widths.append(w)
        # pad to height at top alignment
        data = []
        for row in range(height):
            byte = 0
            bitpos = 7
            for col in range(w):
                # choose pixel from img; align top
                y = row - (height - h)
                pix = 0
                if 0 <= y < h:
                    pix = img.getpixel((col, y))
                if pix:
                    byte |= (1 << bitpos)
                bitpos -= 1
                if bitpos < 0:
                    data.append(byte)
                    byte = 0
                    bitpos = 7
            # flush remaining bits for this row if any
            if bitpos != 7:
                data.append(byte)
        packed.append(data)
    return widths, packed

def write_header(out_path, glyphs, widths, packed, height, chars):
    first = ord(chars[0])
    count = len(chars)
    # Build a char→index map for all 256 ASCII values (255 = not found)
    char_map = [255] * 256
    for idx, ch in enumerate(chars):
        char_map[ord(ch)] = idx
    with open(out_path, 'w') as f:
        f.write('// Auto-generated MS33558 numeric font header\n')
        f.write('#pragma once\n\n')
        f.write('#include <stdint.h>\n\n')
        f.write(f'static const uint8_t MS33558_48_height = {height};\n')
        f.write(f'static const uint8_t MS33558_48_first_char = {first};\n')
        f.write(f'static const uint8_t MS33558_48_glyph_count = {count};\n')
        f.write('\n')
        f.write('// char_map[ascii] = glyph index, 255 = not found\n')
        f.write('static const uint8_t MS33558_48_char_map[256] = {\n')
        for i in range(0, 256, 16):
            f.write('  ' + ', '.join(f'{c:3d}' for c in char_map[i:i+16]) + ',\n')
        f.write('};\n\n')
        f.write('static const uint8_t MS33558_48_glyph_widths[] = {\n')
        f.write('  ' + ', '.join(str(w) for w in widths) + '\n};\n\n')
        # flatten bitmap and indexes
        offsets = []
        flat = []
        off = 0
        for data in packed:
            offsets.append(off)
            flat.extend(data)
            off += len(data)
        f.write(f'static const uint16_t MS33558_48_glyph_offsets[] = {{\n')
        f.write('  ' + ', '.join(str(o) for o in offsets) + '\n};\n\n')
        f.write(f'static const uint8_t MS33558_48_bitmap[] = {{\n')
        # write bytes
        lines = []
        for i, b in enumerate(flat):
            if i % 12 == 0:
                lines.append('')
            lines[-1] += f'0x{b:02X}, '
        for ln in lines:
            f.write('  ' + ln + '\n')
        f.write('};\n')

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--font', required=True)
    p.add_argument('--size', required=True, type=int)
    p.add_argument('--chars', required=True)
    p.add_argument('--out', required=True)
    args = p.parse_args()

    glyphs, max_h = render_glyphs(args.font, args.size, args.chars)
    widths, packed = pack_bitmap(glyphs, max_h)
    write_header(args.out, glyphs, widths, packed, max_h, args.chars)

if __name__ == '__main__':
    main()
