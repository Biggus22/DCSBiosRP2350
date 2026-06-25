#include "ST7789.h"

// If the MS33558 48px numeric font header exists, include it at file scope.
#if defined(__has_include)
#  if __has_include("MS33558_48.h")
#    include "MS33558_48.h"
#    define HAS_MS33558_48 1
#  else
#    define HAS_MS33558_48 0
#  endif
#else
#  define HAS_MS33558_48 0
#endif

// Compute uniform cell width: max glyph width across all glyphs in the font.
static uint16_t ms33558CellWidth() {
    uint16_t cell_w = 0;
    for (int i = 0; i < MS33558_48_glyph_count; ++i) {
        if (MS33558_48_glyph_widths[i] > cell_w) cell_w = MS33558_48_glyph_widths[i];
    }
    return cell_w;
}

void ST7789::drawNumber48_new(uint16_t x, uint16_t y, const char* s, uint16_t color) {
#if HAS_MS33558_48
    // Render with fixed uniform cell width so characters do not shift.
    const uint16_t cell_w = ms33558CellWidth();
    const uint16_t spacing = 2;
    const uint16_t glyph_h = MS33558_48_height;
    uint16_t cursor = x;
    for (const char* p = s; *p; ++p) {
        if (*p == '.') {
            uint16_t dot_w = 6;
            uint16_t dot_h = 6;
            uint16_t dot_x = cursor + (cell_w > dot_w ? (cell_w - dot_w) / 2 : 0);
            uint16_t dot_y = y + (glyph_h > 12 ? glyph_h - 12 : glyph_h - dot_h);
            drawFillRect(dot_x, dot_y, dot_w, dot_h, color);
            cursor += cell_w + spacing;
            continue;
        }
        int idx = MS33558_48_char_map[(int)(uint8_t)*p];
        if (idx >= MS33558_48_glyph_count) idx = 0; // fallback to first glyph
        uint16_t w = MS33558_48_glyph_widths[idx];
        uint32_t off = MS33558_48_glyph_offsets[idx];
        int bytesPerRow = (w + 7) / 8;
        uint16_t xoff = cursor + (cell_w > w ? (cell_w - w) / 2 : 0);
        for (int row = 0; row < glyph_h; ++row) {
            for (int col = 0; col < w; ++col) {
                int byteIndex = off + row * bytesPerRow + (col / 8);
                uint8_t byte = MS33558_48_bitmap[byteIndex];
                int bit = 7 - (col % 8);
                if (byte & (1 << bit)) {
                    drawPixel(xoff + col, y + row, color);
                }
            }
        }
        cursor += cell_w + spacing;
    }
#else
    drawStringScaled(x, y, s, color, 6);
#endif
}

void ST7789::drawNumber48Right(uint16_t right_x, uint16_t y, const char* s, uint16_t color) {
#if HAS_MS33558_48
    const uint16_t cell_w = ms33558CellWidth();
    const uint16_t spacing = 2;
    uint32_t len = 0;
    for (const char* p = s; *p; ++p) ++len;
    uint32_t totalW = len * (uint32_t)(cell_w + spacing);
    int32_t start_x = (int32_t)right_x - (int32_t)totalW;
    if (start_x < 0) start_x = 0;
    drawNumber48_new((uint16_t)start_x, y, s, color);
#else
    const uint8_t scale = 6;
    uint32_t totalW = 0;
    for (const char* p = s; *p; ++p) {
        totalW += (5 + 1) * scale;
    }
    int32_t start_x = (int32_t)right_x - (int32_t)totalW;
    if (start_x < 0) start_x = 0;
    drawStringScaled((uint16_t)start_x, y, s, color, scale);
#endif
}

// Backwards-compatible wrapper for old call sites that still call `drawNumber48`
void ST7789::drawNumber48(uint16_t x, uint16_t y, const char* s, uint16_t color) {
    drawNumber48_new(x, y, s, color);
}
