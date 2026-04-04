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

void ST7789::drawNumber48_new(uint16_t x, uint16_t y, const char* s, uint16_t color) {
#if HAS_MS33558_48
    // Render with fixed per-digit cell width so digits do not shift.
    const uint8_t first = MS33558_48_first_char;
    // Determine a uniform cell width based on digit glyphs '0'..'9' if present.
    uint16_t cell_w = 0;
    for (int i = 0; i < MS33558_48_glyph_count; ++i) {
        int ch = first + i;
        if (ch >= '0' && ch <= '9') {
            if (MS33558_48_glyph_widths[i] > cell_w) cell_w = MS33558_48_glyph_widths[i];
        }
    }
    // If no explicit digit glyphs found, fall back to max glyph width.
    if (cell_w == 0) {
        for (int i = 0; i < MS33558_48_glyph_count; ++i) if (MS33558_48_glyph_widths[i] > cell_w) cell_w = MS33558_48_glyph_widths[i];
    }
    const uint16_t spacing = 2;
    uint16_t cursor = x;
    const uint16_t glyph_h = MS33558_48_height;
    for (const char* p = s; *p; ++p) {
        if (*p == '.') {
            // center a small dot inside the digit cell
            uint16_t dot_w = 6;
            uint16_t dot_h = 6;
            uint16_t dot_x = cursor + (cell_w > dot_w ? (cell_w - dot_w) / 2 : 0);
            uint16_t dot_y = y + (glyph_h > 12 ? glyph_h - 12 : glyph_h - dot_h);
            drawFillRect(dot_x, dot_y, dot_w, dot_h, color);
            cursor += cell_w + spacing;
            continue;
        }
        int ch = (int)(uint8_t)*p;
        if (ch < first || ch >= first + MS33558_48_glyph_count) ch = first; // fallback to first
        int idx = ch - first;
        uint16_t w = MS33558_48_glyph_widths[idx];
        uint32_t off = MS33558_48_glyph_offsets[idx];
        int bytesPerRow = (w + 7) / 8;
        // center glyph horizontally within cell
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
    // fallback to scaled 5x7 if header not present
    drawStringScaled(x, y, s, color, 6);
#endif
}

void ST7789::drawNumber48Right(uint16_t right_x, uint16_t y, const char* s, uint16_t color) {
#if HAS_MS33558_48
    // compute total width using the fixed cell width logic so right-justify matches rendering
    const uint8_t first = MS33558_48_first_char;
    uint16_t cell_w = 0;
    for (int i = 0; i < MS33558_48_glyph_count; ++i) {
        int ch = first + i;
        if (ch >= '0' && ch <= '9') {
            if (MS33558_48_glyph_widths[i] > cell_w) cell_w = MS33558_48_glyph_widths[i];
        }
    }
    if (cell_w == 0) {
        for (int i = 0; i < MS33558_48_glyph_count; ++i) if (MS33558_48_glyph_widths[i] > cell_w) cell_w = MS33558_48_glyph_widths[i];
    }
    const uint16_t spacing = 2;
    // every character occupies one cell (including '.'), so total width is len*(cell_w+spacing)
    uint32_t len = 0;
    for (const char* p = s; *p; ++p) ++len;
    uint32_t totalW = len * (uint32_t)(cell_w + spacing);
    int32_t start_x = (int32_t)right_x - (int32_t)totalW;
    if (start_x < 0) start_x = 0;
    drawNumber48_new((uint16_t)start_x, y, s, color);
#else
    // fallback: use scaled 5x7 width calculation (scale 6 used in fallback)
    const uint8_t scale = 6;
    uint32_t totalW = 0;
    for (const char* p = s; *p; ++p) {
        totalW += (5 + 1) * scale; // 5px glyph + 1 spacing
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
