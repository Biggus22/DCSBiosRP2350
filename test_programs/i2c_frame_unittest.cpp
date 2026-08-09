#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "internal/I2cFrame.h"

static int tests_passed = 0;
static int tests_failed = 0;

static void test_crc8_known_vectors() {
    // Empty data: CRC = 0x00
    uint8_t crc = i2cFrame_crc8(NULL, 0);
    if (crc == 0x00) { tests_passed++; } else { tests_failed++; printf("FAIL: crc8 empty\n"); }

    // Single byte 0x00: CRC = 0x00
    uint8_t d1[] = {0x00};
    crc = i2cFrame_crc8(d1, 1);
    if (crc == 0x00) { tests_passed++; } else { tests_failed++; printf("FAIL: crc8 0x00\n"); }

    // Single byte 0x01: CRC should be 0x07
    uint8_t d2[] = {0x01};
    crc = i2cFrame_crc8(d2, 1);
    if (crc == 0x07) { tests_passed++; } else { tests_failed++; printf("FAIL: crc8 0x01\n"); }

    // [0x01, 0x01, 0x00] = known vector
    uint8_t d3[] = {0x01, 0x01, 0x00};
    crc = i2cFrame_crc8(d3, 3);
    if (crc == 0x07) { tests_passed++; } else { tests_failed++; printf("FAIL: crc8 [1,1,0]\n"); }
}

static void test_encode_decode_roundtrip() {
    uint8_t buf[I2C_FRAME_MAX_SIZE];
    uint8_t payload[] = {0xAB, 0xCD};
    uint8_t total = i2cFrame_encode(buf, 0x01, 0x02, payload, 2);
    if (total != 6) { tests_failed++; printf("FAIL: encode length %d\n", total); return; }

    uint8_t outReg, outCmd, outDataLen;
    const uint8_t *outData;
    bool ok = i2cFrame_decode(buf, total, &outReg, &outCmd, &outData, &outDataLen);
    if (!ok) { tests_failed++; printf("FAIL: decode returned false\n"); return; }

    if (outReg != 0x01) { tests_failed++; printf("FAIL: reg mismatch\n"); return; }
    if (outCmd != 0x02) { tests_failed++; printf("FAIL: cmd mismatch\n"); return; }
    if (outDataLen != 2) { tests_failed++; printf("FAIL: len mismatch\n"); return; }
    if (outData[0] != 0xAB || outData[1] != 0xCD) { tests_failed++; printf("FAIL: data mismatch\n"); return; }
    tests_passed++;
}

static void test_crc_corruption_rejected() {
    uint8_t buf[I2C_FRAME_MAX_SIZE];
    uint8_t payload[] = {0x00};
    uint8_t total = i2cFrame_encode(buf, 0x01, 0x01, payload, 1);
    buf[total - 1] ^= 0xFF; // flip all CRC bits

    uint8_t outReg, outCmd, outDataLen;
    const uint8_t *outData;
    bool ok = i2cFrame_decode(buf, total, &outReg, &outCmd, &outData, &outDataLen);
    if (!ok) { tests_passed++; } else { tests_failed++; printf("FAIL: corrupted CRC accepted\n"); }
}

static void test_oversized_payload_rejected() {
    uint8_t buf[I2C_FRAME_MAX_SIZE];
    // Try to encode a payload that exceeds max
    uint8_t oversized[251];
    uint8_t total = i2cFrame_encode(buf, 0x01, 0x01, oversized, 251);
    if (total == 0) { tests_passed++; } else { tests_failed++; printf("FAIL: oversized encode returned %d\n", total); }
}

static void test_truncated_frame_rejected() {
    uint8_t buf[3] = {0x01, 0x01, 0x00}; // too short for overhead
    uint8_t outReg, outCmd, outDataLen;
    const uint8_t *outData;
    bool ok = i2cFrame_decode(buf, 3, &outReg, &outCmd, &outData, &outDataLen);
    if (!ok) { tests_passed++; } else { tests_failed++; printf("FAIL: truncated frame accepted\n"); }
}

int main() {
    stdio_init_all();
    sleep_ms(500); // wait for USB CDC connection

    printf("I2C Frame Unit Test\n");
    printf("===================\n");

    test_crc8_known_vectors();
    test_encode_decode_roundtrip();
    test_crc_corruption_rejected();
    test_oversized_payload_rejected();
    test_truncated_frame_rejected();

    printf("===================\n");
    printf("Passed: %d  Failed: %d\n", tests_passed, tests_failed);

    if (tests_failed == 0) {
        printf("ALL TESTS PASSED\n");
    } else {
        printf("SOME TESTS FAILED\n");
    }

    while (true) {
        sleep_ms(1000);
    }
}
