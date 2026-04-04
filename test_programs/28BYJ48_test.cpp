// 28BYJ-48 stepper sweep test using BYJ48_stepper driver
// Pins default: GPIO 2,3,4,5. Uses BYJ48 driver (src/internal/BYJ48_stepper.*)

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <array>
#include "pico/time.h"
#include "BYJ48_stepper.h"

static bool ramped_move(byj_motor_t *motor, int32_t final_target, uint32_t start_delay, uint32_t end_delay, int segments, bool s_curve) {
    if (!motor) return false;
    int32_t start_pos = motor->current_position;
    int32_t total_steps = final_target - start_pos;
    if (total_steps == 0) return true;
    int dir = (total_steps > 0) ? 1 : -1;
    int32_t remaining = abs(total_steps);

    if (segments < 1) segments = 1;
    int32_t steps_per_seg = (remaining / segments) > 0 ? (remaining / segments) : 1;

    uint64_t last_moved_time = time_us_64();
    int32_t last_pos = motor->current_position;

    for (int seg = 0; seg < segments && remaining > 0; ++seg) {
        int32_t seg_steps = (remaining > steps_per_seg) ? steps_per_seg : remaining;
        float t = (float)(seg + 1) / (float)segments;
        uint32_t delay_us;
        if (s_curve) {
            float e = t * t * (3.0f - 2.0f * t);
            delay_us = (uint32_t)((float)start_delay + e * ((float)end_delay - (float)start_delay));
        } else {
            delay_us = (uint32_t)((float)start_delay + t * ((float)end_delay - (float)start_delay));
        }
        byj_set_speed(motor, delay_us);

        // set a near-term target and step using non-blocking updates so we can change delay between segments
        int32_t seg_target = motor->current_position + dir * seg_steps;
        motor->target_position = seg_target;
        // loop until this segment target reached or stall detected
        while (motor->current_position != motor->target_position) {
            bool moving = byj_update(motor);
            // detect movement
            if (motor->current_position != last_pos) {
                last_pos = motor->current_position;
                last_moved_time = time_us_64();
            }
            uint64_t now = time_us_64();
            if (now - last_moved_time > 2000000ULL) {
                // stalled for >2s
                printf("ramped_move: stall detected at pos %d\n", motor->current_position);
                return false;
            }
            sleep_us(50);
        }
        remaining -= seg_steps;
    }

    // finalize to exact final_target if any residual
    if (motor->current_position != final_target) {
        motor->target_position = final_target;
        uint64_t last = time_us_64();
        while (motor->current_position != motor->target_position) {
            byj_update(motor);
            if (motor->current_position != last_pos) { last_pos = motor->current_position; last = time_us_64(); }
            if (time_us_64() - last > 2000000ULL) { printf("ramped_move: stall on finalize\n"); return false; }
            sleep_us(50);
        }
    }
    return true;
}

const uint STEP_PINS[4] = {2, 3, 4, 5};

int main() {
    stdio_init_all();

    byj_motor_t motor;
    byj_gpio_config_t cfg = {STEP_PINS[0], STEP_PINS[1], STEP_PINS[2], STEP_PINS[3]};

    // init pins and motor
    if (!byj_init_gpio(&motor, &cfg, BYJ_MODE_FULL_DOUBLE)) {
        printf("Failed to init BYJ48 motor\n");
        return 1;
    }

    // set speed (microseconds per step)
    // enable bipolar H-bridge output mode and start with a safe slower speed
    byj_set_output_mode(&motor, BYJ_OUTPUT_BIPOLAR);
    byj_set_speed(&motor, 3000); // start slower; use serial to tune

    printf("28BYJ-48 BYJ48 driver sweep test\n");
    printf("Pins: %u %u %u %u\n", STEP_PINS[0], STEP_PINS[1], STEP_PINS[2], STEP_PINS[3]);
    printf("Step delay: %u us\n", motor.step_delay_us);

    int32_t sweep_steps = BYJ48_STEPS_PER_REV; // full revolution sweep (adjustable)
    const uint32_t dwell_ms = 1000;
    bool accel_enabled = false;
    const int default_ramp_segments = 20;
    int ramp_segments = default_ramp_segments;
    bool s_curve_enabled = false;
    uint32_t ramp_start_delay = 3000; // minimum start delay for ramp (us)
    printf("Pre-sweep commands (during 3s adjust window): '+' faster, '-' slower, '<' smaller sweep, '>' larger sweep, 'm' cycle mode, 'g' start immediately\n");
    printf("Also: 'r' = interactive range test (will prompt for movement Y/N)\n");

    while (true) {
        // pre-sweep adjust window for forward
        int eff_steps_per_rev = (motor.mode == BYJ_MODE_HALF) ? BYJ48_STEPS_PER_REV : (BYJ48_STEPS_PER_REV / 2);
        double deg_per_step = 360.0 / (double)eff_steps_per_rev;
        printf("Preparing forward sweep: delay=%u us, sweep_steps=%d, mode=%d\n", motor.step_delay_us, sweep_steps, motor.mode);
        printf("Effective steps/rev=%d (deg/step=%.4f)\n", eff_steps_per_rev, deg_per_step);
        printf("Adjust settings now (3s)...\n");
        uint64_t start = time_us_64();
        while ((time_us_64() - start) < 3000000ULL) {
            int ch = getchar_timeout_us(100000);
            if (ch == PICO_ERROR_TIMEOUT) continue;
            if (ch == '\r') printf("<received CR (13)>\n");
            else if (ch == '\n') printf("<received LF (10)>\n");
            else printf("<received '%c' (%d)>\n", (ch >= 32 && ch < 127) ? (char)ch : '?', ch);
            if (ch == '+') {
                uint32_t nd = (uint32_t)((float)motor.step_delay_us * 0.9f);
                if (nd < 100) nd = 100;
                byj_set_speed(&motor, nd);
                printf("Faster -> %u us\n", motor.step_delay_us);
            }
            else if (ch == '-') {
                uint32_t nd = (uint32_t)((float)motor.step_delay_us * 1.1f);
                byj_set_speed(&motor, nd);
                printf("Slower -> %u us\n", motor.step_delay_us);
            }
            else if (ch == '1') {
                uint32_t nd = (uint32_t)((float)motor.step_delay_us * 0.99f);
                if (nd < 100) nd = 100;
                byj_set_speed(&motor, nd);
                printf("Faster 1%% -> %u us\n", motor.step_delay_us);
            }
            else if (ch == '2') {
                uint32_t nd = (uint32_t)((float)motor.step_delay_us * 1.01f);
                byj_set_speed(&motor, nd);
                printf("Slower 1%% -> %u us\n", motor.step_delay_us);
            }
            else if (ch == '1') {
                uint32_t nd = (uint32_t)((float)motor.step_delay_us * 0.99f);
                if (nd < 100) nd = 100;
                byj_set_speed(&motor, nd);
                printf("Faster 1%% -> %u us\n", motor.step_delay_us);
            }
            else if (ch == '2') {
                uint32_t nd = (uint32_t)((float)motor.step_delay_us * 1.01f);
                byj_set_speed(&motor, nd);
                printf("Slower 1%% -> %u us\n", motor.step_delay_us);
            }
            else if (ch == '[') { uint32_t nd = (motor.step_delay_us > 200) ? motor.step_delay_us - 100 : 100; byj_set_speed(&motor, nd); printf("Faster -100 -> %u us\n", motor.step_delay_us); }
            else if (ch == ']') { byj_set_speed(&motor, motor.step_delay_us + 100); printf("Slower +100 -> %u us\n", motor.step_delay_us); }
            else if (ch == '<') { sweep_steps = (int32_t)(sweep_steps * 0.9f); printf("Sweep -> %d\n", sweep_steps); }
            else if (ch == '>') { sweep_steps = (int32_t)(sweep_steps * 1.1f); printf("Sweep -> %d\n", sweep_steps); }
            else if (ch == '3') { sweep_steps = (int32_t)(sweep_steps * 0.99f); if (sweep_steps < 1) sweep_steps = 1; printf("Sweep -1%% -> %d\n", sweep_steps); }
            else if (ch == '4') { sweep_steps = (int32_t)(sweep_steps * 1.01f); printf("Sweep +1%% -> %d\n", sweep_steps); }
            else if (ch == '3') { sweep_steps = (int32_t)(sweep_steps * 0.99f); if (sweep_steps < 1) sweep_steps = 1; printf("Sweep -1%% -> %d\n", sweep_steps); }
            else if (ch == '4') { sweep_steps = (int32_t)(sweep_steps * 1.01f); printf("Sweep +1%% -> %d\n", sweep_steps); }
            else if (ch == 'm') { if (motor.mode == BYJ_MODE_FULL_DOUBLE) motor.mode = BYJ_MODE_FULL_SINGLE; else if (motor.mode == BYJ_MODE_FULL_SINGLE) motor.mode = BYJ_MODE_HALF; else motor.mode = BYJ_MODE_FULL_DOUBLE; printf("Mode -> %d\n", motor.mode); }
            else if (ch == 'g') break;
            else if (ch == 'A') {
                printf("Auto-mapper: testing pin permutations (small steps)\n");
                byj_gpio_config_t orig_cfg = motor.cfg;
                uint32_t orig_speed = motor.step_delay_us;
                bool orig_accel = accel_enabled;
                accel_enabled = false;
                const int test_steps = 16;
                const uint32_t test_delay = 3000;
                byj_set_speed(&motor, test_delay);

                std::array<int,4> perm = {0,1,2,3};
                int pins[4] = { (int)orig_cfg.pin0, (int)orig_cfg.pin1, (int)orig_cfg.pin2, (int)orig_cfg.pin3 };
                int best_moved = -1;
                std::array<int,4> best_map = perm;
                std::sort(perm.begin(), perm.end());
                do {
                    byj_gpio_config_t newcfg = { (uint)pins[perm[0]], (uint)pins[perm[1]], (uint)pins[perm[2]], (uint)pins[perm[3]] };
                    motor.cfg = newcfg;
                    sleep_ms(50);
                    int32_t before = motor.current_position;
                    byj_step_steps(&motor, test_steps);
                    int32_t after = motor.current_position;
                    int moved = abs(after - before);
                    byj_step_steps(&motor, -test_steps);
                    printf("perm [%d %d %d %d] -> moved %d\n", newcfg.pin0, newcfg.pin1, newcfg.pin2, newcfg.pin3, moved);
                    if (moved > best_moved) { best_moved = moved; best_map = perm; }
                    sleep_ms(50);
                } while (std::next_permutation(perm.begin(), perm.end()));

                motor.cfg = orig_cfg;
                byj_set_speed(&motor, orig_speed);
                accel_enabled = orig_accel;

                printf("Auto-mapper complete. Best moved=%d for mapping [%d %d %d %d]\n", best_moved,
                       pins[best_map[0]], pins[best_map[1]], pins[best_map[2]], pins[best_map[3]]);
                printf("To apply best mapping, note the mapping shown and use manual cfg adjust (not automated).\n");
            }
            else if (ch == 'o') {
                motor.output_mode = (motor.output_mode == BYJ_OUTPUT_BIPOLAR) ? BYJ_OUTPUT_UNIPOLAR : BYJ_OUTPUT_BIPOLAR;
                printf("Output mode -> %s\n", motor.output_mode == BYJ_OUTPUT_BIPOLAR ? "BIPOLAR" : "UNIPOLAR");
            }
            else if (ch == 'p') {
                printf("Enter small step count then Enter:\n");
                char sbuf2[16]; int sidx2 = 0;
                while (sidx2 < (int)sizeof(sbuf2)-1) {
                    int c2 = getchar_timeout_us(5000000);
                    if (c2 == PICO_ERROR_TIMEOUT) break;
                    if (c2 == '\r' || c2 == '\n') break;
                    if (c2 >= '0' && c2 <= '9') { sbuf2[sidx2++] = (char)c2; printf("%c", (char)c2); }
                }
                sbuf2[sidx2] = '\0';
                int sc2 = atoi(sbuf2);
                if (sc2 <= 0) { printf("Invalid step count\n"); }
                else {
                    int32_t before2 = motor.current_position;
                    printf("Stepping %d steps...\n", sc2);
                    byj_step_steps(&motor, sc2);
                    printf("Position before: %d after: %d (moved %d)\n", before2, motor.current_position, motor.current_position - before2);
                }
            }
            else if (ch == 'a') { accel_enabled = !accel_enabled; printf("Acceleration %s\n", accel_enabled ? "ENABLED" : "DISABLED"); }
            else if (ch == 'z') { ramp_segments = (ramp_segments > 2) ? (ramp_segments - 1) : 1; printf("Ramp segments -> %d\n", ramp_segments); }
            else if (ch == 'x') { ramp_segments = ramp_segments + 1; printf("Ramp segments -> %d\n", ramp_segments); }
            else if (ch == 'Z') { ramp_segments = (ramp_segments > 5) ? (ramp_segments - 5) : 1; printf("Ramp segments -> %d\n", ramp_segments); }
            else if (ch == 'X') { ramp_segments = ramp_segments + 5; printf("Ramp segments -> %d\n", ramp_segments); }
            else if (ch == 'P') {
                // apply a recommended preset for tuning: conservative preset
                ramp_start_delay = 8000;
                ramp_segments = 40;
                s_curve_enabled = true;
                printf("Applied PRESET: start_delay=%u us, segments=%d, S-curve=ON\n", ramp_start_delay, ramp_segments);
            }
            else if (ch == 'Z') { ramp_segments = (ramp_segments > 5) ? (ramp_segments - 5) : 1; printf("Ramp segments -> %d\n", ramp_segments); }
            else if (ch == 'X') { ramp_segments = ramp_segments + 5; printf("Ramp segments -> %d\n", ramp_segments); }
            else if (ch == 'P') {
                ramp_start_delay = 8000;
                ramp_segments = 40;
                s_curve_enabled = true;
                printf("Applied PRESET: start_delay=%u us, segments=%d, S-curve=ON\n", ramp_start_delay, ramp_segments);
            }
            else if (ch == 'S') { s_curve_enabled = !s_curve_enabled; printf("S-curve %s\n", s_curve_enabled ? "ENABLED" : "DISABLED"); }
            else if (ch == 't') {
                printf("Enter ramp start delay in us (e.g. 3000) then Enter:\n");
                char tbuf[16]; int tidx = 0;
                while (tidx < (int)sizeof(tbuf)-1) {
                    int c = getchar_timeout_us(5000000);
                    if (c == PICO_ERROR_TIMEOUT) break;
                    if (c == '\r' || c == '\n') break;
                    if (c >= '0' && c <= '9') { tbuf[tidx++] = (char)c; printf("%c", (char)c); }
                }
                tbuf[tidx] = '\0';
                int val = atoi(tbuf);
                if (val > 0) { ramp_start_delay = (uint32_t)val; printf("Ramp start delay -> %u us\n", ramp_start_delay); }
                else { printf("Invalid value\n"); }
            }
            else if (ch == 'r') {
                const uint32_t startd = 3000;
                const uint32_t endd = 400;
                const uint32_t stepd = 100;
                const int test_steps = 64;
                printf("Starting interactive range test %u->%u step %u (press y if it MOVED)\n", startd, endd, stepd);
                for (uint32_t d = startd; d >= endd; d -= stepd) {
                    byj_set_speed(&motor, d);
                    printf("Test delay %u us: stepping %d steps...\n", d, test_steps);
                    byj_step_steps(&motor, test_steps);
                    byj_step_steps(&motor, -test_steps);
                    printf("Did it move at %u us? (y/n)\n", d);
                    while (true) {
                        int r = getchar_timeout_us(5000000);
                        if (r == PICO_ERROR_TIMEOUT) { printf("No input (timeout) - assuming 'n'\n"); break; }
                        if (r == 'y' || r == 'Y') { printf("USER: moved at %u us\n", d); break; }
                        if (r == 'n' || r == 'N') { printf("USER: no move at %u us\n", d); break; }
                    }
                }
                printf("Range test complete. Restoring previous delay %u us\n", motor.step_delay_us);
            }
            else if (ch == 'A') {
                printf("Auto-mapper: testing pin permutations (small steps)\n");
                // save original cfg and speed
                byj_gpio_config_t orig_cfg = motor.cfg;
                uint32_t orig_speed = motor.step_delay_us;
                bool orig_accel = accel_enabled;
                accel_enabled = false;
                const int test_steps = 16;
                const uint32_t test_delay = 3000;
                byj_set_speed(&motor, test_delay);

                std::array<int,4> perm = {0,1,2,3};
                int pins[4] = { (int)orig_cfg.pin0, (int)orig_cfg.pin1, (int)orig_cfg.pin2, (int)orig_cfg.pin3 };
                int best_moved = -1;
                std::array<int,4> best_map = perm;
                // ensure start order
                std::sort(perm.begin(), perm.end());
                do {
                    byj_gpio_config_t newcfg = { (uint)pins[perm[0]], (uint)pins[perm[1]], (uint)pins[perm[2]], (uint)pins[perm[3]] };
                    motor.cfg = newcfg; // apply mapping
                    sleep_ms(50);
                    int32_t before = motor.current_position;
                    byj_step_steps(&motor, test_steps);
                    int32_t after = motor.current_position;
                    int moved = abs(after - before);
                    // restore
                    byj_step_steps(&motor, -test_steps);
                    printf("perm [%d %d %d %d] -> moved %d\n", newcfg.pin0, newcfg.pin1, newcfg.pin2, newcfg.pin3, moved);
                    if (moved > best_moved) { best_moved = moved; best_map = perm; }
                    sleep_ms(50);
                } while (std::next_permutation(perm.begin(), perm.end()));

                // restore original cfg and speed
                motor.cfg = orig_cfg;
                byj_set_speed(&motor, orig_speed);
                accel_enabled = orig_accel;

                printf("Auto-mapper complete. Best moved=%d for mapping [%d %d %d %d]\n", best_moved,
                       pins[best_map[0]], pins[best_map[1]], pins[best_map[2]], pins[best_map[3]]);
                printf("To apply best mapping, press 'M' at the prompt (will set mapping)\n");
            }
            else if (ch == 'M') {
                // Apply last reported best mapping if present
                printf("Applying last reported best mapping not implemented persistently; run 'A' first and note mapping above.\n");
            }
            else if (ch == 'o') {
                motor.output_mode = (motor.output_mode == BYJ_OUTPUT_BIPOLAR) ? BYJ_OUTPUT_UNIPOLAR : BYJ_OUTPUT_BIPOLAR;
                printf("Output mode -> %s\n", motor.output_mode == BYJ_OUTPUT_BIPOLAR ? "BIPOLAR" : "UNIPOLAR");
            }
            else if (ch == 'p') {
                printf("Enter small step count then Enter:\n");
                char sbuf[16]; int sidx = 0;
                while (sidx < (int)sizeof(sbuf)-1) {
                    int c = getchar_timeout_us(5000000);
                    if (c == PICO_ERROR_TIMEOUT) break;
                    if (c == '\r' || c == '\n') break;
                    if (c >= '0' && c <= '9') { sbuf[sidx++] = (char)c; printf("%c", (char)c); }
                }
                sbuf[sidx] = '\0';
                int sc = atoi(sbuf);
                if (sc <= 0) { printf("Invalid step count\n"); }
                else {
                    int32_t before = motor.current_position;
                    printf("Stepping %d steps...\n", sc);
                    byj_step_steps(&motor, sc);
                    printf("Position before: %d after: %d (moved %d)\n", before, motor.current_position, motor.current_position - before);
                }
            }
            else if (ch == 'v') { sweep_steps = eff_steps_per_rev; printf("Sweep -> 1 rev (%d steps)\n", sweep_steps); }
            else if (ch == 'd') {
                printf("Enter degrees (0-360) then Enter:\n");
                char buf[16]; int idx = 0;
                while (idx < (int)sizeof(buf)-1) {
                    int c = getchar_timeout_us(5000000);
                    if (c == PICO_ERROR_TIMEOUT) break;
                    if (c == '\r' || c == '\n') break;
                    if (c >= '0' && c <= '9') { buf[idx++] = (char)c; printf("%c", (char)c); }
                }
                buf[idx] = '\0';
                int deg = atoi(buf);
                if (deg <= 0) { printf("Invalid degrees\n"); }
                else {
                    sweep_steps = (int32_t)((double)eff_steps_per_rev * ((double)deg / 360.0));
                    if (sweep_steps < 1) sweep_steps = 1;
                    printf("Sweep -> %d steps (%d deg)\n", sweep_steps, deg);
                }
            }
        }

        // blocking forward sweep (with optional acceleration ramp)
        printf("Sweep forward %d steps\n", sweep_steps);
        int32_t pos_before = motor.current_position;
        printf("Position before: %d\n", pos_before);
        if (accel_enabled) {
            int32_t final_target = motor.current_position + sweep_steps;
            uint32_t target = motor.step_delay_us;
            uint32_t start_delay = (target < ramp_start_delay) ? ramp_start_delay : target;
            bool ok = ramped_move(&motor, final_target, start_delay, target, ramp_segments, s_curve_enabled);
            if (!ok) { printf("Acceleration aborted due to stall, falling back to direct move\n"); byj_set_position(&motor, final_target); byj_wait_complete(&motor); }
        } else {
            byj_set_position(&motor, motor.current_position + sweep_steps);
            byj_wait_complete(&motor);
        }
        int32_t pos_after = motor.current_position;
        printf("Position after: %d (moved %d)\n", pos_after, pos_after - pos_before);
        sleep_ms(dwell_ms);

        // pre-sweep adjust window for backward
        int beff_steps_per_rev = (motor.mode == BYJ_MODE_HALF) ? BYJ48_STEPS_PER_REV : (BYJ48_STEPS_PER_REV / 2);
        double bdeg_per_step = 360.0 / (double)beff_steps_per_rev;
        printf("Preparing backward sweep: delay=%u us, sweep_steps=%d, mode=%d\n", motor.step_delay_us, sweep_steps, motor.mode);
        printf("Effective steps/rev=%d (deg/step=%.4f)\n", beff_steps_per_rev, bdeg_per_step);
        printf("Adjust settings now (3s)...\n");
        start = time_us_64();
        while ((time_us_64() - start) < 3000000ULL) {
            int ch = getchar_timeout_us(100000);
            if (ch == PICO_ERROR_TIMEOUT) continue;
            if (ch == '\r') printf("<received CR (13)>\n");
            else if (ch == '\n') printf("<received LF (10)>\n");
            else printf("<received '%c' (%d)>\n", (ch >= 32 && ch < 127) ? (char)ch : '?', ch);
            if (ch == '+') {
                uint32_t nd = (uint32_t)((float)motor.step_delay_us * 0.9f);
                if (nd < 100) nd = 100;
                byj_set_speed(&motor, nd);
                printf("Faster -> %u us\n", motor.step_delay_us);
            }
            else if (ch == '-') {
                uint32_t nd = (uint32_t)((float)motor.step_delay_us * 1.1f);
                byj_set_speed(&motor, nd);
                printf("Slower -> %u us\n", motor.step_delay_us);
            }
            else if (ch == '[') { uint32_t nd = (motor.step_delay_us > 200) ? motor.step_delay_us - 100 : 100; byj_set_speed(&motor, nd); printf("Faster -100 -> %u us\n", motor.step_delay_us); }
            else if (ch == ']') { byj_set_speed(&motor, motor.step_delay_us + 100); printf("Slower +100 -> %u us\n", motor.step_delay_us); }
            else if (ch == '<') { sweep_steps = (int32_t)(sweep_steps * 0.9f); printf("Sweep -> %d\n", sweep_steps); }
            else if (ch == '>') { sweep_steps = (int32_t)(sweep_steps * 1.1f); printf("Sweep -> %d\n", sweep_steps); }
            else if (ch == 'm') { if (motor.mode == BYJ_MODE_FULL_DOUBLE) motor.mode = BYJ_MODE_FULL_SINGLE; else if (motor.mode == BYJ_MODE_FULL_SINGLE) motor.mode = BYJ_MODE_HALF; else motor.mode = BYJ_MODE_FULL_DOUBLE; printf("Mode -> %d\n", motor.mode); }
            else if (ch == 'g') break;
            else if (ch == 'v') { sweep_steps = beff_steps_per_rev; printf("Sweep -> 1 rev (%d steps)\n", sweep_steps); }
            else if (ch == 'd') {
                printf("Enter degrees (0-360) then Enter:\n");
                char buf2[16]; int idx2 = 0;
                while (idx2 < (int)sizeof(buf2)-1) {
                    int c2 = getchar_timeout_us(5000000);
                    if (c2 == PICO_ERROR_TIMEOUT) break;
                    if (c2 == '\r' || c2 == '\n') break;
                    if (c2 >= '0' && c2 <= '9') { buf2[idx2++] = (char)c2; printf("%c", (char)c2); }
                }
                buf2[idx2] = '\0';
                int deg2 = atoi(buf2);
                if (deg2 <= 0) { printf("Invalid degrees\n"); }
                else {
                    sweep_steps = (int32_t)((double)beff_steps_per_rev * ((double)deg2 / 360.0));
                    if (sweep_steps < 1) sweep_steps = 1;
                    printf("Sweep -> %d steps (%d deg)\n", sweep_steps, deg2);
                }
            }
        }

        // blocking backward sweep (with optional acceleration ramp)
        printf("Sweep backward %d steps\n", sweep_steps);
        int32_t bpos_before = motor.current_position;
        printf("Position before: %d\n", bpos_before);
        if (accel_enabled) {
            int32_t final_target = motor.current_position - sweep_steps;
            uint32_t target = motor.step_delay_us;
            uint32_t start_delay = (target < ramp_start_delay) ? ramp_start_delay : target;
            bool ok = ramped_move(&motor, final_target, start_delay, target, ramp_segments, s_curve_enabled);
            if (!ok) { printf("Acceleration aborted due to stall, falling back to direct move\n"); byj_set_position(&motor, final_target); byj_wait_complete(&motor); }
        } else {
            byj_set_position(&motor, motor.current_position - sweep_steps);
            byj_wait_complete(&motor);
        }
        int32_t bpos_after = motor.current_position;
        printf("Position after: %d (moved %d)\n", bpos_after, bpos_after - bpos_before);
        sleep_ms(dwell_ms);
    }

    return 0;
}
