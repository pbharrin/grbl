"""Compile and exercise the firmware's actual homing function with simulated IO.

No AVR timers, electrical inputs, or physical motion are simulated. The planner
position-copy test extracts the actual system-motion initialization block.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def run_c(source):
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp) / 'test.c'
        path.write_text(source)
        executable = Path(temp) / 'test'
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c99', '-Wall',
                        '-Wextra', '-Werror', '-O1', str(path), '-lm',
                        '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


COMMON = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#define N_AXIS 6
#define COREBC
#define AXIS_1 0
#define AXIS_2 1
#define AXIS_3 2
#define AXIS_4 3
#define AXIS_5 4
#define AXIS_6 5
#define E_MOTOR AXIS_5
#define F_MOTOR AXIS_6
#define bit(i) (1U << (i))
#define bit_istrue(v,m) ((v) & (m))
#define max(a,b) ((a) > (b) ? (a) : (b))
#define PL_COND_FLAG_SYSTEM_MOTION 1
int32_t sys_position[N_AXIS];
int32_t system_convert_corebc_to_b_axis_steps(int32_t *p) {return (p[4]+p[5])/2;}
int32_t system_convert_corebc_to_c_axis_steps(int32_t *p) {return (p[4]-p[5])/2;}
'''


class HomingTests(unittest.TestCase):
    def test_system_motion_positions_and_normal_jog_positions(self):
        source = (ROOT / 'planner.c').read_text()
        start = source.index('// Copy position data based on type of motion')
        end = source.index('  #ifdef COREXY\n    target_steps', start)
        initialization = source[start:end]
        run_c(COMMON + r'''
struct {int32_t position[N_AXIS];} pl;
struct {uint8_t condition;} block_data;
void copy_positions(bool system_motion, int32_t *result) {
    int32_t position_steps[N_AXIS];
    // Deterministically expose any entries the actual code fails to initialize.
    for (int i=0; i<N_AXIS; ++i) position_steps[i] = 1234567;
    block_data.condition = system_motion ? PL_COND_FLAG_SYSTEM_MOTION : 0;
    typeof(block_data) *block = &block_data;
''' .replace('typeof(block_data)', '__typeof__(block_data)') + initialization + r'''
    memcpy(result, position_steps, sizeof(position_steps));
}
int main(void) {
    int32_t physical[] = {120, -300, 440, 60, 800, 200};
    int32_t expected[] = {120, -300, 440, 60, 500, 300};
    int32_t result[N_AXIS];
    memcpy(sys_position, physical, sizeof(physical));
    copy_positions(true, result);
    assert(memcmp(result, expected, sizeof(expected)) == 0);
    for (int i=0; i<N_AXIS; ++i) pl.position[i] = i * 37 - 40;
    copy_positions(false, result);
    assert(memcmp(result, pl.position, sizeof(result)) == 0);
    return 0;
}
''')

    def test_homing_passes_rates_axis_masks_direction_and_abort(self):
        source = (ROOT / 'limits.c').read_text()
        run_c(COMMON + (ROOT / 'tests/homing_stubs.h').read_text() +
              function(source, 'static uint8_t axislock_active(') + '\n' +
              function(source, 'void limits_go_home(') + r'''
void prepare(uint8_t mask) {
    memset(&sys, 0, sizeof(sys));
    memset(sys_position, 0, sizeof(sys_position));
    memset(&settings, 0, sizeof(settings));
    for (int i=0; i<N_AXIS; ++i) {
        settings.max_travel[i] = -360;
        settings.steps_per_mm[i] = 60;
        // Poison masks to check inactive axes are cleared on every pass.
        sys.homing_axis_lock[i] = 255;
    }
    settings.homing_seek_rate = 60;
    settings.homing_feed_rate = 15;
    settings.homing_pulloff = 5;
    active_mask=mask; planned=0; alarm_code=0; sys_rt_exec_state=0;
    inject_reset=false; missing_switch=false;
}
int main(void) {
    const uint8_t masks[] = {bit(0), bit(1), bit(2), bit(0)|bit(1)};
    for (unsigned test=0; test<sizeof(masks); ++test) {
        for (int reverse=0; reverse<2; ++reverse) {
            prepare(masks[test]);
            settings.homing_dir_mask = reverse ? active_mask : 0;
            limits_go_home(active_mask);
            assert(!alarm_code && !sys.abort);
            assert(planned == 4);
            float factor = test == 3 ? sqrtf(2) : 1;
            const float expected[] = {60, 60, 15, 60};
            for (int phase=0; phase<4; ++phase) {
                assert(fabsf(rates[phase] - expected[phase]*factor) < 0.001f);
                for (int axis=0; axis<N_AXIS; ++axis) {
                    if (!(active_mask & bit(axis))) assert(targets[phase][axis] == 0);
                    else {
                        bool positive = (phase%2 == 0) != reverse;
                        assert((targets[phase][axis] > 0) == positive);
                    }
                }
            }
        }
    }
    prepare(bit(0)); inject_reset=true;
    limits_go_home(active_mask);
    assert(sys.abort && alarm_code == EXEC_ALARM_HOMING_FAIL_RESET);
    prepare(bit(0)); missing_switch=true;
    limits_go_home(active_mask);
    assert(sys.abort && alarm_code == EXEC_ALARM_HOMING_FAIL_APPROACH);
    return 0;
}
''')


if __name__ == '__main__':
    unittest.main()
