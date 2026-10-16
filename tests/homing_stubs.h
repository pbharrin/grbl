/* Minimal IO/planner boundary for executing limits_go_home() on the host. */
#define HOMING_AXIS_SEARCH_SCALAR 1.5f
#define HOMING_AXIS_LOCATE_SCALAR 5.0f
#define N_HOMING_LOCATE_CYCLE 1
#define PL_COND_FLAG_NO_FEED_OVERRIDE 2
#define HOMING_CYCLE_LINE_NUMBER 0
#define STEP_CONTROL_EXECUTE_SYS_MOTION 1
#define STEP_CONTROL_NORMAL_OP 0
#define EXEC_SAFETY_DOOR 1
#define EXEC_RESET 2
#define EXEC_CYCLE_STOP 4
#define EXEC_ALARM_HOMING_FAIL_TRAVEL 1
#define EXEC_ALARM_HOMING_FAIL_RESET 2
#define EXEC_ALARM_HOMING_FAIL_DOOR 3
#define EXEC_ALARM_HOMING_FAIL_PULLOFF 4
#define EXEC_ALARM_HOMING_FAIL_APPROACH 5
struct {
    bool abort;
    uint8_t homing_axis_lock[N_AXIS], step_control;
} sys;
struct {
    float max_travel[N_AXIS], steps_per_mm[N_AXIS];
    float homing_seek_rate, homing_feed_rate, homing_pulloff;
    uint8_t homing_dir_mask, homing_debounce_delay;
} settings;
typedef struct {uint8_t condition; int line_number; float feed_rate;} plan_line_data_t;
uint8_t sys_rt_exec_state, sys_rt_exec_alarm;
uint8_t active_mask, alarm_code;
unsigned planned;
float rates[8], targets[8][N_AXIS];
bool inject_reset, missing_switch;
uint8_t get_step_pin_mask(uint8_t axis) {
    const uint8_t pins[] = {1, 64, 8, 16, 2, 1};
    return pins[axis];
}
void limitPinInterruptsDisable(void) {}
void limitPinInterruptsEnable(void) {}
void system_set_exec_alarm(uint8_t alarm) {alarm_code=sys_rt_exec_alarm=alarm;}
void system_clear_exec_state_flag(uint8_t flag) {sys_rt_exec_state &= ~flag;}
void system_convert_array_steps_to_mpos(float *target, int32_t *position) {
    for (int i=0; i<N_AXIS; ++i) target[i]=position[i]/settings.steps_per_mm[i];
}
void plan_buffer_line(float *target, plan_line_data_t *data) {
    assert(planned < 8);
    rates[planned] = data->feed_rate;
    memcpy(targets[planned], target, sizeof(targets[planned]));
    ++planned;
}
void st_prep_buffer(void) {}
void st_wake_up(void) {
    for (int i=0; i<N_AXIS; ++i) {
        if (!(active_mask & bit(i))) assert(sys.homing_axis_lock[i] == 0);
        else assert(sys.homing_axis_lock[i] == get_step_pin_mask(i));
    }
    if (inject_reset) sys_rt_exec_state = EXEC_RESET;
    else if (missing_switch || planned%2 == 0) sys_rt_exec_state=EXEC_CYCLE_STOP;
}
uint8_t limits_get_stateMax(void) {
    return (!missing_switch && planned%2 == 1) ? active_mask : 0;
}
void st_reset(void) {}
void delay_ms(uint8_t delay) {(void)delay;}
void mc_reset(void) {}
void protocol_execute_realtime(void) {sys.abort=true;}
