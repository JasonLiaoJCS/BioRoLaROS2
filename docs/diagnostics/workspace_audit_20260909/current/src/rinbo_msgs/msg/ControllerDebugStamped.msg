Header header

string controller_state
float64 tau
float64 ratio
uint32 cycle_count
bool group_b_active
bool safety_stopped

bool[6] group_b_leg
float32 min_bus_voltage
float32 max_current

float32[6] target_position
float32[6] actual_position
float32[6] target_velocity
float32[6] actual_velocity
float32[6] position_error
float32[6] raw_pwm
float32[6] limited_pwm

string safety_reason
