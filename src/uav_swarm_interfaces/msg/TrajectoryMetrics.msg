std_msgs/Header header

uint8 uav_id
geometry_msgs/Point start_pos
geometry_msgs/Point target_pos

float32 requested_duration
float32 trajectory_duration
string motion_style

float32 path_length
float32 max_velocity
float32 max_acceleration
float32 max_jerk
float32 integrated_squared_jerk

float32 elapsed_time
float32 arrival_time_error
float32 final_position_error

bool is_finished
bool is_hover_stable
