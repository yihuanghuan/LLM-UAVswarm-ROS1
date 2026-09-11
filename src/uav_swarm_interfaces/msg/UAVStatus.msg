std_msgs/Header header
uint8 uav_id
uint32 mission_id
bool system_ready                 # Armed + Offboard + takeoff hold completed
bool is_hover_stable              # Current mission completion state
uint8 STABILITY_UNSTABLE=0
uint8 STABILITY_CANDIDATE=1
uint8 STABILITY_CONFIRMED=2
uint8 stability_state
float32 position_error
float32 speed                     # Filtered position-derived speed (decision signal)
float32 raw_ekf_speed             # PX4 EKF speed (diagnostic only)
uint8 STARTUP_WAIT_ESTIMATOR_READY=0
uint8 STARTUP_PRESTREAM=1
uint8 STARTUP_ARMING=2
uint8 STARTUP_SETTING_OFFBOARD=3
uint8 STARTUP_TAKING_OFF=4
uint8 STARTUP_READY=5
uint8 STARTUP_FAILED=6
uint8 startup_state
bool armed
bool offboard
bool failsafe
bool pre_flight_checks_pass
float32 altitude
