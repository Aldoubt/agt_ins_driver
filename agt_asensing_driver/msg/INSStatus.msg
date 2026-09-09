std_msgs/Header header

uint8 ins_status
uint8 position_type
uint8 heading_type
uint8 num_satellite

bool position_status_valid
bool position_std_valid
bool velocity_std_valid
bool attitude_std_valid
bool gps_week_valid
bool temperature_valid
bool wheel_speed_status_valid

# NavSatFix-compatible ENU position standard deviations.
float32 east_std
float32 north_std
float32 up_std
# Backward-compatible scalar summary: max(east_std, north_std, up_std).
float32 position_std

float32 north_velocity_std
float32 east_velocity_std
float32 ground_velocity_std

float32 roll_std
float32 pitch_std
float32 yaw_std
# Backward-compatible heading summary; currently yaw_std when attitude std is valid.
float32 heading_std

bool rtk_fixed
uint32 gps_week
uint32 gps_time_ms
float32 temperature
uint8 wheel_speed_status
