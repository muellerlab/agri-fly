float64 desired_yaw
geometry_msgs/Vector3 position_estimate_W
geometry_msgs/Vector3 velocity_estimate_W
geometry_msgs/Quaternion attitude_estimate_W

uint64 trajectory_id 
time                  trajectory_time

# redundant with trajectory parameters and trajectory time
geometry_msgs/Vector3 position_reference_W
geometry_msgs/Vector3 velocity_reference_W
geometry_msgs/Vector3 acceleration_reference_W
geometry_msgs/Vector3 angular_velocity_reference_B
float64               thrust_reference_B

float64               current_battery
