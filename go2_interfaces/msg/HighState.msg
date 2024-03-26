
uint8[2] head
uint8 level_flag
uint8 frame_reserve

uint32[2] sn
uint32[2] version
uint16 band_width
IMU imu
MotorState[20] motor_state
BmsState bms
int16[4] foot_force
int16[4] foot_force_est
uint8 mode
float32 progress
uint8 gait_type		   
float32 foot_raise_height		  
float32[3] position 
float32 body_height			  
float32[3] velocity 
float32 yaw_speed				   
float32[4] range_obstacle
Cartesian[4] foot_position2_body 
Cartesian[4] foot_speed2_body	
uint8[40] wireless_remote
uint32 reserve

uint32 crc