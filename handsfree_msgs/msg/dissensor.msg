std_msgs/Header header
uint16[] ult
uint16[] laser
uint16[] drop
int32 collision
float32 uwb_rssi
float32 uwb_distance
float32 ibeacon_rssi
float32 ibeacon_distance

#0(not click) 1(click) 2(double-click) 3(long-press)
uint8 button1
uint8 button2

#0 1(charging) 2(Charged)
uint8 atuo_charger_state 
uint8 hand_charger_state

#unit: 20~2000mm 
uint16 charger_distance 

#0(not find people) 1(find people)
uint8 thermal_infrared

uint8 vcc_motor_state
uint8 vcc_pc_state
uint8 pc_boot_up_state
uint8 break_stop_state

uint8 control_quality
uint8 chassis_online
uint8 col_drop_alarm
uint8 over_speed_state
uint8 motor1_online_state
uint8 motor2_online_state
uint8 motor1_mode_state
uint8 motor2_mode_state
uint8 motor1_fault_state
uint8 motor2_fault_state

uint8 mqtt_online
uint8 mqtt_get_topic_state
uint8 mqtt_command

