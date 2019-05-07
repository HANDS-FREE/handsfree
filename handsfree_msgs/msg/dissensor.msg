std_msgs/Header header
uint16[] ult
uint16[] laser
uint16[] drop
int32 collision
float32 uwb_rssi
float32 uwb_distance
float32 ibeacon_rssi
float32 ibeacon_distance

#0(not click) 1(click) 2(double-click)
uint8 start_button
uint8 gohome_button

#0 1(charging) 2(Charged)
uint8 atuo_charger_state 
uint8 hand_charger_state

#unit: 20~200mm 
uint16 charger_distance 
