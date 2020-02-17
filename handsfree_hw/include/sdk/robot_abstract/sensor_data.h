#ifndef SENSOR_DATA
#define SENSOR_DATA

typedef struct{
    unsigned short int year;
    unsigned char month;
    unsigned char date;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
}__attribute__((packed)) UtcTime;

typedef struct{
    UtcTime uct_time;
    unsigned char satellite_num;
    float altitude;     //unit : m
    float ground_speed;   //unit: m/s
    unsigned int latitude;         //纬度 分扩大100000倍,实际要除以100000
    unsigned char nshemi;	     //北纬/南纬,N:北纬;S:南纬
    unsigned int longitude;	     //经度 分扩大100000倍,实际要除以100000
    unsigned char ewhemi;	     //东经/西经,E:东经;W:西经
}__attribute__((packed)) GPSData;

/*****************************************************************************************/
//IMU (pitch,roll,yaw) (radian,radian,radian) 10HZ ~ 50HZ
typedef struct{
    unsigned char online;
    float frequency;
    float pitch;
    float roll;
    float yaw_gyro;
    float bar_altitude;    //unit : m
    float magnetic_angle;
    float magnetic_fusion_gyro_angle;
}__attribute__((packed)) IMUSensorData;

//10HZ
typedef struct{
    unsigned short int ult[12];  //unit: mm
    unsigned short int laser[12]; //unit: mm
    unsigned short int drop[4]; //unit: mm
    int collision;  //collision bars , 0~31 bit Represents 32 collision points
    float uwb_rssi;
    float uwb_distance;  //unit: m
    float ibeacon_rssi;  //unit: m
    float ibeacon_distance;
    unsigned char button1;   //0(not click) 1(click) 2(double-click)
    unsigned char button2;  //0(not click) 1(click) 2(double-click)
    unsigned char atuo_charger_state; //0 1(charging) 2(Charged)
    unsigned char hand_charger_state; //0 1(charging) 2(Charged)
    unsigned short int charger_distance; //unit: 20~2000mm

    unsigned char thermal_infrared;

    unsigned char vcc_motor_state;
    unsigned char vcc_pc_state;
    unsigned char pc_boot_up_state;
    unsigned char break_stop_state;

    unsigned char control_quality;
    unsigned char chassis_online;
    unsigned char col_drop_alarm;
    unsigned char over_speed_state;
    unsigned char motor1_online_state;
    unsigned char motor2_online_state;
    unsigned char motor1_mode_state;
    unsigned char motor2_mode_state;
    unsigned char motor1_fault_state;
    unsigned char motor2_fault_state;

    unsigned char mqtt_online;
    unsigned char mqtt_get_topic_state;
    unsigned char mqtt_command;
}__attribute__((packed)) IOSensorData;

/*****************************************************************************************/
typedef struct{
    IMUSensorData imu_data;
    IOSensorData disio_data;
    GPSData gps_data;
}__attribute__((packed)) SensorsData;

#endif // SENSOR_DATA
