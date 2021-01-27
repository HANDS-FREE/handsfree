#ifndef SYSTEM_INFO_H
#define SYSTEM_INFO_H

typedef struct {
    char robot_name[24];
    char robot_description[64];
    float firmware_version;
}__attribute__((packed)) RobotInfo;

typedef struct {
    unsigned char battery_series;
    float battery_voltage_alarm;
    unsigned char power_remain_alarm;
}__attribute__((packed)) SystemParameters;

typedef struct{
    unsigned char valid;
    unsigned short int year;
    unsigned char month;
    unsigned char date;
    unsigned char week;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
}__attribute__((packed)) LocalTime;

typedef struct{
    unsigned char valid;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
}__attribute__((packed)) WorkTime;

//1HZ
typedef struct {
    float system_time;
    float cpu_temperature;
    float cpu_usage;
    float battery_voltage;
    float power_remain; // 0 ~ 100 (%)
    LocalTime local_time;
    WorkTime work_time1;
    WorkTime work_time2;
}__attribute__((packed)) SystemInfo;

#endif // SYSTEM_INFO_H
