#ifndef SYSTEM_INFO_H
#define SYSTEM_INFO_H

typedef struct {
    float  system_time;
    float  cpu_temperature;
    float  cpu_usage;
    float  battery_voltage;
    float  power_remain; // 0% ~ 100%
}SystemInfo;

#endif // SYSTEM_INFO_H
