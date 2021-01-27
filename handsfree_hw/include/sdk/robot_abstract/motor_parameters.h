#ifndef MOTOR_PARAMETERS_H
#define MOTOR_PARAMETERS_H

enum MotorDriverType : unsigned char
{
    MotorDriver_PWM12_AND_IO = 0,
    MotorDriver_PWM_AND_IOAB = 1,
    MotorDriver_ZLAC706 = 2,
    MotorDriver_TDE124 = 3,
    MotorDriver_SCOUT = 4,
};

typedef struct{
    float p1;                         //1 is pid mileage loop
    float i1;
    float d1;
    float p2;                         //2 is pid speed loop
    float i2;
    float d2;
    float p3;                         //2 is pid current loop
    float i3;
    float d3;
}__attribute__((packed)) MotorPID;

typedef struct {
    unsigned char enable_flag;
    unsigned char online_state;
    unsigned char mode_state;
    // 0~7 bit(high enable): over current, over voltage , encoder error , under voltage , overload
    unsigned char fault_state;
    unsigned char stop_fault_motor;
    float expect_angle_speed;          //degree/s
    float expect_angle_speed_filter;   //degree/s
    float expect_unit_encoder;
    float expect_total_encoder;
    float measure_unit_encoder;
    float measure_total_encoder;
    float measure_angle_speed;   //degree/s
    float d_past_angle;    //recording d angle for robot coordinate calculation
    float past_total_angle;
    float motor_current;
    float motor_voltage;
    float pwm_output;
}__attribute__((packed)) MotorControlData;

/*****************************************************************************************/

typedef struct {
    //unsigned char motor_id;
    MotorDriverType driver_type;
    unsigned char motor_enable_num;
    unsigned char simulation_model;
    float pid_t;
    float encoder_num;       //the encoder sensor count when the  motor turning one circle
    float pwm_max;           //set the max value for pwm
    float pwm_dead_zone;     //when the pwm in this zone , the motor disable
    float speed_low_filter;  //0~1;  default = 0.3
    float protect_current;   //unit : A default = 1
    float static_damping_coefficient;  //0~0.3;  default = 0.05
    MotorPID pid;
}__attribute__((packed)) MotorParameters;

typedef struct {
    MotorControlData m1;
    MotorControlData m2;
    MotorControlData m3;
    MotorControlData m4;
}__attribute__((packed)) MotorsControlData;

#endif // MOTOR_PARAMETERS_H
