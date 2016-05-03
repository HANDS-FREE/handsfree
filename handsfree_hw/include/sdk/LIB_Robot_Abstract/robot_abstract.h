#ifndef ROBOT_ABSTRACT_H
#define ROBOT_ABSTRACT_H

//robot type
// 2: 2 wheel robot
//    balance car
//    differential car such as turtlebot
//    raider
// 3: 3 universal wheel robot
// 4: 4 mecanum wheels wheel robot

#define  ROBOT_WHEEL_MODEL    3

#if ROBOT_WHEEL_MODEL == 2

#endif

#if ROBOT_WHEEL_MODEL == 3

#endif

#if ROBOT_WHEEL_MODEL == 4


#endif

//MSG struct is the  unit of communication , also is the unit of robot abstract
struct MSGServo3{
    float  servo1;
    float  servo2;
    float  servo3;};

struct MSGServo4{
    float  servo1;
    float  servo2;
    float  servo3;
    float  servo4;};

struct MSGServo5{
    float  servo1;
    float  servo2;
    float  servo3;
    float  servo4;
    float  servo5;};

struct MSGServo6{
    float  servo1;
    float  servo2;
    float  servo3;
    float  servo4;
    float  servo5;
    float  servo6;};

struct MSGCoord{
    float  x;
    float  y;
    float  z;};

struct MSGPose{
    float  pitch;
    float  roll;
    float  yaw;};


struct MSGSystemInfo{
    float  system_time;
    float  cpu_temperature;
    float  cpu_usage;
    float  battery_voltage;
};

struct MSGRobotParameters{
    float robot_wheel_radius;
    float robot_body_radius;
    float speed_low_filter;
};

struct MSGMotorParameters{
    float p1;
    float i1;
    float d1;
    float p2;
    float i2;
    float d2;
};

//unit  distances : metres
//      angle： radian
class RobotAbstract
{
public:
    RobotAbstract()
    {
        expect_global_speed.x=0;
        expect_global_speed.y=0;
        expect_global_speed.z=0;

        measure_global_speed.x=0;
        measure_global_speed.y=0;
        measure_global_speed.z=0;

        expect_robot_speed.x=0;
        expect_robot_speed.y=0;
        expect_robot_speed.z=0;

        measure_robot_speed.x =0;
        measure_robot_speed.y =0;
        measure_robot_speed.z =0;

        expect_motor_speed.servo1=0;
        expect_motor_speed.servo2=0;
        expect_motor_speed.servo3=0;

        measure_motor_speed.servo1=0;
        measure_motor_speed.servo2=0;
        measure_motor_speed.servo3=0;

        measure_motor_mileage.servo1=0;
        measure_motor_mileage.servo2=0;
        measure_motor_mileage.servo3=0;

        measure_global_coordinate.x=0;
        measure_global_coordinate.y=0;
        measure_global_coordinate.z=0;

        measure_robot_coordinate.x=0;
        measure_robot_coordinate.y=0;
        measure_robot_coordinate.z=0;

        expect_arm1_state.servo1=0;
        expect_arm1_state.servo2=0;
        expect_arm1_state.servo3=0;
        expect_arm1_state.servo4=0;
        expect_arm1_state.servo5=0;
        expect_arm1_state.servo6=0;

        measure_arm1_state.servo1=0;
        measure_arm1_state.servo2=0;
        measure_arm1_state.servo3=0;
        measure_arm1_state.servo4=0;
        measure_arm1_state.servo5=0;
        measure_arm1_state.servo6=0;

        expect_arm2_state.servo1=0;
        expect_arm2_state.servo2=0;
        expect_arm2_state.servo3=0;
        expect_arm2_state.servo4=0;
        expect_arm2_state.servo5=0;
        expect_arm2_state.servo6=0;

        measure_arm2_state.servo1=0;
        measure_arm2_state.servo2=0;
        measure_arm2_state.servo3=0;
        measure_arm2_state.servo4=0;
        measure_arm2_state.servo5=0;
        measure_arm2_state.servo6=0;

        expect_head1_state.pitch=0;
        expect_head1_state.yaw=0;
        measure_head1_state.pitch=0;
        measure_head1_state.yaw=0;

        expect_head2_state.pitch=0;
        expect_head2_state.yaw=0;
        measure_head2_state.pitch=0;
        measure_head2_state.yaw=0;


        measure_imu_euler_angle.pitch=0;
        measure_imu_euler_angle.roll=0;
        measure_imu_euler_angle.yaw=0;

        robot_system_info.battery_voltage=0;
        robot_system_info.cpu_temperature=0;
        robot_system_info.cpu_usage=0;
        robot_system_info.system_time=0;

#if ROBOT_WHEEL_MODEL == 2
        robot_parameters.robot_wheel_radius=0.0325;
        robot_parameters.robot_body_radius=0.161;
#endif
#if ROBOT_WHEEL_MODEL == 3
        robot_parameters.robot_wheel_radius=0.029;
        robot_parameters.robot_body_radius=0.161;
#endif
        robot_parameters.speed_low_filter=0.4;
    }
    /***Robot***/
    //Chassis
    MSGCoord   expect_global_speed , measure_global_speed ;
    MSGCoord   expect_robot_speed , measure_robot_speed ;
    MSGServo3  expect_motor_speed , measure_motor_speed , measure_motor_mileage;
    MSGCoord   measure_global_coordinate ;
    MSGCoord   measure_robot_coordinate;
    //arm
    MSGServo6 expect_arm1_state , measure_arm1_state , expect_arm2_state , measure_arm2_state;
    //head
    MSGPose   expect_head1_state , measure_head1_state  ,expect_head2_state , measure_head2_state;
    /***Sensors***/
    //IMU
    MSGPose   measure_imu_euler_angle;
    /***others***/
    MSGSystemInfo robot_system_info;
    MSGRobotParameters robot_parameters;
    MSGMotorParameters motor_parameters;
};

#endif // ROBOT_ABSTRACT_H

