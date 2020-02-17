#include <handsfree_hw/hf_hw_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robothw");
    ros::NodeHandle nh("handsfree");
    ros::NodeHandle nh_private("~");

    std::string robolink_config_file;
    nh_private.param<std::string>("robolink_config_file",robolink_config_file,"");
    std::cerr<<"the configure file path is: "<<robolink_config_file<<std::endl ; 

    while (ros::ok())
    {
        std::string serial_port;
        nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");

        if((access(serial_port.c_str(),0)) != -1)
        {
            std::string serial_port_path="serial://" + serial_port;
            std::cerr<<"the serial_port is: "<<serial_port_path<<std::endl ;
            handsfree_hw::HF_HW_ros hf(nh, serial_port_path , robolink_config_file);
            hf.mainloop();
        }
        else
        {
           std::cerr<<"not find serial_port: "<<serial_port<<std::endl ;
        }
        usleep(2000000);
    }

    return 0;
}
