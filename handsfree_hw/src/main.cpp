#include <handsfree_hw/hf_hw_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robothw");
    ros::NodeHandle nh("handsfree");
    std::string config_filename = "/config.txt" ;
    std::string config_filepath = CONFIG_PATH+config_filename ; 
    std::cerr<<"the configure file path is: "<<config_filepath<<std::endl ; 
    handsfree_hw::HF_HW_ros hf(nh, "serial:///dev/ttyUSB0", config_filepath);
    hf.mainloop();
    return 0;
}
