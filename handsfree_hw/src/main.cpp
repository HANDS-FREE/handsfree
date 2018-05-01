#include <handsfree_hw/hf_hw_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robothw");
    ros::NodeHandle nh("handsfree");

    FILE *pf = popen("rospack find handsfree_hw", "r");
    char res[1024];
    fread(res,1024, 1, pf);
    pclose(pf);
    std::string configfile=res;
    configfile.erase(configfile.end() - 1);
    configfile=configfile+"/config.txt";
    std::cerr << "config path:" << configfile <<std::endl;

    handsfree_hw::HF_HW_ros hf(nh, "serial:///dev/ttyUSB0", configfile);
    hf.mainloop();
    return 0;
}
