#include "RGBD_Capture.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "RGBDCapture");
    ros::NodeHandle n;
    
    boost::shared_ptr<RGBDCapture> capture_node(new RGBDCapture(n));

    ros::spin();

    return 0;

}
