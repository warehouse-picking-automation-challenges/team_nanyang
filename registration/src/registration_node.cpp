#include "registration.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "registration");
    ros::NodeHandle n;
    
    boost::shared_ptr<PoseEstimator> registration_node(new PoseEstimator(n));

    ros::spin();

    return 0;

}
