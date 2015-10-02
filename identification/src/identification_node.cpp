#include "identification.h"

static bool debugMode = false;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "identification");
    ros::NodeHandle n;

    string library_address = string(_SOURCE_DIRECTORY_) + "/yml_library/yml_library";
    string params_directory = string(_SOURCE_DIRECTORY_) + "/params";

    DIR* dir = opendir(library_address.c_str());
    if (!dir) {
        ROS_ERROR("Cannot find YML library at default location (%s). Node terminating.", library_address.c_str());
        return 1;
    } else closedir(dir);

    boost::shared_ptr < objectIdentifier > identifier_node (new objectIdentifier (n, library_address, params_directory));

    ros::spin();

    return 0;

}
