#include "ros/ros.h"
#include "ALLEGROKIN.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "allegro_node");

    ros::NodeHandle nh;
    double frequency = 100.0;

    ROS_INFO("Starting the allegro kin node ...");

    ALLEGROKIN allegro_kin(nh, frequency);

    if (!allegro_kin.Init())
    {
        return -1;
    }
    else
    {
        ROS_INFO("Running the Allegro kinematics node ...");
        allegro_kin.Run();
    }


    return 0;
}

