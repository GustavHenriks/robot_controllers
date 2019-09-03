#include "ros/ros.h"
#include "SVRGRAD.h"


// #include <vector>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "svr_grad_node");

    ros::NodeHandle nh;
    double frequency = 100.0;


    // Parameters
    std::string input_position;
    std::string output_velocity;
    std::string output_gamma;


    std::string path_svr_file;



    if (!nh.getParam("input_position", input_position))
    {
        ROS_ERROR("Couldn't retrieve the topic name for the input position. ");
        // return -1;
    }

    if (!nh.getParam("output_velocity", output_velocity))
    {
        ROS_ERROR("Couldn't retrieve the topic name for the output velocity. ");
        // return -1;
    }

    if (!nh.getParam("output_gamma", output_gamma))
    {
        ROS_ERROR("Couldn't retrieve the topic name for the output gamma. ");
        // return -1;
    }

    if (!nh.getParam("path_svr_file", path_svr_file))
    {
        ROS_ERROR("Couldn't retrieve the topic name for the output gamma. ");
        // return -1;
    }


    ROS_INFO("Starting the SVR grad node ...");

    SVRGRAD svr_grad(nh, frequency,
                     path_svr_file,
                     input_position,
                     output_velocity,
                     output_gamma);
    if (!svr_grad.Init())
    {
        return -1;
    }
    else
    {
        ROS_INFO("Running the SVR grad node ...");
        svr_grad.Run();
    }


    return 0;
}

