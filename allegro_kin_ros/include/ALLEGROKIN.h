#ifndef __ALLEGROKIN_H__
#define __ALLEGROKIN_H__

#include "ros/ros.h"
//#include "geometry_msgs/Pose.h"
// #include "geometry_msgs/Twist.h"
//#include "geometry_msgs/TwistStamped.h"
//#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float64.h"
//#include "nav_msgs/Path.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>
#include "sensor_msgs/JointState.h"
#include <ros/console.h>

// #include <vector>

// #include <mutex>

//#include "eigen3/Eigen/Dense"

using namespace std;
using namespace Eigen;

class ALLEGROKIN
{
public:
    Eigen::Vector3d X0_current_inRef_;
    Eigen::Vector3d X1_current_inRef_;
    Eigen::Vector3d X2_current_inRef_;
    Eigen::Vector3d X3_current_inRef_;

    std::vector<double> joint_values_finger_0_;
    std::vector<double> joint_values_finger_1_;
    std::vector<double> joint_values_finger_2_;
    std::vector<double> joint_values_finger_3_;

    bool found_ik0_;
    bool found_ik1_;
    bool found_ik2_;
    bool found_ik3_;

private:
    std::string path_to_model_;

    robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_;

    robot_state::JointModelGroup *joint_model_finger_0_;
    robot_state::JointModelGroup *joint_model_finger_1_;
    robot_state::JointModelGroup *joint_model_finger_2_;
    robot_state::JointModelGroup *joint_model_finger_3_;

    Eigen::Vector3d test_finger_1_;
    Eigen::Vector3d test_finger_2_;
    Eigen::Vector3d test_finger_3_;
    Eigen::Vector3d test_finger_4_;

    // ROS variables
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    ros::Publisher pub_forward;
    ros::Subscriber sub_joints;

    // for Forward Kinematics
    ros::Subscriber sub_joint_positions_;
    ros::Publisher pub_finger_tips_pose_;

    // for Inverse Kinematics
    ros::Subscriber sub_finger_tips_desired_velocity_;
    ros::Publisher pub_joint_desired_velocity_;

    // Update hand position
    float fk_joints[];
    double current_joint_position_[];

    double dt_;

    std_msgs::Float64 msg_gamma_;

public:
    ALLEGROKIN(ros::NodeHandle &n,
               double frequency);

    // ~ALLEGROKIN();

    bool Init();

    void Run();

private:
    bool InitializeModel();

    bool InitializeROS();

    void ComputeForwardKinematics();

    void UpdateJointPositions(const sensor_msgs::JointState &msg);

    void ComputeInverseKinematics(const Eigen::Vector3d &tip_0_pos,
                                  const Eigen::Vector3d &tip_1_pos,
                                  const Eigen::Vector3d &tip_2_pos,
                                  const Eigen::Vector3d &tip_3_pos);
};

#endif