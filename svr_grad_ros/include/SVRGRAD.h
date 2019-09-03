#ifndef __SVRGRAD_H__
#define __SVRGRAD_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
// #include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int8.h"
// #include <Eigen/Dense>


#include "svm_grad.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// #include <vector>

#include "MathLib.h"
#include "geometry_msgs/Pose.h"
// #include "GMRDynamics.h"
// #include "CDDynamics.h"

// #include <mutex>

// #include "Eigen.h"
// #include <dynamic_reconfigure/server.h>
// #include <ds_motion_generator/SED_paramsConfig.h>


using namespace std;
using namespace Eigen;

class SVRGRAD
{


private:

    std::string path_svr_;
    std::string input_topic_pose_name_;
    std::string output_topic_velocity_name_;
    std::string output_topic_gamme_name_;


    // ROS variables
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;



    ros::Subscriber sub_real_pose_;
    ros::Publisher pub_desired_twist_;
    ros::Publisher pub_desired_twist_thumb_;
    ros::Publisher pub_svr_gamma_;
    ros::Publisher pub_svr_gamma_thumb_;
    ros::Publisher pub_DesiredPath_;
    ros::Publisher _pub_DesiredPath;    



	SVMGrad SVM;

    double dt_;


    // DS variables
    // std::unique_ptr<GMRDynamics> SED_GMM_;

    // int K_gmm_;
    // int dim_;
    // std::vector<double> Priors_;
    // std::vector<double> Mu_;
    // std::vector<double> Sigma_;
    // std::vector<double> attractor_;

    // double max_desired_vel_;

    // // Filter variables
    // std::unique_ptr<CDDynamics> CCDyn_filter_;

    // double Wn_;
    // MathLib::Vector accLimits_;
    // MathLib::Vector velLimits_;




    double gamma_;

    std_msgs::Float64 msg_gamma_;


    geometry_msgs::Pose msg_real_pose_;
    geometry_msgs::TwistStamped msg_desired_velocity_;
    // geometry_msgs::TwistStamped msg_desired_velocity_filtered_;

    nav_msgs::Path msg_DesiredPath_;
    int MAX_FRAME = 100;

    // test of circular path
    nav_msgs::Path _msg_DesiredPath;
    int _MAX_FRAME = 50;
    int _MAX_FRAME_circ = 200;
    Eigen::Vector2d _new_ds_simulation;
    double _dt = 200;


    // //dynamic reconfig settig
    // dynamic_reconfigure::Server<ds_motion_generator::SED_paramsConfig> dyn_rec_srv_;
    // dynamic_reconfigure::Server<ds_motion_generator::SED_paramsConfig>::CallbackType dyn_rec_f_;


    // // Class variables
    // std::mutex mutex_;

    Eigen::Vector3d real_pose_;
    Eigen::Vector3d old_pose_;

    // MathLib::Vector target_pose_;
    // MathLib::Vector target_offset_;


    Eigen::Vector3d svr_desired_velocity_;
    Eigen::Vector3d attractor_desired_velocity_;
    Eigen::Vector3d desired_velocity;

    // MathLib::Vector desired_velocity_filtered_;

    // double scaling_factor_;
    // double ds_vel_limit_;

    // Thumb gamma 
    tf::TransformListener listener_;
    tf::StampedTransform transform_;
    double gamma_thumb_;
    std_msgs::Float64 msg_gamma_thumb_;
    Eigen::Vector3d thumb_pose_;
    Eigen::Vector3d svr_desired_velocity_thumb_; 
    geometry_msgs::TwistStamped msg_desired_velocity_thumb_;

    // Grab
    int _grab, _direction;
    ros::Subscriber _subGrab;                         // Subscriber for grab topic for hand
    ros::Subscriber _subDirection;                         // Subscriber for grab topic for hand

    // Arrow
    ros::Publisher _marker_pub;
    uint32_t _shape;
    Eigen::Vector3d SVRPose_;
    Eigen::VectorXd SVRQuat_;
    ros::Subscriber _subSVRQuat;
    ros::Publisher _marker_pub2;
    Eigen::VectorXd TotQuat_;
    Eigen::Vector3d TotPose_;
    ros::Subscriber _subTotQuat;


public:
    SVRGRAD(ros::NodeHandle &n,
            double frequency,
            std::string path_to_file,
            std::string input_topic_position,
            std::string output_topic_velocity,
            std::string output_topic_gamma);

    bool Init();

    void Run();

private:

    bool InitializeSVR();

    bool InitializeROS();

    void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

    void ComputeDesiredVelocity();

    void PublishDesiredVelocity();

    void ComputeDesiredVelocityThumb();

    void PublishDesiredVelocityThumb();

    void ComputeGamma();
    
    void PublishGamma();

    void ComputeGammaThumb();

    void PublishGammaThumb();

    void PublishFuturePath();

    void LookUpTF();

    void PublishFuturePath2();

    void ds_simulation(Eigen::Vector2d x, double r_value);

    void updateGrabState(const std_msgs::Int8 &msg);
    
    void updateDirectionState(const std_msgs::Int8 &msg);

    void showSVRArrow();

    void showTotArrow();
    
    void quat_from_vec();

    void UpdateSVRQuat(const geometry_msgs::Pose::ConstPtr &msg);

    void UpdateTotQuat(const geometry_msgs::Pose::ConstPtr &msg);


};


#endif