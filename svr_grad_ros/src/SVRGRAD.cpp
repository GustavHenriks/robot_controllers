#include "SVRGRAD.h"


SVRGRAD::SVRGRAD(ros::NodeHandle &n,
                 double frequency,
                 std::string path_svr_file,
                 std::string input_position,
                 std::string output_velocity,
                 std::string output_gamma)
    : nh_(n),
      loop_rate_(frequency),
      path_svr_(path_svr_file),
      input_topic_pose_name_(input_position),
      output_topic_velocity_name_(output_velocity),
      output_topic_gamme_name_(output_gamma),
      dt_(1 / frequency)
{

    ROS_INFO_STREAM("DSGRAD node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}



bool SVRGRAD::Init()
{

    // real_pose_.Resize(3);
    // svr_desired_velocity_.Resize(3);

    if (!InitializeSVR())
    {
        ROS_ERROR_STREAM("ERROR intializing the SVR");
        return false;
    }

    if (!InitializeROS())
    {
        ROS_ERROR_STREAM("ERROR intializing the ROS");
        return false;
    }

    return true;
}


bool SVRGRAD::InitializeSVR()
{



    real_pose_.resize(3);
    svr_desired_velocity_.resize(3);
    real_pose_.Zero();
    svr_desired_velocity_.Zero();
    gamma_ = 0;



    ROS_INFO_STREAM("Loading SVM model from " << path_svr_);
    SVM.loadModel(path_svr_);
    ros::Duration(1).sleep();


    // SED_GMM_.reset (new GMRDynamics(K_gmm_, dim_, dt_, Priors_, Mu_, Sigma_ ));
    // SED_GMM_->initGMR(0, 2, 3, 5 );


    // target_offset_.Resize(3);
    // target_pose_.Resize(3);

    // for (int i = 0; i < attractor_.size(); i++)
    // {
    //     target_pose_(i) = attractor_[i];
    // }


    // // initializing the filter
    // CCDyn_filter_.reset (new CDDynamics(3, dt_, Wn_));

    // // we should set the size automagically
    // velLimits_.Resize(3);
    // CCDyn_filter_->SetVelocityLimits(velLimits_);

    // accLimits_.Resize(3);
    // CCDyn_filter_->SetAccelLimits(accLimits_);


    // MathLib::Vector initial(3);

    // initial.Zero();

    // CCDyn_filter_->SetState(initial);
    // CCDyn_filter_->SetTarget(initial);


    return true;

}


bool SVRGRAD::InitializeROS()
{

    sub_real_pose_ = nh_.subscribe( input_topic_pose_name_, 1000,
                                    &SVRGRAD::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());

    pub_desired_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_velocity_name_, 1);

    pub_svr_gamma_ = nh_.advertise<std_msgs::Float64>(output_topic_gamme_name_, 1);

    pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("SVR/desired_path", 1);

    msg_DesiredPath_.poses.resize(MAX_FRAME);



    if (nh_.ok())   // Wait for poses being published
    {
        ros::spinOnce();
        ROS_INFO("The SVRgrad is ready.");
        return true;
    }
    else
    {
        ROS_ERROR("The ros node has a problem.");
        return false;
    }

}


void SVRGRAD::Run()
{

    while (nh_.ok())
    {

        ComputeDesiredVelocity();

        PublishDesiredVelocity();

        ComputeGamma();

        PublishGamma();

        // PublishFuturePath();

        ros::spinOnce();

        loop_rate_.sleep();
    }
}

void SVRGRAD::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr &msg)
{

    msg_real_pose_ = *msg;

    real_pose_(0) = msg_real_pose_.position.x;
    real_pose_(1) = msg_real_pose_.position.y;
    real_pose_(2) = msg_real_pose_.position.z;

    // double qtx = msg_real_pose_.orientation.x;
    // double qty = msg_real_pose_.orientation.y;
    // double qtz = msg_real_pose_.orientation.z;
    // double qtw = msg_real_pose_.orientation.w;

    // ROS_INFO_STREAM_THROTTLE(1, "Received: " << real_pose_);


}


void SVRGRAD::ComputeDesiredVelocity()
{

    // mutex_.lock();

    // svr_desired_velocity_ = SED_GMM_->getVelocity(real_pose_ - target_pose_ - target_offset_);

    // if (std::isnan(svr_desired_velocity_.Norm2()))
    // {
    //     ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
    //     svr_desired_velocity_.Zero();
    // }

    // svr_desired_velocity_ = svr_desired_velocity_ * scaling_factor_;

    // if (svr_desired_velocity_.Norm() > ds_vel_limit_)
    // {
    //     svr_desired_velocity_ = svr_desired_velocity_ / svr_desired_velocity_.Norm() * ds_vel_limit_;
    // }
    // ROS_INFO_STREAM_THROTTLE(1, "this : " <<real_pose_);
    // std::cout << "This position: " << real_pose_.transpose() << std::endl;
    // std::cout << "Shape " << real_pose_.size() << std::endl;


    svr_desired_velocity_ = SVM.calculateGammaDerivative(real_pose_);

    // std::cout << "This velocity: " << svr_desired_velocity_ << std::endl;


    // svr_desired_velocity_(0) = 0;
    // svr_desired_velocity_(1) = 0;
    // svr_desired_velocity_(2) = 0;

    msg_desired_velocity_.header.stamp = ros::Time::now();
    // msg_desired_velocity_.header.frame_id = "/mocap_svr_rotated";
    msg_desired_velocity_.twist.linear.x  = svr_desired_velocity_(0);
    msg_desired_velocity_.twist.linear.y  = svr_desired_velocity_(1);
    msg_desired_velocity_.twist.linear.z  = svr_desired_velocity_(2);
    msg_desired_velocity_.twist.angular.x = 0;
    msg_desired_velocity_.twist.angular.y = 0;
    msg_desired_velocity_.twist.angular.z = 0;


    // mutex_.unlock();


}
void SVRGRAD::ComputeGamma()
{

    gamma_ = SVM.calculateGamma(real_pose_);
    msg_gamma_.data = gamma_;

}

void SVRGRAD::PublishGamma()
{

    pub_svr_gamma_.publish(msg_gamma_);


}

void SVRGRAD::PublishDesiredVelocity()
{

    pub_desired_twist_.publish(msg_desired_velocity_);

}




void SVRGRAD::PublishFuturePath()
{



    // setting the header of the path
    msg_DesiredPath_.header.stamp = ros::Time::now();
    msg_DesiredPath_.header.frame_id = "mocap_svr_rotated";


    Eigen::Vector3d simulated_pose;
    Eigen::Vector3d simulated_vel;


    simulated_vel.resize(3);
    simulated_pose.resize(3);

    simulated_pose = real_pose_;


    for (int frame = 0; frame < MAX_FRAME; frame++)
    {

        simulated_vel = -1*  SVM.calculateGammaDerivative(simulated_pose);

        if(simulated_vel.norm() > 0.15){
        	simulated_vel = simulated_vel / simulated_vel.norm()  * 0.15;
        }

        simulated_pose[0] +=  simulated_vel[0] * dt_ * 4;
        simulated_pose[1] +=  simulated_vel[1] * dt_ * 4;
        simulated_pose[2] +=  simulated_vel[2] * dt_ * 4;

        msg_DesiredPath_.poses[frame].header.stamp = ros::Time::now();
        msg_DesiredPath_.poses[frame].header.frame_id = "mocap_svr_rotated";
        msg_DesiredPath_.poses[frame].pose.position.x = simulated_pose[0];
        msg_DesiredPath_.poses[frame].pose.position.y = simulated_pose[1];
        msg_DesiredPath_.poses[frame].pose.position.z = simulated_pose[2];

        pub_DesiredPath_.publish(msg_DesiredPath_);


    }


}