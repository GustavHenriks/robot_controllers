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

    sub_real_pose_ = nh_.subscribe(input_topic_pose_name_, 1000,
                                   &SVRGRAD::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());

    _subGrab = nh_.subscribe("/grab", 1, &SVRGRAD::updateGrabState, this);
    _subDirection = nh_.subscribe("/direction", 1, &SVRGRAD::updateDirectionState, this);
    _subSVRQuat = nh_.subscribe("/SVRQuatPub", 1, &SVRGRAD::UpdateSVRQuat, this);
    _subTotQuat = nh_.subscribe("/TotQuatPub", 1, &SVRGRAD::UpdateTotQuat, this);
    pub_desired_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_velocity_name_, 1);
    pub_desired_twist_thumb_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_velocity_name_ + "_thumb", 1);

    pub_svr_gamma_ = nh_.advertise<std_msgs::Float64>(output_topic_gamme_name_, 1);

    pub_svr_gamma_thumb_ = nh_.advertise<std_msgs::Float64>("/svr/gamma_thumb", 1);

    pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("SVR/desired_path", 1);

    _pub_DesiredPath = nh_.advertise<nav_msgs::Path>("SVR/desired_circle", 1);

    _marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    _marker_pub2 = nh_.advertise<visualization_msgs::Marker>("visualization_marker2", 1);

    msg_DesiredPath_.poses.resize(MAX_FRAME);
    _msg_DesiredPath.poses.resize(MAX_FRAME);
    _grab = 0;
    _shape = visualization_msgs::Marker::ARROW;

    SVRQuat_.resize(4);
    TotQuat_.resize(4);

    if (nh_.ok()) // Wait for poses being published
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
        // LookUpTF();

        ComputeDesiredVelocity();

        PublishDesiredVelocity();

        // ComputeDesiredVelocityThumb();

        // PublishDesiredVelocityThumb();

        ComputeGamma();

        PublishGamma();

        // ComputeGammaThumb();

        // PublishGammaThumb();

        // PublishFuturePath();

        PublishFuturePath2();

        showSVRArrow();

        showTotArrow();


        ros::spinOnce();

        loop_rate_.sleep();
    }
}

void SVRGRAD::LookUpTF()
{
    try
    {
        listener_.lookupTransform("/SVR", "/link_15_tip",
                                  ros::Time(0), transform_);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    thumb_pose_ = Vector3d(transform_.getOrigin());
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
    msg_desired_velocity_.twist.linear.x = svr_desired_velocity_(0);
    msg_desired_velocity_.twist.linear.y = svr_desired_velocity_(1);
    msg_desired_velocity_.twist.linear.z = svr_desired_velocity_(2);
    msg_desired_velocity_.twist.angular.x = 0;
    msg_desired_velocity_.twist.angular.y = 0;
    msg_desired_velocity_.twist.angular.z = 0;

    // mutex_.unlock();
}

void SVRGRAD::ComputeDesiredVelocityThumb()
{

    svr_desired_velocity_thumb_ = SVM.calculateGammaDerivative(thumb_pose_);

    msg_desired_velocity_thumb_.header.stamp = ros::Time::now();
    msg_desired_velocity_thumb_.twist.linear.x = svr_desired_velocity_thumb_(0);
    msg_desired_velocity_thumb_.twist.linear.y = svr_desired_velocity_thumb_(1);
    msg_desired_velocity_thumb_.twist.linear.z = svr_desired_velocity_thumb_(2);
    msg_desired_velocity_thumb_.twist.angular.x = 0;
    msg_desired_velocity_thumb_.twist.angular.y = 0;
    msg_desired_velocity_thumb_.twist.angular.z = 0;

    // mutex_.unlock();
}
void SVRGRAD::ComputeGamma()
{

    gamma_ = SVM.calculateGamma(real_pose_);
    msg_gamma_.data = gamma_;
}

void SVRGRAD::ComputeGammaThumb()
{

    gamma_thumb_ = SVM.calculateGamma(thumb_pose_);
    msg_gamma_thumb_.data = gamma_;
}

void SVRGRAD::PublishGamma()
{

    pub_svr_gamma_.publish(msg_gamma_);
}

void SVRGRAD::PublishGammaThumb()
{

    pub_svr_gamma_thumb_.publish(msg_gamma_thumb_);
}

void SVRGRAD::PublishDesiredVelocity()
{

    pub_desired_twist_.publish(msg_desired_velocity_);
}

void SVRGRAD::PublishDesiredVelocityThumb()
{

    pub_desired_twist_thumb_.publish(msg_desired_velocity_thumb_);
}

void SVRGRAD::PublishFuturePath()
{

    // setting the header of the path
    msg_DesiredPath_.header.stamp = ros::Time::now();
    msg_DesiredPath_.header.frame_id = "SVR";

    Eigen::Vector3d simulated_pose;
    Eigen::Vector3d simulated_vel;

    simulated_vel.resize(3);
    simulated_pose.resize(3);

    simulated_pose = real_pose_;
    simulated_pose[0] = simulated_pose[0] - 0.08;
    simulated_pose[1] = simulated_pose[1];
    simulated_pose[2] = simulated_pose[2] - 0.03;
    // simulated_pose[0] = simulated_pose[0] - 0.16;
    // simulated_pose[1] = simulated_pose[1] - 0.035;
    // simulated_pose[2] = simulated_pose[2] - 0.03;
    // simulated_pose = Vector3d(transform_.getOrigin().x(),transform_.getOrigin().y(),transform_.getOrigin().z());
    // simulated_pose = Vector3d(transform_.getOrigin());

    for (int frame = 0; frame < MAX_FRAME; frame++)
    {

        simulated_vel = -1 * SVM.calculateGammaDerivative(simulated_pose);

        if (simulated_vel.norm() > 0.15)
        {
            simulated_vel = simulated_vel / simulated_vel.norm() * 0.15;
        }
        if (SVM.calculateGamma(simulated_pose) > 0.01)
        {
            simulated_pose[0] += simulated_vel[0] * dt_ * 4;
            simulated_pose[1] += simulated_vel[1] * dt_ * 4;
            simulated_pose[2] += simulated_vel[2] * dt_ * 4;
        }
        msg_DesiredPath_.poses[frame].header.stamp = ros::Time::now();
        msg_DesiredPath_.poses[frame].header.frame_id = "iiwa_link_ee";
        // msg_DesiredPath_.poses[frame].header.frame_id = "SVR";
        // msg_DesiredPath_.poses[frame].header.frame_id = "finger3";
        if (_grab == 0)
        {
            msg_DesiredPath_.poses[frame].pose.position.x = simulated_pose[0];
            msg_DesiredPath_.poses[frame].pose.position.y = simulated_pose[1];
            msg_DesiredPath_.poses[frame].pose.position.z = simulated_pose[2];
            pub_DesiredPath_.publish(msg_DesiredPath_);
        }
        else
        {
            msg_DesiredPath_.poses[frame].pose.position.x = 0;
            msg_DesiredPath_.poses[frame].pose.position.y = 0;
            msg_DesiredPath_.poses[frame].pose.position.z = 0;
            pub_DesiredPath_.publish(msg_DesiredPath_);
        }
    }
}

void SVRGRAD::PublishFuturePath2()
{

    // setting the header of the path
    _msg_DesiredPath.header.stamp = ros::Time::now();
    _msg_DesiredPath.header.frame_id = "SVR";

    Eigen::Vector3d simulated_pose;
    Eigen::Vector3d simulated_vel;
    Eigen::Vector2d simulated_ds;

    simulated_vel.resize(3);
    simulated_pose.resize(3);
    simulated_ds.resize(2);

    // simulated_ds=Eigen::Vector2d(_filtered_thumb_pos[1] - _finger[3].X_target_inRef_orig[1], _filtered_thumb_pos[2] - _finger[3].X_target_inRef_orig[2]);
    ds_simulation(simulated_ds, 0.005);

    // simulated_pose[0] = real_pose_[0]*0.01+old_pose_[0]*0.99;
    // simulated_pose[1] = real_pose_[1]*0.01+old_pose_[1]*0.99;
    // simulated_pose[2] = real_pose_[2]*0.01+old_pose_[2]*0.99;
    // old_pose_=real_pose_;

    // simulated_pose = real_pose_;
    // simulated_pose[0] = simulated_pose[0] - 0.16;
    // simulated_pose[1] = simulated_pose[1] - 0.045;
    // simulated_pose[2] = simulated_pose[2] - 0.04;
    simulated_pose[0] = -0.025;
    simulated_pose[1] = 0.015;
    simulated_pose[2] = -0.009;
    // simulated_pose<<0,0,0;

    // simulated_pose = Vector3d(transform_.getOrigin().x(),transform_.getOrigin().y(),transform_.getOrigin().z());
    // simulated_pose = Vector3d(transform_.getOrigin());

    for (int frame = 0; frame < _MAX_FRAME_circ; frame++)
    {

        //         // simulated_vel << (_finger[3].X_target_inRef[0] - _force_term - _finger[3].X_inRef[0]), _new_ds_simulation;
        simulated_vel << 0, _new_ds_simulation;
        // simulated_vel << 0, 0, 0;
        // std::cout<<_new_ds_simulation<<std::endl;

        if (simulated_vel.norm() > 0.15)
        {
            simulated_vel = simulated_vel / simulated_vel.norm() * 0.15;
        }
        if (frame < 80)
        {
            simulated_pose[0] += -simulated_vel[0] * dt_ * 100;
            simulated_pose[1] += -simulated_vel[1] * dt_ * 100 + 0.002; //0.002 old
            simulated_pose[2] += simulated_vel[2] * dt_ * 100;
        }
        _msg_DesiredPath.poses[frame].header.stamp = ros::Time::now();
        _msg_DesiredPath.poses[frame].header.frame_id = "iiwa_link_ee";
        // _msg_DesiredPath.poses[frame].header.frame_id = "palm_link";
        // _msg_DesiredPath.poses[frame].header.frame_id = "thumbpos";
        //   //   msg_DesiredPath_.poses[frame].header.frame_id = "SVR";
        // msg_DesiredPath_.poses[frame].header.frame_id = "Finger 3";
        // std::cout<<_grab<<std::endl;
        if (_grab == 1)
        {
            _msg_DesiredPath.poses[frame].pose.position.x = simulated_pose[0] * 1.7; //1.2 old
            _msg_DesiredPath.poses[frame].pose.position.y = simulated_pose[1] * 1.7;
            _msg_DesiredPath.poses[frame].pose.position.z = simulated_pose[2] * 1.7;
            _pub_DesiredPath.publish(_msg_DesiredPath);
        }
        else
        {
            _msg_DesiredPath.poses[frame].pose.position.x = 0;
            _msg_DesiredPath.poses[frame].pose.position.y = 0;
            _msg_DesiredPath.poses[frame].pose.position.z = 0;
            _pub_DesiredPath.publish(_msg_DesiredPath);
        }
        // simulated_ds<<simulated_pose[1]-real_pose_[1]-0.005,simulated_pose[2]-real_pose_[2];
        simulated_ds[0] = simulated_vel[1];
        simulated_ds[1] = simulated_vel[2];
        ds_simulation(simulated_ds, 0.0055);
    }
}

void SVRGRAD::ds_simulation(Eigen::Vector2d x, double r_value)
{

    double r, theta_circle, theta_circle_dot, r_dot, x_dot, y_dot, limit;
    Eigen::Vector2d v;
    theta_circle = atan2(x(1), x(0));
    r = sqrt(pow(x(0), 2) + pow(x(1), 2));

    theta_circle_dot = -0.4;
    r_dot = -1 * (r - r_value);

    x_dot = r_dot * cos(theta_circle) - r * theta_circle_dot * sin(theta_circle);
    y_dot = r_dot * sin(theta_circle) + r * theta_circle_dot * cos(theta_circle);
    // cout << "_theta_circle" << _theta_circle << endl;
    // cout << "r" << r << endl;
    // cout << "r_dot" << r_dot << endl;
    // cout << "x_dot" << x_dot << endl;

    v(0) = x_dot * 100;
    v(1) = y_dot * 100;
    limit = 0.1;
    if (v.norm() > limit)
    {
        v = v / v.norm() * limit;
    }
    _new_ds_simulation = 0.05 * v + 0.95 * _new_ds_simulation;
    //    _new_ds_simulation = v;
}

void SVRGRAD::updateGrabState(const std_msgs::Int8 &msg)
{
    _grab = msg.data;
    //    _grab_received = 1;
    std::cout << "grab " << _grab << std::endl;
}

void SVRGRAD::updateDirectionState(const std_msgs::Int8 &msg)
{
    _direction = msg.data;
    //    _grab_received = 1;
    //    std::cout << "grab " << _grab << std::endl;
}

void SVRGRAD::showSVRArrow()
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "SVR";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = _shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = real_pose_[0] - 0.08;
    ;
    marker.pose.position.y = real_pose_[1];
    marker.pose.position.z = real_pose_[2] - 0.03;
    // marker.pose.position.x = SVRPose_[0] + 0.08;
    // marker.pose.position.y = SVRPose_[1];
    // marker.pose.position.z = SVRPose_[2] - 0.03;
    // quat_from_vec();
    marker.pose.orientation.x = SVRQuat_(0);
    marker.pose.orientation.y = SVRQuat_(1);
    marker.pose.orientation.z = SVRQuat_(2);
    marker.pose.orientation.w = SVRQuat_(3);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.05;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // marker.lifetime = ros::Duration();

    // while (_marker_pub.getNumSubscribers() < 1)
    // {
    //     ROS_WARN_ONCE("Please create a subscriber to the marker");
    //     sleep(1);
    // }
    _marker_pub.publish(marker);


}

void SVRGRAD::showTotArrow()
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = _shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = TotPose_[0] + 0.08;
    ;
    marker.pose.position.y = TotPose_[1];
    marker.pose.position.z = TotPose_[2] - 0.03;
    // quat_from_vec();
    marker.pose.orientation.x = TotQuat_(0);
    marker.pose.orientation.y = TotQuat_(1);
    marker.pose.orientation.z = TotQuat_(2);
    marker.pose.orientation.w = TotQuat_(3);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.05;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // marker.lifetime = ros::Duration();

    // while (_marker_pub.getNumSubscribers() < 1)
    // {
    //     ROS_WARN_ONCE("Please create a subscriber to the marker");
    //     sleep(1);
    // }
    _marker_pub2.publish(marker);

}

void SVRGRAD::quat_from_vec()
{
    // Eigen::Vector3d axis_y, axis_z, axis_x, axis_z_on_y;
    // axis_y << 0, -1, 0;
    // axis_z = -1 * SVM.calculateGammaDerivative(real_pose_);
    // axis_z_on_y = axis_y.dot(axis_z);
    //     axis_z = axis_z - axis_z_on_y * axis_y
    //     axis_z = axis_z/np.linalg.norm(axis_z)
    //     axis_x = np.cross(axis_y, axis_z)

    //     rot_mat = np.zeros((4, 4))
    //     rot_mat[:3, 0] = axis_x
    //     rot_mat[:3, 1] = axis_y
    //     rot_mat[:3, 2] = axis_z
    //     rot_mat[3, 3] = 1
    //     q_tf = tf.transformations.quaternion_from_matrix(rot_mat)
    //     return q_tf
}

void SVRGRAD::UpdateSVRQuat(const geometry_msgs::Pose::ConstPtr &msg)
{

    msg_real_pose_ = *msg;

    SVRPose_(0) = msg_real_pose_.position.x;
    SVRPose_(1) = msg_real_pose_.position.y;
    SVRPose_(2) = msg_real_pose_.position.z;

    SVRQuat_(0) = msg_real_pose_.orientation.x;
    SVRQuat_(1) = msg_real_pose_.orientation.y;
    SVRQuat_(2) = msg_real_pose_.orientation.z;
    SVRQuat_(3) = msg_real_pose_.orientation.w;

    // ROS_INFO_STREAM_THROTTLE(1, "Received: " << real_pose_);
}

void SVRGRAD::UpdateTotQuat(const geometry_msgs::Pose::ConstPtr &msg)
{

    msg_real_pose_ = *msg;

    TotPose_(0) = msg_real_pose_.position.x;
    TotPose_(1) = msg_real_pose_.position.y;
    TotPose_(2) = msg_real_pose_.position.z;

    TotQuat_(0) = msg_real_pose_.orientation.x;
    TotQuat_(1) = msg_real_pose_.orientation.y;
    TotQuat_(2) = msg_real_pose_.orientation.z;
    TotQuat_(3) = msg_real_pose_.orientation.w;

    // ROS_INFO_STREAM_THROTTLE(1, "Received: " << real_pose_);
}