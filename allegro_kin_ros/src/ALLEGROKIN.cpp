#include "ALLEGROKIN.h"

ALLEGROKIN::ALLEGROKIN(ros::NodeHandle &n,
                       double frequency)
    : nh_(n),
      loop_rate_(frequency),
      dt_(1 / frequency)
{

    ROS_INFO_STREAM("ALLEGROKIN node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}

bool ALLEGROKIN::Init()
{

    // real_pose_.Resize(3);
    // svr_desired_velocity_.Resize(3);

    if (!InitializeModel())
    {
        ROS_ERROR_STREAM("ERROR intializing the Model");
        return false;
    }

    if (!InitializeROS())
    {
        ROS_ERROR_STREAM("ERROR intializing the ROS");
        return false;
    }

    return true;
}

bool ALLEGROKIN::InitializeModel()
{

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model_ = robot_model_loader.getModel();

    ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

    kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
    kinematic_state_->setToDefaultValues();

    joint_model_finger_0_ = kinematic_model_->getJointModelGroup("finger_0");
    joint_model_finger_1_ = kinematic_model_->getJointModelGroup("finger_1");
    joint_model_finger_2_ = kinematic_model_->getJointModelGroup("finger_2");
    joint_model_finger_3_ = kinematic_model_->getJointModelGroup("finger_3");

    const std::vector<std::string> &joint_names_finger_0 = joint_model_finger_0_->getVariableNames();
    const std::vector<std::string> &joint_names_finger_1 = joint_model_finger_1_->getVariableNames();
    const std::vector<std::string> &joint_names_finger_2 = joint_model_finger_2_->getVariableNames();
    const std::vector<std::string> &joint_names_finger_3 = joint_model_finger_3_->getVariableNames();

    kinematic_state_->copyJointGroupPositions(joint_model_finger_0_, joint_values_finger_0_);
    kinematic_state_->copyJointGroupPositions(joint_model_finger_1_, joint_values_finger_1_);
    kinematic_state_->copyJointGroupPositions(joint_model_finger_2_, joint_values_finger_2_);
    kinematic_state_->copyJointGroupPositions(joint_model_finger_3_, joint_values_finger_3_);

    joint_values_finger_0_[0] = 0.0;
    joint_values_finger_0_[1] = 0.5;
    joint_values_finger_0_[2] = 0.5;
    joint_values_finger_0_[3] = 0.7;
    //deco.m_allegro_joint_positions[4] = 1.0;
    joint_values_finger_1_[0] = 0.0;
    joint_values_finger_1_[1] = 0.5;
    joint_values_finger_1_[2] = 0.5;
    joint_values_finger_1_[3] = 0.7;
    //deco.m_allegro_joint_positions[9] = 0.0;
    joint_values_finger_2_[0] = 0.0;
    joint_values_finger_2_[1] = 0.5;
    joint_values_finger_2_[2] = 0.5;
    joint_values_finger_2_[3] = 0.7;
    //deco.m_allegro_joint_positions[14] = 0.0;
    joint_values_finger_3_[0] = 3.14 / 2;
    joint_values_finger_3_[1] = 0.26;
    joint_values_finger_3_[2] = 0.1;
    joint_values_finger_3_[3] = 0.1;

    test_finger_1_ << 0.0399453, 0.0626116, 0.123999;
    test_finger_2_ << 0.0399453, 0.00809732, 0.127429;
    test_finger_3_ << 0.0399453, -0.0464786, 0.12541;
    test_finger_4_ << 0.0350638, 0.156066, -0.0588007;
    // cout << joint_values_finger_3_[] << endl;

    // Subscribers and publishers
    sub_joints = nh_.subscribe("/allegroHand_0/joint_states", 3, &ALLEGROKIN::UpdateJointPositions, this);
    return true;
}

bool ALLEGROKIN::InitializeROS()
{

    // subscriber for q with callback to UpdateJointPositions

    //subscriber for xdot with callback to ComputeInverseKinematics

    // publisher for x

    //publisher for qdot

    // sub_real_pose_ = nh_.subscribe( input_topic_pose_name_, 1000,
    //                                 &ALLEGROKIN::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());

    // pub_desired_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_velocity_name_, 1);

    // pub_svr_gamma_ = nh_.advertise<std_msgs::Float64>(output_topic_gamme_name_, 1);

    // pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("SVR/desired_path", 1);

    if (nh_.ok()) // Wait for poses being published
    {
        ros::spinOnce();
        ROS_INFO("The ALLEGROKIN is ready.");
        return true;
    }
    else
    {
        ROS_ERROR("The ros node has a problem.");
        return false;
    }
}

void ALLEGROKIN::Run()
{

    while (nh_.ok())
    {

        ComputeForwardKinematics();

        // ComputeInverseKinematics(test_finger_1_, test_finger_2_, test_finger_3_, test_finger_4_);

        ros::spinOnce();

        // ROS_INFO_STREAM_THROTTLE(1, "Finger 1 : " << joint_values_finger_0_[0] << " " << joint_values_finger_0_[1] << " " << joint_values_finger_0_[2] << " " << joint_values_finger_0_[3]);
        // ROS_INFO_STREAM_THROTTLE(1, "Finger 2 : " << joint_values_finger_1_[0] << " " << joint_values_finger_1_[1] << " " << joint_values_finger_1_[2] << " " << joint_values_finger_1_[3]);
        // ROS_INFO_STREAM_THROTTLE(1, "Finger 3 : " << joint_values_finger_2_[0] << " " << joint_values_finger_2_[1] << " " << joint_values_finger_2_[2] << " " << joint_values_finger_2_[3]);
        // ROS_INFO_STREAM_THROTTLE(1, "Finger 4 : " << joint_values_finger_3_[0] << " " << joint_values_finger_3_[1] << " " << joint_values_finger_3_[2] << " " << joint_values_finger_3_[3]);

        // ROS_INFO_STREAM_THROTTLE(1, "Finger 1 : " << X0_current_inRef_[0] << " " << X0_current_inRef_[1] << " " << X0_current_inRef_[2]);
        // ROS_INFO_STREAM_THROTTLE(1, "Finger 2 : " << X1_current_inRef_[0] << " " << X1_current_inRef_[1] << " " << X1_current_inRef_[2]);
        // ROS_INFO_STREAM_THROTTLE(1, "Finger 3 : " << X2_current_inRef_[0] << " " << X2_current_inRef_[1] << " " << X2_current_inRef_[2]);
        ROS_INFO_STREAM_THROTTLE(1, "Finger 4 : " << X3_current_inRef_[0] << " " << X3_current_inRef_[1] << " " << X3_current_inRef_[2]);

        // ROS_INFO_STREAM_THROTTLE(1, "Joint 1-4 : " << current_joint_position_[0] << " " << current_joint_position_[1] << " " << current_joint_position_[2] << " " << current_joint_position_[3]);        
        // ROS_INFO_STREAM_THROTTLE(1, "Joint 5-8 : " << current_joint_position_[4] << " " << current_joint_position_[5] << " " << current_joint_position_[6] << " " << current_joint_position_[7]);        
        // ROS_INFO_STREAM_THROTTLE(1, "Joint 9-12 : " << current_joint_position_[8] << " " << current_joint_position_[9] << " " << current_joint_position_[10] << " " << current_joint_position_[11]);        
        ROS_INFO_STREAM_THROTTLE(1, "Joint 13-16 : " << current_joint_position_[12] << " " << current_joint_position_[13] << " " << current_joint_position_[14] << " " << current_joint_position_[15]);        
        // ROS_INFO_STREAM_THROTTLE(1, "Joint 4 : " << current_joint_position_[12] << " " << current_joint_position_[13] << " " << current_joint_position_[14] << " " << current_joint_position_[15]);

        loop_rate_.sleep();
    }
}

void ALLEGROKIN::ComputeForwardKinematics()
{
    double joint_values_finger_0[] = {0};
    double joint_values_finger_1[] = {0};
    double joint_values_finger_2[] = {0};
    double joint_values_finger_3[] = {0};
    for (int j = 0; j < 4; j++)
    {
        joint_values_finger_0[j] = (double)current_joint_position_[j ];
        joint_values_finger_1[j] = (double)current_joint_position_[j + 4];
        joint_values_finger_2[j] = (double)current_joint_position_[j + 8];
        joint_values_finger_3[j] = (double)current_joint_position_[j + 12];
    }

    kinematic_state_->setJointGroupPositions(joint_model_finger_0_, joint_values_finger_0);
    kinematic_state_->setJointGroupPositions(joint_model_finger_1_, joint_values_finger_1);
    kinematic_state_->setJointGroupPositions(joint_model_finger_2_, joint_values_finger_2);
    kinematic_state_->setJointGroupPositions(joint_model_finger_3_, joint_values_finger_3);

    const Eigen::Affine3d &state_fingertip_0 = kinematic_state_->getGlobalLinkTransform("link_3_tip");
    const Eigen::Affine3d &state_fingertip_1 = kinematic_state_->getGlobalLinkTransform("link_7_tip");
    const Eigen::Affine3d &state_fingertip_2 = kinematic_state_->getGlobalLinkTransform("link_11_tip");
    const Eigen::Affine3d &state_fingertip_3 = kinematic_state_->getGlobalLinkTransform("link_15_tip");

    X0_current_inRef_ = state_fingertip_0.translation();
    X1_current_inRef_ = state_fingertip_1.translation();
    X2_current_inRef_ = state_fingertip_2.translation();
    X3_current_inRef_ = state_fingertip_3.translation();
}

void ALLEGROKIN::UpdateJointPositions(const sensor_msgs::JointState &msg)
{
    for (int i = 0; i < 16; i++)
    {
        current_joint_position_[i] = msg.position[i];
    }
}

void ALLEGROKIN::ComputeInverseKinematics(const Eigen::Vector3d &tip_0_pos,
                                          const Eigen::Vector3d &tip_1_pos,
                                          const Eigen::Vector3d &tip_2_pos,
                                          const Eigen::Vector3d &tip_3_pos)
{
    kinematic_state_->setJointGroupPositions(joint_model_finger_0_, joint_values_finger_0_);
    kinematic_state_->setJointGroupPositions(joint_model_finger_1_, joint_values_finger_1_);
    kinematic_state_->setJointGroupPositions(joint_model_finger_2_, joint_values_finger_2_);
    kinematic_state_->setJointGroupPositions(joint_model_finger_3_, joint_values_finger_3_);

    std::size_t attempts = 10;
    double timeout = 0.01;

    Eigen::Affine3d tip_tf;
    tip_tf.setIdentity();
    tip_tf.translate(tip_0_pos);
    found_ik0_ = kinematic_state_->setFromIK(joint_model_finger_0_, tip_tf, attempts, timeout);
    tip_tf.setIdentity();
    tip_tf.translate(tip_1_pos);
    found_ik1_ = kinematic_state_->setFromIK(joint_model_finger_1_, tip_tf, attempts, timeout);
    tip_tf.setIdentity();
    tip_tf.translate(tip_2_pos);
    found_ik2_ = kinematic_state_->setFromIK(joint_model_finger_2_, tip_tf, attempts, timeout);
    tip_tf.setIdentity();
    tip_tf.translate(tip_3_pos);
    //  tip_tf.linear()=( AngleAxisd(M_PI / 2, Vector3d::UnitY())).toRotationMatrix();
    found_ik3_ = kinematic_state_->setFromIK(joint_model_finger_3_, tip_tf, attempts, timeout);

    if (found_ik0_)
    {
        kinematic_state_->copyJointGroupPositions(joint_model_finger_0_, joint_values_finger_0_);
    }
    else
    {
        std::cout << "Did not find IK0 solution" << std::endl
                  << std::endl;
    }

    if (found_ik1_)
    {
        kinematic_state_->copyJointGroupPositions(joint_model_finger_1_, joint_values_finger_1_);
    }
    else
    {
        std::cout << "Did not find IK1 solution" << std::endl
                  << std::endl;
    }

    if (found_ik2_)
    {
        kinematic_state_->copyJointGroupPositions(joint_model_finger_2_, joint_values_finger_2_);
    }
    else
    {
        std::cout << "Did not find IK2 solution" << std::endl
                  << std::endl;
    }

    if (found_ik3_)
    {
        kinematic_state_->copyJointGroupPositions(joint_model_finger_3_, joint_values_finger_3_);
    }
    else
    {
        std::cout << "Did not find IK3 solution" << std::endl
                  << std::endl;
    }

    // publish qdot
}
