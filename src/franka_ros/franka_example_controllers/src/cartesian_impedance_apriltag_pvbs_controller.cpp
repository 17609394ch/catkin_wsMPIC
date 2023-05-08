// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_apriltag_pvbs_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool CartesianImpedanceApriltagPvbsController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceApriltagPvbsController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  Pub_RobotState = node_handle.advertise<franka_example_controllers::State>("Robot_State",1000);
  path_desire_pub = node_handle.advertise<nav_msgs::Path>("trajectory",1, true);
  path_robot_pub = node_handle.advertise<nav_msgs::Path>("trajectory_robot",1, true);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceApriltagPvbsController: Could not read parameter arm_id");
    return false;
  }


  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceApriltagPvbsController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceApriltagPvbsController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceApriltagPvbsController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceApriltagPvbsController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceApriltagPvbsController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceApriltagPvbsController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceApriltagPvbsController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
  ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
  dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

  dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
  boost::bind(&CartesianImpedanceApriltagPvbsController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void CartesianImpedanceApriltagPvbsController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
  position_base_target=initial_transform.translation();
  orientation_base_target= Eigen::Quaterniond(initial_transform.linear());
  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;

  Eigen::Translation3d translation_temp(0,0,0);
  tf2::Quaternion qtn;
  qtn.setRPY(0,0,0);
  Eigen::Quaterniond orientation_temp;
  orientation_temp.coeffs()<<qtn.getX(),qtn.getY(),qtn.getZ(),qtn.getW();
  transformapriltag=translation_temp*orientation_temp.toRotationMatrix();

  path_desire.header.frame_id="world";
  path_desire.header.stamp=ros::Time::now();
  path_robot.header.frame_id="world";
  path_robot.header.stamp=ros::Time::now();
  go_circle=true;
  tau_task.resize(7);
  tau_nullspace.resize(7);
  tau_d.resize(7);
  begin_time = ros::Time::now().toSec();

}

void CartesianImpedanceApriltagPvbsController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // get state variables

 
  ros::param::get("receive_apriltag_flag",receive_apriltag_flag);

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
  model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
  robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  position=transform.translation();
  orientation=transform.linear();

  // compute error to desired pose
  // position error

  if(receive_apriltag_flag ){
    Eigen::Translation3d translationoOtee_apriltag(position_apriltag[0],position_apriltag[1],position_apriltag[2]);
    Eigen::Quaterniond orientationOtee_apriltag;
    orientationOtee_apriltag.coeffs()<<orientation_apriltag.x(),orientation_apriltag.y(),orientation_apriltag.z(),orientation_apriltag.w();
    Eigen::Affine3d transformOtee_apriltag=translationoOtee_apriltag*orientationOtee_apriltag.toRotationMatrix();
    transformapriltag=transform*transformOtee_apriltag;
    }else{

    }
   
    Eigen::Translation3d translationoapriltag_target(0.1*cos(time_add/1000)-0.05,0.1*sin(time_add/1000)-0.05,0.4);
    Eigen::Quaterniond orientationapriltag_target;
    qtn.setRPY(-3.1415,0,-4.71);
    orientationapriltag_target.coeffs()<<qtn.getX(),qtn.getY(),qtn.getZ(),qtn.getW();
    Eigen::Affine3d transformapriltag_target=translationoapriltag_target*orientationapriltag_target.toRotationMatrix();

    Eigen::Affine3d transformtarget=transformapriltag*transformapriltag_target;

    position_base_target=transformtarget.translation();
    orientation_base_target=Eigen::Quaterniond(transformtarget.linear());
    euler_=orientation.toRotationMatrix().eulerAngles(2,1,0);

    euler_d=orientation_base_target.toRotationMatrix().eulerAngles(2,1,0);
    
    if (receive_apriltag_flag||!first_detection)
    {
        first_detection=false;
        if ( go_circle )
        {
            time_add++;
        }
        else{
          time_add=0;
        }
        if (!debug_flag)
        {
          /* code */
          position_d_target_<<position_base_target[0],position_base_target[1],position_base_target[2];
          orientation_d_target_.coeffs()<<orientation_base_target.x(),orientation_base_target.y(),orientation_base_target.z(),orientation_base_target.w();
 
        }
    }
  ROS_INFO("time:%lf",time_add);

  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);

  // compute control
  // allocate variables
  if (first_run)
  {
      s_.head(3)=position_d_;
s_.tail(3)=euler_d;  
      first_run=false;
      // s_=error;
  }
  Eigen::Matrix<double, 6, 1> xd;


  xd.head(3)=position_d_;
  xd.tail(3)=euler_d;


  // dt_error=(xd-s_)/filter_params_;
  if ((end_time - begin_time)!=0)
  {
    /* code */
      dt_error=(xd-s_)/((end_time - begin_time)/filter_params_);
  }
  

   end_time = ros::Time::now().toSec();
  ROS_INFO("t:%lf",   (end_time - begin_time));
  begin_time=end_time;
  s_=(1-filter_params_)*s_+filter_params_*xd;
  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian*dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace+ coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceApriltagPvbsController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceApriltagPvbsController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void CartesianImpedanceApriltagPvbsController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  ros::param::get("receive_apriltag_flag",receive_apriltag_flag);
    if (receive_apriltag_flag)
   {
      position_apriltag << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
      ROS_INFO("%lf",msg->pose.position.y);
      Eigen::Quaterniond last_orientation_apriltag(orientation_apriltag);
      orientation_apriltag.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
          msg->pose.orientation.z, msg->pose.orientation.w;
      if (last_orientation_apriltag.coeffs().dot(orientation_apriltag.coeffs()) < 0.0) {
        orientation_apriltag.coeffs() << -orientation_apriltag.coeffs();
      } 
   }

         for (size_t i = 0; i < 3; i++)
  {
    /* code */
    robot_state_look.position[i]=error[i];
    robot_state_look.orientation[i]=error[3+i];
  }

  for (size_t i = 0; i < 3; i++)
  {
    /* code */
    if (euler_[i]>3)
    {
      euler_[i]=euler_[i]-M_PI;
    }
        if (euler_[i]<-3)
    {
      euler_[i]=euler_[i]+M_PI;
    }
    
      robot_state_look.tau_commanded[i]=euler_[i];

          if (euler_d[i]>3)
    {
      euler_d[i]=euler_d[i]-M_PI;
    }
        if (euler_d[i]<-3)
    {
      euler_d[i]=euler_d[i]+M_PI;
    }
    
      robot_state_look.tau_commanded[i+3]=euler_d[i];
  }
  for (size_t i = 0; i < 3; i++)
  {
      robot_state_look.error_orientation[i]=euler_[i]-euler_d[i];
  }
  
  tf2_ros::TransformBroadcaster broadcaster;

  geometry_msgs::TransformStamped tfs;
  Pub_RobotState.publish(robot_state_look);
  this_pose_stamped.header.frame_id="panda_link0";
  this_pose_stamped.header.stamp=ros::Time::now();
  this_pose_stamped.pose.position.x = position_base_target[0];
  this_pose_stamped.pose.position.y = position_base_target[1];
  this_pose_stamped.pose.position.z = position_base_target[2]; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置
  this_pose_stamped.pose.orientation.x = orientation_base_target.x();
  this_pose_stamped.pose.orientation.y = orientation_base_target.y();
  this_pose_stamped.pose.orientation.z = orientation_base_target.z();
  this_pose_stamped.pose.orientation.w = orientation_base_target.w();
  path_desire.poses.push_back(this_pose_stamped);
  path_desire_pub.publish(path_desire);

    this_pose_stamped.pose.position.x = position[0];
  this_pose_stamped.pose.position.y = position[1];
  this_pose_stamped.pose.position.z = position[2]; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置
  this_pose_stamped.pose.orientation.x = orientation.x();
  this_pose_stamped.pose.orientation.y = orientation.y();
  this_pose_stamped.pose.orientation.z = orientation.z();
  this_pose_stamped.pose.orientation.w = orientation.w();

  path_robot.poses.push_back(this_pose_stamped);
  path_robot_pub.publish(path_robot);

  tfs.header.frame_id = "panda_link0";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "target";

    //  |----坐标系相对信息设置
    tfs.transform.translation.x = position_base_target[0];
    tfs.transform.translation.y = position_base_target[1];
    tfs.transform.translation.z = position_base_target[2]; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置
  
    tfs.transform.rotation.x = orientation_base_target.x();
    tfs.transform.rotation.y = orientation_base_target.y();
    tfs.transform.rotation.z = orientation_base_target.z();
    tfs.transform.rotation.w = orientation_base_target.w();


    broadcaster.sendTransform(tfs);




}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceApriltagPvbsController,
                       controller_interface::ControllerBase)
