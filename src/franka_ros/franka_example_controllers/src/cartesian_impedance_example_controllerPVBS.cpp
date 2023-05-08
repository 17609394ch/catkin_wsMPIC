// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_example_controllerPVBS.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_example_controllers/pseudo_inversion.h>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
// #include <visp3/visual_features/vpFeatureThetaU.h>
// #include <visp3/visual_features/vpFeatureTranslation.h>
#include <franka_example_controllers/State.h>


namespace franka_example_controllers {

bool CartesianImpedanceExampleControllerPVBS::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;
  ros::param::set("receive_apriltag_flag",false);
  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceExampleControllerPVBS::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  Pub_RobotState = node_handle.advertise<franka_example_controllers::State>("Robot_State",1000);
  path_pub = node_handle.advertise<nav_msgs::Path>("trajectory",1, true);
  path1_pub = node_handle.advertise<nav_msgs::Path>("trajectory_robot",1, true);
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleControllerPVBS: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleControllerPVBS: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleControllerPVBS: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleControllerPVBS: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleControllerPVBS: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleControllerPVBS: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleControllerPVBS: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleControllerPVBS: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceExampleControllerPVBS::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  orientation_apriltag.coeffs() << 0.0, 0.0, 0.0, 1.0;

  return true;
}

void CartesianImpedanceExampleControllerPVBS::starting(const ros::Time& /*time*/) {
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
    Eigen::Translation3d translationo_temp(0,0,0);
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    Eigen::Quaterniond orientationoTEE_carame;
    orientationoTEE_carame.coeffs()<<qtn.getX(),qtn.getY(),qtn.getZ(),qtn.getW();
    transformapriltag=translationo_temp*orientationoTEE_carame.toRotationMatrix();
    path.header.frame_id="world";
    path.header.stamp=ros::Time::now();
    path1.header.frame_id="world";
    path1.header.stamp=ros::Time::now();
}

void CartesianImpedanceExampleControllerPVBS::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // get state variables

    ros::param::get("receive_apriltag_flag",receive_apriltag_flag);
    i++;
    static tf2_ros::TransformBroadcaster broadcaster;
    //  5-2.创建 广播的数据(通过 pose 设置)
    geometry_msgs::TransformStamped tfs;
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
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // compute error to desired pose
    // position error
    Eigen::Translation3d translationapriltag(position_d_target_[0],position_d_target_[1],position_d_target_[2]);

    Eigen::Translation3d translationoTEE_carame(0.0337731,-0.00535012,-0.0523339);
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,-1.57);
    Eigen::Quaterniond orientationoTEE_carame;
    orientationoTEE_carame.coeffs()<<qtn.getX(),qtn.getY(),qtn.getZ(),qtn.getW();
    Eigen::Affine3d transformOTEE_Carame=translationoTEE_carame*orientationoTEE_carame.toRotationMatrix();

    Eigen::Affine3d transformB_C=transform*transformOTEE_Carame;

    Eigen::Vector3d position_C(transformB_C.translation());
    Eigen::Quaterniond orientation_C(transformB_C.linear());


    Eigen::Translation3d translationoTEE_carameLink(0.0,0.0,0.0);
    qtn.setRPY(-1.57,0,-1.57);
    Eigen::Quaterniond orientationcamera_cameraLICK;
    orientationcamera_cameraLICK.coeffs()<<qtn.getX(),qtn.getY(),qtn.getZ(),qtn.getW();
    Eigen::Affine3d transformcamera_cameraLICK=translationoTEE_carameLink*orientationcamera_cameraLICK.toRotationMatrix();

    Eigen::Affine3d transformB_CLink=transformB_C*transformcamera_cameraLICK;

    Eigen::Vector3d position_CLink(transformB_CLink.translation());
    Eigen::Quaterniond orientation_CLink(transformB_CLink.linear());


    if(receive_apriltag_flag){
    Eigen::Translation3d translationocarameLink_apriltag(position_apriltag[0],position_apriltag[1],position_apriltag[2]);
    Eigen::Quaterniond orientationcarameLink_apriltag;
    orientationcarameLink_apriltag.coeffs()<<orientation_apriltag.x(),orientation_apriltag.y(),orientation_apriltag.z(),orientation_apriltag.w();
    Eigen::Affine3d transformcarameLink_apriltag=translationocarameLink_apriltag*orientationcarameLink_apriltag.toRotationMatrix();
    Eigen::Affine3d transformapriltag_old(transformapriltag);
    transformapriltag=transformB_C*transformcarameLink_apriltag;

    }else{

    }


    Eigen::Vector3d position_base_apriltag(transformapriltag.translation());
    Eigen::Quaterniond orientation_base_apriltag(transformapriltag.linear());


      /* code */




    Eigen::Translation3d translationapriltag_OTEE(-0.00535012,-0.0337731,-0.0523339);

    qtn.setRPY(0,0,0);
    Eigen::Quaterniond orientationapriltag_OTEE;
    orientationapriltag_OTEE.coeffs()<<qtn.getX(),qtn.getY(),qtn.getZ(),qtn.getW();
    Eigen::Affine3d transformapriltag_OTEE=translationapriltag_OTEE*orientationapriltag_OTEE.toRotationMatrix();

  


    Eigen::Affine3d transformtarget_link=transformapriltag*transformapriltag_OTEE;
    Eigen::Vector3d position_base_apriltag_link(transformtarget_link.translation());
    Eigen::Quaterniond orientation_base_apriltag_link(transformtarget_link.linear());


    Eigen::Translation3d translationoapriltag_target(0.1*cos(i/2000)-0.1,0.1*sin(i/2000),0.4);
    Eigen::Quaterniond orientationapriltag_target;
    qtn.setRPY(-3.1415,0,-4.71);
    orientationapriltag_target.coeffs()<<qtn.getX(),qtn.getY(),qtn.getZ(),qtn.getW();
    Eigen::Affine3d transformapriltag_target=translationoapriltag_target*orientationapriltag_target.toRotationMatrix();

    Eigen::Affine3d transformtarget=transformapriltag*transformapriltag_OTEE*transformapriltag_target;

    position_base_target=transformtarget.translation();
    orientation_base_target=Eigen::Quaterniond(transformtarget.linear());


 tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "target_link";

    //  |----坐标系相对信息设置
    tfs.transform.translation.x = position_base_apriltag_link[0];
    tfs.transform.translation.y = position_base_apriltag_link[1];
    tfs.transform.translation.z = position_base_apriltag_link[2]; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置
  
    tfs.transform.rotation.x = orientation_base_apriltag_link.x();
    tfs.transform.rotation.y = orientation_base_apriltag_link.y();
    tfs.transform.rotation.z = orientation_base_apriltag_link.z();
    tfs.transform.rotation.w = orientation_base_apriltag_link.w();


     //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);

    //  |----头设置
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();

    //  |----坐标系 ID
    tfs.child_frame_id = "end_ODEE";

    //  |----坐标系相对信息设置
    tfs.transform.translation.x = position[0];
    tfs.transform.translation.y = position[1];
    tfs.transform.translation.z = position[2]; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置

    tfs.transform.rotation.x = orientation.x();
    tfs.transform.rotation.y = orientation.y();
    tfs.transform.rotation.z = orientation.z();
    tfs.transform.rotation.w = orientation.w();


    //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);


    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "apriltag_camera";

    //  |----坐标系相对信息设置
    tfs.transform.translation.x = position_C[0];
    tfs.transform.translation.y = position_C[1];
    tfs.transform.translation.z = position_C[2]; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置
  
    tfs.transform.rotation.x = orientation_C.x();
    tfs.transform.rotation.y = orientation_C.y();
    tfs.transform.rotation.z = orientation_C.z();
    tfs.transform.rotation.w = orientation_C.w();

    //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);


    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "carameLink";

    //  |----坐标系相对信息设置
    tfs.transform.translation.x = position_CLink[0];
    tfs.transform.translation.y = position_CLink[1];
    tfs.transform.translation.z = position_CLink[2]; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置
  
    tfs.transform.rotation.x = orientation_CLink.x();
    tfs.transform.rotation.y = orientation_CLink.y();
    tfs.transform.rotation.z = orientation_CLink.z();
    tfs.transform.rotation.w = orientation_CLink.w();


    //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);

    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "apriltag";

    //  |----坐标系相对信息设置
    tfs.transform.translation.x = position_base_apriltag[0];
    tfs.transform.translation.y = position_base_apriltag[1];
    tfs.transform.translation.z = position_base_apriltag[2]; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置
  
    tfs.transform.rotation.x = orientation_base_apriltag.x();
    tfs.transform.rotation.y = orientation_base_apriltag.y();
    tfs.transform.rotation.z = orientation_base_apriltag.z();
    tfs.transform.rotation.w = orientation_base_apriltag.w();


    broadcaster.sendTransform(tfs);


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


     //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);




 
    if (receive_apriltag_flag||!first)
    {
        first=false;
        i++;
        position_d_target_<<position_base_target[0],position_base_target[1],position_base_target[2];
        orientation_d_target_.coeffs()<<orientation_base_target.x(),orientation_base_target.y(),orientation_base_target.z(),orientation_base_target.w();
    }
    
    tfs.header.frame_id = "apriltag_camera";
    tfs.header.stamp = ros::Time::now();

    //  |----坐标系 ID
    tfs.child_frame_id = "apriltag_topic";

    //  |----坐标系相对信息设置
    tfs.transform.translation.x = position_apriltag[0];
    tfs.transform.translation.y = position_apriltag[1];
    tfs.transform.translation.z = position_apriltag[2]; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置

    tfs.transform.rotation.x = orientation_apriltag.x();
    tfs.transform.rotation.y = orientation_apriltag.y();
    tfs.transform.rotation.z = orientation_apriltag.z();
    tfs.transform.rotation.w = orientation_apriltag.w();


    //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);




  Eigen::Matrix<double, 6, 1> error;
 
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
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

    for (size_t i = 0; i < 3; i++)
  {
    /* code */
    robot_state_look.position[i]=error[i];
    robot_state_look.orientation[i]=error[3+i];
  }

  for (size_t i = 0; i < 7; i++)
  {
    /* code */
      robot_state_look.tau_commanded[i]=tau_d[i];
  }
  
  Pub_RobotState.publish(robot_state_look);
  this_pose_stamped.header.frame_id="world";
  this_pose_stamped.header.stamp=ros::Time::now();
  this_pose_stamped.pose.position.x = position_base_target[0];
  this_pose_stamped.pose.position.y = position_base_target[1];
  this_pose_stamped.pose.position.z = position_base_target[2]; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置
  this_pose_stamped.pose.orientation.x = orientation_base_target.x();
  this_pose_stamped.pose.orientation.y = orientation_base_target.y();
  this_pose_stamped.pose.orientation.z = orientation_base_target.z();
  this_pose_stamped.pose.orientation.w = orientation_base_target.w();
  path.poses.push_back(this_pose_stamped);
  path_pub.publish(path);

    this_pose_stamped.pose.position.x = position[0];
  this_pose_stamped.pose.position.y = position[1];
  this_pose_stamped.pose.position.z = position[2]; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置
  this_pose_stamped.pose.orientation.x = orientation.x();
  this_pose_stamped.pose.orientation.y = orientation.y();
  this_pose_stamped.pose.orientation.z = orientation.z();
  this_pose_stamped.pose.orientation.w = orientation.w();

  path1.poses.push_back(this_pose_stamped);
  path1_pub.publish(path1);

  ros::spinOnce();
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

Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleControllerPVBS::saturateTorqueRate(
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

void CartesianImpedanceExampleControllerPVBS::complianceParamCallback(
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

void CartesianImpedanceExampleControllerPVBS::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
   ros::param::get("receive_apriltag_flag",receive_apriltag_flag);
   if (receive_apriltag_flag)
   {
      position_apriltag << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
      Eigen::Quaterniond last_orientation_apriltag(orientation_apriltag);
      orientation_apriltag.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
          msg->pose.orientation.z, msg->pose.orientation.w;
      if (last_orientation_apriltag.coeffs().dot(orientation_apriltag.coeffs()) < 0.0) {
        orientation_apriltag.coeffs() << -orientation_apriltag.coeffs();
      }

   }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleControllerPVBS,
                       controller_interface::ControllerBase)
