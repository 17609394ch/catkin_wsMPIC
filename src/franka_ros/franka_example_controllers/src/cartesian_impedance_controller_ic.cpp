// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_controller_ic.h>

#include <cmath>
#include <memory>
#include <stdio.h>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "franka_example_controllers/State.h"
#include <franka_example_controllers/pseudo_inversion.h>



Eigen::Matrix<double, 3, 1> quatToRotMat(const Eigen::Matrix<double, 4, 1>& q) {
    double e0 = q(0);
    double e1 = q(1);
    double e2 = q(2);
    double e3 = q(3);
  Eigen::Matrix<double, 3, 1> rpy;
    Eigen::Matrix<double, 3, 3> R;
    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
            2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
            1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
            2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
            1 - 2 * (e1 * e1 + e2 * e2);
    rpy(0) = atan2(R(2,1),R(2,2));
    rpy(1) = asin(-R(2,0));
    rpy(2) = atan2(R(1,0),R(0,0));
    return rpy;
}


namespace franka_example_controllers {

bool CartesianImpedanceControllerIc::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;


  node_handle.setParam("constraints_position",10);
  node_handle.setParam("constraints_velocity",10);
  node_handle.setParam("constraints_acceleration",10);


  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceControllerIc::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  Pub_RobotState = node_handle.advertise<franka_example_controllers::State>("Robot_State",1000);
  path_desire_pub = node_handle.advertise<nav_msgs::Path>("trajectory",1, true);
  path_robot_pub = node_handle.advertise<nav_msgs::Path>("trajectory_robot",1, true);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceControllerIc: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceControllerIc: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerIc: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerIc: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerIc: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerIc: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerIc: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceControllerIc: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceControllerIc::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  return true;
}

void CartesianImpedanceControllerIc::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  dynamic_reconfigure_compliance_param_node_.setParam("constraints_position",10);
  dynamic_reconfigure_compliance_param_node_.setParam("constraints_velocity",10);
  dynamic_reconfigure_compliance_param_node_.setParam("constraints_acceleration",10);
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  // std::array<double, 42> jacobian_array =
  //     model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
  orientation_d_target_.coeffs()<<1,0,0,0;
  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
  jacobian_dot.setZero();

   path_desire .header.frame_id="world";
  path_desire.header.stamp=ros::Time::now();
  path_robot.header.frame_id="world";
  path_robot.header.stamp=ros::Time::now();

  int nu=6;
  int nx=18;
  int N=1;
  double Ts=0.001;
  Eigen::MatrixXd K(nu,nx);
  K.block(0,0,nu,nu) = cartesian_mass_.block(0,0,nu,nu).inverse()*cartesian_damping_.block(0,0,nu,nu);
  K.block(0,nu,nu,nu) = cartesian_mass_.block(0,0,nu,nu).inverse()*cartesian_stiffness_.block(0,0,nu,nu);
  K.block(0,2*nu,nu,nu) = cartesian_mass_.block(0,0,nu,nu).inverse()*Eigen::MatrixXd::Identity(nu,nu);
  // ROS_INFO_STREAM(K);
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx,nx);
  A.block(nu,0,nu,nu) = Ts*Eigen::MatrixXd::Identity(nu,nu);

  Eigen::MatrixXd B= Eigen::MatrixXd::Zero(nx,nu);
  B.block(0,0,nu,nu) = Ts*Eigen::MatrixXd::Identity(nu,nu);

  Eigen::MatrixXd selectX = Eigen::MatrixXd::Zero(nx,1);
  Eigen::MatrixXd selectu = Eigen::MatrixXd::Zero(nu,1);

  Eigen::MatrixXd Xmax = Eigen::MatrixXd::Zero(nx,1);
  Eigen::MatrixXd Xmin = Eigen::MatrixXd::Zero(nx,1);
  Eigen::MatrixXd umax = Eigen::MatrixXd::Zero(nu,1);
  Eigen::MatrixXd umin = Eigen::MatrixXd::Zero(nu,1);

  mpic_ = new _MPIC(nx,nu,N);
  mpic_->setSystem(A,B,K);

  for(auto i = 0ul; i < 6; i++){
    double plimits = 10;
    double vlimits = 10;
    double alimits = 10;
    dynamic_reconfigure_compliance_param_node_.getParam("/cartesian_impedance_controller_ic/constraints_position",plimits);
    dynamic_reconfigure_compliance_param_node_.getParam("/cartesian_impedance_controller_ic/constraints_velocity",vlimits);
    dynamic_reconfigure_compliance_param_node_.getParam("/cartesian_impedance_controller_ic/constraints_acceleration",alimits);

    if(vlimits!=0.0){
      Xmax(i,0) = vlimits;
      Xmin(i,0) = -vlimits;
    } 
    
    if(plimits!=0.0){
      selectX(nu+i,0) = 1;
      Xmax(nu+i,0) = plimits;
      Xmin(nu+i,0) = -0;
    }
    // Xmin(7,0) = 0;
    if(alimits!=0.0){
      selectu(i,0) = 1;
      umax(i,0) = alimits;
      umin(i,0) = -alimits;
    }
  } 
  //  ROS_INFO_STREAM(selectX);
  //  ROS_INFO_STREAM( Xmin);

  mpic_->setTimeStep(Ts);

  mpic_->addConstraintsX(selectX,Xmax,Xmin);
  mpic_->addConstraintsU(selectu,umax,umin);
  mpic_->computeQP();

  stateX_ = Eigen::VectorXd::Zero(mpic_->getDimX());
  rk_ = MatrixXd::Zero(mpic_->getHorizon()*(mpic_->getDimU()+mpic_->getDimX()),1);
  rku_tmp_ = MatrixXd::Zero(mpic_->getDimU()*mpic_->getHorizon(),1);
  rkX_tmp_ = MatrixXd::Zero(mpic_->getDimX()*mpic_->getHorizon(),1);
  uOpt_ = MatrixXd::Zero(mpic_->getDimU(),1);

  solved_first_ = false;
// ROS_INFO_STREAM( rk_ );
  FILE * fp;
  fp = fopen ("/home/badboy/outfile/eulerrobot.txt", "w+");
  // fprintf(fp,"%lf\t%lf\t%lf\n",0,0,0);
  fclose(fp);
  fp = fopen ("/home/badboy/outfile/eulerd.txt", "w+"); 
  // fprintf(fp,"%lf\t%lf\t%lf\n",0,0,0);
  fclose(fp);
  fp = fopen ("/home/badboy/outfile/quqtd.txt", "w+");
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\n",0,0,0,0);
  fclose(fp);
  fp = fopen ("/home/badboy/outfile/eulererror.txt", "w+");
  // fprintf(fp,"%lf\t%lf\t%lf\n",0,0,0);
  fclose(fp);
  fp = fopen ("/home/badboy/outfile/quqtrobot.txt", "w+");
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\n",0,0,0,0);
  fclose(fp);
  fp = fopen ("/home/badboy/outfile/error.txt", "w+");//位置误差
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",0,0,0,0,0,0);
  fclose(fp);
  fp = fopen ("/home/badboy/outfile/uOpt.txt", "w+");//优化加速度
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",0,0,0,0,0,0);
  fclose(fp);
  fp = fopen ("/home/badboy/outfile/ddp.txt", "w+");//优化加速度
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",0,0,0,0,0,0);
  fclose(fp);

  fp = fopen ("/home/badboy/outfile/Vrobot.txt", "w+");//机械臂末端速度
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",0,0,0,0,0,0);
  fclose(fp);
  fp = fopen ("/home/badboy/outfile/Vd.txt", "w+");//机械臂末端期望速度
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",0,0,0,0,0,0);
  fclose(fp);

  fp = fopen ("/home/badboy/outfile/Probot.txt", "w+");//机械臂末端轨迹
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",0,0,0,0,0,0);
  fclose(fp);
  fp = fopen ("/home/badboy/outfile/Pd.txt", "w+");//机械臂末端期望轨迹
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",0,0,0,0,0,0);
  fclose(fp);

}

void CartesianImpedanceControllerIc::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 49> mass_array = model_handle_->getMass();
    std::array<double, 7> gravity_array = model_handle_->getGravity();
  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> f_ext_hat(robot_state.K_F_ext_hat_K.data());
   Eigen::Map<Eigen::Matrix<double, 7, 1>> ext_hat(robot_state.tau_ext_hat_filtered.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  orientation_d_target_.coeffs()<<1,0,0,0;
  Eigen::Matrix<double, 4, 1> _quad,_quad_d;
  // f_ext_hat=jacobian*ext_hat;
  _quad<<orientation.w(),orientation.x(),orientation.y(),orientation.z();
  _quad_d<<orientation_d_target_.w(),orientation_d_target_.x(),orientation_d_target_.y(),orientation_d_target_.z();
  Eigen::Vector3d Euler=quatToRotMat(_quad);
  Eigen::Vector3d Euler_p=quatToRotMat(_quad_d);
  for (size_t i = 0; i < 3; i++)
  {
    if(Euler[i]<-3)
      Euler[i]=Euler[i]+M_PI;
       
    if(Euler_p[i]<-3)
      Euler_p[i]=Euler_p[i]+M_PI;
    
    if(Euler[i]>3)
      Euler[i]=Euler[i]- M_PI;
        
    if(Euler_p[i]>3)
      Euler_p[i]=Euler_p[i]-M_PI;
    
  }
  //  Eigen::Vector3d Euler = orientation.toRotationMatrix().eulerAngles(2,1,0);
  // compute error to desired pose 
  // position error
  if (first_run)
  {
      first_run=false;
      s_=jacobian;
  }
  jacobian_dot=(jacobian-s_)/(period.toSec()/filter_params_);
  s_=(1-filter_params_)*s_+filter_params_*jacobian;
  sec_+= period.toSec();
  
  Eigen::Matrix<double, 6, 1>  pr;
  Eigen::Matrix<double, 6, 1>  dpr;
  Eigen::Matrix<double, 6, 1>  ddpr;
  Eigen::Matrix<double, 6, 1>  p;
  Eigen::Matrix<double, 6, 1>  dp;
  Eigen::Matrix<double, 6, 1>  u;
  Eigen::Matrix<double, 7, 1>  v;
  Eigen::Matrix<double, 6, 1>  fext;
  fext.setZero();

  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  // Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // // convert to axis angle
  // Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // // compute "orientation error"
  // error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
  error.tail(3)=(Euler-Euler_p);
  // Eigen::Quaterniond error_quaternion1(orientation * orientation_d_.inverse());
  // Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion1);
  // error.tail(3) << error_quaternion_angle_axis.axis()* error_quaternion_angle_axis.angle();

  if(line_flag==0)
  {
      double a_=0.1;
      double b=1;
      msg_pose.pose.position.x=0.306+a_*cos(sec_/b)+a_;
      
      msg_pose.pose.position.y=a_*sin(sec_/b)*1;
      msg_pose.pose.position.z=0.5;
      position_d_target_ << msg_pose.pose.position.x, msg_pose.pose.position.y, msg_pose.pose.position.z;

      pr<<msg_pose.pose.position.x,msg_pose.pose.position.y, msg_pose.pose.position.z,Euler_p[0],Euler_p[1],Euler_p[2];
      dpr<<-a_*sin(sec_/b)/b,a_*cos(sec_/b)/b*1,0,0,0,0;                           
      ddpr<<-a_*cos(sec_/b)/b/b,-a_*sin(sec_/b)/b/b*1,0,0,0,0; 
      p<<position[0],position[1],position[2],Euler[0],Euler[1],Euler[2];
      dp<<jacobian * dq;
      u=ddpr+cartesian_mass_.inverse()*(cartesian_stiffness_*(-error)+cartesian_damping_*(dpr-dp)-fext);
  }
  for (int i = 0; i < mpic_->getHorizon(); ++i) {
    for (int j =0; j < 6; j++) {
            rku_tmp_(mpic_->getDimU()*i+j) = ddpr[j];
            rkX_tmp_(mpic_->getDimX()*i+j) = dpr[j];
            rkX_tmp_(mpic_->getDimX()*i+mpic_->getDimU()+j) = pr[j];
            rkX_tmp_(mpic_->getDimX()*i+2*mpic_->getDimU()+j) = 0.0;
        }  
    }
  rk_ << rku_tmp_, rkX_tmp_;

  Eigen::VectorXd _q = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd _qv = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd _te = Eigen::VectorXd::Zero(6);

  for (auto index = 0ul; index < 6; ++index){
    // the stats are given in the same order as defines in state_interface_configuration
    _q(index) = p[index];
    _qv(index) = dp[index];
    _te(index) = 0;
  }
  _te(0) = -5*sin(sec_);

  stateX_.head(mpic_->getDimU()) = _qv.head(mpic_->getDimU());
  stateX_.segment(mpic_->getDimU(),mpic_->getDimU()) = _q.head(mpic_->getDimU());
  stateX_.tail(mpic_->getDimU())= _te.head(mpic_->getDimU());

if(!solved_first_){
    mpic_->firstSolveMPIC(stateX_,rk_);
    solved_first_ = true;
  }
  else mpic_->updateSolveMPIC(stateX_,rk_);

  if(mpic_->_QPfail)
    ROS_INFO_STREAM("Faild");
  else 
    uOpt_ = mpic_->getuOpt();





  Eigen::MatrixXd jacobian_pinv;
  pseudoInverse(jacobian, jacobian_pinv);
  v=jacobian_pinv*uOpt_-jacobian_pinv*jacobian_dot*dq;
  Eigen::VectorXd tau_ic(7);
// fext=_te;
  tau_ic=mass*v+jacobian.transpose()*fext;


                         

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
  tau_d << tau_ic  +tau_nullspace+ coriolis;
  // Eigen::Matrix<double, 6, 1> error;
  // error.head(3) << position - position_d_;

  // // orientation error
  // if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
  //   orientation.coeffs() << -orientation.coeffs();
  // }
  // // "difference" quaternion
  // Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  // error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // // Transform to base frame
  // error.tail(3) << -transform.linear() * error.tail(3);;
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
 int nu=6;
  int nx=18;
  Eigen::MatrixXd K(nu,nx);
  K.block(0,0,nu,nu) = cartesian_mass_.block(0,0,nu,nu).inverse()*cartesian_damping_.block(0,0,nu,nu);
  K.block(0,nu,nu,nu) = cartesian_mass_.block(0,0,nu,nu).inverse()*cartesian_stiffness_.block(0,0,nu,nu);
  K.block(0,2*nu,nu,nu) = -cartesian_mass_.block(0,0,nu,nu).inverse()*Eigen::MatrixXd::Identity(nu,nu);
  
  mpic_->updateK(K);
  mpic_->computeQP();
  mpic_->setTimeStep(period.toSec());
  // ROS_INFO_STREAM( K);
  
  // FILE * fp;
  // fp = fopen ("/home/badboy/outfile/eulerrobot.txt", "a+");
  // fprintf(fp,"%lf\t%lf\t%lf\n",Euler[0],Euler[1],Euler[2]);
  // fclose(fp);
  // fp = fopen ("/home/badboy/outfile/eulerd.txt", "a+"); 
  // fprintf(fp,"%lf\t%lf\t%lf\n",Euler_p[0],Euler_p[1],Euler_p[2]);
  // fclose(fp);
  // fp = fopen ("/home/badboy/outfile/quqtd.txt", "a+");
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\n",orientation_d_.w(),orientation_d_.x(),orientation_d_.y(),orientation_d_.z());
  // fclose(fp);
  // fp = fopen ("/home/badboy/outfile/eulererror.txt", "a+");
  // fprintf(fp,"%lf\t%lf\t%lf\n",error[3],error[4],error[5]);
  // fclose(fp);
  // fp = fopen ("/home/badboy/outfile/quqtrobot.txt", "a+");
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\n",orientation.w(),orientation.x(),orientation.y(),orientation.z());
  // fclose(fp);
  // fp = fopen ("/home/badboy/outfile/error.txt", "a+");//位置误差
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",error[0],error[1],error[2],error[3],error[4],error[5]);
  // fclose(fp);
  // fp = fopen ("/home/badboy/outfile/uOpt.txt", "a+");//优化加速度
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",uOpt_(0),uOpt_(1),uOpt_(2),uOpt_(3),uOpt_(4),uOpt_(5));
  // fclose(fp);
  // fp = fopen ("/home/badboy/outfile/ddp.txt", "a+");//优化加速度
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",ddpr(0),ddpr(1),ddpr(2),ddpr(3),ddpr(4),ddpr(5));
  // fclose(fp);

  // fp = fopen ("/home/badboy/outfile/Vrobot.txt", "a+");//机械臂末端速度
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",dp(0),dp(1),dp(2),dp(3),dp(4),dp(5));
  // fclose(fp);
  // fp = fopen ("/home/badboy/outfile/Vd.txt", "a+");//机械臂末端期望速度
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",dpr(0),dpr(1),dpr(2),dpr(3),dpr(4),dpr(5));
  // fclose(fp);

  // fp = fopen ("/home/badboy/outfile/Probot.txt", "a+");//机械臂末端轨迹
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",p(0),p(1),p(2),p(3),p(4),p(5));
  // fclose(fp);
  // fp = fopen ("/home/badboy/outfile/Pd.txt", "a+");//机械臂末端期望轨迹
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",pr(0),pr(1),pr(2),pr(3),pr(4),pr(5));
  // fclose(fp);
  //   fp = fopen ("/home/badboy/outfile/u.txt", "a+");//机械臂末端期望轨迹
  // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",u(0),u(1),u(2),u(3),u(4),u(5));
  // fclose(fp);

  // for (size_t i = 0; i < 3; i++)
  // {
  //   // robot_state_look.error_orientation[i]=Euler[i]-Euler_p[i];
  //   robot_state_look.tau__ext[i]=f_ext_hat[i];
  //   robot_state_look.tau__ext[i+3]=f_ext_hat[i+3];
  //   robot_state_look.error_position[i]=position[i]-position_d_[i];
  //   robot_state_look.error_orientation[i]=Euler[i]-Euler_p[i];
  //   robot_state_look.position[i]=position[i];
  //   robot_state_look.orientation[i]=Euler[i];
  //   robot_state_look.position_d[i]=position_d_[i];
  //   robot_state_look.orientation_d[i]=Euler_p[i];

  // }

  // ROS_INFO("x:%lf,y=%lf,z=%lf",Euler[0],Euler[1],Euler[2]);
  // ROS_INFO("x:%lf,y=%lf,z=%lf",Euler_p[0],Euler_p[1],Euler_p[2]);
  // tf2_ros::TransformBroadcaster broadcaster;

  // geometry_msgs::TransformStamped tfs;
  // Pub_RobotState.publish(robot_state_look);
  // this_pose_stamped.header.frame_id="panda_link0";
  // this_pose_stamped.header.stamp=ros::Time::now();
  // this_pose_stamped.pose.position.x = position_d_[0];
  // this_pose_stamped.pose.position.y = position_d_[1];
  // this_pose_stamped.pose.position.z = position_d_[2]; // 二维实现，pose 中没有z，z 是 0
  //   //  |--------- 四元数设置
  // this_pose_stamped.pose.orientation.x = orientation_d_.x();
  // this_pose_stamped.pose.orientation.y = orientation_d_.y();
  // this_pose_stamped.pose.orientation.z = orientation_d_.z();
  // this_pose_stamped.pose.orientation.w = orientation_d_.w();
  // path_desire.poses.push_back(this_pose_stamped);
  // path_desire_pub.publish(path_desire);

  // this_pose_stamped.pose.position.x = position[0];
  // this_pose_stamped.pose.position.y = position[1];
  // this_pose_stamped.pose.position.z = position[2]; // 二维实现，pose 中没有z，z 是 0
  //   //  |--------- 四元数设置
  // this_pose_stamped.pose.orientation.x = orientation.x();
  // this_pose_stamped.pose.orientation.y = orientation.y();
  // this_pose_stamped.pose.orientation.z = orientation.z();
  // this_pose_stamped.pose.orientation.w = orientation.w();

  // path_robot.poses.push_back(this_pose_stamped);
  // path_robot_pub.publish(path_robot);

  // tfs.header.frame_id = "panda_link0";
  //   tfs.header.stamp = ros::Time::now();
  //   tfs.child_frame_id = "target";

  //   //  |----坐标系相对信息设置
  //   tfs.transform.translation.x = position_base_target[0];
  //   tfs.transform.translation.y = position_base_target[1];
  //   tfs.transform.translation.z = position_base_target[2]; // 二维实现，pose 中没有z，z 是 0
  //   //  |--------- 四元数设置
  
  //   tfs.transform.rotation.x = orientation_base_target.x();
  //   tfs.transform.rotation.y = orientation_base_target.y();
  //   tfs.transform.rotation.z = orientation_base_target.z();
  //   tfs.transform.rotation.w = orientation_base_target.w();


  //   broadcaster.sendTransform(tfs);
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceControllerIc::saturateTorqueRate(
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

void CartesianImpedanceControllerIc::complianceParamCallback(
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

  cartesian_mass_.setIdentity();
    cartesian_mass_.topLeftCorner(3, 3)
      << config.mass_stiffness * Eigen::Matrix3d::Identity();
  cartesian_mass_.bottomRightCorner(3, 3)
      << config.mass_stiffness * Eigen::Matrix3d::Identity();

}

void CartesianImpedanceControllerIc::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& ) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
// position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
position_d_target_ << msg_pose.pose.position.x, msg_pose.pose.position.y, msg_pose.pose.position.z;
  // Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  // orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
  //     msg->pose.orientation.z, msg->pose.orientation.w;
  // if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
  //   orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  // }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceControllerIc,
                       controller_interface::ControllerBase)