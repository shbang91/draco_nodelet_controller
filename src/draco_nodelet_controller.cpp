#include <draco_nodelet_controller/draco_nodelet_controller.hpp>
#include <stdexcept>

using namespace aptk::comm;
using namespace aptk::ctrl;
namespace draco_nodelet_controller {
DracoNodeletController::DracoNodeletController() {

  this->LoadConfigFile();

  medullas_ = {"Medulla", "Medulla_V4"};
  sensillums_ = {"Sensillum_v2"};

  // Full configuration of the robot
  lower_leg_axons_ = {"R_Hip_IE",   "R_Hip_AA",   "R_Hip_FE",   "R_Knee_FE",
                      "R_Ankle_FE", "R_Ankle_IE", "L_Hip_IE",   "L_Hip_AA",
                      "L_Hip_FE",   "L_Knee_FE",  "L_Ankle_FE", "L_Ankle_IE"};

  upper_body_axons_ = {"Neck_Pitch",    "R_Shoulder_FE", "R_Shoulder_AA",
                       "R_Shoulder_IE", "R_Elbow",       "R_Wrist_Roll",
                       "R_Wrist_Pitch", "L_Shoulder_FE", "L_Shoulder_AA",
                       "L_Shoulder_IE", "L_Elbow",       "L_Wrist_Roll",
                       "L_Wrist_Pitch"};

  axons_ = {"Neck_Pitch",    "R_Hip_IE",      "R_Hip_AA",      "R_Hip_FE",
            "R_Knee_FE",     "R_Ankle_FE",    "R_Ankle_IE",    "L_Hip_IE",
            "L_Hip_AA",      "L_Hip_FE",      "L_Knee_FE",     "L_Ankle_FE",
            "L_Ankle_IE",    "L_Shoulder_FE", "L_Shoulder_AA", "L_Shoulder_IE",
            "L_Elbow",       "L_Wrist_Roll",  "L_Wrist_Pitch", "R_Shoulder_FE",
            "R_Shoulder_AA", "R_Shoulder_IE", "R_Elbow",       "R_Wrist_Roll",
            "R_Wrist_Pitch"};

  // joint_names_ = {"l_hip_ie",      "l_hip_aa",      "l_hip_fe",
  //"l_knee_fe_jp", "l_knee_fe_jd",    "l_ankle_fe",    "l_ankle_ie",
  //"l_shoulder_fe", "l_shoulder_aa", "l_shoulder_ie",
  //"l_elbow_fe",    "l_wrist_ps",    "l_wrist_pitch",
  //"neck_pitch",    "r_hip_ie",      "r_hip_aa",
  //"r_hip_fe",      "r_knee_fe_jp", "r_knee_fe_jd",    "r_ankle_fe",
  //"r_ankle_ie",    "r_shourder_fe", "r_shourder_aa",
  //"r_shourder_ie", "r_erbow_fe",    "r_wrist_ps",
  //"r_wrist_pitch"} //pinocchio robot model joint order

  pin_joints_idx_ = {7, 8, 9, 10, 10, 11, 12, 13, 14, 15, 16, 17, 18, 0,
                     1, 2, 3, 4,  4,  5,  6,  19, 20, 21, 22, 23, 24};

  // TEST: when the right leg was disconnected
  // lower_leg_axons_ = {"L_Hip_IE",  "L_Hip_AA",   "L_Hip_FE",
  //"L_Knee_FE", "L_Ankle_FE", "L_Ankle_IE"};
  // upper_body_axons_ = {"Neck_Pitch",    "R_Shoulder_FE", "R_Shoulder_AA",
  //"R_Shoulder_IE", "R_Elbow",       "R_Wrist_Roll",
  //"R_Wrist_Pitch"};

  // axons_ = {"Neck_Pitch",    "L_Hip_IE",      "L_Hip_AA",      "L_Hip_FE",
  //"L_Knee_FE",     "L_Ankle_FE",    "L_Ankle_IE",    "L_Shoulder_FE",
  //"L_Shoulder_AA", "L_Shoulder_IE", "L_Elbow",       "L_Wrist_Roll",
  //"L_Wrist_Pitch", "R_Shoulder_FE", "R_Shoulder_AA", "R_Shoulder_IE",
  //"R_Elbow",       "R_Wrist_Roll",  "R_Wrist_Pitch"};

  // joint_names_ = {
  //"neck_pitch",    "l_hip_ie",      "l_hip_aa",      "l_hip_fe",
  //"l_knee_fe",     "l_ankle_fe",    "l_ankle_ie",    "l_shoulder_fe",
  //"l_shoulder_aa", "l_shoulder_ie", "l_elbow_fe",    "l_wrist_ps",
  //"l_wrist_pitch", "r_shoulder_fe", "r_shoulder_aa", "r_shoulder_ie",
  //"r_elbow_fe",    "r_wrist_ps",    "r_wrist_pitch"};
  // TEST END

  control_mode_ = control_mode::kOff;

  count_ = 0;
  sleep_time_ = 0.7;
  n_joints_ = axons_.size();
  n_medulla_ = medullas_.size();
  n_sensillum_ = sensillums_.size();

  ph_joint_positions_data_.resize(n_joints_);
  ph_motor_positions_data_.resize(n_joints_);
  ph_linkage_speed_ratio_.resize(n_joints_);
  ph_kp_.resize(n_joints_);
  ph_kd_.resize(n_joints_);
  ph_joint_velocities_data_.resize(n_joints_);
  ph_joint_positions_cmd_.resize(n_joints_);
  ph_joint_velocities_cmd_.resize(n_joints_);
  ph_joint_efforts_cmd_.resize(n_joints_);
  ph_current_cmd_.resize(n_joints_);

  b_rpc_alive_ = true;

  actuator_speed_ratio_.resize(n_joints_);
  motor_pos_polarity_.resize(n_joints_);
  diff_jpos_mjpos_ = Eigen::VectorXd::Zero(n_joints_);

  rpc_interface_ = new DracoInterface();
  rpc_sensor_data_ = new DracoSensorData();
  rpc_command_ = new DracoCommand();

  computation_time_ = 0.;

  // for changing orientation standard local NED -> ROS standard by rotating
  // 180deg on x axis
  world_q_local_ned_ = Eigen::Quaternion<double>(0, 1, 0, 0);
  world_q_imu_ = Eigen::Quaternion<double>::Identity();
  world_av_imu_.setZero();

  b_initializing_imu_ = true;
  b_change_to_off_mode_ = false;
  b_change_to_motor_current_mode_ = false;
  b_change_to_joint_impedance_mode_ = false;
  b_change_lb_to_joint_impedance_mode_ = false;
  b_change_ub_to_joint_impedance_mode_ = false;
  b_clear_faults_ = false;
  b_destruct_rpc_ = false;
  b_construct_rpc_ = false;
  b_gains_limits_ = false;
  b_fake_estop_released_ = false;
  b_interrupt_ = false;
  interrupt_data_ = 0;

  lb_low_level_kp_gains_.resize(lower_leg_axons_.size());
  lb_low_level_kd_gains_.resize(lower_leg_axons_.size());
}

DracoNodeletController::~DracoNodeletController() {
  spin_thread_->join();
  for (int i = 0; i < n_joints_; ++i) {
    delete ph_joint_positions_data_[i];
    delete ph_motor_positions_data_[i];
    delete ph_linkage_speed_ratio_[i];
    delete ph_kp_[i];
    delete ph_kd_[i];
    delete ph_joint_velocities_data_[i];
    delete ph_joint_positions_cmd_[i];
    delete ph_joint_velocities_cmd_[i];
    delete ph_joint_efforts_cmd_[i];
    delete ph_current_cmd_[i];
  }
  delete ph_imu_quaternion_w_ned_;
  delete ph_imu_quaternion_x_ned_;
  delete ph_imu_quaternion_y_ned_;
  delete ph_imu_quaternion_z_ned_;
  delete ph_imu_ang_vel_x_;
  delete ph_imu_ang_vel_y_;
  delete ph_imu_ang_vel_z_;
  delete ph_rfoot_sg_;
  delete ph_lfoot_sg_;
}

void DracoNodeletController::onInit() {
  nh_ = getNodeHandle();
  spin_thread_.reset(new boost::thread(
      boost::bind(&DracoNodeletController::spinThread, this)));

  fault_handler_ = nh_.advertiseService(
      "/fault_handler", &DracoNodeletController::FaultHandler, this);
  mode_handler_ = nh_.advertiseService(
      "/mode_handler", &DracoNodeletController::ModeHandler, this);
  rpc_handler_ = nh_.advertiseService(
      "/rpc_handler", &DracoNodeletController::RPCHandler, this);
  gain_limit_handler_ = nh_.advertiseService(
      "/gains_limits_handler", &DracoNodeletController::GainsAndLimitsHandler,
      this);
  fake_estop_handler_ = nh_.advertiseService(
      "/fake_estop_handler", &DracoNodeletController::FakeEstopHandler, this);
  interrupt_handler_ = nh_.advertiseService(
      "/interrupt_handler", &DracoNodeletController::InterruptHandler, this);

  // low level gain handlers
  right_hip_ie_kp_handler_ =
      nh_.advertiseService("/right_hip_ie_kp_handler",
                           &DracoNodeletController::RightHipIEKpCallback, this);
  right_hip_aa_kp_handler_ =
      nh_.advertiseService("/right_hip_aa_kp_handler",
                           &DracoNodeletController::RightHipAAKpCallback, this);
  right_hip_fe_kp_handler_ =
      nh_.advertiseService("/right_hip_fe_kp_handler",
                           &DracoNodeletController::RightHipFEKpCallback, this);
  right_knee_fe_kp_handler_ = nh_.advertiseService(
      "/right_knee_fe_kp_handler",
      &DracoNodeletController::RightKneeFEKpCallback, this);
  right_ankle_fe_kp_handler_ = nh_.advertiseService(
      "/right_ankle_fe_kp_handler",
      &DracoNodeletController::RightAnkleFEKpCallback, this);
  right_ankle_ie_kp_handler_ = nh_.advertiseService(
      "/right_ankle_ie_kp_handler",
      &DracoNodeletController::RightAnkleIEKpCallback, this);
  right_hip_ie_kd_handler_ =
      nh_.advertiseService("/right_hip_ie_kd_handler",
                           &DracoNodeletController::RightHipIEKdCallback, this);
  right_hip_aa_kd_handler_ =
      nh_.advertiseService("/right_hip_aa_kd_handler",
                           &DracoNodeletController::RightHipAAKdCallback, this);
  right_hip_fe_kd_handler_ =
      nh_.advertiseService("/right_hip_fe_kd_handler",
                           &DracoNodeletController::RightHipFEKdCallback, this);
  right_knee_fe_kd_handler_ = nh_.advertiseService(
      "/right_knee_fe_kd_handler",
      &DracoNodeletController::RightKneeFEKdCallback, this);
  right_ankle_fe_kd_handler_ = nh_.advertiseService(
      "/right_ankle_fe_kd_handler",
      &DracoNodeletController::RightAnkleFEKdCallback, this);
  right_ankle_ie_kd_handler_ = nh_.advertiseService(
      "/right_ankle_ie_kd_handler",
      &DracoNodeletController::RightAnkleIEKdCallback, this);

  left_hip_ie_kp_handler_ =
      nh_.advertiseService("/left_hip_ie_kp_handler",
                           &DracoNodeletController::LeftHipIEKpCallback, this);
  left_hip_aa_kp_handler_ =
      nh_.advertiseService("/left_hip_aa_kp_handler",
                           &DracoNodeletController::LeftHipAAKpCallback, this);
  left_hip_fe_kp_handler_ =
      nh_.advertiseService("/left_hip_fe_kp_handler",
                           &DracoNodeletController::LeftHipFEKpCallback, this);
  left_knee_fe_kp_handler_ =
      nh_.advertiseService("/left_knee_fe_kp_handler",
                           &DracoNodeletController::LeftKneeFEKpCallback, this);
  left_ankle_fe_kp_handler_ = nh_.advertiseService(
      "/left_ankle_fe_kp_handler",
      &DracoNodeletController::LeftAnkleFEKpCallback, this);
  left_ankle_ie_kp_handler_ = nh_.advertiseService(
      "/left_ankle_ie_kp_handler",
      &DracoNodeletController::LeftAnkleIEKpCallback, this);
  left_hip_ie_kd_handler_ =
      nh_.advertiseService("/left_hip_ie_kd_handler",
                           &DracoNodeletController::LeftHipIEKdCallback, this);
  left_hip_aa_kd_handler_ =
      nh_.advertiseService("/left_hip_aa_kd_handler",
                           &DracoNodeletController::LeftHipAAKdCallback, this);
  left_hip_fe_kd_handler_ =
      nh_.advertiseService("/left_hip_fe_kd_handler",
                           &DracoNodeletController::LeftHipFEKdCallback, this);
  left_knee_fe_kd_handler_ =
      nh_.advertiseService("/left_knee_fe_kd_handler",
                           &DracoNodeletController::LeftKneeFEKdCallback, this);
  left_ankle_fe_kd_handler_ = nh_.advertiseService(
      "/left_ankle_fe_kd_handler",
      &DracoNodeletController::LeftAnkleFEKdCallback, this);
  left_ankle_ie_kd_handler_ = nh_.advertiseService(
      "/left_ankle_ie_kd_handler",
      &DracoNodeletController::LeftAnkleIEKdCallback, this);
}

void DracoNodeletController::spinThread() {
  sync_.reset(new aptk::comm::Synchronizer(true, "draco_nodelet_controller"));
  sync_->connect();

  debug_interface_.reset(new aptk::util::DebugInterfacer(
      "draco", sync_->getNodeHandle(), sync_->getLogger()));
  debug_interface_->addPrimitive(&computation_time_, "rpc_computation_time");
  debug_interface_->addEigen(&world_q_imu_.coeffs(), "world_q_imu",
                             {"x", "y", "z", "w"});
  debug_interface_->addEigen(&world_av_imu_, "world_av_imu", {"x", "y", "z"});
  debug_interface_->addEigen(&diff_jpos_mjpos_, "diff_jpos_mjpos", axons_);

  aptk::comm::enableRT(5, 2);

  RegisterData();

  SetGainsAndLimits();

  TurnOffMotors();

  ClearFaults();

  // main control loop
  while (sync_->ok()) {
    // wait for bus transaction
    sync_->awaitNextControl();
    // handle service calls based on the flag variables
    ProcessServiceCalls();
    // copy data
    CopyData();
    if (sync_->printIndicatedFaults() && (!b_fake_estop_released_)) {
      // faulted
      SetSafeCommand();
    } else {
      if (control_mode_ == control_mode::kMotorCurrent) {
        // do nothing while going through initial ramp
        SetSafeCommand();
      } else {
        // compute command from rpc
        if (b_measure_computation_time_) {
          init_time_ = Clock::now();
        }
        rpc_interface_->getCommand(rpc_sensor_data_, rpc_command_);
        if (b_measure_computation_time_) {
          end_time_ = Clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
              end_time_ - init_time_);
          computation_time_ = static_cast<double>(duration.count());
        }
        CopyCommand();
      }
    }
    UpdateLowLevelGains();

    // indicate that we're done
    sync_->finishControl();

    ++count_;
    debug_interface_->updateDebug();
  }

  sync_->awaitShutdownComplete();
}

void DracoNodeletController::SetSafeCommand() {
  for (int i = 0; i < n_joints_; ++i) {
    *(ph_joint_positions_cmd_[i]) = *(ph_joint_positions_data_[i]);
    *(ph_joint_velocities_cmd_[i]) = 0.;
    *(ph_joint_efforts_cmd_[i]) = 0.;
    *(ph_current_cmd_[i]) = 0.;
  }
}

void DracoNodeletController::ProcessServiceCalls() {

  if (b_change_to_off_mode_) {
    TurnOffMotors();
    b_change_to_off_mode_ = false;
  }
  if (b_change_to_motor_current_mode_) {
    TurnOnMotorCurrent();
    b_change_to_motor_current_mode_ = false;
  }
  if (b_change_to_joint_impedance_mode_) {
    TurnOnJointImpedance();
    b_change_to_joint_impedance_mode_ = false;
  }
  if (b_change_lb_to_joint_impedance_mode_) {
    TurnOnLowerBodyJointImpedance();
    b_change_lb_to_joint_impedance_mode_ = false;
  }
  if (b_change_ub_to_joint_impedance_mode_) {
    TurnOnUpperBodyJointImpedance();
    b_change_ub_to_joint_impedance_mode_ = false;
  }
  if (b_clear_faults_) {
    ClearFaults();
    b_clear_faults_ = false;
  }
  if (b_construct_rpc_) {
    ConstructRPC();
    b_construct_rpc_ = false;
  }
  if (b_destruct_rpc_) {
    DestructRPC();
    b_destruct_rpc_ = false;
  }
  if (b_gains_limits_) {
    SetGainsAndLimits();
    b_gains_limits_ = false;
  }
  if (b_interrupt_) {
    if (interrupt_data_ == 1) {
      // com swaying
      rpc_interface_->interrupt->PressOne();
    } else if (interrupt_data_ == 3) {
      // com interpolate
    } else {
      // do nothing
    }
    b_interrupt_ = false;
  }
}

void DracoNodeletController::RegisterData() {
  std::cout << "DracoNodeletController::RegisterData()" << std::endl;
  int r_ankle_ie_idx(0), l_ankle_ie_idx(0), r_ankle_fe_idx(0),
      l_ankle_fe_idx(0);
  for (int i = 0; i < n_joints_; ++i) {
    // get parameter
    sync_->getParam<float>("Actuator__Speed_ratio", actuator_speed_ratio_[i],
                           axons_[i]);
    if (axons_[i] == "L_Ankle_IE") {
      motor_pos_polarity_[i] = -1.;
    } else {
      motor_pos_polarity_[i] = 1.;
    }

    // register encoder data
    ph_joint_positions_data_[i] = new float(0.);
    sync_->registerMISOPtr(ph_joint_positions_data_[i],
                           "js__joint__position__rad", axons_[i], false);
    ph_joint_velocities_data_[i] = new float(0.);
    sync_->registerMISOPtr(ph_joint_velocities_data_[i],
                           "js__joint__velocity__radps", axons_[i], false);
    ph_motor_positions_data_[i] = new float(0.);
    sync_->registerMISOPtr(ph_motor_positions_data_[i], "motor__position__rad",
                           axons_[i], false);
    ph_linkage_speed_ratio_[i] = new float(0.);
    sync_->registerMISOPtr(ph_linkage_speed_ratio_[i], "linkage__speedRatio",
                           axons_[i], false);

    // register commands for joint impedance control mode
    ph_joint_positions_cmd_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_joint_positions_cmd_[i],
                           "cmd__joint__position__rad", axons_[i], false);
    ph_joint_velocities_cmd_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_joint_velocities_cmd_[i],
                           "cmd__joint__velocity__radps", axons_[i], false);
    ph_joint_efforts_cmd_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_joint_efforts_cmd_[i], "cmd__joint__effort__nm",
                           axons_[i], false);
    ph_current_cmd_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_current_cmd_[i], "cmd__motor__effort__a",
                           axons_[i], false);

    ph_kp_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_kp_[i], "gain__joint_impedance_kp__nmprad",
                           axons_[i], false);
    ph_kd_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_kd_[i], "gain__joint_impedance_kd__nmsprad",
                           axons_[i], false);

    // for linkage table
    if (axons_[i] == "R_Ankle_FE") {
      sync_->registerMOSIPtr(ph_joint_positions_data_[i],
                             "ext__joint_position__rad", "R_Ankle_IE", false);
    }
    if (axons_[i] == "L_Ankle_FE") {
      sync_->registerMOSIPtr(ph_joint_positions_data_[i],
                             "ext__joint_position__rad", "L_Ankle_IE", false);
    }
  }

  ph_imu_quaternion_w_ned_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_quaternion_w_ned_, "IMU__quaternion_w__mps2",
                         sensillums_[0], false);
  ph_imu_quaternion_x_ned_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_quaternion_x_ned_, "IMU__quaternion_x__mps2",
                         sensillums_[0], false);
  ph_imu_quaternion_y_ned_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_quaternion_y_ned_, "IMU__quaternion_y__mps2",
                         sensillums_[0], false);
  ph_imu_quaternion_z_ned_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_quaternion_z_ned_, "IMU__quaternion_z__mps2",
                         sensillums_[0], false);
  ph_imu_ang_vel_x_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_ang_vel_x_, "IMU__comp_angularRate_x__radps",
                         sensillums_[0], false);
  ph_imu_ang_vel_y_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_ang_vel_y_, "IMU__comp_angularRate_y__radps",
                         sensillums_[0], false);
  ph_imu_ang_vel_z_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_ang_vel_z_, "IMU__comp_angularRate_z__radps",
                         sensillums_[0], false);
  ph_rfoot_sg_ = new float(0.);
  sync_->registerMISOPtr(ph_rfoot_sg_, "foot__sg__x", "R_Ankle_IE", false);
  ph_lfoot_sg_ = new float(0.);
  sync_->registerMISOPtr(ph_lfoot_sg_, "foot__sg__x", "L_Ankle_IE", false);
}

void DracoNodeletController::CopyData() {
  // Process IMU
  Eigen::Quaternion<double> local_ned_q_frame(
      *ph_imu_quaternion_w_ned_, *ph_imu_quaternion_x_ned_,
      *ph_imu_quaternion_y_ned_, *ph_imu_quaternion_z_ned_);
  Eigen::Vector3d frameAVframe(*ph_imu_ang_vel_x_, *ph_imu_ang_vel_y_,
                               *ph_imu_ang_vel_z_);
  world_q_imu_ = world_q_local_ned_ * local_ned_q_frame;
  world_av_imu_ = world_q_imu_ * frameAVframe;

  // Set imu data
  rpc_sensor_data_->imu_frame_quat_ << world_q_imu.vec(), world_q_imu.w();
  rpc_sensor_data_->imu_ang_vel = world_av_imu_;

  // compute joint positions from motor to check gear ratio set correctly
  for (int i = 0; i < n_joints_; ++i) {
    float joint_pos = *(ph_joint_positions_data_[i]);
    float motor_pos = *(ph_motor_positions_data_[i]);
    float dynamic_speed_ratio = *(ph_linkage_speed_ratio_[i]);
    float mom_arm = actuator_speed_ratio_[i] * dynamic_speed_ratio;
    diff_jpos_mjpos_[i] =
        motor_pos_polarity_[i] * (motor_pos / mom_arm) - joint_pos;
  }

  // Set contact bool
  if (*(ph_rfoot_sg_) > contact_threshold_) {
    rpc_sensor_data_->b_rf_contact = true;
  } else {
    rpc_sensor_data_->b_rf_contact = false;
  }
  if (*(ph_lfoot_sg_) > contact_threshold_) {
    rpc_sensor_data_->b_lf_contact = true;
  } else {
    rpc_sensor_data_->b_lf_contact = false;
  }

  // TODO
  // Set encoder data
  for (int i = 0; i < pin_joints_idx_.size(); i++) {
    rpc_sensor_data_->joint_pos_[i] =
        static_cast<double>(*(ph_joint_positions_data_[pin_joints_idx_[i]]));
    rpc_sensor_data_->joint_vel_[i] =
        static_cast<double>(*ph_joint_velocities_data_[pin_joints_idx_[i]]);
  }
  // update rolling contact joint cmd
  rpc_sensor_data_->joint_pos_[3] =
      static_cast<double>(*(ph_joint_positions_data_[10]) * 0.5);
  rpc_sensor_data_->joint_pos_[4] =
      static_cast<double>(*(ph_joint_positions_data_[10]) * 0.5);
  rpc_sensor_data_->joint_pos_[17] =
      static_cast<double>(*(ph_joint_positions_data_[4]) * 0.5);
  rpc_sensor_data_->joint_pos_[18] =
      static_cast<double>(*(ph_joint_positions_data_[4]) * 0.5);

  rpc_sensor_data_->joint_vel_[3] =
      static_cast<double>(*(ph_joint_velocities_data_[10]) * 0.5);
  rpc_sensor_data_->joint_vel_[4] =
      static_cast<double>(*(ph_joint_velocities_data_[10]) * 0.5);
  rpc_sensor_data_->joint_vel_[17] =
      static_cast<double>(*(ph_joint_velocities_data_[4]) * 0.5);
  rpc_sensor_data_->joint_vel_[18] =
      static_cast<double>(*(ph_joint_velocities_data_[4]) * 0.5);
}

// TODO
void DracoNodeletController::CopyCommand() {
  for (int i = 0; i < pin_joints_idx_.size(); i++) {
    *(ph_joint_positions_cmd_[pin_joints_idx_[i]]) =
        static_cast<float>(rpc_command_->joint_pos_cmd_[i]);
    *(ph_joint_velocities_cmd_[pin_joints_idx_[i]]) =
        static_cast<float>(rpc_command_->joint_vel_cmd_[i]);
    *(ph_joint_efforts_cmd_[pin_joints_idx_[i]]) =
        static_cast<float>(rpc_command_->joint_trq_cmd_[i]);
  }
  // update rolling contact joint cmd
  *(ph_joint_positions_cmd_[10]) =
      static_cast<float>(rpc_command_->joint_pos_cmd_[4] * 2.);
  *(ph_joint_positions_cmd_[4]) =
      static_cast<float>(rpc_command_->joint_pos_cmd_[18] * 2.);
  *(ph_joint_velocities_cmd_[10]) =
      static_cast<float>(rpc_command_->joint_vel_cmd_[4] * 2.);
  *(ph_joint_velocities_cmd_[4]) =
      static_cast<float>(rpc_command_->joint_vel_cmd_[18] * 2.);
  *(ph_joint_efforts_cmd_[10]) =
      static_cast<float>(rpc_command_->joint_trq_cmd_[4] * 0.5);
  *(ph_joint_efforts_cmd_[4]) =
      static_cast<float>(rpc_command_->joint_trq_cmd_[18] * 0.5);
}

void DracoNodeletController::SetGainsAndLimits() {
  this->LoadConfigFile();
  bool b_conservative =
      util::ReadParameter<bool>(nodelet_cfg_["service_call"], "conservative");
  apptronik_srvs::Float32 srv_float_kp;
  apptronik_srvs::Float32 srv_float_kd;
  apptronik_srvs::Float32 srv_float_current_limit;

  for (int i = 0; i < n_joints_; ++i) {
    if (b_conservative) {
      *(ph_kp_[i]) = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "weak_kp");
      *(ph_kd_[i]) = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "weak_kd");
      srv_float_kp.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "weak_kp");
      srv_float_kd.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "weak_kd");
      srv_float_current_limit.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "weak_current_limit");
      // nodelet_cfg_["service_call"][joint_names_[i]], "current_limit");
    } else {
      *(ph_kp_[i]) = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "kp");
      *(ph_kd_[i]) = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "kd");
      srv_float_kp.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "kp");
      srv_float_kd.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "kd");
      srv_float_current_limit.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "current_limit");
    }
    CallSetService(axons_[i], "Control__Joint__Impedance__KP", srv_float_kp);
    sleep(sleep_time_);
    CallSetService(axons_[i], "Control__Joint__Impedance__KD", srv_float_kd);
    sleep(sleep_time_);
    CallSetService(axons_[i], "Limits__Motor__Effort__Saturate__Relative_val",
                   srv_float_current_limit);
    sleep(sleep_time_);
  }
  for (int i = 0; i < lower_leg_axons_.size(); ++i) {
    lb_low_level_kp_gains_[i] = util::ReadParameter<float>(
        nodelet_cfg_["service_call"][lower_body_joint_names_[i]], "kp");
    lb_low_level_kd_gains_[i] = util::ReadParameter<float>(
        nodelet_cfg_["service_call"][lower_body_joint_names_[i]], "kd");
  }
}

void DracoNodeletController::ConstructRPC() {
  if (!b_rpc_alive_) {
    rpc_interface_ = new DracoInterface();
    b_rpc_alive_ = true;
  } else {
    std::cerr << "[[[Warning]]] rpc is already alive" << std::endl;
  }
}

void DracoNodeletController::DestructRPC() {
  if (b_rpc_alive_) {
    delete rpc_interface_;
    b_rpc_alive_ = false;
  } else {
    std::cout << "[[[Warning]]] rpc is already desturcted" << std::endl;
  }
}

bool DracoNodeletController::FaultHandler(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "[[[Clear the faults]]]" << std::endl;
    b_clear_faults_ = true;
    return true;
  } else if (data == 1) {
    std::cout << "[[[Clear the faults]]]" << std::endl;
    b_clear_faults_ = true;
  } else {
    std::cout << "[[[Warning]]] Wrong Data Received for ModeHandler()"
              << std::endl;
    return false;
  }
}

bool DracoNodeletController::InterruptHandler(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  b_interrupt_ = true;
  interrupt_data_ = static_cast<int>(req.set_data);
  return true;
}

bool DracoNodeletController::ModeHandler(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "[[[Change to MOTOR_Off Mode]]]" << std::endl;
    b_change_to_off_mode_ = true;
    control_mode_ = control_mode::kOff;
    return true;
  } else if (data == 1) {
    std::cout << "[[[Change to MOTOR_CURRENT Mode]]]" << std::endl;
    b_change_to_motor_current_mode_ = true;
    control_mode_ = control_mode::kMotorCurrent;
  } else if (data == 2) {
    std::cout << "[[[Change to JOINT_IMPEDANCE Mode]]]" << std::endl;
    b_change_to_joint_impedance_mode_ = true;
    control_mode_ = control_mode::kJointImpedance;
    return true;
  } else if (data == 3) {
    b_change_lb_to_joint_impedance_mode_ = true;
    control_mode_ = control_mode::kJointImpedance;
  } else if (data == 4) {
    b_change_ub_to_joint_impedance_mode_ = true;
    control_mode_ = control_mode::kJointImpedance;
  } else {
    std::cout << "[[[Warning]]] Wrong Data Received for ModeHandler()"
              << std::endl;
    return false;
  }
}

bool DracoNodeletController::RPCHandler(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "[[[Destruct RPC]]]" << std::endl;
    b_destruct_rpc_ = true;
    return true;
  } else if (data == 1) {
    std::cout << "[[[Construct RPC]]]" << std::endl;
    b_construct_rpc_ = true;
    return true;
  } else {
    std::cout << "[[[Warning]]] Wrong Data Received for RPCHandler()"
              << std::endl;
    return false;
  }
}

bool DracoNodeletController::GainsAndLimitsHandler(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "[[[Reset Gains and Current Limits]]]" << std::endl;
    b_gains_limits_ = true;
    return true;
  } else if (data == 1) {
    std::cout << "[[[Reset Gains and Current Limits]]]" << std::endl;
    b_gains_limits_ = true;
    return true;
  } else {
    std::cout << "[[[Warning]]] Wrong Data Received for GainsAndLimitsHandler()"
              << std::endl;
    return false;
  }
}

bool DracoNodeletController::FakeEstopHandler(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "[[[Fake Estop Enabled]]]" << std::endl;
    b_fake_estop_released_ = false;
    return true;
  } else if (data == 1) {
    std::cout << "[[[Fake Estop Disabled]]]" << std::endl;
    b_fake_estop_released_ = true;
    return true;
  } else {
    std::cout << "[[[Warning]]] Wrong Data Received for IMUHandler()"
              << std::endl;
    return false;
  }
}

void DracoNodeletController::TurnOffMotors() {
  b_fake_estop_released_ = false;
  for (int i = 0; i < n_joints_; ++i) {
    sync_->changeMode("OFF", axons_[i]);
    sleep(sleep_time_);
  }
}

void DracoNodeletController::TurnOnUpperBodyJointImpedance() {
  b_fake_estop_released_ = true;
  if (b_rpc_alive_) {
    for (int i = 0; i < upper_body_axons_.size(); ++i) {
      sync_->changeMode("JOINT_IMPEDANCE", upper_body_axons_[i]);
      sleep(sleep_time_);
    }
  } else {
    std::cout << "PnC is not alive. Construct PnC before change the mode"
              << std::endl;
  }
}

void DracoNodeletController::TurnOnLowerBodyJointImpedance() {
  b_fake_estop_released_ = true;
  if (b_rpc_alive_) {
    for (int i = 0; i < lower_leg_axons_.size(); ++i) {
      sync_->changeMode("JOINT_IMPEDANCE", lower_leg_axons_[i]);
      sleep(sleep_time_);
    }
  } else {
    std::cout << "PnC is not alive. Construct PnC before change the mode"
              << std::endl;
  }
}

void DracoNodeletController::TurnOnJointImpedance() {
  if (b_rpc_alive_) {
    for (int i = 0; i < n_joints_; ++i) {
      sync_->changeMode("JOINT_IMPEDANCE", axons_[i]);
      sleep(sleep_time_);
    }
  } else {
    std::cout << "PnC is not alive. Construct PnC before change the mode"
              << std::endl;
  }
}

void DracoNodeletController::TurnOnMotorCurrent() {
  for (int i = 0; i < n_joints_; ++i) {
    sync_->changeMode("MOTOR_CURRENT", axons_[i]);
    sleep(1.0);
  }
}

void DracoNodeletController::ClearFaults() {
  for (int i = 0; i < n_joints_; ++i) {
    sync_->clearFaults(axons_[i]);
    sleep(sleep_time_);
  }
}

void DracoNodeletController::LoadConfigFile() {
  nodelet_cfg_ = YAML::LoadFile(THIS_COM "config/draco/nodelet.yaml");
  YAML::Node rpc_cfg_ = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

  // TODO check this
  bool b_sim = util::ReadParameter<bool>(rpc_cfg_, "b_sim");
  while (b_sim) {
    std::cout << "waiting b_sim to be false";
    ros::spinOnce();
    bool b_sim = util::ReadParameter<bool>(rpc_cfg_, "b_sim");
  }
  // try {
  // if (b_sim)
  // throw b_sim:
  //} catch (bool e) {
  // std::cerr << "b_sim is set to true!" << std::endl;
  // return;
  //}

  target_joint_ =
      util::ReadParameter<std::string>(nodelet_cfg_, "target_joint");

  contact_threshold_ =
      util::ReadParameter<double>(nodelet_cfg_, "contact_threshold");
  sleep_time_ = util::ReadParameter<double>(nodelet_cfg_, "sleep_time");

  b_measure_computation_time_ =
      util::ReadParameter<bool>(nodelet_cfg_, "b_measure_computation_time");
}

void DracoNodeletController::UpdateLowLevelGains() {
  for (int i = 0; i < lower_leg_axons_.size(); ++i) {
    auto find_idx =
        std::find(axons_.begin(), axons_.end(), lower_leg_axons_[i]);
    int idx = find_idx - axons_.begin();
    *(ph_kp_[idx]) = lb_low_level_kp_gains_[i];
    *(ph_kd_[idx]) = lb_low_level_kd_gains_[i];
  }
}

bool DracoNodeletController::RightHipIEKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[0] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::RightHipAAKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[1] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::RightHipFEKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[2] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::RightKneeFEKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[3] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::RightAnkleFEKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[4] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::RightAnkleIEKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[5] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftHipIEKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[6] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftHipAAKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[7] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftHipFEKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[8] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftKneeFEKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[9] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftAnkleFEKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[10] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftAnkleIEKpCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kp_gains_[11] = static_cast<float>(req.set_data);
}

bool DracoNodeletController::RightHipIEKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[0] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::RightHipAAKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[1] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::RightHipFEKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[2] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::RightKneeFEKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[3] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::RightAnkleFEKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[4] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::RightAnkleIEKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[5] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftHipIEKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[6] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftHipAAKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[7] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftHipFEKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[8] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftKneeFEKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[9] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftAnkleFEKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[10] = static_cast<float>(req.set_data);
}
bool DracoNodeletController::LeftAnkleIEKdCallback(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  lb_low_level_kd_gains_[11] = static_cast<float>(req.set_data);
}
template <class SrvType>
void DracoNodeletController::CallSetService(const std::string &slave_name,
                                            const std::string &srv_name,
                                            SrvType &srv_obj) {
  std::string full_set_service =
      "/" + slave_name + "/" + srv_name + "/" + "set";
  ros::NodeHandle nh = getPrivateNodeHandle(); // for Nodelets

  ros::ServiceClient client = nh.serviceClient<SrvType>(full_set_service);

  if (client.call(srv_obj)) {
    NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/"
                                   << srv_name.c_str()); // for Nodelets
  } else {
    NODELET_INFO_STREAM(
        "Failed to call service: " << full_set_service.c_str()); // for Nodelets
  }
}

template <class SrvType>
void DracoNodeletController::CallGetService(const std::string &slave_name,
                                            const std::string &srv_name,
                                            SrvType &srv_obj) {
  std::string full_get_service =
      "/" + slave_name + "/" + srv_name + "/" + "get";
  ros::NodeHandle nh = getPrivateNodeHandle(); // for Nodelets

  ros::ServiceClient client = nh.serviceClient<SrvType>(full_get_service);

  if (client.call(srv_obj)) {
    NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/"
                                   << srv_name.c_str()); // for Nodelets
  } else {
    NODELET_INFO_STREAM(
        "Failed to call service: " << full_get_service.c_str()); // for Nodelets
  }
}

} // namespace draco_nodelet_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(draco_nodelet_controller::DracoNodeletController,
                       nodelet::Nodelet)
