#pragma once

#include <Eigen/Dense>
#include <cassert>
#include <chrono>
#include <iostream>

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <apptronik_msgs/Float32Stamped.h>
#include <apptronik_srvs/Float32.h>
#include <cortex_framework/odometry_sensors/vn100_sensor.hpp>
#include <cortex_utils/debug_interfacer.hpp>
#include <rt_utils/synchronizer.hpp>

// RPC related headers
//#include <configuration.hpp>
#include <controller/draco_controller/draco_interface.hpp>
#include <util/util.hpp>

typedef std::chrono::high_resolution_clock Clock;

namespace draco_nodelet_controller {

namespace control_mode {
constexpr int kOff = 0;
constexpr int kMotorCurrent = 1;
constexpr int kJointImpedance = 2;
} // namespace control_mode

class DracoNodeletController : public nodelet::Nodelet {
public:
  void spinThread();
  void onInit();
  DracoNodeletController();
  ~DracoNodeletController();

private:
  int control_mode_;

  // RT kernel
  ros::NodeHandle nh_;
  boost::shared_ptr<aptk::comm::Synchronizer> sync_;
  boost::shared_ptr<boost::thread> spin_thread_;
  boost::shared_ptr<aptk::util::DebugInterfacer> debug_interface_;

  // service calls
  ros::ServiceServer mode_handler_;
  ros::ServiceServer rpc_handler_;
  ros::ServiceServer gain_limit_handler_;
  ros::ServiceServer fault_handler_;
  ros::ServiceServer fake_estop_handler_;
  ros::ServiceServer interrupt_handler_;

  ros::ServiceServer right_hip_ie_kp_handler_;
  ros::ServiceServer right_hip_aa_kp_handler_;
  ros::ServiceServer right_hip_fe_kp_handler_;
  ros::ServiceServer right_knee_fe_kp_handler_;
  ros::ServiceServer right_ankle_fe_kp_handler_;
  ros::ServiceServer right_ankle_ie_kp_handler_;

  ros::ServiceServer right_hip_ie_kd_handler_;
  ros::ServiceServer right_hip_aa_kd_handler_;
  ros::ServiceServer right_hip_fe_kd_handler_;
  ros::ServiceServer right_knee_fe_kd_handler_;
  ros::ServiceServer right_ankle_fe_kd_handler_;
  ros::ServiceServer right_ankle_ie_kd_handler_;

  ros::ServiceServer left_hip_ie_kp_handler_;
  ros::ServiceServer left_hip_aa_kp_handler_;
  ros::ServiceServer left_hip_fe_kp_handler_;
  ros::ServiceServer left_knee_fe_kp_handler_;
  ros::ServiceServer left_ankle_fe_kp_handler_;
  ros::ServiceServer left_ankle_ie_kp_handler_;

  ros::ServiceServer left_hip_ie_kd_handler_;
  ros::ServiceServer left_hip_aa_kd_handler_;
  ros::ServiceServer left_hip_fe_kd_handler_;
  ros::ServiceServer left_knee_fe_kd_handler_;
  ros::ServiceServer left_ankle_fe_kd_handler_;
  ros::ServiceServer left_ankle_ie_kd_handler_;

  std::vector<float> lb_low_level_kp_gains_;
  std::vector<float> lb_low_level_kd_gains_;
  void UpdateLowLevelGains();

  // timing
  int count_;
  double sleep_time_;

  // counting
  int n_joint_;
  int n_medulla_;
  int n_sensillum_;

  // electronic boards naming
  std::vector<std::string> axons_;
  std::vector<std::string> lower_leg_axons_;
  std::vector<std::string> upper_body_axons_;
  std::vector<int> lower_leg_idx_;
  std::vector<std::string> medullas_;
  std::vector<std::string> sensillums_;

  // actuating joint name corresponding to the axons
  std::vector<std::string> joint_names_;
  std::vector<std::string> lower_body_joint_names_;

  std::unordered_map<std::string, int> axons_to_pin_joints_map_;

  // placeholders for data coming through ecat communication
  // this placeholders shares the same order with axons_
  std::vector<float *> ph_joint_positions_data_;
  std::vector<float *> ph_joint_velocities_data_;
  std::vector<float *> ph_joint_positions_cmd_;
  std::vector<float *> ph_joint_velocities_cmd_;
  std::vector<float *> ph_joint_efforts_cmd_;
  std::vector<float *> ph_linkage_speed_ratio_;
  std::vector<float *> ph_motor_positions_data_;
  std::vector<float *> ph_current_cmd_;
  std::vector<float *> ph_kp_;
  std::vector<float *> ph_kd_;
  float *ph_imu_quaternion_w_ned_, *ph_imu_quaternion_x_ned_,
      *ph_imu_quaternion_y_ned_, *ph_imu_quaternion_z_ned_;
  float *ph_imu_ang_vel_x_, *ph_imu_ang_vel_y_, *ph_imu_ang_vel_z_;
  float *ph_rfoot_sg_, *ph_lfoot_sg_;

  std::vector<float> actuator_speed_ratio_;
  std::vector<float> motor_pos_polarity_;
  Eigen::VectorXd diff_jpos_mjpos_;

  // RPC objects
  DracoInterface *rpc_interface_;
  DracoSensorData *rpc_sensor_data_;
  DracoCommand *rpc_command_;

  std::chrono::high_resolution_clock::time_point init_time_, end_time_;
  bool b_measure_computation_time_;
  bool b_use_int_frc_command_;
  double computation_time_;

  Eigen::Quaternion<double> world_q_local_ned_;
  Eigen::Quaternion<double> world_q_imu_;
  Eigen::Vector3d world_av_imu_;

  double contact_threshold_;

  YAML::Node nodelet_cfg_;

  // flags
  bool b_rpc_alive_;
  bool b_initializing_imu_;
  bool b_change_to_joint_impedance_mode_;
  bool b_change_lb_to_joint_impedance_mode_;
  bool b_change_ub_to_joint_impedance_mode_;
  bool b_change_to_off_mode_;
  bool b_change_to_motor_current_mode_;
  bool b_clear_faults_;
  bool b_destruct_rpc_;
  bool b_construct_rpc_;
  bool b_gains_limits_;
  bool b_fake_estop_released_;
  bool b_interrupt_;
  int interrupt_data_;
  std::string target_joint_; // for the purpose of moving joint one by one.

  // register miso and mosi topics to the placeholders
  void RegisterData();

  // copy placeholder data to sensor_data
  void CopyData();

  // update low level gains
  bool RightHipIEKpCallback(apptronik_srvs::Float32::Request &req,
                            apptronik_srvs::Float32::Response &res);
  bool RightHipAAKpCallback(apptronik_srvs::Float32::Request &req,
                            apptronik_srvs::Float32::Response &res);
  bool RightHipFEKpCallback(apptronik_srvs::Float32::Request &req,
                            apptronik_srvs::Float32::Response &res);
  bool RightKneeFEKpCallback(apptronik_srvs::Float32::Request &req,
                             apptronik_srvs::Float32::Response &res);
  bool RightAnkleFEKpCallback(apptronik_srvs::Float32::Request &req,
                              apptronik_srvs::Float32::Response &res);
  bool RightAnkleIEKpCallback(apptronik_srvs::Float32::Request &req,
                              apptronik_srvs::Float32::Response &res);

  bool RightHipIEKdCallback(apptronik_srvs::Float32::Request &req,
                            apptronik_srvs::Float32::Response &res);
  bool RightHipAAKdCallback(apptronik_srvs::Float32::Request &req,
                            apptronik_srvs::Float32::Response &res);
  bool RightHipFEKdCallback(apptronik_srvs::Float32::Request &req,
                            apptronik_srvs::Float32::Response &res);
  bool RightKneeFEKdCallback(apptronik_srvs::Float32::Request &req,
                             apptronik_srvs::Float32::Response &res);
  bool RightAnkleFEKdCallback(apptronik_srvs::Float32::Request &req,
                              apptronik_srvs::Float32::Response &res);
  bool RightAnkleIEKdCallback(apptronik_srvs::Float32::Request &req,
                              apptronik_srvs::Float32::Response &res);

  bool LeftHipIEKpCallback(apptronik_srvs::Float32::Request &req,
                           apptronik_srvs::Float32::Response &res);
  bool LeftHipAAKpCallback(apptronik_srvs::Float32::Request &req,
                           apptronik_srvs::Float32::Response &res);
  bool LeftHipFEKpCallback(apptronik_srvs::Float32::Request &req,
                           apptronik_srvs::Float32::Response &res);
  bool LeftKneeFEKpCallback(apptronik_srvs::Float32::Request &req,
                            apptronik_srvs::Float32::Response &res);
  bool LeftAnkleFEKpCallback(apptronik_srvs::Float32::Request &req,
                             apptronik_srvs::Float32::Response &res);
  bool LeftAnkleIEKpCallback(apptronik_srvs::Float32::Request &req,
                             apptronik_srvs::Float32::Response &res);

  bool LeftHipIEKdCallback(apptronik_srvs::Float32::Request &req,
                           apptronik_srvs::Float32::Response &res);
  bool LeftHipAAKdCallback(apptronik_srvs::Float32::Request &req,
                           apptronik_srvs::Float32::Response &res);
  bool LeftHipFEKdCallback(apptronik_srvs::Float32::Request &req,
                           apptronik_srvs::Float32::Response &res);
  bool LeftKneeFEKdCallback(apptronik_srvs::Float32::Request &req,
                            apptronik_srvs::Float32::Response &res);
  bool LeftAnkleFEKdCallback(apptronik_srvs::Float32::Request &req,
                             apptronik_srvs::Float32::Response &res);
  bool LeftAnkleIEKdCallback(apptronik_srvs::Float32::Request &req,
                             apptronik_srvs::Float32::Response &res);

  // copy command to placeholder
  void CopyCommand();

  // read yaml, and set gains and current limits
  void SetGainsAndLimits();

  // construct rpc
  void ConstructRPC();

  // destruct rpc
  void DestructRPC();

  // load config yaml files
  void LoadConfigFile();

  // process service calls based on the flag variable
  void ProcessServiceCalls();

  // change mode
  // 0: Off
  // 1: CURRENT
  // 2: JOINT_IMPEDANCE
  // 3: LB_JOINT_IMPEDANCE
  // 4: UB_JOINT_IMPEDANCE
  bool ModeHandler(apptronik_srvs::Float32::Request &req,
                   apptronik_srvs::Float32::Response &res);

  // enable or disable RPC
  // 0 : Destruct
  // 1 : Construct
  bool RPCHandler(apptronik_srvs::Float32::Request &req,
                  apptronik_srvs::Float32::Response &res);

  // enable or disable RPC
  // 0 or 1 : Clear the faults
  bool FaultHandler(apptronik_srvs::Float32::Request &req,
                    apptronik_srvs::Float32::Response &res);

  // set gains and limits
  // 0 or 1 : Set service call with current yaml file
  bool GainsAndLimitsHandler(apptronik_srvs::Float32::Request &req,
                             apptronik_srvs::Float32::Response &res);

  // interrupt handler
  // fixed configuration
  //     4: swing left leg
  //     6: swing right leg
  // floating configuration
  //     1: swaying
  //     3: interpolate
  bool InterruptHandler(apptronik_srvs::Float32::Request &req,
                        apptronik_srvs::Float32::Response &res);

  // set gains and limits
  // 0: Fake estop enabled
  // 1 : Fake estop disabled
  bool FakeEstopHandler(apptronik_srvs::Float32::Request &req,
                        apptronik_srvs::Float32::Response &res);

  // turn off the motors
  void TurnOffMotors();
  // turn on the motors to joint impedance mode
  void TurnOnJointImpedance();
  // turn on the motors to joint impedance mode
  void TurnOnUpperBodyJointImpedance();
  // turn on the motors to joint impedance mode
  void TurnOnLowerBodyJointImpedance();
  // turn on the motors to motor current mode
  void TurnOnMotorCurrent();
  // clear faults on motors
  void ClearFaults();
  // set safe commands
  void SetSafeCommand();

  template <class SrvType>
  void CallGetService(const std::string &slave_name,
                      const std::string &srv_name, SrvType &srv_obj);
  template <class SrvType>
  void CallSetService(const std::string &slave_name,
                      const std::string &srv_name, SrvType &srv_obj);
};

} // namespace draco_nodelet_controller
