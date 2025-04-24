#ifndef FSMSTATE_TRADITIONCTRL_H
#define FSMSTATE_TRADITIONCTRL_H

#include <thread>
#include "FSMState.h"
#include "../my_controller_module/Pid/Cpp/include/PIDmethod.h"
#include "../my_filter_module/Cpp/include/my_filters.h"
#include <chrono>

typedef enum _CtrlMode
{
  TEST=0,
  PID,
  LQR
}CtrlMode;

typedef enum _TurnMode
{
  TROLL=0,
  TDIRECTION
}TurnMode;

// params for pid
typedef struct _PidParams
{
  double heading_kp[2] = {0};
  double heading_ki[2] = {0};
  double heading_kd[2] = {0};
  double heading_imax[2] = {0};
  double heading_lim[2] = {0};
  bool heading_enList[2] = {false};

  double motor_kp[3] = {0};
  double motor_ki[3] = {0};
  double motor_kd[3] = {0};
  double motor_imax[3] = {0};
  double motor_lim[3] = {0};
  bool motor_enList[3] = {false};
  
  double balance_kp = 0;
  double balance_ki = 0;
  double balance_kd = 0;
  double balance_imax = 0;
  double balance_lim = 0;
}PidParams;

// params for lqr
typedef struct _LqrParams
{
  double heading_kp[2] = {0};
  double heading_ki[2] = {0};
  double heading_kd[2] = {0};
  double heading_imax[2] = {0};
  double heading_lim[2] = {0};
  bool heading_enList[2] = {false};
  double roll_tar_slope = 0.025;
  double vel_tar_slope = 0.2;

  double motor_kp[3] = {0};
  double motor_ki[3] = {0};
  double motor_kd[3] = {0};
  double motor_imax[3] = {0};
  double motor_lim[3] = {0};
  bool motor_enList[3] = {false};

  Fit_Params lqrs[3] = {0};// 3 state

  double turn_c_kp=1.;
  double turn_c_lim=1.3;

  TurnMode lqr_turn = TurnMode::TROLL;
}LqrParams;

// bike states
typedef struct _BikeCtrlState
{
  double ref_v;
  double obs_v;
  double ref_roll;
  double obs_roll;
  double ref_rollVel;
  double obs_rollVel;
  double ref_yaw;
  double obs_yaw;
  double ref_yawVel;
  double obs_yawVel;
  double ref_turn;
  double obs_turn;

  double wheel_dist;
  double com_dist;
  double com_weight;
  double com_height;
  double wheel_radius;

  double dof_pos[3] = {0};
  double dof_vel[3] = {0};

  double ctrl_output[3] = {0};
}BikeCtrlState;

class FSMState_TraditionCtrl : public FSMState
{
public:
  FSMState_TraditionCtrl(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_TraditionCtrl() {}
  void enter();
  void run();
  void exit();
  FSMStateName checkTransition();

private:
  // state update
  void _bike_state_update();
  BikeCtrlState bike_state;

  // dof test
  void _test_params_update();
  void _test_cal();
  void _test_actuate();

  // generate controller
  PIDmethod bike_motor_pid[3];
  PIDmethod bike_heading_pid[2];
  multiCircle turn_circle = multiCircle(3.1415926);

  // pid controller
  PIDmethod bike_balance_pid;
  PidParams bike_pid_params;
  void _pid_params_update();
  void _high_level_pid_cal();
  void _low_level_pid_cal();
  void _pid_actuate();

  // lqr controller
  PIDmethod bike_lqr_pid[3];
  PIDmethod turn_c_pid;
  LqrParams bike_lqr_params;
  void _lqr_params_update();
  void _high_level_lqr_cal();
  void _low_level_lqr_cal();
  void _lqr_actuate();

  // filters
  MeanFilter<10> real_v_mf;


  CtrlMode ctrl_mode = CtrlMode::TEST;

  // controller thread
  std::thread forward_thread;
  bool threadRunning;
  bool stop_update_ = false;
  bool thread_first_ = true;
  void _controller_loop();

  // clear
  void _pid_clear();
};

#endif