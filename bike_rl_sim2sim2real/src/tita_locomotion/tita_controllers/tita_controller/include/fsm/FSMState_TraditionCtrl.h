#ifndef FSMSTATE_TRADITIONCTRL_H
#define FSMSTATE_TRADITIONCTRL_H

#include "FSMState.h"
#include "../my_controller_module/Pid/Cpp/include/PIDmethod.h"

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

}_LqrParams;

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

  double wheel_dist;
  double com_dist;
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
  PIDmethod bike_motor_pid[3];
  PIDmethod bike_heading_pid[2];
  PIDmethod bike_balance_pid;
  PidParams bike_pid_params;
  BikeCtrlState bike_state;

  void _bike_state_update();

  void _pid_params_update();
  void _high_level_pid_cal();
  void _low_level_pid_cal();
  void _pid_actuate();

};

#endif