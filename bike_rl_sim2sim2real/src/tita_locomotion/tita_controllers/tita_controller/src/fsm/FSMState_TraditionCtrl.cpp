/*============================= TraditionCtrl ==============================*/
/**
 * Control state use traditional method such as pid, lqr, mpc, etc.
 */

#include "fsm/FSMState_TraditionCtrl.h"
#include "common/timeMarker.h"

/**
 * FSM State that use traditional controller.
 */
FSMState_TraditionCtrl::FSMState_TraditionCtrl(std::shared_ptr<ControlFSMData> data)
    : FSMState(data, FSMStateName::TRADITION_CTRL, "tradition_ctrl")
{
  // use 2dof
  this->bike_pid_params.motor_enList[0] = true;
  this->bike_pid_params.motor_enList[1] = false;
  this->bike_pid_params.motor_enList[2] = true;

  // use direct roll ctrl
  this->bike_pid_params.heading_enList[0] = true;
  this->bike_pid_params.heading_enList[1] = true;

  // pid init
  this->bike_balance_pid.PID_Init(Common, 0);
  this->bike_balance_pid.getMicroTick_regist(getSystemTime);
  for (int i = 0; i < 3; i++)
  {
    if (i < 2)
    {
      this->bike_heading_pid[i].PID_Init(Common, 0);
      this->bike_heading_pid[i].getMicroTick_regist(getSystemTime);
      this->bike_motor_pid[i + 1].d_of_current = false;
    }
    this->bike_motor_pid[i].PID_Init(Common, 0);
    this->bike_motor_pid[i].getMicroTick_regist(getSystemTime);
  }
  this->bike_heading_pid[1].d_of_current = false;

  // pid params
  this->bike_pid_params.balance_kp = -20.;
  this->bike_pid_params.balance_ki = 0.;
  this->bike_pid_params.balance_kd = 0.5;
  this->bike_pid_params.balance_imax = 0.5;
  this->bike_pid_params.balance_lim = 2.;

  this->bike_pid_params.heading_kp[0] = 0.5;
  this->bike_pid_params.heading_ki[0] = 0.;
  this->bike_pid_params.heading_kd[0] = -0.;
  this->bike_pid_params.heading_imax[0] = 0.;
  this->bike_pid_params.heading_lim[0] = 1.;

  this->bike_pid_params.heading_kp[1] = 0.;
  this->bike_pid_params.heading_ki[1] = -0.2;
  this->bike_pid_params.heading_kd[1] = 0.;
  this->bike_pid_params.heading_imax[1] = 0.15;
  this->bike_pid_params.heading_lim[1] = 0.15;

  // high gain
  // this->bike_pid_params.motor_kp[0] = 40.;
  // this->bike_pid_params.motor_ki[0] = 0.;
  // this->bike_pid_params.motor_kd[0] = -3.;
  // this->bike_pid_params.motor_imax[0] = 0.;
  // this->bike_pid_params.motor_lim[0] = 20.;

  // low gain
  this->bike_pid_params.motor_kp[0] = 20.;
  this->bike_pid_params.motor_ki[0] = 0.;
  this->bike_pid_params.motor_kd[0] = -2.;
  this->bike_pid_params.motor_imax[0] = 0.;
  this->bike_pid_params.motor_lim[0] = 20.;

  this->bike_pid_params.motor_kp[1] = 20.;
  this->bike_pid_params.motor_ki[1] = 0.;
  this->bike_pid_params.motor_kd[1] = 0.;
  this->bike_pid_params.motor_imax[1] = 0.;
  this->bike_pid_params.motor_lim[1] = 30.;

  this->bike_pid_params.motor_kp[2] = 10.;
  this->bike_pid_params.motor_ki[2] = 2.;
  this->bike_pid_params.motor_kd[2] = -0.2;
  this->bike_pid_params.motor_imax[2] = 10.;
  this->bike_pid_params.motor_lim[2] = 30.;

  // bike discription
  this->bike_state.com_dist = 0.4651025;
  this->bike_state.wheel_dist = 1.02065;
  this->bike_state.wheel_radius = 0.33;
}

void FSMState_TraditionCtrl::enter()
{
  // set pid params
  this->_pid_params_update();
  // update body state
  this->_bike_state_update();
}

void FSMState_TraditionCtrl::run()
{
  this->_data->low_cmd->qd.setZero();
  this->_data->low_cmd->qd_dot.setZero();
  this->_data->low_cmd->kp.setZero();
  this->_data->low_cmd->kd.setZero();
  this->_data->low_cmd->tau_cmd.setZero();

  // set pid params
  this->_pid_params_update();
  // update body state
  this->_bike_state_update();

  // update commands
  this->bike_state.ref_v = this->_data->state_command->rc_data_->twist_linear[point::X];
  this->bike_state.ref_yaw = this->_data->state_command->rc_data_->twist_angular[point::Z];
  this->bike_state.ref_yawVel = this->_data->state_command->rc_data_->twist_angular[point::Z];
  this->bike_state.ref_roll = 0.5 * this->_data->state_command->rc_data_->twist_angular[point::Z];

  // use pid controller
  this->_high_level_pid_cal();
  this->_low_level_pid_cal();
  this->_pid_actuate();

  for (int i = 0; i < 3; i++)
  {
    this->_data->low_cmd->tau_cmd[i] = this->bike_state.ctrl_output[i];
  }
}

void FSMState_TraditionCtrl::exit() {}

FSMStateName FSMState_TraditionCtrl::checkTransition()
{
  // std::cout << "rl state check" << std::endl;
  this->_nextStateName = this->_stateName;

  // Switch FSM control mode
  switch (this->_data->state_command->desire_data_->fsm_state_name)
  {
  case FSMStateName::RECOVERY_STAND:
    this->_nextStateName = FSMStateName::RECOVERY_STAND;
    break;

  case FSMStateName::RL: // normal c
    this->_nextStateName = FSMStateName::RL;
    break;

  case FSMStateName::TRADITION_CTRL:
    break;

  case FSMStateName::TRANSFORM_DOWN:
    this->_nextStateName = FSMStateName::TRANSFORM_DOWN;
    break;

  case FSMStateName::PASSIVE: // normal c
    this->_nextStateName = FSMStateName::PASSIVE;
    break;
  default:
    break;
  }
  return this->_nextStateName;
}

void FSMState_TraditionCtrl::_bike_state_update()
{
  Vec3<double> linvel = this->_data->state_estimator->getResult().vBody;
  Vec3<double> angvel = this->_data->state_estimator->getResult().omegaBody;
  this->bike_state.obs_rollVel = angvel(0);
  this->bike_state.obs_yawVel = angvel(2);
  this->bike_state.obs_v = linvel(0);
  Vec3<double> ang = this->_data->state_estimator->getResult().rpy;
  this->bike_state.obs_roll = ang(0);
  this->bike_state.obs_yaw = ang(2);

  for (int i = 0; i < 3; i++)
  {
    this->bike_state.dof_pos[i] = this->_data->low_state->q[i];
    this->bike_state.dof_vel[i] = this->_data->low_state->dq[i];
  }
}

void FSMState_TraditionCtrl::_pid_params_update()
{
  this->bike_balance_pid.Params_Config(this->bike_pid_params.balance_kp,
                                       this->bike_pid_params.balance_ki,
                                       this->bike_pid_params.balance_kd,
                                       this->bike_pid_params.balance_imax,
                                       this->bike_pid_params.balance_lim);
  for (int i = 0; i < 3; i++)
  {
    if (i < 2)
    {
      this->bike_heading_pid[i].Params_Config(this->bike_pid_params.heading_kp[i],
                                              this->bike_pid_params.heading_ki[i],
                                              this->bike_pid_params.heading_kd[i],
                                              this->bike_pid_params.heading_imax[i],
                                              this->bike_pid_params.heading_lim[i]);
    }
    this->bike_motor_pid[i].Params_Config(this->bike_pid_params.motor_kp[i],
                                          this->bike_pid_params.motor_ki[i],
                                          this->bike_pid_params.motor_kd[i],
                                          this->bike_pid_params.motor_imax[i],
                                          this->bike_pid_params.motor_lim[i]);
  }
}

void FSMState_TraditionCtrl::_high_level_pid_cal()
{
  // task1: heading error -> yaw velocity error
  double angle_err = this->bike_state.ref_yaw - this->bike_state.obs_yaw;
  angle_err = fmod(angle_err, 2.0 * M_PI);
  while (angle_err > M_PI)
  {
    angle_err = angle_err - 2.0 * M_PI;
  }
  while (angle_err < -M_PI)
  {
    angle_err = angle_err + 2.0 * M_PI;
  }
  this->bike_heading_pid[0].target = angle_err;
  this->bike_heading_pid[0].current = 0;
  this->bike_heading_pid[0].Adjust(0, this->bike_state.obs_yawVel);

  // task2: yaw velocity error -> roll error
  if (this->bike_pid_params.heading_enList[0] == true)
  {
    this->bike_state.ref_yawVel = this->bike_heading_pid[0].out;
  }
  this->bike_heading_pid[1].target = this->bike_state.ref_yawVel;
  this->bike_heading_pid[1].current = this->bike_state.obs_yawVel;
  this->bike_heading_pid[1].Adjust(0);

  if (this->bike_pid_params.heading_enList[1] == true)
  {
    this->bike_state.ref_roll = this->bike_heading_pid[1].out;
  }
  else if (this->bike_pid_params.heading_enList[0] == true)
  {
    this->bike_state.ref_roll = this->bike_heading_pid[0].out;
  }

  // task3: velocity error -> rear motor velocity error
}

void FSMState_TraditionCtrl::_low_level_pid_cal()
{
  // task1: roll error -> turn angle -> turn motor pid
  this->bike_balance_pid.target = this->bike_state.ref_roll;
  this->bike_balance_pid.current = this->bike_state.obs_roll;
  this->bike_balance_pid.Adjust(0, this->bike_state.obs_rollVel);
  double real_v = this->bike_state.dof_vel[2]*0.33*0.7;
  // avoid approach 0
  if(real_v<0.5)
  {
    real_v=0.5;
  }
  double ref_com_angle = asin(upper::constrain(this->bike_balance_pid.out * this->bike_state.com_dist / pow(real_v, 2),0.99));
  // ref_com_angle = upper::constrain(ref_com_angle,this->bike_pid_params.balance_lim);
  this->bike_motor_pid[0].target = upper::constrain(atan(this->bike_state.wheel_dist / this->bike_state.com_dist * tan(ref_com_angle)),this->bike_pid_params.balance_lim);
  // this->bike_motor_pid[0].target = -this->bike_state.ref_roll;
  this->bike_motor_pid[0].current = this->bike_state.dof_pos[0];
  this->bike_motor_pid[0].Adjust(0, this->bike_state.dof_vel[0]);

  // task2: rear motor velocity error -> rear motor pid
  this->bike_motor_pid[2].target = this->bike_state.ref_v / this->bike_state.wheel_radius;
  this->bike_motor_pid[2].current = this->bike_state.dof_vel[2];
  this->bike_motor_pid[2].Adjust(0, this->bike_state.dof_vel[2]);
  std::cout << "target: " << this->bike_motor_pid[2].target << std::endl;
  std::cout << "current: " << this->bike_motor_pid[2].current << std::endl;
  std::cout << "out: " << this->bike_motor_pid[2].out << std::endl;
}

void FSMState_TraditionCtrl::_pid_actuate()
{
  // motor ref -> motor torque
  for (int i = 0; i < 3; i++)
  {
    if (this->bike_pid_params.motor_enList[i] == true)
    {
      this->bike_state.ctrl_output[i] = this->bike_motor_pid[i].out;
    }
    else
    {
      this->bike_state.ctrl_output[i] = 0;
    }
  }
}