/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm/FSMState_JointPD.h"
#include "common/timeMarker.h"
// #include <Utilities/Utilities_print.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
FSMState_JointPD::FSMState_JointPD(std::shared_ptr<ControlFSMData> data)
: FSMState(data, FSMStateName::JOINT_PD, "joint_pd")
{
  // Do nothing
  // Set the pre controls safety checks
  // this->checkSafeOrientation = false;

  // Post control safety checks
  // this->checkPDesFoot = false;
  // this->checkForceFeedForward = false;
  size_t dof = 3;
  initial_jpos.setZero(dof);

  for(int i=0;i<3;i++)
  {
    this->motor_pid_test[i].PID_Init(Common, 0);
    this->motor_pid_test[i].getMicroTick_regist(getSystemTime);
  }
  this->motor_pid_test[2].Params_Config(0.25,10.,0.,2.,2.);
}

void FSMState_JointPD::enter()
{
  std::cout << "checkin" << std::endl;
  // Default is to not transition
  this->_nextStateName = this->_stateName;
  initial_jpos = this->_data->low_state->q;
  initial_jpos(0) = 0.;

  this->threadRunning = true;
  if (this->thread_first_)
  {
    this->forward_thread = std::thread(&FSMState_JointPD::_controller_loop, this);
    this->thread_first_ = false;
  }
  this->stop_update_ = false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */

void FSMState_JointPD::run()
{
  std::cout << "passive running" << std::endl;
  _data->low_cmd->zero();
  // Eigen::Map<DVec<scalar_t>> kp_joint(
  //   _data->params->joint_kp.data(), _data->params->joint_kp.size()),
  //   kd_joint(_data->params->joint_kd.data(), _data->params->joint_kd.size());
  Vec3<scalar_t> kp_joint = Vec3<scalar_t>(14,10,10);
  Vec3<scalar_t> kd_joint = Vec3<scalar_t>(1,1,1);
  DVec<scalar_t> initial_djpos(initial_jpos.size());
  initial_djpos.setZero();
  this->_data->low_cmd->tau_cmd[0] = 0.;
  this->_data->low_cmd->tau_cmd[1] = 0.;
  this->_data->low_cmd->tau_cmd[2] = this->test_out[2];
  
  // _data->low_cmd->tau_cmd = kp_joint.cwiseProduct(initial_jpos - _data->low_state->q) +
  //                           kd_joint.cwiseProduct(initial_djpos - _data->low_state->dq);
  // for (Eigen::Index i(0); i < initial_jpos.size(); ++i) {
  //   bound(_data->low_cmd->tau_cmd(i), _data->params->torque_limit[i]);
  // }
}

void FSMState_JointPD::_controller_loop()
{
  while(this->threadRunning)
  {
    long long _start_time = getSystemTime();
    if(!this->stop_update_)
    {
      this->motor_pid_test[2].target = ref_v_mf.f(this->_data->state_command->rc_data_->twist_linear[point::X]/0.175);
      if(abs(this->motor_pid_test[2].target)<0.1)
      {
        this->motor_pid_test[2].target = 0;
      }
      this->motor_pid_test[2].current = _data->low_state->dq[2];
      this->motor_pid_test[2].Adjust(0,_data->low_state->dq[2]);
      this->test_out[2] = this->motor_pid_test[2].out;
    }
    absoluteWait(_start_time, (long long)(0.0025 * 1000000));
  }
  this->threadRunning = false;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */

FSMStateName FSMState_JointPD::checkTransition()
{
  this->_nextStateName = this->_stateName;

  // Switch FSM control mode
  switch (_data->state_command->desire_data_->fsm_state_name) {
    case FSMStateName::JOINT_PD:
    break;
    case FSMStateName::RL: // normal c
    this->_nextStateName = FSMStateName::RL;
      break;
    case FSMStateName::TRADITION_CTRL:
      this->_nextStateName = FSMStateName::TRADITION_CTRL;
      break;
    case FSMStateName::PASSIVE:  // normal c
      this->_nextStateName = FSMStateName::PASSIVE;
      break;
    default:
      break;
      // std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
      //           << K_RECOVERY_STAND << " to "
      //           << this->_data->controlParameters->control_mode << std::endl;
  }
  // Get the next state
  return this->_nextStateName;
}

/**
 * Cleans up the state information on exiting the state.
 */

void FSMState_JointPD::exit()
{
  // Nothing to clean up when exiting
  this->stop_update_ = true;
}
