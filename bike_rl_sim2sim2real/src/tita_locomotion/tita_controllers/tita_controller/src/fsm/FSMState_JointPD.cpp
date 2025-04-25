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
  // 航向位置式pid位置控制参数
  // this->motor_pid_test[0].Params_Config(20.,500.,-2.5,3.,3);
  this->motor_pid_test[0].Params_Config(20.,0.,-2.,1.5,3);
  // 航向位置式pid速度控制参数
  // this->motor_pid_test[0].Params_Config(5.,60,0,3,3);
  // 航向增量式pid速度控制参数
  // this->motor_pid_test[0].Params_Config(1.,0.1,0.0,3.,3.);

  // 轮毂位置式pid速度控制参数
  // this->motor_pid_test[2].Params_Config(0.25,10.,0.,4.,4.);
  // 轮毂增量式pid控制参数
  this->motor_pid_test[2].Params_Config(0.5,0.01,0.05,4.,4.);

  // 转向偏置
  this->turn_circle.set_offset(TURN_OFFSET);
}

void FSMState_JointPD::enter()
{
  std::cout << "checkin" << std::endl;
  // Default is to not transition
  this->_nextStateName = this->_stateName;
  // initial_jpos = this->_data->low_state->q;
  // initial_jpos(0) = 0.;
  for(auto& p:this->motor_pid_test)
  {
    p.Clear();
  }

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
  // std::cout << "passive running" << std::endl;
  _data->low_cmd->zero();

  this->_data->low_cmd->tau_cmd[0] = this->test_out[0];
  this->_data->low_cmd->tau_cmd[1] = 0.;
  this->_data->low_cmd->tau_cmd[2] = this->test_out[2];

  std::cout << "\033[20A\r"; // \033[3A 表示上移3行
  std::cout << "tPr: " << this->motor_pid_test[0].target << "\033[K" << std::endl;
  for (int i = 0; i < _data->low_state->q.rows(); i++)
  {
    std::cout << "motor" << i << ":" << "\033[K" << "\n"
              << "q:  " << _data->low_state->q[i] << "\033[K" << "\n"
              << "dq: " << _data->low_state->dq[i] << "\033[K" << std::endl;
  }
  std::cout << "angVel: \n" << _data->state_estimator->getResult().omegaBody << "\033[K" << std::endl;
  std::cout << "rpy: \n" << _data->state_estimator->getResult().rpy << "\033[K" << std::endl;
  std::cout << "turnOut: " << this->test_out[0] << "\033[K" << std::endl;
  std::cout << "wheelOut: " << this->test_out[2] << "\033[K" << std::endl; 
}

void FSMState_JointPD::_controller_loop()
{
  while(this->threadRunning)
  {
    long long _start_time = getSystemTime();
    if(!this->stop_update_)
    {
      // 航向位置控制测试
      this->motor_pid_test[0].target = -1.5*this->_data->state_command->rc_data_->twist_angular[point::Z];
      this->motor_pid_test[0].current = turn_circle.f(_data->low_state->q[0]);
      this->motor_pid_test[0].Adjust(0,_data->low_state->dq[0]);

      // // 航向速度控制测试
      // this->motor_pid_test[0].target = -2.*this->_data->state_command->rc_data_->twist_angular[point::Z];
      // this->motor_pid_test[0].current = _data->low_state->dq[0];
      // // 位置式pid
      // this->motor_pid_test[0].Adjust(0,_data->low_state->dq[0]);
      // // 增量式pid
      // // this->motor_pid_test[0].Adjust_Incremental();

      this->test_out[0] = this->motor_pid_test[0].out;

      // 轮毂测试
      this->motor_pid_test[2].target = 1.5*this->_data->state_command->rc_data_->twist_linear[point::X]/0.175;
      if(abs(this->motor_pid_test[2].target)<0.1)
      {
        this->motor_pid_test[2].target = 0;
      }
      this->motor_pid_test[2].current = _data->low_state->dq[2];
      // 传统位置式pid
      // this->motor_pid_test[2].Adjust(0,_data->low_state->dq[2]);
      // 使用增量式pid
      this->motor_pid_test[2].Adjust_Incremental();
      this->test_out[2] = this->motor_pid_test[2].out;
    }
    absoluteWait(_start_time, (long long)(0.002 * 1000000));
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
  for(auto& p:this->motor_pid_test)
  {
    p.Clear();
  }
}
