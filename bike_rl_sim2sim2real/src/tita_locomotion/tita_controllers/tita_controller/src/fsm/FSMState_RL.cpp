/*============================= RL ==============================*/
/**
 * Control state use reinforcement learning model.
 */

#include "fsm/FSMState_RL.h"
#include "common/timeMarker.h"

/**
 * FSM State that use reinforcement learning controller.
 */

FSMState_RL::FSMState_RL(std::shared_ptr<ControlFSMData> data)
    : FSMState(data, FSMStateName::RL, "rl")
{
  // this->cuda_test_ = std::make_shared<CudaTest>(data->params->model_engine_path);
  this->model_name = data->params->model_engine_path;

  this->inference_test_ = std::make_shared<RL_InferenceModule>(NUM_OBS, OBS_BUF, NUM_OUTPUT,
                                                               "/home/lu/Git_Project/gitlab/bike_rl/engine/head_4model_11000_P.engine");
  // 使用lambda表达式传入函数指针
  this->inference_test_->set_obs_func([this]() -> const std::vector<float>
                                      { return this->_GetObs(); });

  // this->params_.p_gains[0] = data->params->turn_kp;
  // this->params_.p_gains[1] = data->params->wheel_kp;
  // this->params_.p_gains[2] = data->params->wheel_kp;
  // this->params_.d_gains[0] = data->params->turn_kd;
  // this->params_.d_gains[1] = data->params->wheel_kd;
  // this->params_.d_gains[2] = data->params->wheel_kd;

  // init pid tick and params mode
  this->heading_pid.getMicroTick_regist(getSystemTime);
  this->heading_pid.PID_Init(Common, 0);
  this->heading_pid.Params_Config(0.4, 0., 0., 0.5, 0.8, -0.8);
  // this->heading_pid.integral = 0.3/0.01;
  this->heading_pid.d_of_current = false;

  // 使用 Lambda 捕获 this 指针
  this->dof_actuate[0] = [this]()
  { this->_P_actuate(); };
  this->dof_actuate[1] = [this]()
  { this->_V_actuate(); };
  this->dof_actuate[2] = [this]()
  { this->_T_actuate(); };
  // 配置关节执行模式
  this->dofs_.dof_mode = DofCtrlType::P;
}

void FSMState_RL::enter()
{
  // std::cout << "rl state enter" << std::endl;

  this->_data->state_command->firstRun = true;
  this->use_lpf_actions = true;

  for (int i = 0; i < NUM_OUTPUT; i++)
  {
    this->desired_pos[i] = this->_data->low_state->q[i];
    this->obs_.dof_pos[i] = this->_data->low_state->q[i];
    this->obs_.dof_vel[i] = this->_data->low_state->dq[i];
  }

  this->params_.action_scale = 0.25;
  this->params_.num_of_dofs = 3;
  this->params_.lin_vel_scale = 2.0;
  this->params_.ang_vel_scale = 0.25;
  this->params_.dof_pos_scale = 1.0;
  this->params_.dof_vel_scale = 0.05;
  this->params_.commands_scale[0] = this->params_.lin_vel_scale;
  this->params_.commands_scale[1] = this->params_.lin_vel_scale;
  this->params_.commands_scale[2] = this->params_.ang_vel_scale;
  this->params_.commands_scale[3] = this->params_.lin_vel_scale;

  this->dofs_.P_p[0] = 20;
  this->dofs_.P_d[0] = 1.;
  // this->dofs_.P_p[0] = 40;
  // this->dofs_.P_d[0] = 5.;
  this->dofs_.P_p[1] = 10;
  this->dofs_.P_d[1] = 0.2;
  if (NUM_OUTPUT == 3)
  {
    this->dofs_.P_p[2] = 10;
    this->dofs_.P_d[2] = 0.2;
  }

  this->dofs_.V_p[0] = 10.;
  this->dofs_.V_d[0] = 0.02;
  this->dofs_.V_p[1] = 10.;
  this->dofs_.V_d[1] = 0.015;
  if (NUM_OUTPUT == 3)
  {
    this->dofs_.V_p[2] = 10.;
    this->dofs_.V_d[2] = 0.015;
  }

  this->dofs_.T_d[0] = 0.3;
  this->dofs_.T_d[1] = 0.2;
  if (NUM_OUTPUT == 3)
  {
    this->dofs_.T_d[2] = 0.2;
  }

  this->params_.torque_limits[0] = 3;
  this->params_.torque_limits[1] = 20;
  this->params_.torque_limits[2] = 20;

  // const float default_dof_pos_tmp[NUM_OUTPUT] = {0.};
  this->heading_cmd_ = 0.;

  this->obs_.forward_vec[0] = 1.0;
  this->obs_.forward_vec[1] = 0.0;
  this->obs_.forward_vec[2] = 0.0;

  for (int j = 0; j < NUM_OUTPUT; j++)
  {
    // this->action[j] = obs_.dof_pos[j];
    this->action[j] = 0;
  }

  this->inference_test_->run_prepare();

  this->threadRunning[0] = true;
  this->threadRunning[1] = true;
  if (this->thread_first_[0])
  {
    this->ctrl_thread[0] = std::thread(&FSMState_RL::_Run_Forward, this);
    this->thread_first_[0] = false;
  }

  if (this->thread_first_[1])
  {
    this->ctrl_thread[1] = std::thread(&FSMState_RL::_Run_Lowlevel, this);
    this->thread_first_[1] = false;
  }
  this->stop_update_[0] = false;
  this->stop_update_[1] = false;

  for (int i = 0; i < 3; i++)
  {
    this->torques[i] = 0;
  }
}

void FSMState_RL::run()
{
  // std::cout << "rl state run" << std::endl;
  // update cmds
  this->x_vel_cmd_ = this->_data->state_command->rc_data_->twist_linear[point::X];
  this->heading_cmd_ = this->_data->state_command->rc_data_->twist_angular[point::Z];
  // this->x_vel_cmd_=1.;
  // _data->state_command->rc_data_->twist_angular[point::Z]
  this->_data->low_cmd->qd.setZero();
  this->_data->low_cmd->qd_dot.setZero();
  this->_data->low_cmd->kp.setZero();
  this->_data->low_cmd->kd.setZero();
  this->_data->low_cmd->tau_cmd.setZero();
  std::cout << "torques: \n" << this->torques[0] << "\n"
            << this->torques[1] << "\n"
            << this->torques[2] << std::endl;
  for (int i = 0; i < NUM_OUTPUT; i++)
  {
    //  this->_data->low_cmd->tau_cmd[i] = upper::constrain(this->torques[i],this->params_.torque_limits[i]);
    this->_data->low_cmd->tau_cmd[i] = 0;
  }

  for (int i = 0; i < NUM_OUTPUT; i++)
  {
    this->obs_.dof_pos_last[i] = this->obs_.dof_pos[i];
    this->obs_.dof_vel_last[i] = this->obs_.dof_vel[i];
  }
}

void FSMState_RL::exit()
{
  // std::cout << "rl state exit" << std::endl;
  this->stop_update_[0] = true;
  this->stop_update_[1] = true;
}

FSMStateName FSMState_RL::checkTransition()
{
  // std::cout << "rl state check" << std::endl;
  this->_nextStateName = this->_stateName;

  // Switch FSM control mode
  switch (this->_data->state_command->desire_data_->fsm_state_name)
  {
  case FSMStateName::RL: // normal c
    break;

  case FSMStateName::TRADITION_CTRL:
    this->_nextStateName = FSMStateName::TRADITION_CTRL;
    break;

  case FSMStateName::JOINT_PD:
    this->_nextStateName = FSMStateName::JOINT_PD;
    break;

  case FSMStateName::PASSIVE: // normal c
    this->_nextStateName = FSMStateName::PASSIVE;
    break;
  default:
    break;
  }
  return this->_nextStateName;
}

const std::vector<float> FSMState_RL::_GetObs()
{
  std::vector<float> obs_buff;
  // compute gravity
  Mat3<double> _B2G_RotMat = this->_data->state_estimator->getResult().rBody;
  Mat3<double> _G2B_RotMat = this->_data->state_estimator->getResult().rBody.transpose();

  Vec3<double> linvel = this->_data->state_estimator->getResult().vBody;
  // Vec3<double> angvel = a_l;
  // a_l = 0.97 * this->_data->state_estimator->getResult().omegaBody + 0.03 * a_l;
  Vec3<double> angvel = this->_data->state_estimator->getResult().omegaBody;
  Vec3<double> projected_gravity = _B2G_RotMat * Vec3<double>(0.0, 0.0, -1.0);
  Vec3<double> projected_forward = _G2B_RotMat * Vec3<double>(1.0, 0.0, 0.0);

  // obs_tmp.push_back(linvel(0)*this->params_.lin_vel_scale);
  // obs_tmp.push_back(linvel(1)*this->params_.lin_vel_scale);
  // obs_tmp.push_back(linvel(2)*this->params_.lin_vel_scale);

  obs_buff.push_back(angvel(0) * this->params_.ang_vel_scale);
  obs_buff.push_back(angvel(1) * this->params_.ang_vel_scale);
  obs_buff.push_back(angvel(2) * this->params_.ang_vel_scale);

  // std::cout << "angvelC: " << angvel(0) << ", " << angvel(1) << ", " << angvel(2) << std::endl;

  if (NUM_OBS == 22)
  {
    for (int i = 0; i < 3; ++i)
    {
      obs_buff.push_back(projected_forward(i));
    }
  }

  for (int i = 0; i < 3; ++i)
  {
    obs_buff.push_back(projected_gravity(i));
  }

  // cmd
  double angle_err = (double)this->heading_cmd_ - this->_data->state_estimator->getResult().rpy(2, 0);
  angle_err = fmod(angle_err, 2.0 * M_PI);
  while (angle_err > M_PI)
  {
    angle_err = angle_err - 2.0 * M_PI;
  }
  while (angle_err < -M_PI)
  {
    angle_err = angle_err + 2.0 * M_PI;
  }

  this->heading_pid.target = angle_err;
  this->heading_pid.current = 0;
  angle_err = this->heading_pid.Adjust(0);

  // angle_err = (double)this->heading_cmd_;

  obs_buff.push_back(this->x_vel_cmd_ * this->params_.commands_scale[0]);
  obs_buff.push_back(0.0);
  obs_buff.push_back(angle_err * this->params_.commands_scale[2]);
  if (NUM_OBS == 22)
  {
    obs_buff.push_back((double)this->heading_cmd_);
    // obs_buff.push_back(0);
  }

  // std::cout << "commend: " << this->x_vel_cmd_ << ", " << this->heading_cmd_ << ", " << angle_err << std::endl;
  // std::cout << "angVel: " << angvel(2) << std::endl;
  

  // pos
  for (int i = 0; i < NUM_OUTPUT; ++i)
  {
    float pos = (this->obs_.dof_pos[i] - this->params_.default_dof_pos[i]) * params_.dof_pos_scale;
    obs_buff.push_back(pos);
  }
  // vel
  for (int i = 0; i < NUM_OUTPUT; ++i)
  {
    float vel = this->obs_.dof_vel[i] * params_.dof_vel_scale;
    obs_buff.push_back(vel);
  }

  // last action
  for (int i = 0; i < NUM_OUTPUT; ++i)
  {
    obs_buff.push_back(this->action_last[i]);
  }

  return obs_buff;
}

void FSMState_RL::_Run_Forward()
{
  while (this->threadRunning[0])
  {
    long long _start_time = getSystemTime();

    if (!this->stop_update_[0])
    {
      // update current dof positions and velocities
      for (int i = 0; i < NUM_OUTPUT; i++)
      {
        this->obs_.dof_pos[i] = this->_data->low_state->q[i];
        this->obs_.dof_vel[i] = this->_data->low_state->dq[i];
      }

      this->inference_test_->forward();
      std::shared_ptr<float[]> get_output(this->inference_test_->get_action());

      // calculate actions
      for (int j = 0; j < NUM_OUTPUT; j++)
      {
        if (this->use_lpf_actions == true)
        {
          this->action[j] = (0.8 * get_output.get()[j] + 0.2 * this->action_last[j]) * this->params_.action_scale;
        }
        else
        {
          this->action[j] = get_output.get()[j] * this->params_.action_scale;
        }

        this->action_last[j] = get_output.get()[j];
      }

      for (int i = 0; i < NUM_OUTPUT; i++)
      {
        this->desired_pos[i] = this->action[i] + this->params_.default_dof_pos[i];
      }
    }

    absoluteWait(_start_time, (long long)(0.01 * 1000000));
  }
  this->threadRunning[0] = false;
}

void FSMState_RL::_Run_Lowlevel()
{
  while (this->threadRunning[1])
  {
    long long _start_time = getSystemTime();

    if (!this->stop_update_[1])
    {
      this->dof_actuate[this->dofs_.dof_mode]();
    }
    absoluteWait(_start_time, (long long)(0.0025 * 1000000));
  }
  this->threadRunning[1] = false;
}

void FSMState_RL::_P_actuate()
{
  for (int i = 0; i < NUM_OUTPUT; i++)
  {
    if (i == 0)
    {
      this->torques[i] = this->dofs_.P_p[i] * (this->desired_pos[i] - this->_data->low_state->q[i]) + this->dofs_.P_d[i] * (0 - this->_data->low_state->dq[i]);
    }
    else
    {
      if (NUM_OUTPUT == 2)
      {
        this->torques[2] = this->dofs_.P_p[i] * this->desired_pos[i] + this->dofs_.P_d[i] * (0 - this->_data->low_state->dq[i]);
        this->torques[1] = 0;
      }
      else
      {
        this->torques[i] = this->dofs_.P_p[i] * this->desired_pos[i] + this->dofs_.P_d[i] * (0 - this->_data->low_state->dq[i]);
      }
    }
  }
}

void FSMState_RL::_V_actuate()
{
  for (int i = 0; i < NUM_OUTPUT; i++)
  {
    if (i == 0)
    {
      this->torques[i] = this->dofs_.V_p[i] * (this->action[i] - this->obs_.dof_vel[i]) - this->dofs_.V_d[i] * (this->obs_.dof_vel[i] - this->obs_.dof_vel_last[i]) / 0.01;
    }
    else
    {
      if (NUM_OUTPUT == 2)
      {
        this->torques[2] = this->dofs_.V_p[i] * (this->action[i] - this->obs_.dof_vel[i]) - this->dofs_.V_d[i] * (this->obs_.dof_vel[i] - this->obs_.dof_vel_last[i]) / 0.01;
        this->torques[1] = 0;
      }
      else
      {
        this->torques[i] = this->dofs_.V_p[i] * (this->action[i] - this->obs_.dof_vel[i]) - this->dofs_.V_d[i] * (this->obs_.dof_vel[i] - this->obs_.dof_vel_last[i]) / 0.01;
      }
    }
  }
}

void FSMState_RL::_T_actuate()
{
  for (int i = 0; i < NUM_OUTPUT; i++)
  {
    if (i == 0)
    {
      this->torques[i] = this->action[i] - this->dofs_.T_d[i] * this->obs_.dof_vel[i];
    }
    else
    {
      if (NUM_OUTPUT == 2)
      {
        this->torques[2] = this->action[i] - this->dofs_.V_d[i] * this->obs_.dof_vel[i];
        this->torques[1] = 0;
      }
      else
      {
        this->torques[i] = this->action[i] - this->dofs_.V_d[i] * this->obs_.dof_vel[i];
      }
    }
  }
}
