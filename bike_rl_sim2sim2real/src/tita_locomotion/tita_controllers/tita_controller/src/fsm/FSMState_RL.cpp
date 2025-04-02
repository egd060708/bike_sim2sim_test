/*============================= RL ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm/FSMState_RL.h"
#include "common/timeMarker.h"

/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

FSMState_RL::FSMState_RL(std::shared_ptr<ControlFSMData> data)
    : FSMState(data, FSMStateName::RL, "rl"),
      input_0(new float[NUM_OBS]),
      input_1(new float[NUM_OBS * OBS_BUF]),
      output(new float[NUM_OUTPUT]),
      output_last(new float[NUM_OUTPUT]),
      input_1_temp(new float[NUM_OBS * (OBS_BUF - 1)])
{
  // this->cuda_test_ = std::make_shared<CudaTest>(data->params->model_engine_path);
  this->model_name = data->params->model_engine_path;
  // std::cout << "111111111111111111111111111111111111111111111111111111" << std::endl;
  this->cuda_test_ = std::make_shared<CudaTest>("/home/lu/Git_Project/gitlab/bike_rl/engine/17model_10000_simple.engine");
  std::cout << "cuda init :" << this->cuda_test_->get_cuda_init() << std::endl;
  // std::cout << "222222222222222222222222222222222222222222222222222222" << std::endl;
  // this->params_.p_gains[0] = data->params->turn_kp;
  // this->params_.p_gains[1] = data->params->wheel_kp;
  // this->params_.p_gains[2] = data->params->wheel_kp;
  // this->params_.d_gains[0] = data->params->turn_kd;
  // this->params_.d_gains[1] = data->params->wheel_kd;
  // this->params_.d_gains[2] = data->params->wheel_kd;

  // init pid tick and params mode
  this->heading_pid.getMicroTick_regist(getSystemTime);
  this->heading_pid.PID_Init(Common,0);
  this->heading_pid.Params_Config(0.75,0.,0.,0.5,1.5,-1.5);
  // this->heading_pid.integral = 0.3/0.01;
  this->heading_pid.d_of_current = false;
}

void FSMState_RL::enter()
{
  // std::cout << "rl state enter" << std::endl;

  this->_data->state_command->firstRun = true;

  for (int i = 0; i < NUM_OUTPUT; i++)
  {
    this->desired_pos[i] = this->_data->low_state->q[i];
    this->obs_.dof_pos[i] = this->_data->low_state->q[i];
    this->obs_.dof_vel[i] = this->_data->low_state->d
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
  this->params_.p_gains[0] = 40;
  this->params_.p_gains[1] = 10;

  this->params_.d_gains[0] = 5.;
  this->params_.d_gains[1] = 1.;

  if(NUM_OUTPUT == 3)
  {
    this->params_.p_gains[2] = 10;
    this->params_.d_gains[2] = 1.;
  }

  // const float default_dof_pos_tmp[NUM_OUTPUT] = {0.};
  this->heading_cmd_ = 0.;

  for (int i = 0; i < NUM_OBS * (OBS_BUF - 1); i++)
    this->input_1.get()[i] = 0;
  for (int i = 0; i < NUM_OUTPUT; i++)
    this->output_last.get()[i] = 0;

  this->obs_.forward_vec[0] = 1.0;
  this->obs_.forward_vec[1] = 0.0;
  this->obs_.forward_vec[2] = 0.0;

  for (int j = 0; j < NUM_OUTPUT; j++)
  {
    this->action[j] = obs_.dof_pos[j];
  }
  this->a_l.setZero();

  for (int i = 0; i < OBS_BUF; i++)
  {
    // torch::Tensor obs_tensor = GetObs();
    // // append obs_ to obs_ bufferNUM_OBS
    // obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);
    this->_GetObs(true);

    for (int i = 0; i < NUM_OBS * (OBS_BUF - 1); i++)
      input_1_temp.get()[i] = input_1.get()[i + NUM_OBS];

    for (int i = 0; i < NUM_OBS * (OBS_BUF - 1); i++)
      input_1.get()[i] = input_1_temp.get()[i];

    for (int i = 0; i < NUM_OBS; i++)
      input_1.get()[i + NUM_OBS * (OBS_BUF - 1)] = input_0.get()[i];
  }
  std::cout << "init finised predict" << std::endl;

  for (int i = 0; i < OBS_BUF; i++)
  {
    this->_Forward(true);
  }

  this->threadRunning = true;
  if (this->thread_first_)
  {
    this->forward_thread = std::thread(&FSMState_RL::_Run_Forward, this);
    this->thread_first_ = false;
  }
  this->stop_update_ = false;
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
  for (int i = 0; i < NUM_OUTPUT; i++)
  {
    if (i == 0)
    {
      this->_data->low_cmd->tau_cmd[i] = this->params_.p_gains[i] * (this->desired_pos[i] - this->_data->low_state->q[i]) + this->params_.d_gains[i] * (0 - this->_data->low_state->dq[i]);
    }
    else
    {
      if(NUM_OUTPUT == 2)
      {
        this->_data->low_cmd->tau_cmd[2] = this->params_.p_gains[i] * this->desired_pos[i] + this->params_.d_gains[i] * (0 - this->_data->low_state->dq[i]);
        this->_data->low_cmd->tau_cmd[1] = 0;
      }
      else
      {
        this->_data->low_cmd->tau_cmd[i] = this->params_.p_gains[i] * this->desired_pos[i] + this->params_.d_gains[i] * (0 - this->_data->low_state->dq[i]);
      }
    }
  }
}

void FSMState_RL::exit()
{
  // std::cout << "rl state exit" << std::endl;
  this->stop_update_ = true;
}

FSMStateName FSMState_RL::checkTransition()
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

void FSMState_RL::_GetObs(bool _is_init)
{
  // omegawb = state_estimate_->omegaBody;
  // qwb = state_estimate_->orientation;
  // pwb = state_estimate_->position;
  // vwb = state_estimate_->vWorld;
  std::vector<float> obs_tmp;
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

  obs_tmp.push_back(angvel(0) * this->params_.ang_vel_scale);
  obs_tmp.push_back(angvel(1) * this->params_.ang_vel_scale);
  obs_tmp.push_back(angvel(2) * this->params_.ang_vel_scale);

  std::cout << "angvelC: " << angvel(0) << ", " << angvel(1) << ", " << angvel(2) << std::endl;

  for (int i = 0; i < 3; ++i)
  {
    obs_tmp.push_back(projected_gravity(i));
  }

  // cmd
  double angle_err = (double)this->heading_cmd_ - this->_data->state_estimator->getResult().rpy(2,0);
  angle_err = fmod(angle_err, 2.0 * M_PI);
  while (angle_err > M_PI)
  {
    angle_err = angle_err - 2.0 * M_PI;
  }
  while (angle_err < -M_PI)
  {
    angle_err = angle_err + 2.0 * M_PI;
  }
  std::cout << "commend: " << this->x_vel_cmd_ << ", " << 0 << ", " << angle_err << std::endl;
  // angle_err = 0.75 * angle_err;
  // angle_err = ((angle_err > 1.) ? 1. : (angle_err < -1.) ? -1. : angle_err)/* - 0.35*/;
  if(_is_init == false)
  {
    this->heading_pid.target = angle_err;
    this->heading_pid.current = 0;
    angle_err = this->heading_pid.Adjust(0);
    std::cout << "i_term: " << this->heading_pid.I_Term << std::endl;
    std::cout << "out:    " << angle_err << std::endl;
    std::cout << "dt:     " << this->heading_pid.dt << std::endl;
  }
  else
  {
    angle_err = 0;
  }
  
  // angle_err = (double)this->heading_cmd_;
  obs_tmp.push_back(this->x_vel_cmd_ * this->params_.commands_scale[0]);
  obs_tmp.push_back(0.0);
  obs_tmp.push_back(angle_err * this->params_.commands_scale[2]);
  // obs_tmp.push_back(this->x_vel_cmd_ * this->params_.commands_scale[0]);
  // obs_tmp.push_back(angle * this->params_.commands_scale[0]);
  // obs_tmp.push_back(0.0);
  
  // std::cout << "model_path: " << this->model_name << std::endl;

  // pos
  for (int i = 0; i < NUM_OUTPUT; ++i)
  {
    float pos = (this->obs_.dof_pos[i] - this->params_.default_dof_pos[i]) * params_.dof_pos_scale;
    obs_tmp.push_back(pos);
  }
  // vel
  for (int i = 0; i < NUM_OUTPUT; ++i)
  {
    float vel = this->obs_.dof_vel[i] * params_.dof_vel_scale;
    obs_tmp.push_back(vel);
  }

  // last action
  for (int i = 0; i < NUM_OUTPUT; ++i)
  {
    obs_tmp.push_back(output_last.get()[i]);
  }

  for (int i = 0; i < NUM_OBS; i++)
  {
    input_0.get()[i] = obs_tmp[i];
  }
}

void FSMState_RL::_Forward(bool _is_init)
{
  _GetObs(_is_init);
  cuda_test_->do_inference(input_0.get(), input_1.get(), output.get());

  for (int i = 0; i < NUM_OBS * (OBS_BUF - 1); i++)
    input_1_temp.get()[i] = input_1.get()[i + NUM_OBS];

  for (int i = 0; i < NUM_OBS * (OBS_BUF - 1); i++)
    input_1.get()[i] = input_1_temp.get()[i];

  for (int i = 0; i < NUM_OBS; i++)
    input_1.get()[i + NUM_OBS * (OBS_BUF - 1)] = input_0.get()[i];

  for (int i = 0; i < NUM_OUTPUT; i++)
    output_last.get()[i] = output.get()[i];
}

void FSMState_RL::_Run_Forward()
{
  while (this->threadRunning)
  {
    long long _start_time = getSystemTime();

    if (!this->stop_update_)
    {
      // update current dof positions and velocities
      for (int i = 0; i < NUM_OUTPUT; i++)
      {
        this->obs_.dof_pos[i] = this->_data->low_state->q[i];
        this->obs_.dof_vel[i] = this->_data->low_state->dq[i];
      }

      _Forward(false);

      // calculate actions
      for (int j = 0; j < NUM_OUTPUT; j++)
      {
        this->action[j] = this->output.get()[j] * this->params_.action_scale + this->params_.default_dof_pos[j];
        // std::cout << "action" << j << " : " << this->action[j] << std::endl;
      }

      for (int i = 0; i < NUM_OUTPUT; i++)
      {
        this->desired_pos[i] = this->action[i];
      }
    }

    absoluteWait(_start_time, (long long)(0.01 * 1000000));
  }
  this->threadRunning = false;
}