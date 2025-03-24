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
  input_1(new float[NUM_OBS*OBS_BUF]),
  output(new float[NUM_OUTPUT]),
  output_last(new float[NUM_OUTPUT]),
  input_1_temp(new float[NUM_OBS*(OBS_BUF-1)])
{
  this->cuda_test_ = std::make_shared<CudaTest>("/home/hxt/Downloads/LocomotionWithNP3O_8dofs_126/tita_rl/model_gn.engine");
  std::cout << "cuda init :" << this->cuda_test_->get_cuda_init() << std::endl;
}

void FSMState_RL::enter()
{ 
  this->_data->state_command->firstRun = true;

  for(int i=0;i<3;i++)
  {
    this->desired_pos[i] = this->_data->low_state->q[i];
    this->obs.dof_pos[i] = this->_data->low_state->q[i];
    this->obs.dof_vel[i] = this->_data->low_state->dq[i];
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
  this->params_.p_gains[2] = 10;
  this->params_.d_gains[0] = 1.;
  this->params_.d_gains[1] = 0.5;
  this->params_.d_gains[2] = 0.5;

  const float default_dof_pos_tmp[NUM_OUTPUT] = {0.};
  for (int i = 0; i < NUM_OUTPUT; i++)
  {
    this->params_.default_dof_pos[i] = default_dof_pos_tmp[i];
  }

  this->x_vel_cmd_ = 0.;
  this->heading_cmd_ = 0.;

  for (int i = 0; i < NUM_OBS*(OBS_BUF-1); i++)
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
    // // append obs to obs buffer
    // obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);
    this->_GetObs();

    for (int i = 0; i < NUM_OBS*(OBS_BUF-1); i++)
      input_1_temp.get()[i] = input_1.get()[i + NUM_OBS];

    for (int i = 0; i < NUM_OBS*(OBS_BUF-1); i++)
      input_1.get()[i] = input_1_temp.get()[i];

    for (int i = 0; i < NUM_OBS; i++)
      input_1.get()[i + NUM_OBS*(OBS_BUF-1)] = input_0.get()[i];
  }
  std::cout << "init finised predict" << std::endl;

  for (int i = 0; i < OBS_BUF; i++)
  {
    this->_Forward();
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
  // update cmds
  this->x_vel_cmd_ = this->_data->state_command->rc_data_->twist_linear[point::X];
  this->heading_cmd_ = this->_data->state_command->rc_data_->twist_angular[point::Z];
  // _data->state_command->rc_data_->twist_angular[point::Z]
  this->_data->low_cmd->qd.setZero();
  this->_data->low_cmd->qd_dot.setZero();
  this->_data->low_cmd->kp.setZero();
  this->_data->low_cmd->kd.setZero();
  this->_data->low_cmd->tau_cmd.setZero();
  for(int i = 0; i < NUM_OUTPUT; i++)
  {
    if(i == 0)
    {
      this->_data->_low_cmd->tau_cmd[i] = this->_params->p_gains[i] * (this->desired_pos[i] - this->_data->low_state->q[i]) \
                                         + this->_params->d_gains[i] * (0 - this->_data->low_state->dq[i]);
    }
    else
    {
      this->_data->_low_cmd->tau_cmd[i] = this->_params->p_gains[i] * this->desired_pos[i] \
                                         + this->_params->d_gains[i] * (0 - this->_data->low_state->dq[i]);
    }
  }
}

void FSMState_RL::exit() 
{
  this->stop_update_ = true;
}

FSMStateName FSMState_RL::checkTransition()
{
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

void FSMState_RL::_GetObs()
{
  // omegawb = state_estimate_->omegaBody;
  // qwb = state_estimate_->orientation;
  // pwb = state_estimate_->position;
  // vwb = state_estimate_->vWorld;
    std::vector<float> obs_tmp;
    // compute gravity
    Mat3<double> _B2G_RotMat = this->_data->state_estimator->getResult().rBody;
    Mat3<double> _G2B_RotMat = this->_data->state_estimator->getResult().rBody.transpose();

    Vec3<double> limvel = this->_data->state_estimator->getResult().vBody
    Vec3<double> angvel = a_l;
    a_l = 0.97*this->_data->state_estimator->getResult().omegaBody + 0.03*a_l;
    Vec3<double> projected_gravity = _B2G_RotMat * Vec3<double>(0.0, 0.0, -1.0);
    Vec3<double> projected_forward = _G2B_RotMat * Vec3<double>(1.0, 0.0, 0.0);
 
    this->obs_tmp.push_back(limvel(0)*this->params_.lin_vel_scale);
    this->obs_tmp.push_back(limvel(1)*this->params_.lin_vel_scale);
    this->obs_tmp.push_back(limvel(2)*this->params_.lin_vel_scale);

    this->obs_tmp.push_back(angvel(0)*this->params_.ang_vel_scale);
    this->obs_tmp.push_back(angvel(1)*this->params_.ang_vel_scale);
    this->obs_tmp.push_back(angvel(2)*this->params_.ang_vel_scale);

    for (int i = 0; i < 3; ++i)
    {
        this->obs_tmp.push_back(projected_gravity(i));
    }

    // cmd
    double angle = (double)this->heading_cmd_;
    angle = fmod(angle,2.0*M_PI);
    if(angle > M_PI)
    {
        angle = angle - 2.0*M_PI;
    }

    obs_tmp.push_back(this->x_vel_cmd_*this->params_.commands_scale[0]);
    obs_tmp.push_back(0.0);
    obs_tmp.push_back(angle*this->params_.commands_scale[2]);

    // pos
    for (int i = 0; i < NUM_OUTPUT; ++i)
    {
        float pos = (this->obs_.dof_pos[i]  - this->params_.default_dof_pos[i]) * params_.dof_pos_scale;
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

    for(int i = 0; i < NUM_OBS; i++)
    {
        input_0.get()[i] = obs_tmp[i];
    }

}

void FSMState_RL::_Forward()
{
    _GetObs();
    cuda_test_->do_inference(input_0.get(), input_1.get(), output.get());

    for (int i = 0; i < NUM_OBS*(OBS_BUF-1); i++)
        input_1_temp.get()[i] = input_1.get()[i + 33];

    for (int i = 0; i < NUM_OBS*(OBS_BUF-1); i++)
        input_1.get()[i] = input_1_temp.get()[i];

    for (int i = 0; i < NUM_OBS; i++)
        input_1.get()[i + NUM_OBS*(OBS_BUF-1)] = input_0.get()[i];

    for (int i = 0; i < NUM_OUTPUT i++)
        output_last.get()[i] = output.get()[i];

}

void FSMState_RL::_Run_Forward()
{
  while (this->threadRunning)
  {
    long long _start_time = this->getSystemTime();

    if (!this->stop_update_)
    {
      // update current dof positions and velocities
      for(int i=0;i<3;i++)
      {
        this->obs.dof_pos[i] = this->_data->low_state->q[i];
        this->obs.dof_vel[i] = this->_data->low_state->dq[i];
      }

      _Forward();

      // calculate actions
      for (int j = 0; j < NUM_OBS; j++)
      {
        this->action[j] = this->output.get()[j] * this->params_.action_scale + this->params_.default_dof_pos[j];
      }

      for (int i = 0; i < NUM_OBS; i++)
      {
        this->desired_pos[i] = this->action[i];
      }
    }

    absoluteWait(_start_time, (long long)(0.01 * 1000000));
  }
  this->threadRunning = false;
}