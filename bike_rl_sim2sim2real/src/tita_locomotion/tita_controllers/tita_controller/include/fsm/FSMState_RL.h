#ifndef FSMSTATE_RL_H
#define FSMSTATE_RL_H

#include <thread>
#include <functional>
#include "FSMState.h"
#include "tensorrt_cuda/tensor_cuda_test.hpp"
#include "../my_controller_module/Pid/Cpp/include/PIDmethod.h"
#include "tensorrt_cuda/rl_inference_module.hpp"

#define NUM_OBS 22
#define OBS_BUF 10
#define NUM_OUTPUT 3
/**
 *
 */
// 初始模型感知维度
struct ModelParams
{
    float damping;
    float stiffness;
    float action_scale;
    float hip_scale_reduction;
    float num_of_dofs;
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float clip_obs;
    float clip_actions;
    float torque_limits[NUM_OUTPUT];
    // float d_gains[NUM_OUTPUT];
    // float p_gains[NUM_OUTPUT];
    float commands_scale[4];
    float default_dof_pos[NUM_OUTPUT];
};
// 观测数据维度
struct Observations
{
    float lin_vel[3];           
    float ang_vel[3];  
    float gravity_vec[3];
    float forward_vec[3];       
    float commands[3];        
    float base_quat[4];   
    float dof_pos[NUM_OUTPUT];
    float dof_pos_last[NUM_OUTPUT];           
    float dof_vel[NUM_OUTPUT];
    float dof_vel_last[NUM_OUTPUT];           
    float actions[NUM_OUTPUT];
};
enum DofCtrlType
{
  P=0,
  V,
  T
};
// 电机执行维度
struct DofCtrl
{
  DofCtrlType dof_mode = P;
  float P_p[NUM_OUTPUT];
  float P_d[NUM_OUTPUT];
  float V_p[NUM_OUTPUT];
  float V_d[NUM_OUTPUT];
  float T_d[NUM_OUTPUT];
};

class FSMState_RL : public FSMState
{
public:
  FSMState_RL(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_RL() {}

  // Behavior to be carried out when entering a state
  void enter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSMStateName checkTransition();

  // Manages state specific transitions
  //   TransitionData transition();

  // Behavior to be carried out when exiting a state
  void exit();

  //   TransitionData testTransition();

private:
  // Keep track of the control iterations
  float wheel_init_pos_abs_[4];
  float x_vel_cmd_;
  float heading_cmd_;
  std::string model_name;

  PIDmethod heading_pid;
private:
  ModelParams params_;
  Observations obs_;
  DofCtrl dofs_;
  using Dof_Actuate = std::function<void()>;  // 改用 std::function
  Dof_Actuate dof_actuate[3];  // 存储可调用对象
  void _P_actuate();
  void _V_actuate();
  void _T_actuate();

  void _Run_Forward();
  void _Run_Lowlevel();

  std::shared_ptr<RL_InferenceModule> inference_test_;

  std::thread ctrl_thread[2];
  bool threadRunning[2] = {false,false};
  float desired_pos[NUM_OUTPUT] = {0.};

  const std::vector<float> _GetObs();

  float action[NUM_OUTPUT];
  float action_last[NUM_OUTPUT];
  float torques[3] = {0};

  bool stop_update_[2] = {false,false};
  bool thread_first_[2] = {true,true};
  
  bool use_lpf_actions = false;
};

#endif  // FSMSTATE_RL_H
