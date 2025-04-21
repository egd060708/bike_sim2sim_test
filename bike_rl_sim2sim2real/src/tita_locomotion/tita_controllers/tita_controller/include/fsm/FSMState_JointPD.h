#ifndef FSM_TEST_H
#define FSM_TEST_H

#include "FSMState.h"
#include "../my_controller_module/Pid/Cpp/include/PIDmethod.h"
#include "../my_filter_module/Cpp/include/my_filters.h"
#include <thread>

class FSMState_JointPD : public FSMState
{
public:
  FSMState_JointPD(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_JointPD() {}
  void enter();
  void run();
  void exit();
  FSMStateName checkTransition();

private:
  // data
  DVec<scalar_t> initial_jpos;
  // test pid
  PIDmethod motor_pid_test[3];
  double test_out[3] = {0};
  MeanFilter<10> ref_v_mf;
  MeanFilter<1> obs_v_mf;

  // controller thread
  std::thread forward_thread;
  bool threadRunning;
  bool stop_update_ = false;
  bool thread_first_ = true;
  void _controller_loop();
};

#endif