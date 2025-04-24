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
   
   // control mode
   this->ctrl_mode = CtrlMode::LQR;
   // turn mode
   this->bike_lqr_params.lqr_turn = TurnMode::TDIRECTION;

   // use 2dof
   this->bike_pid_params.motor_enList[0] = true;
   this->bike_pid_params.motor_enList[1] = false;
   this->bike_pid_params.motor_enList[2] = true;
 
   // use direct roll ctrl
   this->bike_pid_params.heading_enList[0] = false;
   this->bike_pid_params.heading_enList[1] = true;
 
   // pid controller pid init
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
  //  this->bike_heading_pid[1].d_of_current = false;
 
 #if USE_REAL_BIKE == 0
   // multiCircle offset
   this->turn_circle.set_offset(0);
 
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
 
   // this->bike_pid_params.motor_kp[2] = 10.;
   // this->bike_pid_params.motor_ki[2] = 2.;
   // this->bike_pid_params.motor_kd[2] = -0.2;
   // this->bike_pid_params.motor_imax[2] = 10.;
   // this->bike_pid_params.motor_lim[2] = 30.;
 
   this->bike_pid_params.motor_kp[2] = 15.;
   this->bike_pid_params.motor_ki[2] = 30.;
   this->bike_pid_params.motor_kd[2] = -0.2;
   this->bike_pid_params.motor_imax[2] = 10.;
   this->bike_pid_params.motor_lim[2] = 30.;
 
 
   // lqr controller pid init
   for(int i=0;i<3;i++)
   {
     this->bike_lqr_pid[i].PID_Init(Fit, 0);
     this->bike_lqr_pid[i].getMicroTick_regist(getSystemTime);
     this->bike_lqr_pid[i].fit_degree = 7;
   }
 
   // lqr params init
   this->bike_lqr_params.heading_kp[0] = 0.5;
   this->bike_lqr_params.heading_ki[0] = 0.;
   this->bike_lqr_params.heading_kd[0] = -0.;
   this->bike_lqr_params.heading_imax[0] = 0.;
   this->bike_lqr_params.heading_lim[0] = 1.;
 
   this->bike_lqr_params.heading_kp[1] = 0.;
   this->bike_lqr_params.heading_ki[1] = -0.2;
   this->bike_lqr_params.heading_kd[1] = 0.;
   this->bike_lqr_params.heading_imax[1] = 0.2;
   this->bike_lqr_params.heading_lim[1] = 0.2;
 
   this->bike_lqr_params.motor_kp[0] = 15.;
   this->bike_lqr_params.motor_ki[0] = 60.;
   this->bike_lqr_params.motor_kd[0] = -0.3;
   this->bike_lqr_params.motor_imax[0] = 5.;
   this->bike_lqr_params.motor_lim[0] = 20.;
 
   this->bike_lqr_params.motor_kp[1] = 20.;
   this->bike_lqr_params.motor_ki[1] = 0.;
   this->bike_lqr_params.motor_kd[1] = 0.;
   this->bike_lqr_params.motor_imax[1] = 0.;
   this->bike_lqr_params.motor_lim[1] = 30.;
 
   this->bike_lqr_params.motor_kp[2] = 10.;
   this->bike_lqr_params.motor_ki[2] = 10.;
   this->bike_lqr_params.motor_kd[2] = -0.2;
   this->bike_lqr_params.motor_imax[2] = 10.;
   this->bike_lqr_params.motor_lim[2] = 30.;
 
   /*参数*/
   this->bike_lqr_params.lqrs[0].a = -4.437549e-01;
   this->bike_lqr_params.lqrs[0].b = 9.522741e+00;
   this->bike_lqr_params.lqrs[0].c = -8.506836e+01;
   this->bike_lqr_params.lqrs[0].d = 4.097284e+02;
   this->bike_lqr_params.lqrs[0].e = -1.150538e+03;
   this->bike_lqr_params.lqrs[0].f = 1.894709e+03;
   this->bike_lqr_params.lqrs[0].g = -1.723706e+03;
   this->bike_lqr_params.lqrs[0].h = 8.005830e+02;
   /*参数*/
   this->bike_lqr_params.lqrs[1].a = -9.064959e-02;
   this->bike_lqr_params.lqrs[1].b = 1.948763e+00;
   this->bike_lqr_params.lqrs[1].c = -1.746025e+01;
   this->bike_lqr_params.lqrs[1].d = 8.452224e+01;
   this->bike_lqr_params.lqrs[1].e = -2.394803e+02;
   this->bike_lqr_params.lqrs[1].f = 4.011097e+02;
   this->bike_lqr_params.lqrs[1].g = -3.781801e+02;
   this->bike_lqr_params.lqrs[1].h = 1.738160e+02;
   /*参数*/
   this->bike_lqr_params.lqrs[2].a = -4.738319e-03;
   this->bike_lqr_params.lqrs[2].b = 9.471532e-02;
   this->bike_lqr_params.lqrs[2].c = -7.542719e-01;
   this->bike_lqr_params.lqrs[2].d = 2.955858e+00;
   this->bike_lqr_params.lqrs[2].e = -5.265100e+00;
   this->bike_lqr_params.lqrs[2].f = 2.865951e-01;
   this->bike_lqr_params.lqrs[2].g = 1.513493e+01;
   this->bike_lqr_params.lqrs[2].h = -1.377900e+01;
 
   // bike discription
   this->bike_state.com_dist = 0.4651025;
   this->bike_state.com_height = 0.4631022;
   this->bike_state.com_weight = 12.04277;
   this->bike_state.wheel_dist = 1.02065;
   this->bike_state.wheel_radius = 0.33;
 
 #else
   // multiCircle offset
  //  this->turn_circle.set_offset(0.82);
  this->turn_circle.set_offset(0.);

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
   this->bike_pid_params.heading_ki[1] = -0.4;
   this->bike_pid_params.heading_kd[1] = 0;
   this->bike_pid_params.heading_imax[1] = 0.05;
   this->bike_pid_params.heading_lim[1] = 0.05;
 
   // low gain
   this->bike_pid_params.motor_kp[0] = 20.;
   this->bike_pid_params.motor_ki[0] = 0.;
   this->bike_pid_params.motor_kd[0] = -2.;
   this->bike_pid_params.motor_imax[0] = 0.;
   this->bike_pid_params.motor_lim[0] = 3.;
 
   this->bike_pid_params.motor_kp[1] = 0.5;
   this->bike_pid_params.motor_ki[1] = 0.01;
   this->bike_pid_params.motor_kd[1] = 0.05;
   this->bike_pid_params.motor_imax[1] = 4.;
   this->bike_pid_params.motor_lim[1] = 4.;
 
   this->bike_pid_params.motor_kp[2] = 0.5;
   this->bike_pid_params.motor_ki[2] = 0.01;
   this->bike_pid_params.motor_kd[2] = 0.05;
   this->bike_pid_params.motor_imax[2] = 4.;
   this->bike_pid_params.motor_lim[2] = 4.;
 
 
   // lqr controller pid init
   for(int i=0;i<3;i++)
   {
     this->bike_lqr_pid[i].PID_Init(Fit, 0);
     this->bike_lqr_pid[i].getMicroTick_regist(getSystemTime);
     this->bike_lqr_pid[i].fit_degree = 7;
   }
   this->turn_c_pid.PID_Init(Common, 0);
   this->turn_c_pid.getMicroTick_regist(getSystemTime);
 
   // lqr params init
   this->bike_lqr_params.heading_kp[0] = 0.5;
   this->bike_lqr_params.heading_ki[0] = 0.;
   this->bike_lqr_params.heading_kd[0] = -0.;
   this->bike_lqr_params.heading_imax[0] = 0.;
   this->bike_lqr_params.heading_lim[0] = 1.;
   if(this->bike_lqr_params.lqr_turn == TurnMode::TROLL)
   {
    this->bike_lqr_params.heading_kp[1] = 0.;
    this->bike_lqr_params.heading_ki[1] = -0.2;
    this->bike_lqr_params.heading_kd[1] = 0.0;
    this->bike_lqr_params.heading_imax[1] = 0.1;
    this->bike_lqr_params.heading_lim[1] = 0.1;
    this->bike_lqr_params.roll_tar_slope = 0.025;
   }
   else
   {
    this->bike_lqr_params.heading_kp[1] = 0.;
    this->bike_lqr_params.heading_ki[1] = -1.;
    this->bike_lqr_params.heading_kd[1] = 0.;
    this->bike_lqr_params.heading_imax[1] = 1.;
    this->bike_lqr_params.heading_lim[1] = 1.;
    // this->bike_lqr_params.roll_tar_slope = 0.5;
   }
 
   this->bike_lqr_params.motor_kp[0] = 5.;
   this->bike_lqr_params.motor_ki[0] = 60.;
   this->bike_lqr_params.motor_kd[0] = 0.;
   this->bike_lqr_params.motor_imax[0] = 3.;
   this->bike_lqr_params.motor_lim[0] = 3.;
 
   this->bike_lqr_params.motor_kp[1] = 0.5;
   this->bike_lqr_params.motor_ki[1] = 0.01;
   this->bike_lqr_params.motor_kd[1] = 0.05;
   this->bike_lqr_params.motor_imax[1] = 4.;
   this->bike_lqr_params.motor_lim[1] = 4.;
 
   this->bike_lqr_params.motor_kp[2] = 0.5;
   this->bike_lqr_params.motor_ki[2] = 0.01;
   this->bike_lqr_params.motor_kd[2] = 0.05;
   this->bike_lqr_params.motor_imax[2] = 4.;
   this->bike_lqr_params.motor_lim[2] = 4.;

   this->bike_lqr_params.turn_c_kp = 5.;
   this->bike_lqr_params.turn_c_lim = 1.;

   this->bike_lqr_params.vel_tar_slope = 0.2;
 
  /*参数*/
this->bike_lqr_params.lqrs[0].a = -9.063404e-02;
this->bike_lqr_params.lqrs[0].b = 1.932797e+00;
this->bike_lqr_params.lqrs[0].c = -1.713936e+01;
this->bike_lqr_params.lqrs[0].d = 8.186656e+01;
this->bike_lqr_params.lqrs[0].e = -2.279759e+02;
this->bike_lqr_params.lqrs[0].f = 3.736884e+02;
this->bike_lqr_params.lqrs[0].g = -3.443513e+02;
this->bike_lqr_params.lqrs[0].h = 1.756409e+02;
/*参数*/
this->bike_lqr_params.lqrs[1].a = -1.560181e-02;
this->bike_lqr_params.lqrs[1].b = 3.329101e-01;
this->bike_lqr_params.lqrs[1].c = -2.954741e+00;
this->bike_lqr_params.lqrs[1].d = 1.413355e+01;
this->bike_lqr_params.lqrs[1].e = -3.945927e+01;
this->bike_lqr_params.lqrs[1].f = 6.503630e+01;
this->bike_lqr_params.lqrs[1].g = -6.089175e+01;
this->bike_lqr_params.lqrs[1].h = 3.030269e+01;
/*参数*/
this->bike_lqr_params.lqrs[2].a = 4.369293e-04;
this->bike_lqr_params.lqrs[2].b = -9.332416e-03;
this->bike_lqr_params.lqrs[2].c = 8.289935e-02;
this->bike_lqr_params.lqrs[2].d = -3.967833e-01;
this->bike_lqr_params.lqrs[2].e = 1.110095e+00;
this->bike_lqr_params.lqrs[2].f = -1.887723e+00;
this->bike_lqr_params.lqrs[2].g = 4.366517e+00;
this->bike_lqr_params.lqrs[2].h = 2.033561e-01;
 
   // bike discription
   this->bike_state.com_dist = 0.32617799;
   this->bike_state.com_height = 0.11459196+0.175;
   this->bike_state.com_weight = 13.55852069;
   this->bike_state.wheel_dist = 0.64;
   this->bike_state.wheel_radius = 0.175;
 #endif
 
 }
 
 void FSMState_TraditionCtrl::enter()
 {
   this->_pid_clear();
   if(this->ctrl_mode == TEST)
   {
     // set test params
     this->_test_params_update();
   }
   else if(this->ctrl_mode == PID)
   {
     // set pid params
     this->_pid_params_update();
   }
   else if(this->ctrl_mode == LQR)
   {
     // set lqr params
     this->_lqr_params_update();
   }
   
   // update body state
   this->bike_state.ref_roll = 0.;
   this->bike_state.ref_rollVel = 0.;
   this->bike_state.ref_turn = 0.;
   this->_bike_state_update();
 
   this->threadRunning = true;
   if (this->thread_first_)
   {
     this->forward_thread = std::thread(&FSMState_TraditionCtrl::_controller_loop, this);
     this->thread_first_ = false;
   }
   this->stop_update_ = false;
 }
 
 void FSMState_TraditionCtrl::run()
 {
   this->_data->low_cmd->qd.setZero();
   this->_data->low_cmd->qd_dot.setZero();
   this->_data->low_cmd->kp.setZero();
   this->_data->low_cmd->kd.setZero();
   this->_data->low_cmd->tau_cmd.setZero();

   std::cout << "\033[19A\r"; // \033[3A 表示上移3行
   for (int i = 0; i < _data->low_state->q.rows(); i++)
   {
     std::cout << "motor" << i << ":" << "\033[K" << "\n"
               << "q:  " << _data->low_state->q[i] << "\033[K" << "\n"
               << "dq: " << _data->low_state->dq[i] << "\033[K" << std::endl;
   }
   std::cout << "angVel: \n" << _data->state_estimator->getResult().omegaBody << "\033[K" << std::endl;
   std::cout << "rpy: \n" << _data->state_estimator->getResult().rpy << "\033[K" << std::endl;
   std::cout << "turn_c: " << this->bike_lqr_pid[2].target << ", " << this->bike_lqr_pid[2].current << ", " << this->bike_lqr_pid[2].out << std::endl;
 
   // update commands
   this->bike_state.ref_v = 2.*this->_data->state_command->rc_data_->twist_linear[point::X];
   this->bike_state.ref_yaw = this->_data->state_command->rc_data_->twist_angular[point::Z];
   this->bike_state.ref_yawVel = this->_data->state_command->rc_data_->twist_angular[point::Z];
   if(this->bike_lqr_params.lqr_turn == TurnMode::TROLL)
   {
    this->bike_state.ref_roll = -0.05 * this->_data->state_command->rc_data_->twist_angular[point::Z];
    this->bike_state.ref_rollVel = 0.;
    this->bike_state.ref_turn = 0.;
   }
   else
   {
    // 由于系统平衡需要，前叉实际稳定方向与目标值相反，要自行车正转就要设置负的目标值
    this->bike_state.ref_roll = 0.;
    this->bike_state.ref_rollVel = 0.;
    this->bike_state.ref_turn = -0.5 * this->_data->state_command->rc_data_->twist_angular[point::Z];
   }
 
   for (int i = 0; i < 3; i++)
   {
     this->_data->low_cmd->tau_cmd[i] = this->bike_state.ctrl_output[i];
    //  this->_data->low_cmd->tau_cmd[i] = 0;
   }
 }
 
 void FSMState_TraditionCtrl::exit() 
 {
   this->stop_update_ = true;
   this->_pid_clear();
 }
 
 void FSMState_TraditionCtrl::_controller_loop()
 {
   while(this->threadRunning)
   {
     long long _start_time = getSystemTime();
 
     if(!this->stop_update_)
     {
       if(this->ctrl_mode == TEST)
       {
         // set test params
         this->_test_params_update();
       }
       else if(this->ctrl_mode == PID)
       {
         // set pid params
         this->_pid_params_update();
       }
       else if(this->ctrl_mode == LQR)
       {
         // set lqr params
         this->_lqr_params_update();
       }
       // update body state
       this->_bike_state_update();
 
       if(this->ctrl_mode == TEST)
       {
         // use test controller
         this->_test_cal();
         this->_test_actuate();
       }
       else if(this->ctrl_mode == PID)
       {
         // use pid controller
         this->_high_level_pid_cal();
         this->_low_level_pid_cal();
         this->_pid_actuate();
       }
       else if(this->ctrl_mode == LQR)
       {
         // use pid controller
         this->_high_level_lqr_cal();
         this->_low_level_lqr_cal();
         this->_lqr_actuate();
       }
     }
     absoluteWait(_start_time, (long long)(0.002 * 1000000));
   }
   this->threadRunning = false;
 }
 
 FSMStateName FSMState_TraditionCtrl::checkTransition()
 {
   // std::cout << "rl state check" << std::endl;
   this->_nextStateName = this->_stateName;
 
   // Switch FSM control mode
   switch (this->_data->state_command->desire_data_->fsm_state_name)
   {
   case FSMStateName::RL: // normal c
     this->_nextStateName = FSMStateName::RL;
     break;
 
   case FSMStateName::TRADITION_CTRL:
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
     if(i==0)
     {
      this->bike_state.dof_pos[i] = this->turn_circle.f(this->_data->low_state->q[i]);
     }
     else
     {
      this->bike_state.dof_pos[i] = this->_data->low_state->q[i];
     }
     this->bike_state.dof_vel[i] = this->_data->low_state->dq[i];
   }
 
   this->bike_state.obs_turn = -this->bike_state.dof_pos[0];
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
   std::cout << this->bike_heading_pid[0].dt << std::endl;
 
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
   double real_v = this->real_v_mf.f(this->bike_state.dof_vel[2]*0.33*0.7);
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
  //  this->bike_motor_pid[2].Adjust(0, this->bike_state.dof_vel[2]);
  this->bike_motor_pid[2].Adjust_Incremental();
  //  std::cout << "target: " << this->bike_motor_pid[2].target << std::endl;
  //  std::cout << "current: " << this->bike_motor_pid[2].current << std::endl;
  //  std::cout << "out: " << this->bike_motor_pid[2].out << std::endl;
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
 
 void FSMState_TraditionCtrl::_lqr_params_update()
 {
   for(int i=0;i<3;i++)
   {
     this->bike_lqr_pid[i].Params_Config(this->bike_lqr_params.lqrs[i],0,10.);
     if (i < 2)
     {
       this->bike_heading_pid[i].Params_Config(this->bike_lqr_params.heading_kp[i],
                                               this->bike_lqr_params.heading_ki[i],
                                               this->bike_lqr_params.heading_kd[i],
                                               this->bike_lqr_params.heading_imax[i],
                                               this->bike_lqr_params.heading_lim[i]);
     }
     this->bike_motor_pid[i].Params_Config(this->bike_lqr_params.motor_kp[i],
                                           this->bike_lqr_params.motor_ki[i],
                                           this->bike_lqr_params.motor_kd[i],
                                           this->bike_lqr_params.motor_imax[i],
                                           this->bike_lqr_params.motor_lim[i]);
   }
   this->turn_c_pid.Params_Config(this->bike_lqr_params.turn_c_kp,0,1000.);
 }
 
 void FSMState_TraditionCtrl::_high_level_lqr_cal()
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
  
   double* use_turn_ptr = nullptr;
   if(this->bike_lqr_params.lqr_turn == TurnMode::TROLL)
   {
    use_turn_ptr = &this->bike_state.ref_roll;
   }
   else
   {
    use_turn_ptr = &this->bike_state.ref_turn;
   }
   if (this->bike_pid_params.heading_enList[1] == true)
   {
     *use_turn_ptr = upper::constrain(this->bike_heading_pid[1].out,
                                                  bike_state.ref_roll+bike_lqr_params.roll_tar_slope,
                                                  bike_state.ref_roll-bike_lqr_params.roll_tar_slope);
   }
   else if (this->bike_pid_params.heading_enList[0] == true)
   {
    *use_turn_ptr = upper::constrain(this->bike_heading_pid[0].out,
                                                  bike_state.ref_roll+bike_lqr_params.roll_tar_slope,
                                                  bike_state.ref_roll-bike_lqr_params.roll_tar_slope);
   }
 }
 
 void FSMState_TraditionCtrl::_low_level_lqr_cal()
 {
   // task1: roll error, rollVel error, turn error -> turnVel ref
   double real_v = this->real_v_mf.f(this->bike_state.dof_vel[2]*this->bike_state.wheel_radius);
   // avoid approach 0
   if(real_v<0.5)
   {
     real_v=0.5;
   }
   else if(real_v>4.)
   {
     real_v=4.;
   }
   // 如果是使用车头转向，那么考虑加入角速度计算前馈
  //  if(this->bike_lqr_params.lqr_turn == TurnMode::TDIRECTION && this->bike_pid_params.heading_enList[1] == true)
  //  {
  //   this->bike_state.ref_turn -= this->bike_state.ref_yawVel * this->bike_state.wheel_dist / real_v;
  //  }
   this->bike_lqr_pid[0].target = this->bike_state.ref_roll;
   this->bike_lqr_pid[0].current = this->bike_state.obs_roll;
   this->bike_lqr_pid[0].Adjust(real_v);
   this->bike_lqr_pid[1].target = this->bike_state.ref_rollVel;
   this->bike_lqr_pid[1].current = this->bike_state.obs_rollVel;
   this->bike_lqr_pid[1].Adjust(real_v);
  //  this->bike_lqr_pid[2].target = upper::constrain(this->bike_state.ref_turn,this->bike_lqr_params.turn_c_lim);
   this->bike_lqr_pid[2].target = this->bike_state.ref_turn;
   this->bike_lqr_pid[2].current = this->bike_state.obs_turn;
   this->bike_lqr_pid[2].Adjust(real_v);
 
   // task2: turnVel error -> turn motor pid
   this->turn_c_pid.target = upper::constrain((this->bike_lqr_pid[0].out + this->bike_lqr_pid[1].out + this->bike_lqr_pid[2].out)/this->turn_c_pid.kp - this->bike_state.dof_pos[0],this->bike_lqr_params.turn_c_lim);
   this->turn_c_pid.current = -this->bike_state.dof_pos[0];
   this->turn_c_pid.Adjust(0);
   this->bike_motor_pid[0].target = -this->turn_c_pid.out;
  //  this->bike_motor_pid[0].target = -(this->bike_lqr_pid[0].out + this->bike_lqr_pid[1].out + this->bike_lqr_pid[2].out);
   // this->bike_motor_pid[0].target = -this->bike_state.ref_roll;
   this->bike_motor_pid[0].current = this->bike_state.dof_vel[0];
   this->bike_motor_pid[0].Adjust(0, this->bike_state.dof_vel[0]);
 
   // task3: rear motor velocity error -> rear motor pid
  //  this->bike_motor_pid[2].target = upper::constrain(this->bike_state.ref_v / this->bike_state.wheel_radius,
  //                                                     this->bike_state.dof_vel[2]+this->bike_lqr_params.vel_tar_slope,
  //                                                     this->bike_state.dof_vel[2]-this->bike_lqr_params.vel_tar_slope);
   this->bike_motor_pid[2].target = this->bike_state.ref_v / this->bike_state.wheel_radius;
   this->bike_motor_pid[2].current = this->bike_state.dof_vel[2];
   //  this->bike_motor_pid[2].Adjust(0, this->bike_state.dof_vel[2]);
   this->bike_motor_pid[2].Adjust_Incremental();
 }
 
 void FSMState_TraditionCtrl::_lqr_actuate()
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
 
 void FSMState_TraditionCtrl::_test_params_update()
 {
   // // pid ctrl
   // this->bike_motor_pid[0].Params_Config(20,0,-2.,0.,20.);
   // this->bike_motor_pid[1].Params_Config(15,30,-0.2,10.,30.);
   // this->bike_motor_pid[2].Params_Config(15,30,-0.2,10.,30.);
 
   // // lqr ctrl
   // this->bike_motor_pid[0].Params_Config(20,0,-2.,0.,20.);
   // this->bike_motor_pid[1].Params_Config(15,30,-0.2,10.,30.);
   // this->bike_motor_pid[2].Params_Config(15,30,-0.2,10.,30.);
 
   // test ctrl
   this->bike_motor_pid[0].Params_Config(5,0.,-0.01,0.,20.);
   this->bike_motor_pid[1].Params_Config(10,10,-0.2,10.,30.);
   this->bike_motor_pid[2].Params_Config(10,0.,-0.015,0.,30.);
   this->bike_motor_pid[0].d_of_current = true;
   this->bike_motor_pid[2].d_of_current = true;
 }
 
 void FSMState_TraditionCtrl::_test_cal()
 {
   this->bike_motor_pid[0].target = this->bike_state.ref_yaw*5;
   this->bike_motor_pid[2].target = this->bike_state.ref_v*10;
   this->bike_motor_pid[0].current = this->bike_state.dof_vel[0];
   this->bike_motor_pid[2].current = this->bike_state.dof_vel[2];
   this->bike_motor_pid[0].Adjust(0);
   this->bike_motor_pid[2].Adjust(0);
 }
 
 void FSMState_TraditionCtrl::_test_actuate()
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

 void FSMState_TraditionCtrl::_pid_clear()
 {
    bike_balance_pid.Clear();
    for(int i=0;i<3;i++)
    {
      bike_motor_pid[i].Clear();
      bike_lqr_pid[i].Clear();
      if(i<2)
      {
        this->bike_heading_pid[i].Clear();
      }
    }
 }