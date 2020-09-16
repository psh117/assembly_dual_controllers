#include <assembly_dual_controllers/servers/idle_control_server.h>

IdleControlServer::IdleControlServer(const std::string &name, ros::NodeHandle &nh,
                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu) :
nh_(nh), mu_(mu)
{
  for (auto & pair : mu_)
  {
    auto msg = assembly_msgs::IdleControl::Request();
    msg.mode = assembly_msgs::IdleControl::Request::TASK_SPACE;
    params_.emplace(std::make_pair(pair.first, msg));
  }
  // params_["panda_right"].mode = assembly_msgs::IdleControl::Request::TASK_SPACE_DEV_TEST;
  server_  = nh_.advertiseService(name, &IdleControlServer::setTarget, this);
}

bool IdleControlServer::compute(ros::Time time)
{
  for (auto & pair : mu_)
  {
    auto & arm = *(pair.second);
    if (arm.target_updated_)
    {
      arm.idle_controlled_ = false;
    }
    else
    {
      if (arm.idle_controlled_ == false)
      {
        // initialize
        arm.setInitialValues();
      }
      computeArm(time, arm, params_[pair.first]);
    }
  }
}

void IdleControlServer::computeArm(ros::Time time, FrankaModelUpdater &arm, assembly_msgs::IdleControl::Request &param)
{

  // For safety
  // Eigen::Quaterniond qt_init(arm.initial_transform_.linear());
  // Eigen::Quaterniond qt_cur(arm.transform_.linear());
  // if (qt_init.angularDistance(qt_cur) > 0.174532889) // 10 DEGREE
  // {
  //   arm.setInitialValues();
  //   std::cout << "Large angular difference detected. Set initial position to this position" << std::endl;
  // }
  
  // if ((arm.initial_transform_.translation() -arm.position_ ).norm() > 0.05)
  // {
  //   arm.setInitialValues();
  //   std::cout << "Large translational difference detected. Set initial position to this position" << std::endl;
  // }
  
  switch (param.mode)
  {

    case assembly_msgs::IdleControl::Request::DISABLED:
    {
      arm.setTorque(Eigen::Vector7d::Zero(), true);
      arm.idle_controlled_ = false;
      break;
    }
    case assembly_msgs::IdleControl::Request::JOINT_SPACE:
    {
      
      // arm.setTorque(~~~, true); // <<- idle_control = true
      break;
    }
    case assembly_msgs::IdleControl::Request::TASK_SPACE:
    {
      // default gain (gains are not set)
      if (param.p_gain <= 0.01)
      {
        ROS_INFO("IdleController::computeArm -- Set P gain to default(2000) %s", param.arm_name.c_str());
        param.p_gain = 1000.;
      } 
      if (param.d_gain <= 0.01)
      {
        ROS_INFO("IdleController::computeArm -- Set D gain to default(89) %s", param.arm_name.c_str());
        param.d_gain = 15;
      }
      if (param.vel_p_gain <= 0.01)
      {
        ROS_INFO("IdleController::computeArm -- Set velocity P gain to default(1500) %s", param.arm_name.c_str());
        param.vel_p_gain = 1500.; // q_dot = kp/kv(del_q)
      }
      if (param.vel_d_gain <= 0.01)
      {
        ROS_INFO("IdleController::computeArm -- Set velocity D gain to default(20) %s", param.arm_name.c_str());
        param.vel_d_gain = 10.0; // q_dot = kp/kv(del_q)
      }
      if (param.qdot_max <= 0.01)
      {
        ROS_INFO("IdleController::computeArm -- Set qdot_max to default(0.2) %s", param.arm_name.c_str());
        param.qdot_max = 0.2; // joint velocity when a joint escapes from joint limit
      }
      // auto fstar = PegInHole::keepCurrentState(
      //   arm.initial_transform_.translation(), arm.initial_transform_.linear(), 
      //   arm.position_, arm.rotation_, arm.xd_, param.p_gain, param.d_gain);

      auto fstar = PegInHole::keepCurrentPose(arm.initial_transform_, arm.transform_,arm.xd_,
      param.p_gain, param.d_gain, param.p_gain * 2.0, param.d_gain, arm.modified_lambda_matrix_);

      Eigen::Vector7d tau_desired = arm.jacobian_.transpose() * fstar;
      // std::cout << "fstar: " << fstar.transpose() << std::endl
      // << "tau: " << tau_desired.transpose() << std::endl;

      arm.setTorque(tau_desired, true);
      arm.idle_controlled_ = true;
      
      // auto fstar = PegInHole::keepCurrentPose(
      //   arm.initial_transform_, arm.transform_, arm.xd_, param.p_gain, param.d_gain);

      // Eigen::Vector7d tau_desired = arm.jacobian_.transpose() * fstar;
     
      // std::cout << "fstar: " << fstar.transpose() << std::endl
      // << "tau: " << tau_desired.transpose() << std::endl;

      // arm.setTorque(tau_desired, true);
      // arm.idle_controlled_ = true;
      break;
    }
    case assembly_msgs::IdleControl::Request::TASK_SPACE_WITH_NULL:
    {
      // default gain (gains are not set)
      if (param.p_gain <= 0.01)
      {
        ROS_INFO("IdleController::computeArm -- Set P gain to default(3000) %s", param.arm_name.c_str());
        param.p_gain = 3000.;
      } 
      if (param.d_gain <= 0.01)
      {
        ROS_INFO("IdleController::computeArm -- Set D gain to default(100) %s", param.arm_name.c_str());
        param.d_gain = 100.;
      }
      if (param.vel_p_gain <= 0.01)
      {
        ROS_INFO("IdleController::computeArm -- Set velocity P gain to default(1500) %s", param.arm_name.c_str());
        param.vel_p_gain = 1500.; // q_dot = kp/kv(del_q)
      }
      if (param.vel_d_gain <= 0.01)
      {
        ROS_INFO("IdleController::computeArm -- Set velocity D gain to default(20) %s", param.arm_name.c_str());
        param.vel_d_gain = 10.0; // q_dot = kp/kv(del_q)
      }
      if (param.qdot_max <= 0.01)
      {
        ROS_INFO("IdleController::computeArm -- Set qdot_max to default(0.2) %s", param.arm_name.c_str());
        param.qdot_max = 0.2; // joint velocity when a joint escapes from joint limit
      }
      
      auto fstar = PegInHole::keepCurrentState(
        arm.initial_transform_.translation(), arm.initial_transform_.linear(), 
        arm.position_, arm.rotation_, arm.xd_, param.p_gain, param.d_gain);

      // std::cout << fstar.transpose() << std::endl;
      double qdot_max = param.qdot_max;
      Eigen::Vector7d qdot_desired;
      Eigen::Vector7d vel_kp, vel_kv;

      vel_kp.setConstant(param.vel_p_gain);
      vel_kv.setConstant(param.vel_d_gain);

      for(int i = 0; i < 7; i++)
      {
        double del_q = arm.q_limit_center_(i) - arm.q_(i);

        qdot_desired(i) = vel_kp(i)/vel_kv(i)*del_q;       
        
        if(qdot_desired(i) > qdot_max) qdot_desired(i) = qdot_max;
        else if(qdot_desired(i) < - qdot_max) qdot_desired(i) = - qdot_max;
      }

      Eigen::Vector7d tau0;
      tau0 = vel_kv.asDiagonal()*(qdot_desired - arm.qd_);
      
      Eigen::Vector7d tau_desired = arm.jacobian_.transpose() * fstar + arm.null_projector_ * tau0;
     
      arm.setTorque(tau_desired, true);
      arm.idle_controlled_ = true;
      break;
    }
    case assembly_msgs::IdleControl::Request::TASK_SPACE_DEV_TEST:
    {
      auto fstar = PegInHole::keepCurrentPose(arm.initial_transform_, arm.transform_,arm.xd_,
      3000, 40, 5000, 50, arm.modified_lambda_matrix_);

      Eigen::Vector7d tau_desired = arm.jacobian_.transpose() * fstar;
      // std::cout << "fstar: " << fstar.transpose() << std::endl
      // << "tau: " << tau_desired.transpose() << std::endl;

      arm.setTorque(tau_desired, true);
      arm.idle_controlled_ = true;
      break;
    }
    default:
      arm.setTorque(Eigen::Vector7d::Zero(), true);
  }

} 

bool IdleControlServer::setTarget(assembly_msgs::IdleControl::Request  &req,
                assembly_msgs::IdleControl::Response &res)
{
  for (auto & pair : mu_)
  {
    if (pair.first == req.arm_name)
    {
      ROS_INFO("IdleController::setTarget -- receiving request. %s",pair.first.c_str());
      params_[pair.first] = req;
      res.is_succeed = true;
    }
  }
  return true;
}

