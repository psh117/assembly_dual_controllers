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
        arm.setInitialTransform();
      }
      computeArm(time, arm, params_[pair.first]);
    }
  }
}

void IdleControlServer::computeArm(ros::Time time, FrankaModelUpdater &arm, assembly_msgs::IdleControl::Request &param)
{
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
        ROS_INFO("IdleController::computeArm -- Set P gain to default(5000) %s", param.arm_name.c_str());
        param.p_gain = 5000.;
      } 
      if (param.d_gain <= 0.01)
      {
        ROS_INFO("IdleController::computeArm -- Set D gain to default(100) %s", param.arm_name.c_str());
        param.d_gain = 100.;
      }
      
      auto fstar = PegInHole::keepCurrentState(
        arm.initial_transform_.translation(), arm.initial_transform_.linear(), 
        arm.position_, arm.rotation_, arm.xd_, param.p_gain, param.d_gain);

      // std::cout << fstar.transpose() << std::endl;
      arm.setTorque(arm.jacobian_.transpose() * fstar, true);
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
