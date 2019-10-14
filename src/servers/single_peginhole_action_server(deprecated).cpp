#include <assembly_dual_controllers/single_peginhole_action_server.h>

SinglePegInHoleActionServer::SinglePegInHoleActionServer(std::string name, ros::NodeHandle &nh, 
                                std::shared_ptr<FrankaModelUpdater> &mu_r, std::shared_ptr<FrankaModelUpdater> &mu_l)
: ActionServerBase(name,nh,mu_r,mu_l),
as_(nh,name,false)
{
  as_.registerGoalCallback(boost::bind(&SinglePegInHoleActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&SinglePegInHoleActionServer::preemptCallback, this));
  as_.start();
}

void SinglePegInHoleActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  ROS_INFO("AssembleApproach goal has been received.");

  start_time_ = ros::Time::now();
  initParameters();  
}  

void SinglePegInHoleActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool SinglePegInHoleActionServer::getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque_main, Eigen::Matrix<double, 7, 1> & torque_sub)
{
  Eigen::Matrix<double, 6, 1> f_main;
  Eigen::Matrix<double, 6, 1> f_sub;

  if (!as_.isActive())  return false; 

  readState();
  parametersAllocator();

  f_main = mainArm(time);
  f_sub = subArm(time);

  torque_main = J_main_.transpose()*f_main;
  torque_sub = J_sub_.transpose()*f_sub;

  return true;
}

void SinglePegInHoleActionServer::readState()
{
  auto & rotation_r = mu_r_->rotation_;
  auto & position_r = mu_r_->position_;
  auto & jacobian_r = mu_r_->jacobian_;
  auto & mass_r = mu_r_->mass_matrix_;
  auto & tau_measured_r = mu_r_->tau_measured_;
  auto & gravity_r = mu_r_->gravity_;
  auto & xd_r = mu_r_->xd_;

  auto & rotation_l = mu_l_->rotation_;
  auto & position_l = mu_l_->position_;
  auto & jacobian_l = mu_l_->jacobian_;
  auto & mass_l = mu_l_->mass_matrix_;
  auto & tau_measured_l = mu_l_->tau_measured_;
  auto & gravity_l = mu_l_->gravity_;
  auto & xd_l = mu_l_->xd_;

  task_type_ = goal_ -> task_type;

  Eigen::Matrix<double, 6, 6> lambda_r;
  Eigen::Matrix<double, 6, 6> lambda_l;

  lambda_r = (jacobian_r*mass_r.inverse()*jacobian_r.transpose()).inverse();
  lambda_l = (jacobian_l*mass_l.inverse()*jacobian_l.transpose()).inverse();

  pos_r_ = position_r;
  rot_r_ = rotation_r;
  vel_r_ = xd_r;
  J_r_ = jacobian_r;

  pos_l_ = position_l;
  rot_l_ = rotation_l;
  vel_l_ = xd_l;
  J_l_ = jacobian_l;
   
  J_bar_r_ = mass_r.inverse()*J_r_.transpose()*lambda_r;
  J_bar_l_ = mass_l.inverse()*J_l_.transpose()*lambda_l;

  f_sensing_r_ = J_bar_r_.transpose()*(tau_measured_r - gravity_r);
  f_sensing_l_ = J_bar_l_.transpose()*(tau_measured_l - gravity_l);

  f_xy_r_ = sqrt(f_sensing_r_(0)*f_sensing_r_(0) +f_sensing_r_(1)*f_sensing_r_(1));
  f_xy_l_ = sqrt(f_sensing_l_(0)*f_sensing_l_(0) +f_sensing_l_(1)*f_sensing_l_(1));
}


Eigen::Matrix<double, 6, 1> SinglePegInHoleActionServer::mainArm(ros::Time time)
{
    
  switch(stage_)
  {
    case 0:
      approach(pos_ref_main_, rot_ref_main_, vel_ref_main_, f_ref_main_(2), time);
      break;
    case 1:
      // crossSearch();
      break;
    case 2:
      spiral(pos_ref_main_, rot_ref_main_, vel_ref_main_, f_xy_main_, time);
      break;
    
    case 3:
      insertion(pos_ref_main_, rot_ref_main_, time);
      break;
    
    case 4:
      // inspection();
      break;
    
    case 5:
      break;
    
    case 6:
      break;
    
    case 7: //recovery process
      // retreat();
      break;
  }

  f_star_zero_.head<3>() = f_star_;
  f_star_zero_.tail<3>() = m_star_;

  return f_star_zero_;
}

Eigen::Matrix<double, 6, 1> SinglePegInHoleActionServer::subArm(ros::Time time)
{ 
  hold(pos_ref_sub_, rot_ref_sub_, vel_ref_sub_);

  f_star_zero_.head<3>() = f_star_;
  f_star_zero_.tail<3>() = m_star_;

  return f_star_zero_;
}

void SinglePegInHoleActionServer::approach(const Eigen::Vector3d pos_ref,
  const Eigen::Matrix3d rot_ref,
  const Eigen::Matrix<double, 6, 1> vel_ref,
  const double f_ref,
  ros::Time time) //stage = 0;
{
  if(is_first_ == true) setForStage(pos_ref, rot_ref);
  
  if(checkContact(f_ref))
  {
    std::cout<<"CHECK CONTATCT!!!!!"<<std::endl;
    completeStage();
    stage_ = 2;   
  }
  else
  {
    f_star_ = straightMove(origin_, init_rot_, rot_ref, pos_ref, vel_ref, time.toSec(), start_time_.toSec()).head<3>();
    m_star_ = keepOrientationPerpenticular(init_rot_, rot_ref, vel_ref, 2.0, time.toSec(), start_time_.toSec());
  }   
}

void SinglePegInHoleActionServer::spiral(const Eigen::Vector3d pos_ref,
  const Eigen::Matrix3d rot_ref,
  const Eigen::Matrix<double, 6, 1> vel_ref,
  const double f_ref,
  ros::Time time) //stage = 2;
{
  if(is_first_ == true) setForStage(pos_ref, rot_ref);
  
  if(timeOut(time.toSec(), start_time_.toSec(), spiral_duration_)) stage_ = 7;//setForIncomplete(); //the process is ended

  f_star_ = generateSpiral(origin_, init_rot_, rot_ref, pos_ref, vel_ref, time.toSec(), start_time_.toSec(), spiral_duration_);
  f_star_(2) = -4.0; // put some value!!! //-6 //-12
  m_star_ = keepOrientationPerpenticular(init_rot_, rot_ref, vel_ref, 2.0, time.toSec(), start_time_.toSec());
  
  if(detectHole(origin_(2),pos_ref(2),f_ref,0.003,15.0))
  {
    std::cout<<"HOLE IS DETECTED"<<std::endl;
    completeStage();
    stage_ = 3; // detect a hole then, move to insertion stage.
  }
}

void SinglePegInHoleActionServer::insertion(const Eigen::Vector3d pos_ref,
  const Eigen::Matrix3d rot_ref,
  ros::Time time) //stage = 3;
{
  if(is_first_ == true) setForStage(pos_ref, rot_ref);
            
  if(start_time_.toSec() + 3.0 <= time.toSec()) //after pushing for 3 seconds
  {
    completeStage();
    std::cout<<"INSERTION IS FINISHED"<<std::endl;
    //stage_ = 4;
    f_star_.setZero();
    m_star_.setZero();
  }

  else
  {
    f_star_ = 500 * (origin_ - pos_ref);
    m_star_.setZero();
    f_star_(2) = -20.0; 
  }
}

void SinglePegInHoleActionServer::hold(const Eigen::Vector3d pos_ref,
  const Eigen::Matrix3d rot_ref, const Eigen::Matrix<double, 6, 1> vel_ref)
{
  if(is_first_ == true) setForStage(pos_ref, rot_ref);

  f_star_ = keepCurrentState(origin_, init_rot_, rot_ref, pos_ref, vel_ref).head<3>();
  m_star_ = keepCurrentState(origin_, init_rot_, rot_ref, pos_ref, vel_ref).tail<3>();
}

void SinglePegInHoleActionServer::setForStage(const Eigen::Vector3d pos_ref, const Eigen::Matrix3d rot_ref)
{
  start_time_ = ros::Time::now();
  init_rot_ = rot_ref;
  origin_ = pos_ref;

  is_first_ = false;
}

void SinglePegInHoleActionServer::completeStage()
{
  is_first_ = true;

  std::cout<<"go to the next step"<<std::endl;
}

void SinglePegInHoleActionServer::parametersAllocator()
{
  uint32_t hash = HashCode(task_type_.c_str());

  switch(hash)
  {
    case HashCode("left"):
      pos_ref_main_ = pos_l_;
      rot_ref_main_ = rot_l_;
      vel_ref_main_ = vel_l_;
      pos_ref_sub_ = pos_r_;
      rot_ref_sub_ = rot_r_;
      vel_ref_sub_ = vel_r_;
      f_ref_main_ = f_sensing_l_;
      f_ref_sub_ = f_sensing_r_;
      f_xy_main_ = f_xy_l_;
      f_xy_sub_ = f_xy_r_;
      J_bar_main_ = J_bar_l_;
      J_bar_sub_ = J_bar_r_;
      J_main_ = J_l_;
      J_sub_ = J_r_;
      break;
    case HashCode("right"):
      pos_ref_main_ = pos_r_;
      rot_ref_main_ = rot_r_;
      vel_ref_main_ = vel_r_;
      pos_ref_sub_ = pos_l_;
      rot_ref_sub_ = rot_l_;
      vel_ref_sub_ = vel_l_;
      f_ref_main_ = f_sensing_r_;
      f_ref_sub_ = f_sensing_l_;
      f_xy_main_ = f_xy_r_;
      f_xy_sub_ = f_xy_l_;
      J_bar_main_ = J_bar_r_;
      J_bar_sub_ = J_bar_l_;
      J_main_ = J_r_;
      J_sub_ = J_l_;
      break;
  }
}

void SinglePegInHoleActionServer::taskArmDefine(const std::string left_arm_id, const std::string right_arm_id,
  std::string main_arm_id, std::string sub_arm_id)
{
  uint32_t hash = HashCode(task_type_.c_str());

  switch(hash)
  {
    case HashCode("left"):
      main_arm_id = left_arm_id;
      sub_arm_id = right_arm_id;
      break;
    case HashCode("right"):
      main_arm_id = right_arm_id;
      sub_arm_id = left_arm_id;
      break;
  }
}

void SinglePegInHoleActionServer::initParameters()
{
  pos_r_.setZero(); //current position of right arm
  pos_l_.setZero(); //current position of left arm

  vel_r_.setZero(); //linear + angular
  vel_l_.setZero();

  f_sensing_r_.setZero();
  f_sensing_l_.setZero();
  
  J_r_.setZero();
  J_l_.setZero();        
  J_main_.setZero();
  J_sub_.setZero();
  J_bar_r_.setZero();
  J_bar_l_.setZero();
  J_bar_main_.setZero();
  J_bar_sub_.setZero();

  rot_r_.setZero();
  rot_l_.setZero();
  init_rot_.setZero();                

  f_star_.setZero();
  m_star_.setZero();
  f_star_zero_.setZero();

  f_star_zero_r_.setZero();
  f_star_zero_l_.setZero();

  origin_.setZero();
  
  pos_ref_main_.setZero();
  pos_ref_sub_.setZero();
  rot_ref_main_.setZero();
  rot_ref_sub_.setZero();
  vel_ref_main_.setZero();
  vel_ref_sub_.setZero();
  f_ref_main_.setZero();
  f_ref_sub_.setZero();
  
  is_first_ = true;
  
  spiral_duration_ = 100.0;
  
  f_xy_r_ = 0.0;
  f_xy_l_ = 0.0;
  f_xy_main_ = 0.0;
  f_xy_sub_ = 0.0;

  stage_ = 0;

  // left_arm_id_= "172.16.0.2";
  // right_arm_id_ = "172.16.1.3";
}


