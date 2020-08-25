#include <assembly_dual_controllers/servers/grasp_update_server.h>

GraspUpdateServer::GraspUpdateServer(const std::string &name, ros::NodeHandle &nh,
                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu) :
nh_(nh), mu_(mu)
{
  server_  = nh_.advertiseService(name, &GraspUpdateServer::updater, this);
}

bool GraspUpdateServer::updater(assembly_msgs::GraspUpdate::Request  &req,
                assembly_msgs::GraspUpdate::Response &res)
{
  mu_[req.arm_name] -> mass_obj_ = req.mass_obj;
  mu_[req.arm_name] -> p_7l_ = Eigen::Vector3d(req.p_7l.x, req.p_7l.y, req.p_7l.z);
  mu_[req.arm_name] -> p_7l_hat_ = skew(mu_[req.arm_name]->p_7l_);
  res.is_succeed = true;
  return true;
}