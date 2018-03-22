#include <velocity_share_layer/velocity_share_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(velocity_share_layer::VelocityShareLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace velocity_share_layer
{
void VelocityShareLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  server_ = new dynamic_reconfigure::Server<VelocityShareLayerConfig>(nh);
  f_ = boost::bind(&VelocityShareLayer::configure, this, _1, _2);
  server_->setCallback(f_);

  enabled_ = true;

  robot_vel_sub_ = nh.subscribe("/robot_velocities", 1, &VelocityShareLayer::robotVelCallback, this);
}

void VelocityShareLayer::robotVelCallback(const velocity_share_msgs::RobotVelInfoStamped& robot_vel_info) {
  boost::recursive_mutex::scoped_lock lock(lock_);

  // try to find robot_name in the message within the map
  std::string robot_name(robot_vel_info.robotvelinfo.robot_name);

  std::map<std::string, velocity_share_msgs::RobotVelInfoStamped>::iterator it =
      robot_vel_infos_.find(robot_name);

  if (it != robot_vel_infos_.end()) {
    // if it is found, update
    it->second = robot_vel_info;
  } else {
    // otherwise create a new element in the map
    robot_vel_infos_.insert(
          std::make_pair(robot_name, robot_vel_info));
  }
}


void VelocityShareLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  std::map<std::string, velocity_share_msgs::RobotVelInfoStamped>::iterator r_it;

  for(r_it = robot_vel_infos_.begin(); r_it != robot_vel_infos_.end(); ++r_it){
    velocity_share_msgs::RobotVelInfoStamped robot = r_it->second;

    *min_x = (double) std::min(*min_x, std::min(robot.robotvelinfo.pose.position.x + robot.robotvelinfo.vel_endpoint.x,
                                                robot.robotvelinfo.pose.position.x - robot.robotvelinfo.vel_endpoint.x));
    *min_y = (double) std::min(*min_y, std::min(robot.robotvelinfo.pose.position.y + robot.robotvelinfo.vel_endpoint.y,
                                                robot.robotvelinfo.pose.position.y - robot.robotvelinfo.vel_endpoint.y));
    *max_x = (double) std::max(*max_x, std::max(robot.robotvelinfo.pose.position.x + robot.robotvelinfo.vel_endpoint.x,
                                                robot.robotvelinfo.pose.position.x - robot.robotvelinfo.vel_endpoint.x));
    *max_y = (double) std::max(*max_y, std::max(robot.robotvelinfo.pose.position.y + robot.robotvelinfo.vel_endpoint.y,
                                                robot.robotvelinfo.pose.position.y - robot.robotvelinfo.vel_endpoint.y));
  }
}

void VelocityShareLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
  boost::recursive_mutex::scoped_lock lock(lock_);

  if(!enabled_)
    return;

  if( robot_vel_infos_.size() == 0 )
    return;

  std::map<std::string, velocity_share_msgs::RobotVelInfoStamped>::iterator robot_it;
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();

  for(robot_it = robot_vel_infos_.begin(); robot_it != robot_vel_infos_.end(); ++robot_it){
    velocity_share_msgs::RobotVelInfoStamped robot_vel_info = robot_it->second;

    ros::Time robottime(robot_vel_info.header.stamp);
    ros::Duration dur = ros::Time::now() - robottime;

    // TODO: setup as paremeter!
    ros::Duration max_dur(5.0);
    double res = costmap->getResolution();

    if (dur < max_dur) {
      unsigned int mx, my;

      double dx = (robot_vel_info.robotvelinfo.vel_endpoint.x - robot_vel_info.robotvelinfo.pose.position.x);
      double dy = (robot_vel_info.robotvelinfo.vel_endpoint.y - robot_vel_info.robotvelinfo.pose.position.y);

      double x_step = dx * res;
      double y_step = dy * res;

      int num_steps;

      if (fabs(x_step) > fabs(y_step)) {
        num_steps = fabs(dx / res);
      } else {
        num_steps = fabs(dy / res);
      }

      double cur_x = robot_vel_info.robotvelinfo.pose.position.x;
      double cur_y = robot_vel_info.robotvelinfo.pose.position.y;

      for (int i = 0; i < num_steps; i++) {
        if(costmap->worldToMap(cur_x, cur_y, mx, my)){
          costmap->setCost(mx, my, LETHAL_OBSTACLE);
        }
        cur_x += x_step;
        cur_y += y_step;
      }
    }
  }
}

void VelocityShareLayer::configure(VelocityShareLayerConfig &config, uint32_t level) {
  cutoff_ = config.cutoff;
  amplitude_ = config.amplitude;
  covar_ = config.covariance;
  factor_ = config.factor;
  enabled_ = config.enabled;
}


};
