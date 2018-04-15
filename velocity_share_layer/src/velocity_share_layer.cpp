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
  ROS_INFO("Velocity share layer initialized.");
}

void VelocityShareLayer::robotVelCallback(const velocity_share_msgs::RobotVelInfo& robot_vel_info) {
  boost::recursive_mutex::scoped_lock lock(lock_);

  // try to find robot_name in the message within the map
  std::string robot_name(robot_vel_info.robot_name);

  std::map<std::string, velocity_share_msgs::RobotVelInfo>::iterator it =
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

  std::map<std::string, velocity_share_msgs::RobotVelInfo>::iterator r_it;

  for(r_it = robot_vel_infos_.begin(); r_it != robot_vel_infos_.end(); ++r_it){
    velocity_share_msgs::RobotVelInfo robot = r_it->second;

    if (robot.path.poses.size() < 2) {
      break;
    }
    geometry_msgs::PoseStamped wp_start = robot.path.poses.front();
    geometry_msgs::PoseStamped wp_end = robot.path.poses.back();
    // TODO!!

    *min_x = (double) std::min(*min_x, std::min(wp_start.pose.position.x, wp_end.pose.position.x));
    *min_y = (double) std::min(*min_y, std::min(wp_start.pose.position.y, wp_end.pose.position.y));
    *max_x = (double) std::max(*max_x, std::max(wp_start.pose.position.x, wp_end.pose.position.x));
    *max_y = (double) std::max(*max_y, std::max(wp_start.pose.position.y, wp_end.pose.position.y));
  }
}

void VelocityShareLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
  boost::recursive_mutex::scoped_lock lock(lock_);

  if(!enabled_)
    return;

  if( robot_vel_infos_.size() == 0 )
    return;

  std::map<std::string, velocity_share_msgs::RobotVelInfo>::iterator robot_it;
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();

  for(robot_it = robot_vel_infos_.begin(); robot_it != robot_vel_infos_.end(); ++robot_it){
    velocity_share_msgs::RobotVelInfo robot_vel_info = robot_it->second;

    ros::Time robottime(robot_vel_info.path.header.stamp);
    ros::Duration dur = ros::Time::now() - robottime;

    // TODO: setup as paremeter!
    ros::Duration max_dur(5.0);
    double res = costmap->getResolution() / 2.;

    int num_steps = 0;

    if (dur < max_dur && robot_vel_info.high_prio == true) {
      unsigned int mx, my;
      for(size_t i_p = 0; i_p < robot_vel_info.path.poses.size() - 1; i_p++){
        geometry_msgs::Pose cur_pose = robot_vel_info.path.poses[i_p].pose;
        geometry_msgs::Pose next_pose = robot_vel_info.path.poses[i_p + 1].pose;

        double dx = (next_pose.position.x - cur_pose.position.x);
        double dy = (next_pose.position.y - cur_pose.position.y);
        double dphi = atan2(dy, dx);

        double x_step = cos(dphi) * res;
        double y_step = sin(dphi) * res;

        if (fabs(x_step) > fabs(y_step)) {
          num_steps = std::max((int) fabs(dx / res), 1);
        } else {
          num_steps = std::max((int) fabs(dy / res), 1);
        }

        double cur_x = cur_pose.position.x;
        double cur_y = cur_pose.position.y;

        for (int i = 0; i < num_steps; i++) {
          if(costmap->worldToMap(cur_x, cur_y, mx, my)){
              costmap->setCost(mx, my, 254);
          }
          cur_x += x_step;
          cur_y += y_step;
        }
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
