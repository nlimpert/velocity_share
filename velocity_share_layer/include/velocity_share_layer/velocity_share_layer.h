#ifndef VELOCITY_SHARE_LAYER_H_
#define VELOCITY_SHARE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <velocity_share_msgs/RobotVelInfoStamped.h>
#include <velocity_share_layer/VelocityShareLayerConfig.h>
#include <nav_msgs/Path.h>
#include <map>
#include <string>

namespace velocity_share_layer
{
class VelocityShareLayer : public costmap_2d::Layer
{
public:
  VelocityShareLayer() { layered_costmap_ = NULL; }

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
  void robotVelCallback(const velocity_share_msgs::RobotVelInfo& robot_vel_info);
  ros::Subscriber robot_vel_sub_;
  ros::Subscriber global_path_sub_;
  std::map<std::string, velocity_share_msgs::RobotVelInfo> robot_vel_infos_;

  void configure(VelocityShareLayerConfig &config, uint32_t level);
  double cutoff_, amplitude_, covar_, factor_;
  ros::Duration robot_keep_time_;
  nav_msgs::Path path_;
  boost::recursive_mutex lock_;
  dynamic_reconfigure::Server<VelocityShareLayerConfig>* server_;
  dynamic_reconfigure::Server<VelocityShareLayerConfig>::CallbackType f_;
private:

};
};


#endif

