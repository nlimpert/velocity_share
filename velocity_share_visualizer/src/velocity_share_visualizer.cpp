#include <ros/ros.h>
#include <tf/tf.h>
#include <velocity_share_msgs/RobotVelInfoStamped.h>
#include <visualization_msgs/Marker.h>

// set certaing arrow scaling factor
#define SCALE_FACTOR 2.0

class RobotVelVisualizer
{
private:
  ros::Publisher marker_pub;
  ros::Subscriber sub;
public:
  RobotVelVisualizer();
  void robotvelCallback(const velocity_share_msgs::RobotVelInfoStamped::ConstPtr& msg);
};

RobotVelVisualizer::RobotVelVisualizer()
{
  ros::NodeHandle n;
  sub = n.subscribe("/robot_velocities", 1000, &RobotVelVisualizer::robotvelCallback, this);
  marker_pub = n.advertise<visualization_msgs::Marker>("/robot_velocities_visualizer", 1);
}

void
RobotVelVisualizer::robotvelCallback(const velocity_share_msgs::RobotVelInfoStamped::ConstPtr& msg)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.type = marker.ARROW;
  marker.action = marker.ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.g = 1.0;
  marker.color.a = 1.0;

  geometry_msgs::Point p1;
  p1.x = msg->robotvelinfo.pose.position.x;
  p1.y = msg->robotvelinfo.pose.position.y;

  geometry_msgs::Point p2;
  p2.x = msg->robotvelinfo.vel_endpoint.x;
  p2.y = msg->robotvelinfo.vel_endpoint.y;

  marker.points.push_back(p1);
  marker.points.push_back(p2);

  marker_pub.publish(marker);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "RobotVelVisualizer");

  RobotVelVisualizer visualizer;

  ros::spin();

  return 0;
}
