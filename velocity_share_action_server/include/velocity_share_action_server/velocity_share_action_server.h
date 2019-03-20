/***************************************************************************
 *  velocity_share_action_server.h
 *
 *  Created on Fri Nov  2 20:28:23 CET 2018
 *  Copyright (C) 2018 by Nicolas Limpert, limpert@fh-aachen.de
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 */

#ifndef VELOCITY_SHARE_ACTIONSERVER_H_
#define VELOCITY_SHARE_ACTIONSERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>

#define COST_THRESHOLD 10

namespace velocity_share_action_server {

  enum GOAL_MODE {
    DIRECT,
    INIT_GIVE_PRIORITY,
    GIVE_PRIORITY
  };

  // We want to expose the same interface like move_base so we make use of the same message type.
  // This is taken from move_base.h
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

  class VelocityShareActionServer {
  public:
    /**
     * @brief  Constructor
     */
    VelocityShareActionServer();

    /**
     * @brief  Destructor
     */
    virtual ~VelocityShareActionServer();

    /**
     * @brief The callback needed for the MoveBaseAction messages.
     * @param move_base_goal The goal to be used for move_base.
     */
    void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

    /**
     * @brief Called once when the goal completes.
     * @param state - The final state of the actionserver.
     * @param result - The result of the move_base execution. Currently unused.
     */
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const move_base_msgs::MoveBaseResultConstPtr& result);

    /**
     * @brief feedbackCb - Called every time feedback is received for the goal.
     * @param feedback - The current robot's position.
     */
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    /**
     * @brief activeCb - Called once when the goal becomes active.
     */
    void activeCb();

    /**
     * @brief goalCB - Needed to use this ActionServer with plain PoseStamped messages.
     * @param goal
     */
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

    /**
     * @brief sendGoal - Method to send the goal to the move_base.
     * @param goal The MoveBaseGoal to navigate to.
     */
    void sendGoal(const move_base_msgs::MoveBaseGoal &goal);
    void sendGoal(const geometry_msgs::PoseStamped &goal);



  private:
    /**
     * @brief pathCb - The callback for new paths calculated by the global planner.
     * @param path
     */
    void pathCb(const nav_msgs::PathConstPtr& path);

    /**
     * @brief validateGlobalPath - Validate the latest path published by move_base against
     * possible collisions according to the costmap's occupancy grid.
     *
     * @return true if the plan doesn't contain any collisions.
     *         false if the plan contains at least one collision.
     */
    bool validateGlobalPath();

    /**
     * @brief getCostsAt - Compute the costmap's cell costs at a certain position.
     * @param pose - The pose of interest.
     * @return cell cost value
     */
    unsigned char getCostsAt(geometry_msgs::PoseStamped &pose);

    MoveBaseActionServer* as_; // The actionserver to expose the move_base functionality
    MoveBaseActionClient* ac_; // The actionclient to talk to the real move_base

    // an instance of Costmap2D needed for the velocity_share_layer
    tf::TransformListener tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    geometry_msgs::PoseStamped base_position_;
    geometry_msgs::PoseStamped cur_goal_;
    move_base_msgs::MoveBaseGoal orig_move_base_goal_;

    // the temp_pose is needed to let the robot navigate
    // to a pose close to another robot's path
    geometry_msgs::PoseStamped temp_pose_;

    nav_msgs::Path path_;
    bool path_updated_;
    int cur_goal_mode_;

    std::string global_plan_topic_name_;
    bool active_;

    ros::Publisher action_goal_pub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber global_plan_sub_;

    double controller_frequency_;

    // variables needed for the getCostsAt method
    unsigned int cur_world_to_map_x;
    unsigned int cur_world_to_map_y;
  };
}
#endif
