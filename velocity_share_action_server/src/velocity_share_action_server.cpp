/***************************************************************************
 *  velocity_share_action_server.cpp
 *
 *  Created on Fri Nov  2 20:54:16 CET 2018
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

#include <velocity_share_action_server/velocity_share_action_server.h>

namespace velocity_share_action_server {

  VelocityShareActionServer::VelocityShareActionServer()
  {
    ros::NodeHandle nh;

    // TODO: make these parameterizable
    controller_frequency_ = 10.0;
    global_plan_topic_name_ = "/move_base/GlobalPlanner/plan";
    global_plan_sub_ = nh.subscribe<nav_msgs::Path>(global_plan_topic_name_, 1, boost::bind(&VelocityShareActionServer::pathCb, this, _1));

    path_updated_ = false;
    active_ = false;
    cur_goal_mode_ = GOAL_MODE::DIRECT;

    costmap_ros_ = new costmap_2d::Costmap2DROS("vel_share_server_costmap", tf_);
    costmap_ros_->start();
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "vel_share_server", boost::bind(&VelocityShareActionServer::executeCb, this, _1), false);
    ac_ = new MoveBaseActionClient("move_base");

    ROS_INFO("velocity_share_action_server waiting for move_base");
    ac_->waitForServer();
    as_->start();

    ros::NodeHandle action_nh("vel_share_server");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

    ros::NodeHandle simple_nh("vel_share_server_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&VelocityShareActionServer::goalCB, this, _1));

    ROS_INFO("velocity_share_action_server Initialized");
  }

  VelocityShareActionServer::~VelocityShareActionServer()
  {
    delete as_;
    delete ac_;
    delete costmap_ros_;
  }

  void VelocityShareActionServer::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    // simply forward the desired goal to the move_base
    orig_move_base_goal_ = *move_base_goal;
    sendGoal(orig_move_base_goal_);

    ros::Rate r(controller_frequency_);

    ros::NodeHandle n;
    while(n.ok() && active_ == true)
    {
        if(as_->isPreemptRequested()){
          if(as_->isNewGoalAvailable()){
            ROS_WARN("vel_share_server: got a new goal while moving");
            orig_move_base_goal_ = *as_->acceptNewGoal();
            sendGoal(orig_move_base_goal_);
            cur_goal_mode_ = GOAL_MODE::DIRECT;
            } else {
              as_->setPreempted();
            }
        }

        std::cout << "state: " << ac_->getState().toString() << std::endl;
        if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED && cur_goal_mode_ == GOAL_MODE::DIRECT) {
          as_->setSucceeded();
          active_ = false;
          return;
        } else if (ac_->getState() == actionlib::SimpleClientGoalState::PENDING) {
        } else if (ac_->getState() == actionlib::SimpleClientGoalState::ACTIVE) {
        } else if (ac_->getState() == actionlib::SimpleClientGoalState::ABORTED) {
          as_->setAborted();
          active_ = false;
          return;
        } else if (ac_->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
          as_->setPreempted();
          active_ = false;
          return;
        }

        // check whether we need to validate the path for possible collisions
        // with robots
        if (path_updated_ && cur_goal_mode_ == GOAL_MODE::DIRECT) {
          if (!validateGlobalPath())
            cur_goal_mode_ = GOAL_MODE::INIT_GIVE_PRIORITY;
        }
        if (cur_goal_mode_ == GOAL_MODE::INIT_GIVE_PRIORITY) {
          sendGoal(temp_pose_);
          cur_goal_mode_ = GOAL_MODE::GIVE_PRIORITY;
        }
        if (cur_goal_mode_ == GOAL_MODE::GIVE_PRIORITY) {
          if (validateGlobalPath()) {
            cur_goal_mode_ = GOAL_MODE::DIRECT;
            sendGoal(orig_move_base_goal_);
          }
        }
      r.sleep();
    }
  }

  void VelocityShareActionServer::doneCb(const actionlib::SimpleClientGoalState& state,
              const move_base_msgs::MoveBaseResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  }

  void VelocityShareActionServer::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
  {
    as_->publishFeedback(*feedback);
  }

  void VelocityShareActionServer::activeCb()
  {
  }

  void VelocityShareActionServer::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void VelocityShareActionServer::pathCb(const nav_msgs::PathConstPtr& path)
  {
    // Do not overwrite the path when we are not in DIRECT GOAL_MODE
    if (cur_goal_mode_ == GOAL_MODE::DIRECT) {
      path_ = *path;
      path_updated_ = true;
    }
  }

  void VelocityShareActionServer::sendGoal(const move_base_msgs::MoveBaseGoal &goal) {

    ac_->sendGoal(goal,
                  boost::bind(&VelocityShareActionServer::doneCb, this, _1, _2),
                  boost::bind(&VelocityShareActionServer::activeCb, this),
                  boost::bind(&VelocityShareActionServer::feedbackCb, this, _1));
    active_ = true;
  }

  void VelocityShareActionServer::sendGoal(const geometry_msgs::PoseStamped &goal) {
    move_base_msgs::MoveBaseGoal move_base_goal;
    move_base_goal.target_pose = goal;

    sendGoal(move_base_goal);
  }

  bool VelocityShareActionServer::validateGlobalPath() {
    bool retVal = true;

    unsigned char costs_pose_1;
    unsigned char costs_pose_2;
    unsigned char costs_pose_3;

    for (int i = 0; i < path_.poses.size() - 2; i++) {

      // we want to check up to three poses to validate whether the costs rise or not.
      // If the costs increase we know that the path is leading towards a possible collision.
      // If the costs decrease we know that the path is leading outwards of a collision and is not harmful.

      geometry_msgs::PoseStamped pose_1 = path_.poses[i];
      geometry_msgs::PoseStamped pose_2 = path_.poses[i+1];
      geometry_msgs::PoseStamped pose_3 = path_.poses[i+2];

      costs_pose_1 = getCostsAt(pose_1);

      if (costs_pose_1 > COST_THRESHOLD) {
        costs_pose_2 = getCostsAt(pose_2);
        costs_pose_3 = getCostsAt(pose_3);

        ROS_INFO("costs: %i %i %i", (int) costs_pose_1, (int) costs_pose_2, (int) costs_pose_3);

        if (costs_pose_1 <= costs_pose_2 && costs_pose_2 <= costs_pose_3) {
          retVal = false;
          temp_pose_ = pose_1;
          break;
        }
      }
    }
    return retVal;
  }

  unsigned char VelocityShareActionServer::getCostsAt(geometry_msgs::PoseStamped &pose) {

    ROS_INFO("get costs at %f %f", pose.pose.position.x, pose.pose.position.y);
    costmap_ros_->getCostmap()->
        worldToMap(pose.pose.position.x, pose.pose.position.y, cur_world_to_map_x, cur_world_to_map_y);
    return costmap_ros_->getCostmap()->getCost(cur_world_to_map_x, cur_world_to_map_y);
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "velocity_share_action_server");

  ROS_INFO("MAIN");

  velocity_share_action_server::VelocityShareActionServer vel_share_server;
  ROS_INFO("SPIN");

  ros::spin();

  return(0);
}





/*
  class FibonacciAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;

public:

  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }

  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}

*/
