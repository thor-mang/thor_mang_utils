//=================================================================================================
// Copyright (c) 2018, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef THOR_MANG_HEAD_TRACKING_NODE_H__
#define THOR_MANG_HEAD_TRACKING_NODE_H__

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>



namespace thor_mang_head_tracking
{
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryActionClient;

class HeadTrackingNode
{
public:
  HeadTrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  virtual ~HeadTrackingNode();

protected:
  void jointStateCB(const sensor_msgs::JointState::ConstPtr msg);

  /**
   * @brief Main update loop to be called in regular intervals.
   */
  void update(const ros::TimerEvent& event = ros::TimerEvent());

  // timer for updating periodically
  ros::Timer update_timer_;

  tf::TransformListener tf_listener_;

  std::string camera_frame_;
  std::string tracked_frame_;

  boost::shared_ptr<TrajectoryActionClient> head_traj_client_;

  bool valid_head_state_;
  double head_p_;
  double head_y_;

  double max_vel_; // [rad/s]

  // subscriber
  ros::Subscriber joint_state_sub_;
};
}

#endif
