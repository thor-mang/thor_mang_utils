#include <thor_mang_head_tracking/head_tracking_node.h>



namespace thor_mang_head_tracking
{
HeadTrackingNode::HeadTrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : valid_head_state_(false)
{
  // get parameters
  if (!pnh.getParam("camera_frame", camera_frame_))
  {
    ROS_ERROR("Camera frame was not given! Please add 'camera_frame' to parameter server.");
    return;
  }
  if (!pnh.getParam("tracked_frame", tracked_frame_))
  {
    ROS_ERROR("Tracked frame was not given! Please add 'camera_frame' to parameter server.");
    return;
  }

  // init trajectory controller
  head_traj_client_.reset(new TrajectoryActionClient("joints/head_traj_controller/follow_joint_trajectory", true));

  // subscriber
  joint_state_sub_ = nh.subscribe("joints/joint_states", 1, &HeadTrackingNode::jointStateCB, this);

  // schedule main update loop
  update_timer_ = nh.createTimer(nh.param("control_rate", 20.0), &HeadTrackingNode::update, this);
}

HeadTrackingNode::~HeadTrackingNode()
{

}

void HeadTrackingNode::jointStateCB(const sensor_msgs::JointState::ConstPtr msg)
{
  bool has_head_p = false;
  bool has_head_y = false;

  // get head joint state
  for (size_t i = 0; i < msg->name.size(); i++)
  {
    if (msg->name[i] == "head_p")
    {
      head_p_ = msg->position[i];
      has_head_p = true;
    }

    if (msg->name[i] == "head_y")
    {
      head_y_ = msg->position[i];
      has_head_y = true;
    }

    if (!valid_head_state_ && has_head_p && has_head_y)
      valid_head_state_ = true;
  }
}

void HeadTrackingNode::update(const ros::TimerEvent& /*event*/)
{
  if (!valid_head_state_)
    return;

  // get transform between frames
  tf::StampedTransform transform;
  if (tf_listener_.canTransform(camera_frame_, tracked_frame_, ros::Time(0)))
  {
    tf_listener_.lookupTransform(camera_frame_, tracked_frame_, ros::Time(0), transform);
  }
  else
  {
    ROS_WARN_THROTTLE(1.0, "No transform from '%s' to '%s' available.", tracked_frame_.c_str(), camera_frame_.c_str());
    return;
  }

  // compute new joint values
  double d_pitch = -atan2(transform.getOrigin().z(), transform.getOrigin().x());
  double d_yaw = atan2(transform.getOrigin().y(), transform.getOrigin().x());

  //ROS_WARN("transf: %.2f, %.2f, %.2f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  //ROS_WARN("delta: %.2f (%.2f), %.2f (%.2f)", d_pitch, d_pitch/M_PI*180.0, d_yaw, d_yaw/M_PI*180.0);

  std::vector<double> positions;
  positions.push_back(head_p_ + d_pitch);
  positions.push_back(head_y_ + d_yaw);

  //ROS_INFO(">>> %.2f (%.2f), %.2f (%.2f)", positions[0], positions[0]/M_PI*180.0, positions[1], positions[1]/M_PI*180.0);

  // send new trajectory
  if (!head_traj_client_->waitForServer(ros::Duration(1.0)))
    ROS_WARN("Time out while waiting for head trajectory controller");

  if (head_traj_client_->isServerConnected())
  {
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names.push_back("head_p");
    joint_trajectory.joint_names.push_back("head_y");

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = ros::Duration(0.05);
    joint_trajectory.points.push_back(point);

    control_msgs::FollowJointTrajectoryGoal trajectory_goal;
    trajectory_goal.trajectory = joint_trajectory;

    // Send goals to controllers
    head_traj_client_->sendGoal(trajectory_goal);
  }
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "l3_walk_controller");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  thor_mang_head_tracking::HeadTrackingNode node(nh, pnh);

  ros::spin();

  return 0;
}
