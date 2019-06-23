#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <drive_ros_uavcan/phoenix_msgs__NucDriveCommand.h>
#include <drive_ros_uavcan/phoenix_msgs__RemoteControl.h>

const int FRONT_LEFT = 0;
const int FRONT_RIGHT = 1;
const int REAR_LEFT = 2;
const int REAR_RIGHT = 3;

class GazeboRosBodyControl {
public:
  GazeboRosBodyControl(ros::NodeHandle nh, ros::NodeHandle pnh):
    nh_(nh),
    pnh_(pnh),
    steer_(4),
    velocities_(4),
    manual_driving_(false)
  {
    front_left_steering_pub_ = nh_.advertise<std_msgs::Float64>("front_left_steering_out", 5);
    front_left_velocity_pub_ = nh_.advertise<std_msgs::Float64>("front_left_velocity_out", 5);
    front_right_steering_pub_ = nh_.advertise<std_msgs::Float64>("front_right_steering_out", 5);
    front_right_velocity_pub_ = nh_.advertise<std_msgs::Float64>("front_right_velocity_out", 5);
    rear_left_steering_pub_ = nh_.advertise<std_msgs::Float64>("rear_left_steering_out", 5);
    rear_left_velocity_pub_ = nh_.advertise<std_msgs::Float64>("rear_left_velocity_out", 5);
    rear_right_steering_pub_ = nh_.advertise<std_msgs::Float64>("rear_right_steering_out", 5);
    rear_right_velocity_pub_ = nh_.advertise<std_msgs::Float64>("rear_right_velocity_out", 5);
    pnh_.getParam("manual", manual_driving_);

    if (manual_driving_)
      command_sub_ = nh_.subscribe("command_in", 1, &GazeboRosBodyControl::manualDrivingCB, this);
    else
      command_sub_ = nh_.subscribe("command_in", 1, &GazeboRosBodyControl::driveCommandCB, this);
  }

  void dynamicsControl()
  {
    ROS_INFO_STREAM("Velocity: "<<velocities_[FRONT_LEFT]<<", "<<velocities_[FRONT_RIGHT]);
    std_msgs::Float64 cmd_msg;
    cmd_msg.data = steer_[FRONT_LEFT];
    front_left_steering_pub_.publish(cmd_msg);
    cmd_msg.data = velocities_[FRONT_LEFT];
    front_left_velocity_pub_.publish(cmd_msg);
    cmd_msg.data = steer_[FRONT_RIGHT];
    front_right_steering_pub_.publish(cmd_msg);
    cmd_msg.data = velocities_[FRONT_RIGHT];
    front_right_velocity_pub_.publish(cmd_msg);
    cmd_msg.data = steer_[REAR_LEFT];
    rear_left_steering_pub_.publish(cmd_msg);
    cmd_msg.data = velocities_[REAR_LEFT];
    rear_left_velocity_pub_.publish(cmd_msg);
    cmd_msg.data = steer_[REAR_RIGHT];
    rear_right_steering_pub_.publish(cmd_msg);
    cmd_msg.data = velocities_[REAR_RIGHT];
    rear_right_velocity_pub_.publish(cmd_msg);
  }

  void manualDrivingCB(const drive_ros_uavcan::phoenix_msgs__RemoteControlConstPtr &msg) {
    ROS_INFO_STREAM("Driving command vel "<<msg->velocity);
    steer_[FRONT_LEFT] = msg->steer_front;
    steer_[FRONT_RIGHT] = msg->steer_front;
    steer_[REAR_LEFT] = msg->steer_rear;
    steer_[REAR_RIGHT] = msg->steer_rear;

    std::fill(velocities_.begin(), velocities_.end(), msg->velocity);
    dynamicsControl();
  }

  void driveCommandCB(const drive_ros_uavcan::phoenix_msgs__NucDriveCommandConstPtr &msg) {
    steer_[FRONT_LEFT] = msg->phi_f;
    steer_[FRONT_RIGHT] = msg->phi_f;
    steer_[REAR_LEFT] = msg->phi_r;
    steer_[REAR_RIGHT] = msg->phi_r;

    std::fill(velocities_.begin(), velocities_.end(), msg->lin_vel*20);
    dynamicsControl();
  }

private:
  ros::Publisher front_left_steering_pub_;
  ros::Publisher front_left_velocity_pub_;
  ros::Publisher front_right_steering_pub_;
  ros::Publisher front_right_velocity_pub_;
  ros::Publisher rear_left_steering_pub_;
  ros::Publisher rear_left_velocity_pub_;
  ros::Publisher rear_right_steering_pub_;
  ros::Publisher rear_right_velocity_pub_;

  ros::Subscriber command_sub_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  bool manual_driving_;

  std::vector<float> steer_;
  std::vector<float> velocities_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_gazebo_body_control_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  GazeboRosBodyControl body_control(nh, pnh);

#ifdef DEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}

