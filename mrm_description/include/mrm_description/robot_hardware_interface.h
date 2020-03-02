#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <rospy_tutorials/Floats.h>
#include <mrm_description/Floats_array.h>
#include <angles/angles.h>
#include <sensor_msgs/JointState.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot(ros::NodeHandle& nh);
  void init();
  void update(const ros::TimerEvent& e);
  void read(const sensor_msgs::JointState::ConstPtr& msg);
  void write(ros::Duration elapsed_time);
  ros::Publisher pub;
  ros::ServiceClient client;
  rospy_tutorials::Floats joints_pub;
  mrm_description::Floats_array joint_read;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;

  int num_joints_;
  std::string joint_names_[5]; 
  double cmd[5];
  double pos[5];
  double vel[5];
  double eff[5];

  ros::NodeHandle nh_;
  ros::Timer non_realtime_loop_;
  ros::Duration elapsed_time_;
  double loop_hz_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};