#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class MyOperator{
private:
// ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;
// ROS Topic Publisher
  ros::Publisher joint_trajectory_pub_;

  trajectory_msgs::JointTrajectory *jnt_tra_msg_;
public:
    MyOperator();
    bool getTrajectoryInfo(const std::string yaml_file, trajectory_msgs::JointTrajectory *jnt_tra_msg);
    void publishTra();
};

