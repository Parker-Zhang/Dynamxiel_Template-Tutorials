#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include<eigen3/Eigen/Eigen>
using namespace Eigen;
using namespace std;

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT  2
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0


typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class MyDynamixelController
{
private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;      

  std::map<std::string, uint32_t> dynamixel_;

  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list_;

  // ROS Service Server

  //ROS public
  ros::Publisher dynamixel_state_list_pub_;

  //ROS subscriber
  ros::Subscriber trajectory_sub_;

  // timer period
  double write_period_;
  double read_period_ ;
  double pub_period_;

  
public:
  // Dynamixel Workbench Parameters
    DynamixelWorkbench *dxl_wb_;
      std::map<std::string, const ControlItem*> control_items_;
    MyDynamixelController();
    bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
    bool getDynamixelsInfo(const std::string yaml_file);
    bool loadDynamixels(void);
    bool initDynamixels(void);
    bool initControlItems(void);
    bool initSDKHandlers(void);
 

    void syncReadData(uint8_t *id_array,uint8_t id_cnt, Eigen::MatrixXd & state,Eigen::MatrixXd & current);
    void syncWriteData(int control_mode ,uint8_t *id_array,uint8_t id_cnt,int32_t * goal);
    void torqueOffALL(void);
    void torqueOnALL(void);
                       
};