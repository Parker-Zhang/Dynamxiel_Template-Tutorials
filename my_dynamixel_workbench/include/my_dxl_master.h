#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include<trajectory_generator.h>

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
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list_;

  trajectory_msgs::JointTrajectory *jnt_tra_msg_;
  JointTrajectory *jnt_tra_;
  std::vector<WayPoint> pre_goal_;
  trajectory_msgs::JointTrajectory *jnt_tra_test_;

  bool is_moving_ = false;

  // ROS Service Server
  // ros::ServiceServer changeGoalPositonSrv;


  //ROS public
  ros::Publisher dynamixel_state_list_pub_;

  //ROS subscriber
  ros::Subscriber trajectory_sub_;

  // timer period
  double write_period_;
  double read_period_ ;
  double pub_period_;

  // pid controller parameters
  int32_t * goal_pos;
  int32_t * init_pos;
  int32_t * pos_err ;
  int32_t * last_pos_err ;
  int32_t * pos_err_integral ;
  int32_t * pos_com_err_integral;
  int32_t * last_pos_com_err;
  int32_t * pos_com_err;
  int16_t * goal_cur;
  

  float p_gain = 0.5;
  float i_gain = 0.01;
  float d_gain = 0.05;
  int16_t limit_current = 50;

  
public:
  // Dynamixel Workbench Parameters
    DynamixelWorkbench *dxl_wb_;
    MyDynamixelController();
    bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
    bool getDynamixelsInfo(const std::string yaml_file);
    bool loadDynamixels(void);
    bool initDynamixels(void);
    bool initControlItems(void);
    bool initSDKHandlers(void);
    void initServer();
    void initSubscriber(void);
    void initPublisher(void);

  // callback
    // bool changePositionCallback(my_dynamixel_workbench::ChangeGoalPosition::Request &req
    //                               ,my_dynamixel_workbench::ChangeGoalPosition::Response &res);


    void trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);     
                            
  // ros timer callback 
    double getReadPeriod(){return read_period_;}
    double getWritePeriod(){return write_period_;}
    double getPublishPeriod(){return pub_period_;}
    void writeCallback(const ros::TimerEvent&);
    void readCallback(const ros::TimerEvent&);
    void publishCallback(const ros::TimerEvent&);

  // pid controller function
    bool setGoalPosition(int position);
    bool getGoalPosition(int &position);
    bool setPidGain(float p_gain,float i_gain,float d_gain);
    bool setLimitCurrent(int lim_cur);
    int pidController(int goal_position_ ,int id);
    void pidControllerInit();
  

  // load trajectory
    void initGoalPos();
    bool getPresentPosition(std::vector<std::string> dxl_name);

};