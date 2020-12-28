/**********************************************************************
Templete.cpp
Author: Parker Zhang
Date: 2020.12.16
Description: This is a templete introduce how to 
**********************************************************************/
#include<dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include<ros/ros.h>
#include<RL_3DofSerialRobot.h>

#include <iostream>
#include <fstream>
#include <sstream>

// #include <Eigen/Eigen>
//#include <eigen3/unsupported/Eigen/KroneckerProduct>

using namespace std;
using namespace Eigen;

#define PI 3.1415926
#define USE_OPERATOR_CONTROL

MyDynamixelController::MyDynamixelController()
 :node_handle_(""),
  priv_node_handle_("~")
{
    dxl_wb_ = new DynamixelWorkbench;
    jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
    jnt_tra_test_ = new trajectory_msgs::JointTrajectory;
    read_period_ = priv_node_handle_.param<double>("dxl_read_period", 0.005f);
    write_period_ = priv_node_handle_.param<double>("dxl_write_period", 0.010f);
    pub_period_ = priv_node_handle_.param<double>("publish_period", 0.05f);
    jnt_tra_ = new JointTrajectory;
    is_moving_ = false;
}

/******************************/
//          控制器初始化       
/******************************/
bool MyDynamixelController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(),baud_rate,&log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  return result;
}
bool MyDynamixelController::getDynamixelsInfo(const std::string yaml_file)
{
  //node可以理解为 树的根节点，这里什么时候分叉，需要看缩进符的结构
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (dynamixel == NULL)
    return false;
  //iterator 翻译为 迭代器，有时又称游标（cursor）是程序设计的软件设计模式，可在容器上遍访的接口
  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    //这里觉得是读到用户自定义舵机的别名
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      //把ID号赋值给舵机的别名
      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }
  return true;
}
//按照加载的配置文件，显示当前舵机在线的数量，名字，以及id号
bool MyDynamixelController::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {      
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}
//初始化舵机，注意写入的时候，舵机的力矩要torqueoff，写入完成，再torque on
bool MyDynamixelController::initDynamixels(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for (auto const& info:dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }

    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}
//这是初始化控制表，即获得舵机当前值以及目标值
bool MyDynamixelController::initControlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == NULL) return false;

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL)  goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == NULL)  return false;

  const ControlItem *goal_current = dxl_wb_->getItemInfo(it->second, "Goal_Current");
  if (goal_current == NULL) {
    ROS_INFO("goal_current NULL");
    return false;  }

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL) return false;

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL)  present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == NULL) return false;

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL)  present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  if (present_current == NULL) return false;

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;
  control_items_["Goal_Current"] = goal_current;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  return true;
}
bool MyDynamixelController::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Current"]->address, control_items_["Goal_Current"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  //采用通讯协议2.0版本
  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {  
    uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

    /* 
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */    
    // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
    uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length+2;

    result = dxl_wb_->addSyncReadHandler(start_address,
                                          read_length,
                                          &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
  }

  return result;
}
// subscriber 接受operator发布的轨迹
void MyDynamixelController::initSubscriber(){
  trajectory_sub_ = node_handle_.subscribe("joint_trajectory", 100, &MyDynamixelController::trajectoryMsgCallback, this);
}


// 获取当前的位置，给定舵机的名称，返回
bool MyDynamixelController::getPresentPosition(std::vector<std::string> dxl_name)
{
  bool result = false;
  const char* log = NULL;
  int32_t get_position[dxl_name.size()];  //生成给定名称数组的数组

  uint8_t id_array[dxl_name.size()];
  uint8_t id_array2[dxl_name.size()];
  uint8_t id_cnt = 0;

  for (auto const& dxl:dynamixel_)
  {
    id_array2[id_cnt++] = (uint8_t)dxl.second;  //得到对应名称对应的ID号
  }

  uint8_t index_array[dynamixel_.size()];
  uint8_t cnt = 0;

  id_cnt = 0;
  for (auto const& name:dxl_name)
  {

    id_array[id_cnt++] = (uint8_t)dynamixel_[name];
  }
  for (auto const& name:dxl_name)
  {
    uint8_t index = 0;
    for (;index<id_cnt;index++)
    {
      if(id_array2[index]==(uint8_t)dynamixel_[name])
      {
        break;
      }
    }
    index_array[cnt++]=index;
  }

  //读操作要求输入：handler id序列 先遍历以下是否包含所给ID的舵机
  result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                               id_array,
                               dxl_name.size(),
                               &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
  WayPoint wp;

  //输入：ID序列。舵机数量，读取位置的地址，关节长度，输出保存的位置，log
  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                    id_array,
                                    id_cnt,
                                    control_items_["Present_Position"]->address,
                                    control_items_["Present_Position"]->data_length,
                                    get_position,
                                    &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
  else
  {
    for(uint8_t index = 0; index < id_cnt; index++)
    {
      wp.position = dxl_wb_->convertValue2Radian(id_array[index], get_position[index]-init_pos[index_array[index]]+2048);
      pre_goal_.push_back(wp);
    }
  }
  return result;
}
// 接受到operator发布的轨迹后执行的回调函数
void MyDynamixelController::trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  ROS_INFO("receive msg!");
  uint8_t id_cnt = 0;
  bool result = false;
  WayPoint wp;
  if(is_moving_ == false)
  {
    jnt_tra_msg_->joint_names.clear();
    jnt_tra_msg_->points.clear();
    pre_goal_.clear();

    result = getPresentPosition(msg->joint_names);
    if (result == false)
      ROS_ERROR("Failed to get Present Position");

    for (auto const& joint:msg->joint_names)
    {
      ROS_INFO("'%s' is ready to move", joint.c_str());
      jnt_tra_msg_->joint_names.push_back(joint);
      id_cnt++;
    }

    // tightenRope();

    if (id_cnt != 0)
    {
      uint8_t cnt = 0;
      // 遍历轨迹上的点
      while(cnt < msg->points.size())
      {
        std::vector<WayPoint> goal;
        //遍历一个点上的所有舵机位置
        for (std::vector<int>::size_type id_num = 0; id_num < msg->points[cnt].positions.size(); id_num++)
        {
          wp.position = msg->points[cnt].positions.at(id_num);

          if (msg->points[cnt].velocities.size() != 0)  wp.velocity = msg->points[cnt].velocities.at(id_num);
          else wp.velocity = 0.0f;

          if (msg->points[cnt].accelerations.size() != 0)  wp.acceleration = msg->points[cnt].accelerations.at(id_num);
          else wp.acceleration = 0.0f;

          goal.push_back(wp);
        }
        // size的值 对应要控制的舵机的数量
        jnt_tra_->setJointNum((uint8_t)msg->points[cnt].positions.size());
        // 获取运动的时长，时长在msg中获取
        double move_time = 0.0f;
        if (cnt == 0) move_time = msg->points[cnt].time_from_start.toSec();
        else move_time = msg->points[cnt].time_from_start.toSec() - msg->points[cnt-1].time_from_start.toSec();
        // pre_goal 表示上一个终点的位置，生成点需要初始点和重点，然后在中间处根据写的频率进行插值
        jnt_tra_->init(move_time,
                        write_period_,
                        pre_goal_,
                        goal);

        std::vector<WayPoint> way_point;
        trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;
        // 从trajectory中得到目标按写频率分布的轨迹，存放到jnt_tra_point_msg中
        for (double index = 0.0; index < move_time; index = index + write_period_)
        {
          way_point = jnt_tra_->getJointWayPoint(index);

          for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
          {
            jnt_tra_point_msg.positions.push_back(way_point[id_num].position);
            jnt_tra_point_msg.velocities.push_back(way_point[id_num].velocity);
            jnt_tra_point_msg.accelerations.push_back(way_point[id_num].acceleration);
          }

          jnt_tra_msg_->points.push_back(jnt_tra_point_msg);
          jnt_tra_point_msg.positions.clear();
          jnt_tra_point_msg.velocities.clear();
          jnt_tra_point_msg.accelerations.clear();
        }

        pre_goal_ = goal;
        cnt++;
      } 

      ROS_INFO("Succeeded to get joint trajectory!");
      is_moving_ = true;
    }
    else
    {
      ROS_WARN("Please check joint_name");
    }
  }
  else
  {
    ROS_WARN("Dynamixel is moving");
  }

}
//define  publisher
void MyDynamixelController::initPublisher()
{
  dynamixel_state_list_pub_ = priv_node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
  // desired_tra_pub_ = priv_node_handle_.advertise<my_dynamixel_workbench_test::desired_trajectory>("desired_tra",100);
}
//自定义service callback
void MyDynamixelController::initServer()
{
  // 例如：需要自己写callback函数和定义srv
  // changeGoalPositonSrv = node_handle_.advertiseService("changePosition",&MyDynamixelController::changePositionCallback,this);
}
void MyDynamixelController::initGoalPos(){
  bool result = false;
  const char* log = NULL;

  dynamixel_workbench_msgs::DynamixelState  dynamixel_state[dynamixel_.size()];

  int32_t get_position[dynamixel_.size()];
  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  for (auto const& dxl:dynamixel_)
  {
    dynamixel_state[id_cnt].name = dxl.first;
    dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

  result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                              id_array,
                              dynamixel_.size(),
                              &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                id_cnt,
                                                control_items_["Present_Position"]->address,
                                                control_items_["Present_Position"]->data_length,
                                                get_position,
                                                &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  for(uint8_t index = 0; index < id_cnt; index++)
  {
    
    init_pos[index] = get_position[index];
    goal_pos[index] = get_position[index];
    ROS_INFO("id:%d present_position:%d  index:%d",id_array[index],get_position[index],index);
  }
}

//****************************//
//  callback 函数 开发者接口
//****************************//
//ros timer callback
#ifndef USE_OPERATOR_CONTROL
void MyDynamixelController::writeCallback(const ros::TimerEvent&){
//舵机的目标角度 goal_pos 单位为 unit

//反馈控制律 使用 Egien 库



}
#endif

#ifdef USE_OPERATOR_CONTROL
void MyDynamixelController::writeCallback(const ros::TimerEvent&){
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  int32_t dynamixel_current[dynamixel_.size()];
  int32_t dynamixel_position[dynamixel_.size()];//应该是舵机的目标点的位置

  static uint32_t point_cnt = 0;
  static uint32_t position_cnt = 0;

  if (is_moving_==true)
  {
    for (auto const& dxl:dynamixel_)
    {
      id_array[id_cnt++] = (uint8_t)dxl.second;
    }
    uint8_t index_array[dynamixel_.size()];
    uint8_t cnt = 0;
    for (auto const& joint:jnt_tra_msg_->joint_names)
    {
      uint8_t index = 0;
      for (;index<id_cnt;index++)
      {
        if(id_array[index]==(uint8_t)dynamixel_[joint])
        {
          break;
        }
      }
      index_array[cnt++]=index;
    }
    for (uint8_t index = 0; index < cnt; index++)
    {
      dynamixel_position[index] = dxl_wb_->convertRadian2Value(id_array[index], jnt_tra_msg_->points[point_cnt].positions.at(index));
      goal_pos[index_array[index]] = dynamixel_position[index]-2048 + init_pos[index_array[index]]; //为了保持序号的一致
      position_cnt++;
      
    }    
    //如果遍历了所有点的话，表明结束轨迹，不再运动
    //position_cnt++;
    //jnt_tra_msg_->points[point_cnt].positions.size() 表示第point_cnt个点，舵机的数量
    if (position_cnt >= jnt_tra_msg_->points[point_cnt].positions.size())
    {
      point_cnt++;
      position_cnt = 0;
      //ROS_INFO("the %d th point",point_cnt);
      // jnt_tra_msg_->points.size() 表示路径上点的总数
      if (point_cnt >= jnt_tra_msg_->points.size())
      {
        //ROS_INFO("totoal points size:%d",jnt_tra_msg_->points.size());
        is_moving_ = false;
        point_cnt = 0;
        position_cnt = 0;
        ROS_INFO("Complete Execution");
      }
    }
  }
  else{
      for (auto const& dxl:dynamixel_)
    {
      id_array[id_cnt++] = (uint8_t)dxl.second;
    }
  }

  // 调用自定义的控制器
  // for (uint8_t index = 0; index < id_cnt; index++)
  // {
  //   dynamixel_current[index] = pidController(goal_pos[index],id_array[index]);
  // }

  // 写入目标电流
  // result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, id_array, id_cnt, dynamixel_current, 1, &log);
  // 写入目标速度
  // result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, id_cnt, dynamixel_current, 1, &log);
  //写入目标位置
  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, id_cnt, goal_pos, 1, &log);

  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
}
#endif


void MyDynamixelController::readCallback(const ros::TimerEvent&){
  bool result = false;
  const char* log = NULL;

  dynamixel_workbench_msgs::DynamixelState  dynamixel_state[dynamixel_.size()];
  dynamixel_state_list_.dynamixel_state.clear();

  int32_t get_current[dynamixel_.size()];
  int32_t get_velocity[dynamixel_.size()];
  int32_t get_position[dynamixel_.size()];

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  for (auto const& dxl:dynamixel_)
  {
    dynamixel_state[id_cnt].name = dxl.first;
    dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

    id_array[id_cnt++] = (uint8_t)dxl.second;
  }
  

  result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                            id_array,
                            dynamixel_.size(),
                            &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                id_cnt,
                                                control_items_["Present_Current"]->address,
                                                control_items_["Present_Current"]->data_length,
                                                get_current,
                                                &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                id_cnt,
                                                control_items_["Present_Velocity"]->address,
                                                control_items_["Present_Velocity"]->data_length,
                                                get_velocity,
                                                &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                id_cnt,
                                                control_items_["Present_Position"]->address,
                                                control_items_["Present_Position"]->data_length,
                                                get_position,
                                                &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  for(uint8_t index = 0; index < id_cnt; index++)
  {
    dynamixel_state[index].present_current = get_current[index];
    dynamixel_state[index].present_velocity = get_velocity[index];
    dynamixel_state[index].present_position = get_position[index];
    dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index]);
  }
  
}

void MyDynamixelController::publishCallback(const ros::TimerEvent&)
{
  // desired_tra_pub_.publish(d_tra); //发布期望轨迹
  dynamixel_state_list_pub_.publish(dynamixel_state_list_);
}

/******************************
//  控制器定义 以pid控制器为例
******************************/
// pid controller function
void MyDynamixelController::pidControllerInit(){
  pos_err = new int[dynamixel_.size()];
  last_pos_err = new int[dynamixel_.size()];
  pos_err_integral = new int[dynamixel_.size()];
  goal_pos = new int[dynamixel_.size()];
  init_pos = new int[dynamixel_.size()];
  goal_cur = new int16_t[dynamixel_.size()];
  for (int i=0;i<dynamixel_.size();i++)
  {
    pos_err[i] = 0 ;
    last_pos_err[i] = 0 ;
    pos_err_integral[i] = 0;

    // 这里初始化回零了，应该以读出来的位置作为初始值
    goal_pos[i] = 0;
  }
  ROS_INFO("P Gain: %f I Gain:%f  D Gain: %f",p_gain,i_gain,d_gain);
}
bool MyDynamixelController::setPidGain(float p,float i,float d){
  p_gain = p;
  i_gain = i;
  d_gain = d;
  return true;
}
bool MyDynamixelController::setLimitCurrent(int lim_cur)
{
  limit_current = lim_cur;
  return true;
}

// new API
// 同时读函数
void MyDynamixelController::syncReadData(uint8_t *id_array,uint8_t id_cnt, Eigen::MatrixXd & state,Eigen::MatrixXd & current)
{
  bool result = false;
  const char* log = NULL;
  int32_t get_current[id_cnt];
  int32_t get_velocity[id_cnt];
  int32_t get_position[id_cnt];

  Eigen::MatrixXd state_(2,id_cnt);
  Eigen::MatrixXd current_(1,id_cnt);

  result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                          id_array,
                          id_cnt,
                          &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                id_cnt,
                                                control_items_["Present_Current"]->address,
                                                control_items_["Present_Current"]->data_length,
                                                get_current,
                                                &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                id_cnt,
                                                control_items_["Present_Velocity"]->address,
                                                control_items_["Present_Velocity"]->data_length,
                                                get_velocity,
                                                &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                id_cnt,
                                                control_items_["Present_Position"]->address,
                                                control_items_["Present_Position"]->data_length,
                                                get_position,
                                                &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
  
  for(uint8_t index = 0; index < id_cnt; index++)
  {
    state_(0,index) = get_position[index]*0.088/180*PI;
    state_(1,index) = get_velocity[index]*0.229*2*PI/60;
    current_(0,index) = (int16_t)get_current[index]*2.69;
    // std::cout<< "ID: "<< id_array[index] <<"current:"<< int16_t(get_current[index])<<endl;
    // std::cout<< "ID: "<< id_array[index] <<"velocity:"<< get_velocity[index]<<endl;
    // std::cout<< "ID: "<< id_array[index] <<"position:"<< get_position[index]<<endl;
  }
  state = state_;
  current = current_;

  // cout<<"state:\n"<<state<<endl;
}

void MyDynamixelController::torqueOffALL()
{
    for (auto const& dxl:dynamixel_)
    {
       dxl_wb_->torqueOff((uint8_t)dxl.second);
    }
}

void MyDynamixelController::torqueOnALL()
{
    for (auto const& dxl:dynamixel_)
    {
       dxl_wb_->torqueOn((uint8_t)dxl.second);
    }
}


//同时写函数
void MyDynamixelController::syncWriteData(int control_mode , uint8_t *id_array,uint8_t id_cnt,int32_t * goal)
{
  bool result = false;
  const char* log = NULL;
  int32_t * goal_ = goal;

  switch(control_mode)
  {
    case SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY:
      result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, id_cnt, goal_, 1, &log);
      break;
    case SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT:
      result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, id_array, id_cnt, goal_, 1, &log);
      break;
    case SYNC_WRITE_HANDLER_FOR_GOAL_POSITION:
      result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, id_cnt, goal_, 1, &log);
      // cout<<"goal:"<<goal_[0]<<goal_[1]<<endl;
      break;
    default: ROS_ERROR("ERROR! NO SUCH CONTROL MODE");
  }
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
}

//定义控制器，反馈控制律 K，输入控制律 
int MyDynamixelController::pidController(int goal_position_ , int id)
{
  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;
  int index = 0;
  for (auto const& dxl:dynamixel_)
  {
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }
  for(int i=0;i<id_cnt;i++)
  {
    if(id_array[i]==id)
    {
      index = i;
    }
  }
  int32_t present_position = dynamixel_state_list_.dynamixel_state[index].present_position;

  present_position = present_position - init_pos[index];  //采用增量式的控制方法，这一步的目的是将基准设为开机时的舵机状态

  int16_t goal_current = 0;
  last_pos_err[index] = pos_err[index];
  pos_err[index] = goal_position_ - present_position ; 
  pos_err_integral[index] += pos_err[index];
  goal_current = (int16_t) (p_gain * pos_err[index]+ i_gain * pos_err_integral[index] +d_gain * (pos_err[index]-last_pos_err[index])); 

  if(goal_current>limit_current)
  {
    goal_current = limit_current;
  }
  if(goal_current<-limit_current)
  {
    goal_current = -limit_current;
  }

  return goal_current;
}


// #define TRACK_TEST
#define Debug


// main function 要不要定时器？ 控制周期需要改动 
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"my_dxl_master");
    ros::NodeHandle node_handle;

    std::string port_name = "/dev/ttyUSB0";
    uint32_t baud_rate = 57600;

    if (argc < 2)
    {
        ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
        return 0;
    }
    else
    {
        port_name = argv[1];
        baud_rate = atoi(argv[2]);
    }

    MyDynamixelController dynamixel_controller;

    bool result = false;

    std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

// 把这个封装成一个函数；
    result = dynamixel_controller.initWorkbench(port_name, baud_rate);
    if (result == false)
    {
        ROS_ERROR("Please check USB port name");
        return 0;
    }
    
    result = dynamixel_controller.getDynamixelsInfo(yaml_file);
    if (result == false)
    {
        ROS_ERROR("Please check YAML file");
        return 0;
    }

    result = dynamixel_controller.loadDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check Dynamixel ID or BaudRate");
        return 0;
    }

    result = dynamixel_controller.initDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
        return 0;
    }

    result = dynamixel_controller.initControlItems();
    if (result == false)
    {
        ROS_ERROR("Please check control items");
        return 0;
    }

    result = dynamixel_controller.initSDKHandlers();
    if (result == false)
    {
        ROS_ERROR("Failed to set Dynamixel SDK Handler");
        return 0;
    }
    
    
    dynamixel_controller.pidControllerInit();
    ROS_INFO("Welcome my dynamixel workbench!");

    std::string stateRecordPath = node_handle.param<std::string>("stateRecordPath", "");
    std::ofstream StatesRecoder(stateRecordPath);
     //state: motor 1: position velocity current goal_position input motor2:repeate
    //  StatesRecoder<<"motor 1\t\t\t\t motor 2\t\t\t\t"<<endl;
     StatesRecoder<<"radian(rad)\t velocity(rad/s) \t current(mA) \t goal_radian(rad) \t input(rad/s) \t radian(rad)\t velocity(rad/s) \t current(mA) \t goal_radian(rad) \t input(rad/s)"<<endl;

    DynamixelWorkbench * dxl_wb = dynamixel_controller.dxl_wb_; //便于引用
     
    //ros service init
    dynamixel_controller.initServer();

    // /*
    dynamixel_controller.initSubscriber();
    dynamixel_controller.initPublisher();
    dynamixel_controller.initGoalPos();


    // 多臂连动
    const int motorNum = 2;
    uint8_t id_cnt = 2;
    uint8_t id_array[motorNum] = {1,2};    //预定义的舵机id

    //学习参数
    int eps_max  = 10;
    int step_max = 105;		int step = 0;

    //状态参数
    Eigen::MatrixXd state_goal(2,motorNum);   state_goal = Eigen::MatrixXd::Zero(2,motorNum);
    Eigen::MatrixXd state(2,motorNum);        state = Eigen::MatrixXd::Zero(2,motorNum);
    Eigen::MatrixXd state_err(2,motorNum);    state_err = Eigen::MatrixXd::Zero(2,motorNum);
    Eigen::MatrixXd current(1,motorNum);      current = Eigen::MatrixXd::Zero(1,motorNum);


    double goal_pos_ID1 = 800 *0.088/180*PI; 
    double goal_pos_ID2 = 2000 *0.088/180*PI;
    state_goal(0,0) = goal_pos_ID1; state_goal(0,1) = goal_pos_ID2;

    #ifdef TRACK_TEST
    Eigen::MatrixXd state_goal_offset(2,motorNum);    state_goal_offset = Eigen::MatrixXd::Zero(2,motorNum);
    Eigen::MatrixXd state_goal_am(2,motorNum);        state_goal_am = Eigen::MatrixXd::Zero(2,motorNum);
    Eigen::MatrixXd tra_frequence(1,motorNum);        tra_frequence = Eigen::MatrixXd::Zero(1,motorNum);

    state_goal_offset(0,0) = state_goal(0,0);         state_goal_offset(0,1) = state_goal(0,1);
    state_goal_am(0,0) = 200 *0.088/180*PI;           
    state_goal_am(0,1) = 400 *0.088/180*PI;
    tra_frequence(0,0) = 1.5;
    tra_frequence(0,1) = 1.5;
    #endif

    //控制参数
    double K1 = -35.5301 ,K2 = -6.6606;
    Eigen::MatrixXd K(1,2); K(0,0)=K1; K(0,1)=K2;
    int dt = 100*1000;

    double u1,u2;
    Eigen::MatrixXd uMat(1,motorNum) ;  uMat= Eigen::MatrixXd::Zero(1,motorNum); 
    
    int32_t	 u[motorNum] = {0};
    int32_t u_limit = 300;

    
    // 测试一次性读取舵机的位置、速度、电流信息并显示
    // Input： ID的序列 读取状态保存到的地址
    
    //测试同时写函数
    //Input：handle id_array id_cnt goal 
    while(step<(step_max - 5)&&ros::ok())
    {
      step++;
      dynamixel_controller.syncReadData(id_array,id_cnt,state,current);
      state_err = state - state_goal;
      uMat = K * state_err;
      for (int i=0;i<motorNum;i++)
      {
        u[i] = int32_t(uMat(0,i));
        if(abs(u[i])>u_limit)
        {
          u[i] = u[i]/abs(u[i])*u_limit;
        }

      #ifdef TRACK_TEST
         state_goal(0,i) = state_goal_offset(0,i) 
                          + state_goal_am(0,i)*sin(2*PI*tra_frequence(0,i)*step/(step_max-5)); 
      #endif
      }


      #ifdef Debug
      cout<<"state_err\n"<<state_err<<endl;
      cout<<"output ID 1 :"<<u[0]<<endl;
      cout<<"output ID 2 :"<<u[1]<<endl;
      #endif


      dynamixel_controller.syncWriteData(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY,
                          id_array,id_cnt,u);
      // dynamixel_controller.syncWriteData(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT,
      //               id_array,id_cnt,u);

      //record data
      //state: motor 1: position velocity current goal_position input motor2:repeate
      StatesRecoder<<state(0,0)<<"\t"<<state(1,0)<<"\t"<<current(0,0)<<"\t"<<state_goal(0,0)<<"\t"<<u[0]*0.229*2*PI/60<<"\t"
                    <<state(0,1)<<"\t"<<state(1,1)<<"\t"<<current(0,1)<<"\t"<<state_goal(0,1)<<"\t"<<u[1]*0.229*2*PI/60<<"\t"<<endl;

      usleep(dt);
    }
    u[0] = 0;
    u[1] = 0;
    dynamixel_controller.syncWriteData(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY,
                    id_array,id_cnt,u);

    // dynamixel_controller.syncWriteData(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT,
    //               id_array,id_cnt,u);
    // dynamixel_controller.torqueOffALL();
    
    StatesRecoder.close();
    ROS_INFO("******      executing terminated      ******");
    //测试文件写入
    // StatesRecoder<<"this is test data!\n"<<state(0,0)<<endl;
    
    
    // ros::Timer read_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getReadPeriod()),
    //                                        &MyDynamixelController::readCallback, &dynamixel_controller);
    // ros::Timer write_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getWritePeriod()),
    //                                        &MyDynamixelController::writeCallback, &dynamixel_controller);
    // ros::Timer publish_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()), 
    //                                       &MyDynamixelController::publishCallback, &dynamixel_controller);

    //ros::spin();
}
