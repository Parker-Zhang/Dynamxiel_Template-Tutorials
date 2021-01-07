/**********************************************************************
RL_3DofSerialRobot.cpp
Author: Parker Zhang
Date: 2020.12.16
Description: This is a reinforcement test program in which we try several
RL Algorithms to control a 3Dof(3R) serial manilupator.
**********************************************************************/
#include<dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include<ros/ros.h>
#include<my_dxl_controller.h>

//#include <eigen3/unsupported/Eigen/KroneckerProduct>

using namespace std;

#define PI 3.1415926


MyDynamixelController::MyDynamixelController()
 :node_handle_(""),
  priv_node_handle_("~")
{
    dxl_wb_ = new DynamixelWorkbench;

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



// new API: based on official codes
// sync read
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


//sync write
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



