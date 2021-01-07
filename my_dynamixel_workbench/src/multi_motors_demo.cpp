#include <my_dxl_controller.h>
#include<eigen3/Eigen/Eigen>
#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace Eigen;


#define PI 3.1415926
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
    
    
    ROS_INFO("Welcome my dynamixel workbench!");

    std::string stateRecordPath = node_handle.param<std::string>("stateRecordPath", "");
    std::ofstream StatesRecoder(stateRecordPath);
     StatesRecoder<<"radian(rad)\t velocity(rad/s) \t current(mA) \t goal_radian(rad) \t input(rad/s) \t radian(rad)\t velocity(rad/s) \t current(mA) \t goal_radian(rad) \t input(rad/s)"<<endl;

    DynamixelWorkbench * dxl_wb = dynamixel_controller.dxl_wb_; //便于引用
     


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
