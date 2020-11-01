//////////////////////////    headfiles    //////////////////////////
//math
#include <math.h>
#include <Eigen/Eigen>
#include <eigen3/unsupported/Eigen/KroneckerProduct>

#include <cstdlib>

using namespace Eigen;
//ros
#include<dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include<ros/ros.h>
#include<my_dynamixel_workbench_test/dxl_state.h>
//iostream
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
//predefine
#define BAUDRATE 57600
#define ID 2
#define random(x) (rand()%x)
#define PI 3.1415926



//////////////////////////    begin    //////////////////////////
int main(int argc,char **argv)
{
	//////////////////////////	  parameters    //////////////////////////
	int eps_max  = 1;
	int step_max = 104;		int step = 0;
	int update_step =  2;
	double gam = 0.85;
	Eigen::MatrixXd Qx(2, 2);
	Qx(0,0) = 1000;		Qx(1,0) = 0;	Qx(0,1) = 0;
	Qx(1,1) = 10;
	double Qu = 1;
	int       tar_position = 4600/2;
	double 	  tar_pos = tar_position*0.088/180*PI;
	int noise_trial = 20;
	bool noise_change = true;
	bool K_limit = false;
	bool is_current_control = false;
	bool position_limit_flag = true;
	bool AC_control = true;
	bool index_jsum_change = false;
	int P_scale = 1000;
	int ini_pos = 1600;
	int dt = 300*1000;
	float K2 = -2.656;	float K1 = -20.01;
	
	int32_t 		limit_current = 450;	//2.69 [mA]

	std::ofstream CoeffRecoder("Z_CoeffRecoder.txt");
    CoeffRecoder <<"Qx:  " << std::endl << Qx <<endl<<"K :"<<endl<< K1<<" "<<K2 <<endl
			<<"	Qu:	"<<Qu<<"  gamma: "<< gam <<"   P  "<<P_scale<<"	ini_pos:"<<ini_pos
			<<"	dt"<<dt/1000<<"	limit_current:"<<limit_current<<"	nose:"<<noise_trial<<"  "<<noise_change<<endl
			<<"eps_max= "<<eps_max<<"	step_max:"<<step_max<<"  update_step:"<<update_step<<"	is_current_control"<<is_current_control<<"  AC_control:"<<AC_control<<endl
			<<"  index_jsum_change:"<<index_jsum_change<<endl;  

	//////////////////////////	  on-offs    //////////////////////////
	bool check = false;


	//////////////////////////	  state variable    //////////////////////////
	Eigen::MatrixXd state(2,1) ;	Eigen::MatrixXd statePre(2,1) ;	
	Eigen::MatrixXd dstate(2,1) ;
	double f;						double u;			Eigen::MatrixXd uMat(1, 1);	
	Eigen::MatrixXd K(1, 2);		Eigen::MatrixXd Kac(1, 2);	 
	Eigen::MatrixXd P(6, 6);		P = MatrixXd::Identity(6,6)*P_scale;
	Eigen::MatrixXd theta(6, 1);    theta = MatrixXd::Zero(6,1);
	Eigen::MatrixXd phi_all(9, 1);   phi_all = MatrixXd::Zero(9,1);
	Eigen::MatrixXd gradient(6, 1);
	double cost;					Eigen::MatrixXd costMat(1, 1);	
	Eigen::MatrixXd xu(3,1);		Eigen::MatrixXd xuPre(3,1) ;
	Eigen::MatrixXd phi(6,1);
	Eigen::MatrixXd H(3,3);
	Eigen::MatrixXd H21(1,2);		double H22;
	Eigen::MatrixXd H44(3, 3);
	double index_Jum;				Eigen::MatrixXd Jsum(1, 2);
	Eigen::MatrixXd eye(1,1);		eye(0,0) = 1;
	Eigen::MatrixXd temp(1,1);		temp(0,0) = 1;
	
	//////////// 给定参数
	Eigen::MatrixXd state_est(2,1) ;
	Eigen::MatrixXd A_sys(2,2) ;
	Eigen::MatrixXd B_sys(2,1) ;
	Eigen::MatrixXd C_sys(1,2) ;
	Eigen::MatrixXd L_sys(2,1) ;
	
	K(0,0)=K1; K(0,1)=K2; 			Kac(0,0)=K1; Kac(0,1)=K2;
	// A_sys(0,0)=0.9458; A_sys(0,1)=2.4837; A_sys(1,0)=0.0; A_sys(1,1)=-0.33;
	A_sys(0,0)=0.9042 ; A_sys(0,1)=0.1519; A_sys(1,0)=0.0609; A_sys(1,1)=0.0807;
    B_sys(0,0)=0; B_sys(1,0)=0.0240;	//dt/1000000;
	C_sys(0,0)=1; C_sys(0,1)=0.0;
	L_sys(0,0)=2.6614; L_sys(1,0)=-29.8183;
	

  ros::init(argc,argv,"ACimpedent");
  ros::NodeHandle n;
  ros::Publisher  dxl_state_pub = n.advertise<my_dynamixel_workbench_test::dxl_state>("dxl_state_topic",100);  //定义舵机状态发布器
  my_dynamixel_workbench_test::dxl_state msg;
  


  //////////////////////////    舵机数据定义
  int32_t 		position_data = 0;
  double 	position = 0.0;
  int32_t 	velocity_data = 0;
  double 	velocity = 0.0;
  int32_t	current_data = 0;
  double 	current = 0.0;
  int32_t	 	goal_current;


 
  //////////////////////////    连接舵机，设置电流控制模式
  DynamixelWorkbench dxl_wb; 
  dxl_wb.begin("/dev/ttyUSB0",BAUDRATE);
  dxl_wb.ping(ID);
  dxl_wb.ledOff(ID);
  ROS_INFO("Welcome my dynamixel workbench!");
  dxl_wb.setCurrentControlMode(ID);
  dxl_wb.torqueOn(ID);
  dxl_wb.torqueOn(1);
  dxl_wb.setPositionControlMode(1);
  
  //////////////////////////    舵机初始化
  

  dxl_wb.itemRead(ID,"Present_Position",&position_data);
	// position = dxl_wb.convertValue2Position(ID,position_data);
	position = position_data*0.088/180*PI;
  dxl_wb.itemRead(ID,"Present_Velocity",&velocity_data);
	// velocity = dxl_wb.convertValue2Velocity(ID,velocity_data);
	velocity = velocity_data*0.229*2*PI/60;
  dxl_wb.itemRead(ID,"Present_Current",&current_data);
	// current  = dxl_wb.convertValue2Current(ID,current_data);
	current	 = current_data*2.69;   // mA
  
  //////////////////////////	定义信息流
  // 在home下创建文件夹 ACimpedent
  std::ofstream StatesRecoder("Z_statusRecoder.txt");
  std::ofstream KRecoder("Z_kRecoder.txt");
  std::ofstream JsumRecoder("Z_JsumRecoder.txt");
  
  //////////////////////////	学习过程     //////////////////////////
  for (int eps = 0; eps < eps_max; eps++) 
	{		// episode
	 ROS_INFO("******      episode begin      ******");
	 //////////////////////////    找到起始位置
	 ROS_INFO("******      go to init pos      ******");
	 dxl_wb.torqueOff(ID);
	 dxl_wb.setPositionControlMode(ID);
	 dxl_wb.torqueOn(ID);
	 dxl_wb.itemWrite(ID,"Goal_Position",ini_pos);
	 sleep(2);
	 dxl_wb.itemRead(ID,"Present_Position",&position_data);

	 usleep(2000 * 1000);  	 
	 ROS_INFO("******      reach init pos      ******");
	
	
	 //////////////////////////	   学习开始
	 //修改控制模型并获得当前状态
	 dxl_wb.torqueOff(ID);
	 usleep(10);
	//  dxl_wb.setVelocityControlMode(ID);
	if (is_current_control)
	{
		dxl_wb.setCurrentControlMode(ID);
	}
	else
	{
		dxl_wb.setVelocityControlMode(ID);
	}
	 
	 usleep(10);
	 dxl_wb.torqueOn(ID);
 	 
	 dxl_wb.itemRead(ID,"Present_Position",&position_data);
	 position = position_data*0.088/180*PI;
	 dxl_wb.itemRead(ID,"Present_Velocity",&velocity_data);
	 velocity = velocity_data*0.229*2*PI/60;
	 dxl_wb.itemRead(ID,"Present_Current" ,&current_data);
	 current	 = current_data*2.69;   // mA
	 
	 StatesRecoder << position_data << ";   " << velocity_data << ";   " << int16_t(current_data) << ";   " <<0<<std::endl;
	 KRecoder      << K(0,0) 		   << ";   " << K(0,1) 		 << ";   " << std::endl;
	 JsumRecoder <<	0	<<";   " <<	0	<<";   " << std::endl;
	 
	 //


	 // 获得起始状态，通过除以一个系数使得系统计算值不要太大，通过减去tar_position将目标位置调节至零位
     state(0,0) = position-tar_pos;	 state(1,0) = velocity;
	 state_est(0,0) = 0;	 state_est(1,0) = 0;

	 ROS_INFO("******      start learning   episode %d   ******",eps);


	 step = 0;			check = true;
	 P =MatrixXd::Identity(6,6)*P_scale;
	 while (step < (step_max - 5) && ros::ok() && check ) 
		{	//   step
		
		step++;
		 //////////////////////////    计算输出并发送给舵机
		 
		//uMat = K*state_est;
		uMat = K*state;
		
		if(uMat(0,0)>(limit_current-noise_trial))
		{
			uMat(0,0)= (limit_current-noise_trial);
		}
		if(uMat(0,0)<-(limit_current-noise_trial))
		{
			uMat(0,0)= -(limit_current-noise_trial);
		}
		u        = uMat(0,0);

		
		//////////////////////////   观测系统，用观测器观测系统状态，已知position，求系统状态
		temp(0,0)=state(0,0);		
		state_est=A_sys*state_est+B_sys*int32_t(u)+L_sys*(temp-C_sys*state_est);
		
		if(is_current_control)
		{
			goal_current = int32_t(u);
			ROS_INFO("Goal_Current:%d",goal_current);
			dxl_wb.itemWrite(ID,"Goal_Current",goal_current);
		}
		else
		{
			goal_current = int32_t(u);
			ROS_INFO("Goal_Velocityt:%d",goal_current);
			dxl_wb.itemWrite(ID,"Goal_Velocity",goal_current);			
		}
		// dxl_wb.itemWrite(ID,"Goal_Velocity",goal_current);
		usleep(dt);
		 
		 //////////////////////////	  更新状态
		 dxl_wb.itemRead(ID,"Present_Position",&position_data);
		 position = position_data*0.088/180*PI;
		 dxl_wb.itemRead(ID,"Present_Velocity",&velocity_data);
		 velocity = velocity_data*0.229*2*PI/60;
		 dxl_wb.itemRead(ID,"Present_Current" ,&current_data);
		 current	 = current_data*2.69;   // mA
		 cout<< "current_data:" << int16_t(current_data);
		 f = 0;        				//f =  externalF();
		 // 通过除以一个系数使得系统计算值不要太大，通过减去tar_position将目标位置调节至零位
		 state(0,0) = position - tar_pos;	
		 state(1,0) = velocity;
		 
		 //////////////////////////	  检查位置是否超限,未超限则进行调整
		 if (position_limit_flag)
		 {
			if ( position_data > 4000)		{check = 0;}
			if ( position_data < 100)		{check = 0;}
			ROS_INFO("check: %d ",check);
		 }
		 
		
		 //////////////////////////	  更新agent

		temp(0,0) = (K*state)(0,0);
		if(temp(0,0)>limit_current)
		{
			temp(0,0)=limit_current;
		}
		if(temp(0,0)<-limit_current)
		{
			temp(0,0)=-limit_current;
		}

		 //////////////////////////	数据显示和记录
		//  std::cout << "Kac    :   " << Kac(0,0) 		  << " " << Kac(0,1)  		 << " " << std::endl;
		 std::cout << "K    :   " << K  <<endl;

         // save data state
        //  StatesRecoder << position_data << ";   " << velocity_data << ";   " << goal_current << ";   " <<std::endl;
	     StatesRecoder << position_data << ";   " << velocity_data << ";   " << state_est(0,0) << ";   " <<state_est(1,0)<<std::endl;
		 KRecoder      << K(0,0) 		   << ";   " << K(0,1) 		 << ";   " << std::endl;
		 JsumRecoder <<	u	<<";   " <<	uMat(0,0)	<<";   " << std::endl;
		//  TestRecoder << u << ";  " << position_data <<";  " << velocity_data<< std::endl; 
		
		}
	

	 dxl_wb.itemWrite(ID,"Goal_Current",0);	// 关闭电流，保证安全	
	 ROS_INFO("******      episode end      ******");
	 usleep(1000 * 1000);  // sleep for 6 ms
	 
	}
	
  ROS_INFO("******      learning terminated      ******");
}
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  















