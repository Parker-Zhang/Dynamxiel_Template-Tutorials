#include<my_operator.h>
MyOperator::MyOperator()
:node_handle_(""),
priv_node_handle_("~")
{
    std::string yaml_file = node_handle_.param<std::string>("trajectory_info", "");
    jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
    bool result = getTrajectoryInfo(yaml_file, jnt_tra_msg_);
    if (result == false)
    {
        ROS_ERROR("Please check YAML file");
        exit(0);
    }
    joint_trajectory_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);
    
    
}
bool MyOperator::getTrajectoryInfo(const std::string yaml_file,
                     trajectory_msgs::JointTrajectory *jnt_tra_msg)
{
    YAML::Node file;
    file = YAML::LoadFile(yaml_file.c_str());
    if (file == NULL)
        return false;
    //
    YAML::Node joint = file["joint"];
    uint16_t joint_size = joint["names"].size();

    for (uint16_t index = 0; index < joint_size; index++)
    {
        std::string joint_name = joint["names"][index].as<std::string>();
        jnt_tra_msg->joint_names.push_back(joint["names"][index].as<std::string>());
    }
    //
    YAML::Node trajectory = file["trajectory"];
    uint16_t trajectory_size = trajectory["index"].size();
    for (uint16_t index = 0;index<trajectory_size;index++)
    {
        trajectory_msgs::JointTrajectoryPoint jnt_tra_point;
        std::string index_ = trajectory["index"][index].as<std::string>();
        YAML::Node wp_num = trajectory[index_];
        if (joint_size != wp_num["pos"].size())
        {
            ROS_ERROR("Please check way point pos size. It must be equal to joint size");
            return 0;
        }
        for (uint16_t size=0;size<wp_num["pos"].size();size++)
        {
            jnt_tra_point.positions.push_back(wp_num["pos"][size].as<double>());
            ROS_INFO("NO.way point : %s, position : %f", index_.c_str(), wp_num["pos"][size].as<double>());
        }
        if (wp_num["vel"] != NULL)
        {
            // 速度的size也要和pos一样
            for (uint16_t size=0;size<wp_num["vel"].size();size++)
            {
                jnt_tra_point.velocities.push_back(wp_num["vel"][size].as<double>());
                ROS_INFO("NO.way point : %s, velocity : %f", index_.c_str(), wp_num["vel"][size].as<double>());
            }
        }
        if (wp_num["acc"] != NULL)
        {
            //加速度的size也要和pos一样
            for (uint16_t size=0;size<wp_num["acc"].size();size++)
            {
                jnt_tra_point.accelerations.push_back(wp_num["acc"][size].as<double>());
                ROS_INFO("NO.way point : %s, acceleration : %f", index_.c_str(), wp_num["acc"][size].as<double>());
            }
        }
        if (wp_num["time_from_start"] == NULL)
        {
            ROS_ERROR("Please check time_from_start. It must be set time_from_start each waypoint");
            return 0;
        }
        jnt_tra_point.time_from_start.fromSec(wp_num["time_from_start"].as<double>());

        ROS_INFO("time_from_start : %f", wp_num["time_from_start"].as<double>());

        jnt_tra_msg->points.push_back(jnt_tra_point);     
    }
    return true;
}
void MyOperator::publishTra(){
    joint_trajectory_pub_.publish(*jnt_tra_msg_);
    ROS_INFO("Success to publish joint trajectory");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_operator");
    MyOperator myOperator;
    sleep(1);
    myOperator.publishTra();
    ros::spin();
    return 0;
}