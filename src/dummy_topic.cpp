#include "local_path_planner/dummy_topic.h"

Dummy::Dummy():private_nh("~")
{
    private_nh.param("DT",DT,{0.05});
    private_nh.param("ROBOT_FRAME",ROBOT_FRAME,{"base_link"});
    private_nh.param("MAP_FRAME",MAP_FRAME,{"map"});

    pub_estimated_pose=nh.advertise<geometry_msgs::PoseStamped>("dummy_estimated_pose",1);
    pub_local_goal=nh.advertise<geometry_msgs::PoseStamped>("dummy_local_goal",1);
    pub_odometry=nh.advertise<nav_msgs::Odometry>("dummy_odometry",1);

    sub_cmd_vel=nh.subscribe("/roomba/control",100,&Dummy::cmd_vel_callback,this);
}

void Dummy::cmd_vel_callback(const roomba_500driver_meiji::RoombaCtrl::ConstPtr& msg)
{
    roomba_500driver_meiji::RoombaCtrl roomba_ctrl=*msg;
}

void Dummy::dummy_estimated_pose()
{
    geometry_msgs::PoseStamped estimated_pose;

    estimated_pose.header.frame_id=MAP_FRAME;
    estimated_pose.header.stamp=ros::Time::now();

    estimated_pose.pose.position.x=1.0;
    estimated_pose.pose.position.y=2.0;
    estimated_pose.pose.position.z=0.0;
    estimated_pose.pose.orientation.x=0.0;
    estimated_pose.pose.orientation.y=0.0;
    estimated_pose.pose.orientation.z=0.0;
    estimated_pose.pose.orientation.w=1.0;

    pub_estimated_pose.publish(estimated_pose);
}

void Dummy::dummy_local_goal()
{
    geometry_msgs::PoseStamped local_goal;

    local_goal.header.frame_id=MAP_FRAME;
    local_goal.header.stamp=ros::Time::now();

    local_goal.pose.position.x=10.0;
    local_goal.pose.position.y=20.0;
    local_goal.pose.position.z=0.0;
    local_goal.pose.orientation.x=0.0;
    local_goal.pose.orientation.y=0.0;
    local_goal.pose.orientation.z=0.0;
    local_goal.pose.orientation.w=1.0;

    pub_local_goal.publish(local_goal);
}

void Dummy::dummy_odometry()
{
}

void Dummy::process()
{
    ros::Rate loop_rate(1/DT);
    while(ros::ok()){
        dummy_estimated_pose();
        dummy_local_goal();

        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO_STREAM("loop time:"<<ros::Time::now().toSec()<<"[s]");
    }
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"dummy_topic");
    Dummy dummy;
    dummy.process();
    return 0;
}
