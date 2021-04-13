#include "local_path_planner/dummy_topic.h"

Dummy::Dummy():private_nh("~")
{
    private_nh.param("DT",DT,{0.05});
    private_nh.param("pre_x",pre_x,{0.0});
    private_nh.param("pre_y",pre_y,{0.0});
    private_nh.param("pre_yaw",pre_yaw,{0.0});
    private_nh.param("ROBOT_FRAME",ROBOT_FRAME,{"base_link"});
    private_nh.param("MAP_FRAME",MAP_FRAME,{"map"});

    pub_estimated_pose=nh.advertise<geometry_msgs::PoseStamped>("/dummy_estimated_pose",1);
    pub_local_goal=nh.advertise<geometry_msgs::PoseStamped>("/dummy_local_goal",1);
    pub_twist=nh.advertise<geometry_msgs::Twist>("/dummy_twist",1);

    sub_cmd_vel=nh.subscribe("/roomba/control",100,&Dummy::cmd_vel_callback,this);
}

void Dummy::cmd_vel_callback(const roomba_500driver_meiji::RoombaCtrl::ConstPtr& msg)
{
    cmd_vel=*msg;

    // ROS_INFO_STREAM("cmd_vel.cntl.linear.x = "<<cmd_vel.cntl.linear.x);
}

void Dummy::dummy_estimated_pose()
{
    geometry_msgs::PoseStamped estimated_pose;

    estimated_pose.header.frame_id=MAP_FRAME;
    estimated_pose.header.stamp=ros::Time::now();

    double yaw=pre_yaw+cmd_vel.cntl.angular.z*DT;
    estimated_pose.pose.position.x=pre_x+cmd_vel.cntl.linear.x*std::cos(yaw)*DT;
    estimated_pose.pose.position.y=pre_y+cmd_vel.cntl.linear.x*std::sin(yaw)*DT;
    estimated_pose.pose.position.z=0.0;
    estimated_pose.pose.orientation.x=0.0;
    estimated_pose.pose.orientation.y=0.0;
    estimated_pose.pose.orientation.z=std::sin(yaw/2.0);
    estimated_pose.pose.orientation.w=std::cos(yaw/2.0);

    pre_yaw=yaw;
    pre_x=estimated_pose.pose.position.x;
    pre_y=estimated_pose.pose.position.y;
    pub_estimated_pose.publish(estimated_pose);
}

void Dummy::dummy_local_goal()
{
    geometry_msgs::PoseStamped local_goal;

    local_goal.header.frame_id=MAP_FRAME;
    local_goal.header.stamp=ros::Time::now();

    local_goal.pose.position.x=-20.0;
    local_goal.pose.position.y=-10.0;
    local_goal.pose.position.z=0.0;
    local_goal.pose.orientation.x=0.0;
    local_goal.pose.orientation.y=0.0;
    local_goal.pose.orientation.z=0.0;
    local_goal.pose.orientation.w=1.0;

    pub_local_goal.publish(local_goal);
}

void Dummy::dummy_twist()
{
    geometry_msgs::Twist twist;

    twist.linear.x=cmd_vel.cntl.linear.x*std::cos(pre_yaw);
    twist.linear.y=cmd_vel.cntl.linear.x*std::sin(pre_yaw);
    twist.linear.z=0.0;
    twist.angular.x=0.0;
    twist.angular.y=0.0;
    twist.angular.z=cmd_vel.cntl.angular.z;

    pub_twist.publish(twist);
}

void Dummy::process()
{
    ros::Rate loop_rate(1/DT);
    while(ros::ok()){
        dummy_estimated_pose();
        dummy_local_goal();
        dummy_twist();

        ros::spinOnce();
        loop_rate.sleep();
        // ROS_INFO_STREAM("loop time:"<<ros::Time::now().toSec()<<"[s]");
    }
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"dummy_topic");
    Dummy dummy;
    dummy.process();
    return 0;
}
