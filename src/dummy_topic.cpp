#include "local_path_planner/dummy_topic.h"

Dummy::Dummy():private_nh("~")
{
    private_nh.param("DT",DT,{0.05});
    private_nh.param("pre_x",pre_x,{0.0});
    private_nh.param("pre_y",pre_y,{0.0});
    private_nh.param("pre_yaw",pre_yaw,{0.0});
    private_nh.param("ROBOT_FRAME",ROBOT_FRAME,{"base_link"});
    private_nh.param("MAP_FRAME",MAP_FRAME,{"map"});
    private_nh.param("LOCAL_GOAL_X",LOCAL_GOAL_X,{10.0});
    private_nh.param("LOCAL_GOAL_Y",LOCAL_GOAL_Y,{10.0});
    private_nh.param("LOCAL_GOAL_YAW",LOCAL_GOAL_YAW,{0.0});

    pub_estimated_pose=nh.advertise<geometry_msgs::PoseStamped>("/dummy_estimated_pose",1);
    pub_local_goal=nh.advertise<geometry_msgs::PoseStamped>("/dummy_local_goal",1);
    pub_twist=nh.advertise<geometry_msgs::Twist>("/dummy_twist",1);
    // pub_obstacle=nh.advertise<geometry_msgs::PoseArray>("/dummy_obstacle",1);

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

    local_goal.pose.position.x=LOCAL_GOAL_X;
    local_goal.pose.position.y=LOCAL_GOAL_Y;
    local_goal.pose.position.z=0.0;
    local_goal.pose.orientation.x=0.0;
    local_goal.pose.orientation.y=0.0;
    local_goal.pose.orientation.z=std::sin(LOCAL_GOAL_YAW/2.0);
    local_goal.pose.orientation.w=std::cos(LOCAL_GOAL_YAW/2.0);

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

    // ROS_INFO_STREAM("linear.x = "<<twist.linear.x<<" || linear.y = "<<twist.linear.y<<" || angular.z = "<<twist.angular.z);
    pub_twist.publish(twist);
}

void Dummy::dummy_obstacle()
{
/*    std::vector<double> obs_list={
        {5.0,5.0},
        {3.0,7,0},
        {3.0,5.0},
        {5.0,3.0}
    };

    geometry_msgs::PoseArray pub_obs_list;
    pub_obs_list.header.stamp=ros::Time::now();
    pub_obs_list.header.frame_id=MAP_FRAME;

    geometry_msgs::Pose obs_point;
    for(auto& obs:obs_list){
        obs_point.position.x=obs[0];
        obs_point.position.y=obs[1];
        obs_point.position.z=0;
        obs_point.orientation.x=0;
        obs_point.orientation.y=0;
        obs_point.orientation.z=0;
        obs_point.orientation.w=0;

        // pub_obs_list.push_back(obs_point);
    }
    pub_obstacle.publish(pub_obs_list);
*/}

void Dummy::process()
{
    ros::Rate loop_rate(1/DT);
    while(ros::ok()){
        dummy_estimated_pose();
        dummy_local_goal();
        dummy_twist();
        // dummy_obstacle();

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
