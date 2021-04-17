#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include <ros/ros.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

class LocalGoal
{
    public:
        LocalGoal();
        void process();

    private:
    //method
    void global_path_callback(const nav_msgs::Path::ConstPtr &msg);
    void mcl_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void select_goal();

    //parameter
    //double distanse;
    int goal_num;
    bool global_path_checker;
    bool mcl_checker;



    //member
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber sub_path;
    ros::Subscriber sub_pose;
    ros::Publisher pub_local_goal;
    nav_msgs::Path global_path;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped local_goal;
    geometry_msgs::PoseStamped mcl_pose;

};
#endif

