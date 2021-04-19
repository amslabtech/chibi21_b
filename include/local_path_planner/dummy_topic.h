#ifndef DUMMY_TOPIC
#define DUMMY_TOPIC

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>

class Dummy
{
    public:
        Dummy();
        void process();
    private:
        void cmd_vel_callback(const roomba_500driver_meiji::RoombaCtrl::ConstPtr &);
        void dummy_estimated_pose();
        void dummy_local_goal();
        void dummy_twist();
        void dummy_obstacle();

        double DT;
        double pre_x;
        double pre_y;
        double pre_yaw;
        double LOCAL_GOAL_X;
        double LOCAL_GOAL_Y;
        double LOCAL_GOAL_YAW;
        roomba_500driver_meiji::RoombaCtrl cmd_vel;
        std::string ROBOT_FRAME;
        std::string MAP_FRAME;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Publisher pub_estimated_pose;
        ros::Publisher pub_local_goal;
        ros::Publisher pub_twist;
        ros::Publisher pub_obstacle;
        ros::Subscriber sub_cmd_vel;

};
#endif
