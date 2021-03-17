#ifndef LOCALIZER_H
#define LOCALIZER_H

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<roomba_500driver_meiji/RoombaCtrl.h>
#include<tf/tf.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<nav_msgs/OccupancyGrid.h>


class Localizer
{
    public:
        Localizer();
        void process();
    private:
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

        class Particle
        {
            public:
                Particle();
            private:
        };

        ros::NodeHandle private_nh;
        ros::NodeHandle nh;
        ros::Subscriber map_sub;
        ros::Subscriber laser_sub;
        ros::Subscriber odometry_sub;
        ros::Publisher mcl_pose_pub;

        nav_msgs::OccupancyGrid map;
        sensor_msgs::LaserScan laser;
        nav_msgs::Odometry odometry;



};
#endif
