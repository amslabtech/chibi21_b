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
#include<random>
#include<tf/tf.h>
class Localizer
{
    public:
        Localizer();
        void process();
    private:
        class Particle
        {
            public:
                Particle(Localizer* localizer);
                geometry_msgs::PoseStamped p_pose;
                void set_p(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma);
                void p_move(double dx, double dy, double dyaw);

            private:
                int p_pose_to_map_index(geometry_msgs::PoseStamped p_pose);
                double create_yaw_from_msg(geometry_msgs::Quaternion q);
                double substract_yawA_from_yawB(double yawA, double yawB);



                Localizer* mcl;
                ros::NodeHandle private_nh;
        };
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

        Particle make_particle();
        double gaussian(double mu, double sigma);
        void create_p_pose_array_from_p_array(std::vector<Particle> p_array);
        double substract_yawA_from_yawB(double yawA, double yawB);
        double create_yaw_from_msg(geometry_msgs::Quaternion q);
        void motion_update();

        int hz;
        int particle_number;
        double init_x;
        double init_y;
        double init_yaw;
        double init_x_sigma;
        double init_y_sigma;
        double init_yaw_sigma;
        double move_noise_ratio;

        bool map_get_ok = false;
        bool odometry_get_ok = false;

        ros::NodeHandle private_nh;
        ros::NodeHandle nh;
        ros::Subscriber map_sub;
        ros::Subscriber laser_sub;
        ros::Subscriber odometry_sub;
        ros::Publisher mcl_pose_pub;
        ros::Publisher p_pose_array_pub;


        nav_msgs::OccupancyGrid map;
        sensor_msgs::LaserScan laser;
        nav_msgs::Odometry current_odometry;
        nav_msgs::Odometry previous_odometry;
        geometry_msgs::PoseStamped mcl_pose;
        geometry_msgs::PoseArray p_pose_array;
        std::vector<Particle> p_array;

};
#endif
