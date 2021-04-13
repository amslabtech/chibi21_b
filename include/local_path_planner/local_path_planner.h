#ifndef LOCAL_PATH_PLANNER
#define LOCAL_PATH_PLANNER

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <vector>
#include <roomba_500driver_meiji/RoombaCtrl.h>

class DynamicWindowApproach
{
    public:
        DynamicWindowApproach();
        void process();

        struct Window
        {
            double vel_max;
            double vel_min;
            double omega_max;
            double omega_min;
        };

        struct State
        {
            double x;
            double y;
            double yaw;
            double velocity;
            double omega;
        };

    private:
        Window Vs;
        Window Vd;
        Window dw;
        int RESOLUTION_VELOCITY_NUM;
        int RESOLUTION_OMEGA_NUM;
        double DT;
        std::string ROBOT_FRAME;
        bool USE_DUMMY_TOPIC;

        //roomba's spec
        double LINEAR_SPEED_MAX;
        double LINEAR_SPEED_MIN;
        double ANGULAR_SPEED_MAX;
        double LINEAR_ACCL;
        double ANGULAR_ACCL;

        //cost gain
        double COST_HEADING_GAIN;
        double COST_VELOCITY_GAIN;
        double COST_OBSTACLE_GAIN;

        //trajectories
        State current_state;
        State local_goal;
        std::vector<State> best_traj;
        std::vector<std::vector<State>> trajectories;
        std::vector<std::vector<double>> obs_list;
        sensor_msgs::LaserScan scan;
        geometry_msgs::PoseStamped estimated_pose;
        // geometry_msgs::PoseStamped local_goal_sim;
        nav_msgs::Odometry odometry;
        double PREDICT_TIME;

        //method
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &);
        void estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &);
        void local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &);
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &);
        void twist_to_odometry_callback(const geometry_msgs::Twist::ConstPtr &);
        void calc_dynamic_window();
        void calc_trajectory();
        void roomba_motion(State&, double, double);
        double calc_cost_heading(State&);
        double calc_cost_velocity(double);
        double calc_cost_obstacle(std::vector<State>&);
        void scan_to_obs();
        void init();

        //ros
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_scan;
        ros::Subscriber sub_estimated_pose;
        ros::Subscriber sub_local_map;
        ros::Subscriber sub_odometry;
        ros::Subscriber sub_twist;
        ros::Publisher pub_roomba_ctrl;

        //rviz
        ros::Publisher pub_best_traj;
        ros::Publisher pub_trajectries;
        // ros::Publisher pub_local_goal;
        void visualize_traj(std::vector<State>&,const ros::Publisher&);
        // void visualize_trajectories();
};
#endif
