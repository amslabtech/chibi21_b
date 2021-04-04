#ifndef LOCAL_PATH_PLANNER
#define LOCAL_PATH_PLANNER

#include "ros/ros.h"
// #include ""


class DynamicWindowApproach
{
    public:
        DynamicWindowApproach();
        void process();

    private:
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

        Window Vs;
        Window Vd;
        Window dw;
        int RESOLUTION_VELOCITY_NUM;
        int RESOLUTION_OMEGA_NUM
        double DT;

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


};
#endif
