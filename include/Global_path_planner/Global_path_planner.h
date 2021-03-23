#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

struct open
{
    int f;
    int g;
    int h;
    int x;
    int y;
};



class AStarPath
{
    public:
        AStarPath();
        void process();

    private:
        //method
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void thick_wall(); //壁を増やして本当の壁にぶつからないようにする。
        void set_goal(); //goalを決める
        void make_heuristic(); //heuristic関数をつくる
        void A_star(); //A*プロセス


        //parameter
        //mapを2次元にする
        int row;
        int col;
        std::vector<std::vector<int>> map_grid; //2Dmap

        std::vector<std::vector<int>> heuristic; //heuristic関数

        bool map_checker;//map貰ったかどうか✓


        //member
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber sub_map;
        ros::Publisher pub_path;
        nav_msgs::OccupancyGrid the_map;//map格納
        nav_msgs::Path global_path;//Local_path_plannerにおくる
        geometry_msgs::PoseStamped mcl_pose;//Localizerからもらう


};
#endif

