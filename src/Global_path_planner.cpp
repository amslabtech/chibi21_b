#include "Global_path_planner/Global_path_planner.h"

AStarPath::AStarPath():private_nh
{//paramきめたらそれ書く
    private_nh.param("map_checker",map_checker,{false});

}

void AStarPath::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(!map_checker)
    {
        the_map = *msg;

        //the_mapの1次元の情報を2次元に変える
        row = the_map.info.height;
        col = the_map.info.width;
        map_grid = std::vector<std::vector<int>>(row,std::vector<int>(col,0));

        for(int i=0;i<row,i++)
        {
            for(int j=0;j<col;j++)
            {
                map_grid[i][j] = the_map.data[j+i*j];
            }
        }

        map_checker = true;
        std::cout<<"map_grid"std::<<endl;
    }

}

void thick_wall()
{
    for(int i=0;i<row;i++)
    {
        for(int j=0;j<col;j++)
        {
        }
    }


}

void set_goal()
{

}

void make_heuristic()
{

}

void A_star()
{

}

void AStarPath::process
{
    ros::Rate loop_rate(Hz);
    while(ros::ok())
    {
        if(map_checker)
        {
            std::cout<<"ok1"<<std::endl;
        }
    }

}

int main(int argc,char **argv)
{
    ros::init(int argc,argv,"Global_path_planner");
    AStarPath astarpath
    std::cout<<"Global_path_planner will begin"std::endl;
    astarpath process;
    return 0;

}

