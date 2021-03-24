#include "Global_path_planner/Global_path_planner.h"

AStarPath::AStarPath():private_nh("")
{
    //paramきめたらそれ書く
    private_nh.param("hz",hz,{10});
    private_nh.param("map_checker",map_checker,{false});
    sub_map = nh.subscribe("/map",10,&AStarPath::map_callback,this);
    pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/map",1);

}

void AStarPath::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(map_checker) return;
    else
    {
        the_map = *msg;

        //the_mapの1次元の情報を2次元に変える
        row = the_map.info.height;
        col = the_map.info.width;
        map_grid = std::vector<std::vector<int>>(row,std::vector<int>(col,0));
        map_grid_copy = std::vector<std::vector<int>>(row,std::vector<int>(col,0));

        for(int i=0;i<row;i++)
        {
            for(int j=0;j<col;j++)
            {
                map_grid[i][j] = the_map.data[j+i*row];
            }
        }

        map_checker = true;
        std::cout<<"map_grid"<<std::endl;
    }

}

void AStarPath::thick_wall()
{
    if(!wall_checker)
    {
        //map_grid_copy = map_grid;

        for(int i=0;i<row;i++)
        {
            for(int j=0;j<col;j++)
            {
                map_grid_copy[i][j] = map_grid[i][j];
            }
        }


        for(int i=1;i<row-1;i++)
        {
            for(int j=1;j<col-1;j++)
            {
                if(map_grid[i][j]==100)
                {
                    map_grid_copy[i+1][j]==100;
                    map_grid_copy[i][j+1]==100;
                    map_grid_copy[i+1][j+1]==100;
                }

            }
        }
        //map_grid = map_grid_copy;

        for(int i=0;i<row;i++)
        {
            for(int j=0;j<col;j++)
            {
                map_grid[i][j] = map_grid_copy[i][j];
            }
        }


        for(int i=0;i<row;i++)
        {
            for(int j=0;j<col;j++)
            {
                the_map.data[j+i*row] = map_grid[i][j];
            }
        }


        pub_map.publish(the_map);
        wall_checker = true;
    }

}

void AStarPath::set_goal()
{
    //goal = {{,},{,},{,},{,},{,},{,}};
    //goal[0]:startpoint
    //goal[5]:goalpoint
}

void AStarPath::make_heuristic(int next,int pre)
{
    dx = abs(goal[next].x-goal[pre].x);
    dy = abs(goal[next].y-goal[pre].y);

    heuristic = std::vector<std::vector<int>>(dx,std::vector<int>(dy,0));
    for(int i=dx-1;i<=0;i--)
    {
        for(int j=dy-1;j<=0;j--)
        {
            heuristic[i][j] = abs(goal[next].x-i+1) + abs(goal[next].y-j+1);
        }
    }

}

void AStarPath::A_star()
{

    for(int i=1;i<5;i++)
    {
        make_heuristic(i,i-1);
        reached = false;

        while(!reached)
        {

            if(i=1) reached = true;//ここよくわかんなくなってる。

        }


    }

}

void AStarPath::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(map_checker)
        {
            std::cout<<"ok1"<<std::endl;
            thick_wall();
            std::cout<<"ok2"<<std::endl;

        }
        else
        {
            // std::cout<<"waaa"<<std::endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"Global_path_planner");
    AStarPath astarpath;
    std::cout<<"Global_path_planner will begin"<<std::endl;
    astarpath.process();
    return 0;

}

