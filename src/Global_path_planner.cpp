#include "Global_path_planner/Global_path_planner.h"

AStarPath::AStarPath():private_nh("")
{
    //param
    private_nh.param("hz",hz,{10});
    private_nh.param("map_checker",map_checker,{false});
    private_nh.param("wall_checker",wall_checker,{false});
    private_nh.param("heu_checker",heu_checker,{false});
    private_nh.param("path_checker",path_checker,{false});
    sub_map = nh.subscribe("/map",10,&AStarPath::map_callback,this);
    // pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/fixed_map",1);
    pub_path = nh.advertise<nav_msgs::Path>("/path",1);
    //pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/mini_goal",1);

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
                //map_grid[i][j] = the_map.data[j+i*row];
                map_grid[i][j] = the_map.data[i+j*row];
            }
        }

        origin.x = the_map.info.origin.position.x;
        origin.y = the_map.info.origin.position.y;
        std::cout<<"origin:"<<origin.x<<","<<origin.y<<std::endl;

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
                //map_cleaner
                //if((i>2282 && i<2295 && j>1547 && j<2213) || (i>1997 && i<2282 && j>1540 && j<1555) || (i>1995 && i<2007 && j>1547 && j<2213) || (i>2003 && i<2290 && j>2205 && j<2220))
                if((j>2282 && j<2295 && i>1547 && i<2213) || (j>1997 && j<2282 && i>1540 && i<1555) || (j>1995 && j<2007 && i>1547 && i<2213) || (j>2003 && j<2290 && i>2205 && i<2220))
                {
                    if(map_grid[i][j]==100)
                    {
                        map_grid[i][j] = 0;
                    }
                }
                //map_copy
                map_grid_copy[i][j] = map_grid[i][j];
            }
           // std::cout<<std::endl;
        }


        int count = 0;

        for(int i=5;i<row-5;i++)
        {
            for(int j=5;j<col-5;j++)
            {
                if(map_grid_copy[i][j]==100)
                {
                    map_grid[i+1][j]=100;
                    map_grid[i+1][j+1]=100;
                    map_grid[i+1][j+2]=100;
                    map_grid[i][j+1]=100;
                    map_grid[i][j+2]=100;
                    map_grid[i+2][j]=100;
                    map_grid[i+2][j+1]=100;
                    map_grid[i+2][j+2]=100;

                    map_grid[i][j-1]=100;
                    map_grid[i][j-2]=100;
                    map_grid[i-1][j]=100;
                    map_grid[i-1][j-1]=100;
                    map_grid[i-1][j-2]=100;
                    map_grid[i-2][j]=100;
                    map_grid[i-2][j-1]=100;
                    map_grid[i-2][j-2]=100;

                    //std::cout<<"wo!"<<std::endl;
                //    count += 1;
                }


            }
        }

        /*for(int i=0;i<row;i++)
        {
            for(int j=0;j<col;j++)
            {
                map_grid[i][j] = map_grid_copy[i][j];
            }
        }*/

        /*for(int i=0;i<100;i++)
        {
            int j = 2285;
            map_grid[j][1895+i] = 80;
            map_grid[j][1896+i] = 60;
            map_grid[j+1][1895+i] = 60;
            map_grid[j+1][1896+i] = 60;
            map_grid[j][1550+i] = 80;
            map_grid[j][1551+i] = 60;
            map_grid[j+1][1550+i] = 60;
            map_grid[j+1][1551+i] = 60;

        }*/

        // map_grid[1550][2000] = 80;
        // map_grid[2210][2000] = 80;
        // map_grid[1550][2285] = 80;
        // map_grid[2210][2285] = 80;

        res = the_map.info.resolution;
        std::cout<<res<<std::endl;

        // std::cout<<map_grid[2000][1550]<<std::endl;
        // std::cout<<map_grid[1999][1666]<<std::endl;
        // std::cout<<map_grid[2285][1642]<<std::endl;
        // std::cout<<map_grid[2285][1641]<<std::endl;


        for(int i=0;i<row;i++)
        {
            for(int j=0;j<col;j++)
            {
                //the_map.data[j+i*row] = map_grid[i][j];
                the_map.data[i+j*row] = map_grid[i][j];
            }
        }

        std::cout<<"wall"<<std::endl;
        //std::cout<<count<<std::endl;

        // pub_map.publish(the_map);
        wall_checker = true;
    }

}

void AStarPath::set_goal()
{
    //goal = {{2285,1890},{2285,1550},{2000,1550},{2000,2210},{2285,2210},{2285,1890}}; スタート位置間違えてた。↓が正しい方です。
    //goal = {{2000,1890},{2000,2210},{2285,2210},{2285,1550},{2000,1550},{2000,1890}};
    goal = {{1890,2000},{2210,2000},{2210,2285},{1550,2285},{1550,2000},{1890,2000}};
    //goal[0]:startpoint
    //goal[5]:goalpoint
    std::cout<<"set_goal"<<std::endl;
}

void AStarPath::make_heuristic(int next)
{
    //dx = abs(goal[next].x-goal[pre].x)+1;
    //dy = abs(goal[next].y-goal[pre].y)+1;

    heuristic = std::vector<std::vector<int>>(row,std::vector<int>(col,0));
    std::cout<<"startheu"<<std::endl;
    //std::cout<<dx<<std::endl;
    //std::cout<<dy<<std::endl;
    for(int i=0;i<row;i++)
    {
        for(int j=0;j<col;j++)
        {
            heuristic[i][j] = abs(goal[next].x - i) + abs(goal[next].y - j);
            //std::cout<<heuristic[i][j];
            //std::cout<<",";
        }
        //std::cout<<std::endl;
    }
    //std::cout<<"--------------------"<<std::endl;

}

void AStarPath::A_star()
{
    std::cout<<"A*start"<<std::endl;

    delta = {{0,-1},{-1,0},{0,1},{1,0}};//R,Up,L,Down
    open_list = std::vector<std::vector<open>>(row,std::vector<open>(col));

    /*origin.x = the_map.info.origin.position.x;
    origin.y = the_map.info.origin.position.y;
    std::cout<<"origin:"<<origin.x<<","<<origin.y<<std::endl;
*/

    for(int r=0;r<row;r++)
    {
        for(int c=0;c<col;c++)
        {
            open_list[r][c].f = 0;
            open_list[r][c].g = 0;
            open_list[r][c].pre_x = 0;
            open_list[r][c].pre_y = 0;
        }
    }
    std::cout<<"setopen_list"<<std::endl;

    for(int i=0;i<5;i++)
    {
        mini_path.poses.clear();
        make_heuristic(i+1);
        reached = false;
        goal_point.header.frame_id = "map";
        goal_point.pose.position.x = goal[i+1].x;
        goal_point.pose.position.y = goal[i+1].y;

        close_list = std::vector<std::vector<open>>(row,std::vector<open>(col));
        for(int r=0;r<row;r++)
        {
            for(int c=0;c<col;c++)
            {
                close_list[r][c].f = 0;
                close_list[r][c].g = 0;
                close_list[r][c].pre_x = 0;
                close_list[r][c].pre_y = 0;
                open_list[r][c].f = 0;
                open_list[r][c].g = 0;
                open_list[r][c].pre_x = 0;
                open_list[r][c].pre_y = 0;
            }
        }

        std::cout<<"setclose_list"<<std::endl;

        int gx = goal[i+1].x;
        int gy = goal[i+1].y;
        int x = goal[i].x;
        int y = goal[i].y;
        int g = 0;

        //std::cout<<goal[i].x<<","<<goal[i].y<<","<<goal[i+1].x<<","<<goal[i+1].y<<std::endl;


        std::cout<<"start_searching"<<std::endl;

        while(!reached)
        {
            if(x == gx && y == gy)
            {
                std::cout<<"reached"<<std::endl;
                reached = true;
            }
            else
            {
                g += 1;
                for(int j=0;j<4;j++)
                {
                    //std::cout<<j;
                    x2 = x + delta[j].x;
                    y2 = y + delta[j].y;

                    if(x2 >= 0 && x2<row && y2 >= 0 && y2<col)
                    {
                        if(close_list[x2][y2].g == 0 && map_grid[x2][y2] != 100)
                        {
                            f[j] = g + heuristic[x2][y2];
                            open_list[x2][y2].g = g;
                            open_list[x2][y2].f = f[j];
                            open_list[x2][y2].pre_x = x;
                            open_list[x2][y2].pre_y = y;
                            close_list[x2][y2].pre_x = x;
                            close_list[x2][y2].pre_y = y;
                            close_list[x2][y2].g = 1;
                        }
                        else
                        {
                            //close_list[x2][y2].g = 1;
                            f[j] = 4000;
                        }
                    }
                }

                int fmin = f[0];
                int kmin = 0;
                for(int k=0;k<4;k++)//f値が一番小さいのいつだった？
                {
                    if(f[k]<fmin)
                    {
                        fmin = f[k];
                        kmin = k;
                    }
                }

                if(fmin==4000)
                {
                    close_list[old_x][old_y].g = 0;
                    for(int k=0;k<4;k++)
                    {
                        close_list[old_x + delta[k].x][old_y + delta[k].y].g = 0;
                        close_list[old_x + delta[k].x][old_y + delta[k].y].pre_x = 0;
                        close_list[old_x + delta[k].x][old_y + delta[k].y].pre_y = 0;
                    }
                    close_list[x][y].g = 1;
            //        map_grid[x][y] = 100;
                    x = old_x;
                    y = old_y;

                }

                if(fmin!=4000)
                {
                    old_x = x;
                    old_y = y;
            /*std::cout<<close_list[x][y].pre_x;
            std::cout<<",";
            std::cout<<close_list[x][y].pre_y<<std::endl;*/
                    x = x + delta[kmin].x;
                    y = y + delta[kmin].y;
                    // std::cout<<close_list[x][y].pre_x;
                    // std::cout<<",";
                    // std::cout<<close_list[x][y].pre_y;
                    // std::cout<<",";
                    // std::cout<<x;
                    // std::cout<<",";
                    // std::cout<<y;
                    // std::cout<<",";
                    // std::cout<<open_list[x][y].f<<std::endl;
                }
            }


        }
        startpoint = false;
        //mini_path
        mini_path.header.frame_id = "map";
        geometry_msgs::PoseStamped point;
        point.pose.position.x = (x-row/2)*res;// + origin.x;
        point.pose.position.y = (y-col/2)*res;// + origin.y;
        mini_path.poses.push_back(point);

        mom.x = close_list[x][y].pre_x;
        mom.y = close_list[x][y].pre_y;
        std::cout<<"momdad"<<std::endl;
        std::cout<<mom.x<<","<<mom.y<<std::endl;

        while(!startpoint)
        {
            if(mom.x == goal[i].x && mom.y == goal[i].y)
            {
                point.pose.position.x = (mom.x-row/2)*res;
                point.pose.position.y = (mom.y-col/2)*res;
                mini_path.poses.push_back(point);
                startpoint = true;
            }

            else
            {
                    point.pose.position.x = (mom.x-row/2)*res;// + origin.x;
                    point.pose.position.y = (mom.y-col/2)*res;// + origin.y;
                    mini_path.poses.push_back(point);

                    dad.x = mom.x;
                    dad.y = mom.y;
                    mom.x = close_list[dad.x][dad.y].pre_x;
                    mom.y = close_list[dad.x][dad.y].pre_y;

                    //std::cout<<mom.x<<","<<mom.y<<std::endl;

                    //if(mom.y != 0) std::cout<<mom.y<<std::endl;
            }
        }

        std::cout<<"============="<<std::endl;
        std::reverse(mini_path.poses.begin(),mini_path.poses.end());
        global_path.header.frame_id = "map";
        global_path.poses.insert(global_path.poses.end(),mini_path.poses.begin(),mini_path.poses.end());
    }
    std::cout<<"--------------------"<<std::endl;
}

void AStarPath::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(map_checker && !path_checker)
        {
            //std::cout<<"ok1"<<std::endl;
            thick_wall();
            set_goal();
            // if(!heu_checker)
            // {
            //     for(int i=0;i<5;i++)
            //     {
            //         make_heuristic(i+1);
            //     }
            //     heu_checker = true;
            // }
            A_star();
            std::cout<<"ok2"<<std::endl;
            pub_path.publish(global_path);
            std::cout<<"fin"<<std::endl;
            path_checker = true;

        }
        std::cout<<"FIN"<<std::endl;
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

