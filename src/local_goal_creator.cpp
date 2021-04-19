#include "Global_path_planner/local_goal_creator.h"

LocalGoal::LocalGoal():private_nh("")
{
    //param
    private_nh.param("global_path_checker",global_path_checker,{false});
    private_nh.param("mcl_checker",mcl_checker,{false});
    // private_nh.param("",,{});
    // private_nh.param("",,{});

    sub_path = nh.subscribe("/path",10,&LocalGoal::global_path_callback,this);
    sub_pose = nh.subscribe("/mcl_pose",10,&LocalGoal::mcl_callback,this);

    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>("/local_goal",1);
}

void LocalGoal::global_path_callback(const nav_msgs::Path::ConstPtr &msg)
{
    local_goal.header.frame_id = "map";
    if(!global_path_checker)
    {
        global_path = *msg;
        goal_num = 0;
        local_goal = global_path.poses[goal_num];
        global_path_checker = true;
    }

}

void LocalGoal::mcl_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(!mcl_checker)
    {
        current_pose = *msg;
        mcl_checker = true;
    }
}

void LocalGoal::select_goal()
{
    float distance = sqrt(pow(current_pose.pose.position.x - local_goal.pose.position.x , 2) + pow(current_pose.pose.position.y - local_goal.pose.position.y , 2));
    if(distance < 10)
    {
        goal_num += 20; //ちょっと先に新しいゴールを設置
        if(goal_num < global_path.poses.size()) local_goal = global_path.poses[goal_num];
        else
        {
            goal_num = global_path.poses.size()-1;
            local_goal = global_path.poses[goal_num];
        }
    }

}

void LocalGoal::process()
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        if(global_path_checker && mcl_checker)
        {
            select_goal();
            std::cout<<"local_goal ok"<<std::endl;
            pub_local_goal.publish(local_goal);
        }

        std::cout<<"FIN"<<std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"local_goal_creator");
    LocalGoal creator;
    std::cout<<"local_goal_creator start"<<std::endl;
    creator.process();
    return 0;
}
