#include "local_path_planner/local_path_planner.h"

DynamicWindowApproach::DynamicWindowApproach():private_nh("~")
{
    private_nh.param("RESOLUTION_VELOCITY_NUM",RESOLUTION_VELOCITY_NUM,{10});
    private_nh.param("RESOLUTION_OMEGA_NUM",RESOLUTION_OMEGA_NUM,{10});
    private_nh.param("DT",DT,{0.05});
    private_nh.param("ROBOT_FRAME",ROBOT_FRAME,{"base_link"});
    private_nh.param("LINEAR_SPEED_MAX",LINEAR_SPEED_MAX,{1.0});
    private_nh.param("LINEAR_SPEED_MIN",LINEAR_SPEED_MIN,{0.0});
    private_nh.param("ANGULAR_SPEED_MAX",ANGULAR_SPEED_MAX,{0.8});
    private_nh.param("LINEAR_ACCL",LINEAR_ACCL,{1.0});
    private_nh.param("ANGULAR_ACCL",ANGULAR_ACCL,{2.0});
    private_nh.param("COST_HEADING_GAIN",COST_HEADING_GAIN,{1.0});
    private_nh.param("COST_VELOCITY_GAIN",COST_VELOCITY_GAIN,{1.0});
    private_nh.param("COST_OBSTACLE_GAIN",COST_OBSTACLE_GAIN,{1.0});
    private_nh.param("PREDICT_TIME",PREDICT_TIME,{3.0});
    private_nh.param("USE_DUMMY_TOPIC",USE_DUMMY_TOPIC,{false});
    // private_nh.param("",,{});

    // sub_scan=nh.subscribe("/scan",100,&DynamicWindowApproach::scan_callback,this);
    sub_estimated_pose=nh.subscribe("/mcl_pose",100,&DynamicWindowApproach::estimated_pose_callback,this);
    sub_local_map=nh.subscribe("/local_goal",100,&DynamicWindowApproach::local_goal_callback,this);
    if(USE_DUMMY_TOPIC){sub_twist=nh.subscribe("/dummy_twist",100,&DynamicWindowApproach::twist_to_odometry_callback,this);
    }else{
        sub_odometry=nh.subscribe("/roomba/odometry",100,&DynamicWindowApproach::odometry_callback,this);
    }
    pub_roomba_ctrl=nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);
    pub_best_traj=nh.advertise<nav_msgs::Path>("best_traj",1);
    pub_trajectries=nh.advertise<nav_msgs::Path>("trajectories",1);
}

void DynamicWindowApproach::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan=*msg;
}

void DynamicWindowApproach::estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    estimated_pose=*msg;

    current_state.x=estimated_pose.pose.position.x;
    current_state.y=estimated_pose.pose.position.y;
    current_state.yaw=tf::getYaw(estimated_pose.pose.orientation);
}

void DynamicWindowApproach::local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped _local_goal=*msg;

    local_goal.x=_local_goal.pose.position.x;
    local_goal.y=_local_goal.pose.position.y;
    local_goal.yaw=tf::getYaw(_local_goal.pose.orientation);
    local_goal.velocity=LINEAR_SPEED_MAX;
    local_goal.omega=0.0;
}

void DynamicWindowApproach::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry=*msg;

    current_state.velocity=odometry.twist.twist.linear.x;
    current_state.omega=odometry.twist.twist.angular.z;
}

void DynamicWindowApproach::twist_to_odometry_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    geometry_msgs::Twist twist=*msg;

    current_state.velocity=sqrt(std::pow(twist.linear.x,2)+std::pow(twist.linear.y,2));
    current_state.omega=twist.angular.z;

    // ROS_INFO_STREAM("current_state.velocity = "<<current_state.velocity<<" || current_state.omega ="<<current_state.omega);
}

void DynamicWindowApproach::calc_dynamic_window()
{
    Vs.vel_max=LINEAR_SPEED_MAX;
    Vs.vel_min=LINEAR_SPEED_MIN;
    Vs.omega_max=ANGULAR_SPEED_MAX;
    Vs.omega_min=-1*ANGULAR_SPEED_MAX;

    Vd.vel_max=current_state.velocity+LINEAR_ACCL*DT;
    Vd.vel_min=current_state.velocity-LINEAR_ACCL*DT;
    Vd.omega_max=current_state.omega+ANGULAR_ACCL*DT;
    Vd.omega_min=current_state.omega-ANGULAR_ACCL*DT;

    dw.vel_max=std::min(Vs.vel_max,Vd.vel_max);
    dw.vel_min=std::max(Vs.vel_min,Vd.vel_min);
    dw.omega_max=std::min(Vs.omega_max,Vd.omega_max);
    dw.omega_min=std::max(Vs.omega_min,Vd.omega_min);

    // ROS_INFO_STREAM("dw.vel_max = "<<dw.vel_max<<" || dw.vel_min = "<<dw.vel_min);
    // ROS_INFO_STREAM("dw.omega_max = "<<dw.omega_max<<" || dw.omega_min = "<<dw.omega_min);

}

void DynamicWindowApproach::calc_trajectory()
{
    best_traj.clear();
    trajectories.clear();

    double resolution_velocity=(dw.vel_max-dw.vel_min)/RESOLUTION_VELOCITY_NUM;
    double resolution_omega=(dw.omega_max-dw.omega_min)/RESOLUTION_OMEGA_NUM;

    double cost_min=1e3;
    double best_velocity=LINEAR_SPEED_MAX;
    double best_omega=0.0;

    int count=0;
    for(double v=dw.vel_min;v<dw.vel_max;v+=resolution_velocity){
        for(double w=dw.omega_min;w<dw.omega_max;w+=resolution_omega){
            State state;
            state.x=0.0;
            state.y=0.0;
            state.yaw=0.0;
            state.velocity=current_state.velocity;
            state.omega=current_state.omega;

            std::vector<State> traj;
            for(double t=0;t<=PREDICT_TIME;t+=DT){
                roomba_motion(state,v,w);
                traj.push_back(state);
            }
            trajectories.push_back(traj);
            visualize_traj(trajectories[0],pub_trajectries);

            double cost_heading=calc_cost_heading(traj.back());
            double cost_velocity=calc_cost_velocity(v);
            double cost_obstacle=calc_cost_obstacle(traj);
            double cost_sum=COST_HEADING_GAIN*cost_heading+COST_VELOCITY_GAIN*cost_velocity+COST_OBSTACLE_GAIN*cost_obstacle;

            if(cost_sum<cost_min){
                cost_min=cost_sum;
                best_traj=traj;
                best_velocity=v;
                best_omega=w;
            }
            count++;
        }
    }
    ROS_INFO_STREAM("best_velocity = "<<best_velocity<<" || best_omega = "<<best_omega<<" || count = "<<count);

    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    cmd_vel.cntl.linear.x=best_velocity;
    cmd_vel.cntl.angular.z=best_omega;
    pub_roomba_ctrl.publish(cmd_vel);

    visualize_traj(best_traj,pub_best_traj);
    // ROS_INFO_STREAM(best_traj);
    // visualize_trajectries(trajectories);
}

void DynamicWindowApproach::roomba_motion(State& state, double velocity, double omega)
{
    state.yaw+=omega*DT;
    if(state.yaw>M_PI) state.yaw-=2*M_PI;
    else if(state.yaw<-1*M_PI) state.yaw+=2*M_PI;
    state.x+=velocity*std::cos(state.yaw)*DT;
    state.y+=velocity*std::sin(state.yaw)*DT;
    state.velocity=velocity;
    state.omega=omega;
}

double DynamicWindowApproach::calc_cost_heading(State& traj_last_state)
{
    double angle_to_goal=std::atan2((local_goal.y-current_state.y)-traj_last_state.y,(local_goal.x-current_state.x)-traj_last_state.x);

    double angle_diff=angle_to_goal-(current_state.yaw+traj_last_state.yaw);
    if(angle_diff>M_PI) angle_diff-=2*M_PI;
    else if(angle_diff<-M_PI) angle_diff+=2*M_PI;
    angle_diff=std::abs(angle_diff);

    // ROS_INFO_STREAM("local_goal.x = "<<local_goal.x<<" || local_goal.y = "<<local_goal.y);
    // ROS_INFO_STREAM("traj_last_state.x = "<<traj_last_state.x<<" || traj_last_state.y = "<<traj_last_state.y);
    // ROS_INFO_STREAM("angle_diff = "<<angle_diff);

    return std::max(angle_diff,0.0);
}

double DynamicWindowApproach::calc_cost_velocity(double velocity)
{
    return std::max(local_goal.velocity-velocity,0.0);
}

double DynamicWindowApproach::calc_cost_obstacle(std::vector<State>& traj)
{
    double dist_min=1e3; //最も近くなる時の距離
    for(auto& state : traj){
        for(auto& obs : obs_list){
            double dist=sqrt(pow(state.x-obs[0],2)+pow(state.y-obs[1],2));
            if(dist<dist_min) dist_min=dist;
        }
    }
    return std::max(1/dist_min,0.0); //距離が小さくなるほどコストが大きくなる
}

void DynamicWindowApproach::scan_to_obs()
{
    obs_list.clear();
    double angle=scan.angle_min;
    for(auto r : scan.ranges){
        double x=r*std::cos(angle);
        double y=r*std::sin(angle);
        std::vector<double> obs_state={x,y};
        obs_list.push_back(obs_state);
        angle+=scan.angle_increment;
    }
}

void DynamicWindowApproach::process()
{
    ros::Rate loop_rate(1/DT);
    init();
    while(ros::ok()){
        calc_dynamic_window();
        calc_trajectory();

        ros::spinOnce();
        loop_rate.sleep();
        // ROS_INFO_STREAM("loop time:"<<ros::Time::now().toSec()<<"[s]");
    }
}

void DynamicWindowApproach::visualize_traj(std::vector<State>& traj,const ros::Publisher& pub)
{
    nav_msgs::Path v_traj;
    v_traj.header.frame_id=ROBOT_FRAME;
    v_traj.header.stamp=ros::Time::now();

    for(auto& traj_state :traj){
        geometry_msgs::PoseStamped state;
        state.pose.position.x=traj_state.x;
        state.pose.position.y=traj_state.y;
        v_traj.poses.push_back(state);
    }
    pub.publish(v_traj);
}

// void DynamicWindowApproach::visualize_trajectries()
// {
// }

void DynamicWindowApproach::init()
{
    current_state.x=0.0;
    current_state.y=0.0;
    current_state.yaw=0.0;
    current_state.velocity=0.0;
    current_state.omega=0.0;

    local_goal.x=LINEAR_SPEED_MAX*PREDICT_TIME;
    local_goal.y=+5.0;
    local_goal.yaw=0.0;
    local_goal.velocity=0.0;
    local_goal.omega=0.0;

    obs_list.clear();
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"local_path_planner");
    DynamicWindowApproach dwa;
    dwa.process();
    return 0;
}
