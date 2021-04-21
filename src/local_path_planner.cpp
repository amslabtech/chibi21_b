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
    private_nh.param("ROBOT_RADIUS",ROBOT_RADIUS,{0.5});
    private_nh.param("COST_HEADING_GAIN",COST_HEADING_GAIN,{1.0});
    private_nh.param("COST_VELOCITY_GAIN",COST_VELOCITY_GAIN,{1.0});
    private_nh.param("COST_OBSTACLE_GAIN",COST_OBSTACLE_GAIN,{1.0});
    private_nh.param("COST_HEADING_OBS_GAIN",COST_HEADING_OBS_GAIN,{1.0});
    private_nh.param("PREDICT_TIME",PREDICT_TIME,{3.0});
    private_nh.param("USE_DUMMY_TOPIC",USE_DUMMY_TOPIC,{false});
    private_nh.param("flag_scan",flag_scan,{false});
    private_nh.param("flag_mcl_pose",flag_mcl_pose,{false});
    private_nh.param("flag_local_goal",flag_local_goal,{false});
    private_nh.param("flag_odom",flag_odom,{false});

    sub_scan=nh.subscribe("/scan",100,&DynamicWindowApproach::scan_callback,this);
    sub_estimated_pose=nh.subscribe("/mcl_pose",100,&DynamicWindowApproach::estimated_pose_callback,this);
    sub_local_map=nh.subscribe("/local_goal",100,&DynamicWindowApproach::local_goal_callback,this);
    if(USE_DUMMY_TOPIC){sub_twist=nh.subscribe("/dummy_twist",100,&DynamicWindowApproach::twist_to_odometry_callback,this);
    }else{
        sub_odometry=nh.subscribe("/roomba/odometry",100,&DynamicWindowApproach::odometry_callback,this);
    }
    pub_roomba_ctrl=nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);
    pub_best_traj=nh.advertise<nav_msgs::Path>("best_traj",1);
    pub_trajectries=nh.advertise<nav_msgs::Path>("trajectories",1);
    pub_obs=nh.advertise<geometry_msgs::PointStamped>("obstacles",1);
    pub_obs_robot_frame=nh.advertise<geometry_msgs::PointStamped>("obstacles_robot_frame",1);
}

void DynamicWindowApproach::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    scan=*msg;
    flag_scan=true;
}

void DynamicWindowApproach::estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    estimated_pose=*msg;

    current_state.x=estimated_pose.pose.position.x;
    current_state.y=estimated_pose.pose.position.y;
    current_state.yaw=tf::getYaw(estimated_pose.pose.orientation);
    flag_mcl_pose=true;
}

void DynamicWindowApproach::local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped _local_goal=*msg;

    local_goal.x=_local_goal.pose.position.x;
    local_goal.y=_local_goal.pose.position.y;
    local_goal.yaw=tf::getYaw(_local_goal.pose.orientation);
    local_goal.velocity=LINEAR_SPEED_MAX;
    local_goal.omega=0.0;
    flag_local_goal=true;
}

void DynamicWindowApproach::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry=*msg;

    current_state.velocity=odometry.twist.twist.linear.x;
    current_state.omega=odometry.twist.twist.angular.z;
    flag_odom=true;
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

    double cost_min=1e9;
    // double cost_heading_min=1e6;
    // double cost_velocity_min=1e6;
    // double cost_obstacle_min=1e6;
    double best_velocity=0.0;
    double best_omega=0.0;

    int count=0;
    for(double v=dw.vel_max; v>=dw.vel_min; v-=resolution_velocity){
        for(double w=0.0; w<=dw.omega_max; w+=resolution_omega){
            for(double omega_direction=1.0; omega_direction>=-1.0; omega_direction-=2.0){
                State state;
                state.x=0.0;
                state.y=0.0;
                state.yaw=0.0;
                state.velocity=current_state.velocity;
                state.omega=current_state.omega;

                std::vector<State> traj;
                for(double t=0.0;t<=PREDICT_TIME;t+=DT){
                    roomba_motion(state,v,w*omega_direction,DT);
                    traj.push_back(state);
                }
                trajectories.push_back(traj);
                visualize_traj(trajectories[count],pub_trajectries);
                double cost_heading=calc_cost_heading(traj.back());
                double cost_velocity=calc_cost_velocity(v,w*omega_direction);
                double cost_obstacle=calc_cost_obstacle(traj);
                // double cost_obstacle_min=calc_cost_obstacle(traj);
                double cost_heading_obs=0.0;
                if(USE_DUMMY_TOPIC) cost_heading_obs=calc_cost_heading_obs(map_frame_obs_list,w*omega_direction);
                else{
                    scan_to_obs();
                    cost_heading_obs=calc_cost_heading_obs(obs_list,w*omega_direction);
                }
                double cost_sum=COST_HEADING_GAIN*cost_heading+COST_VELOCITY_GAIN*cost_velocity+COST_OBSTACLE_GAIN*cost_obstacle+COST_HEADING_OBS_GAIN*cost_heading_obs;

                if(cost_sum<=cost_min){
                    cost_min=cost_sum;
                    // cost_heading_min=cost_heading;
                    // cost_velocity_min=cost_velocity;
                    // cost_obstacle_min=cost_obstacle;
                    best_traj=traj;
                    best_velocity=v;
                    best_omega=w*omega_direction;
                    // ROS_INFO_STREAM("best_velocity = "<<best_velocity<<" || best_omega = "<<best_omega<<" || count = "<<count);
                }
                count++;
            }
        }
        if(count > RESOLUTION_VELOCITY_NUM*RESOLUTION_OMEGA_NUM*100) break;
    }
    // if(cost_obstacle_min>=1e3){
        // best_velocity=0.0;
        // best_omega=1.0;
    // }
    ROS_INFO_STREAM("best_velocity = "<<best_velocity<<" || best_omega = "<<best_omega<<" ||  count = "<<count);
    // ROS_INFO_STREAM("best_omega = "<<best_omega);

    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    cmd_vel.mode=11;
    cmd_vel.cntl.linear.x=best_velocity;
    cmd_vel.cntl.angular.z=best_omega;
    pub_roomba_ctrl.publish(cmd_vel);

    visualize_traj(best_traj,pub_best_traj);
    // ROS_INFO_STREAM(best_traj);
    // visualize_trajectries(trajectories);
}

void DynamicWindowApproach::roomba_motion(State& state, double velocity, double omega,double dt)
{
    state.yaw+=omega*dt;
    if(state.yaw>M_PI) state.yaw-=2*M_PI;
    else if(state.yaw<-1*M_PI) state.yaw+=2*M_PI;
    state.x+=velocity*std::cos(state.yaw)*dt;
    state.y+=velocity*std::sin(state.yaw)*dt;
    state.velocity=velocity;
    state.omega=omega;
}

double DynamicWindowApproach::calc_cost_heading(State& traj_last_state)
{
    double angle_to_goal=std::atan2((local_goal.y-current_state.y)-traj_last_state.y,(local_goal.x-current_state.x)-traj_last_state.x);
    if(angle_to_goal>M_PI) angle_to_goal-=2*M_PI;
    if(angle_to_goal<-M_PI) angle_to_goal+=2*M_PI;
    // double angle_to_goal=std::atan2((local_goal.x-current_state.x)-traj_last_state.x,(local_goal.y-current_state.y)-traj_last_state.y);
    double angle_diff=angle_to_goal-(current_state.yaw+traj_last_state.yaw);
    if(angle_diff>M_PI) angle_diff-=2*M_PI;
    if(angle_diff<-M_PI) angle_diff+=2*M_PI;

    // ROS_INFO_STREAM("local_goal.x = "<<local_goal.x<<" || local_goal.y = "<<local_goal.y);
    // ROS_INFO_STREAM("traj_last_state.x = "<<traj_last_state.x<<" || traj_last_state.y = "<<traj_last_state.y);
    // ROS_INFO_STREAM("angle_diff = "<<angle_diff<<" || angle_to_goal = "<<angle_to_goal<<" || current_state.yaw = "<<current_state.yaw);

    return std::max(std::abs(angle_diff),0.0)/M_PI;
}

double DynamicWindowApproach::calc_cost_velocity(double velocity,double omega)
{
    return std::max(LINEAR_SPEED_MAX-velocity,0.0)/LINEAR_SPEED_MAX;
}

double DynamicWindowApproach::calc_cost_obstacle(std::vector<State>& traj)
{
    double dist_min=1e3; //最も近くなる時の距離
    // State state=traj.back();
    for(auto& state : traj){
        if(USE_DUMMY_TOPIC){
            for(auto& obs : map_frame_obs_list){
                // double dist=std::sqrt(std::pow(state.x-(std::cos(state.yaw+current_state.yaw)*(obs[0]-state.x)+std::sin(state.yaw+current_state.yaw)*(obs[1]-state.y)),2)+std::pow(state.y-(-1*std::sin(state.yaw+current_state.yaw)*(obs[0]-state.x)+std::cos(state.yaw+current_state.yaw)*(obs[1]-state.y)),2));
                std::vector<double> obs_position_from_state={0.0,0.0};
                map_to_robot_frame(obs[0]-(current_state.x), obs[1]-(current_state.y), current_state.yaw+state.yaw, obs_position_from_state);
                double dist=std::sqrt(std::pow(state.x-obs_position_from_state[0],2)+std::pow(state.y-obs_position_from_state[1],2));
                // ROS_INFO_STREAM("obs[0] = "<<obs[0]<<" || obs[1] = "<<obs[1]<<" || dist = "<<dist);
                if(dist<dist_min) {
                    dist_min=dist;
                }
                if(dist_min<ROBOT_RADIUS){
                    // return 1e3;
                    // ROS_INFO_STREAM("dist_min < ROBOT_RADIUS!!!");
                }
                visualize_obs(obs[0],obs[1],pub_obs);
                visualize_obs(obs_position_from_state[0],obs_position_from_state[1],pub_obs_robot_frame);


            }
        }else{
            for(auto& obs : obs_list){
                double dist=std::sqrt(pow(state.x-obs[0],2)+pow(state.y-obs[1],2));
                if(dist<dist_min) dist_min=dist;
            }
        }
    }
    // ROS_INFO_STREAM("1/dist_min = "<<std::max(1.0/dist_min,0.0));
    //
    // std::vector<double> position={0.0,0.0};
    // map_to_robot_frame(1.0,std::sqrt(3.0),M_PI/6.0,position);
    // ROS_INFO_STREAM("position[0] = "<<position[0]<<" || position[1] = "<<position[1]);
    return std::max(ROBOT_RADIUS/dist_min,0.0); //距離が小さくなるほどコストが大きくなる
}

double DynamicWindowApproach::calc_cost_heading_obs(std::vector<std::vector<double>>& obstacles,double omega)
{
    double dist_min=1e3;
    int nearest_obs_num=0;
    int count=0;
    for(auto& obs:obstacles){
        double dist=std::sqrt(std::pow(current_state.x-obs[0],2)+pow(current_state.y-obs[1],2));
        if(dist_min>dist){
            dist_min=dist;
            nearest_obs_num=count;
        }
        count++;
    }

    double angle_to_nearest_obs=std::atan2(obstacles[nearest_obs_num][1]-current_state.y,obstacles[nearest_obs_num][0]-current_state.x);
    if(angle_to_nearest_obs>M_PI) angle_to_nearest_obs-=2*M_PI;
    if(angle_to_nearest_obs<-M_PI) angle_to_nearest_obs+=2*M_PI;
    double angle_diff=angle_to_nearest_obs-current_state.yaw-omega*DT;
    // if(angle_diff>M_PI) angle_diff-=2*M_PI;
    // if(angle_diff<-M_PI) angle_diff+=2*M_PI;
    // ROS_INFO_STREAM("nearest_obs_num = "<<nearest_obs_num<<" || angle_to_nearest_obs = "<<angle_to_nearest_obs);
    if(angle_diff==0.0)return 1e2;
    else return 1.0/std::abs(angle_diff);
}

void DynamicWindowApproach::map_to_robot_frame(double position_map_x, double position_map_y, double yaw, std::vector<double>& position_robot)
{
    if(yaw>M_PI) yaw-=2*M_PI;
    if(yaw<M_PI) yaw+=2*M_PI;
    position_robot[0]=std::cos(yaw)*position_map_x+std::sin(yaw)*position_map_y;
    position_robot[1]=-1*std::sin(yaw)*position_map_x+std::cos(yaw)*position_map_y;
}

void DynamicWindowApproach::scan_to_obs()
{
    if(!USE_DUMMY_TOPIC) obs_list.clear();
    double angle=scan.angle_min;
    int count=0;
    for(auto r : scan.ranges){
        if((r<0.2)||(10<=count&&count<=70)||(count<=290&&count<=350)||(730<=count&&count<=790)||(1010<=count&&count<=1070)){
            double x=r*std::cos(scan.angle_min+scan.angle_increment*count);
            double y=r*std::sin(scan.angle_min+scan.angle_increment*count);
            std::vector<double> obs_state={x,y};
            obs_list.push_back(obs_state);
            visualize_obs(x,y,pub_obs);
            // angle+=scan.angle_increment;
        }
        count++;
    }
}

void DynamicWindowApproach::process()
{
    ros::Rate loop_rate(1/DT);
    init();
    while(ros::ok()){
            // ROS_INFO_STREAM("scan = "<<flag_scan<<" | goal = "<<flag_local_goal<<" | mcl = "<<flag_mcl_pose<<" | odom= "<<flag_odom);
        if(flag_scan&&flag_local_goal&&flag_mcl_pose&&flag_odom){
        // ROS_INFO_STREAM("current_state.x = "<<current_state.x<<" || current_state.y = "<<current_state.y<<" || current_state.yaw = "<<current_state.yaw);
            calc_dynamic_window();
            calc_trajectory();
        }
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

void DynamicWindowApproach::visualize_obs(double x,double y,const ros::Publisher& pub)
{
    geometry_msgs::PointStamped v_obs;
    v_obs.header.stamp=ros::Time::now();
    v_obs.header.frame_id="map";

    v_obs.point.x=x;
    v_obs.point.y=y;
    v_obs.point.z=0.0;

    pub.publish(v_obs);
}


void DynamicWindowApproach::init()
{
    current_state.x=0.0;
    current_state.y=0.0;
    current_state.yaw=0.0;
    current_state.velocity=0.0;
    current_state.omega=0.0;

    local_goal.x=LINEAR_SPEED_MAX*PREDICT_TIME;
    local_goal.y=5.0;
    local_goal.yaw=0.0;
    local_goal.velocity=0.0;
    local_goal.omega=0.0;

    obs_list.clear();
    map_frame_obs_list={
        {5.0,-5.0},
        // {5.0,-4.5},
        // {5.0,-4.0},
        // {5.0,-3.5},
        // {5.0,-3.0},
        // {5.0,-2.5},
        // {5.0,-2.0},
        // {5.0,-1.5},
        // {5.0,-1.0},
        // {5.0,0.0},
        // {5.0,1.0},
        // {5.0,2.0},
        // {5.0,3.0},
        // {5.0,4.0},
        // {5.0,5.0}
    };
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"local_path_planner");
    DynamicWindowApproach dwa;
    dwa.process();
    return 0;
}
