#include "local_path_planner.h"

DynamicWindowApproach::DynamicWindowApproach():private_nh("~")
{
    //private_nh.param("",,{});

    //sub_;
}

void DynamicWindowApproach::scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
}

void DynamicWindowApproach::estimated_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
}

void DynamicWindowApproach::local_goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
}

// void DynamicWindowApproach::get_current_state()
// {
// }

// void DynamicWindowApproach::get_local_goal()
// {
// }
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

    return 0;
}

void DynamicWindowApproach::calc_trajectory()
{
    double resolution_velocity=(dw.vel_max-dw.vel_min)/RESOLUTION_VELOCITY_NUM;
    double resolution_omega=(dw.omega_max-dw.omega_min)/RESOLUTION_OMEGA_NUM;

    double cost_max=0;
    for(double v=dw.vel_min;v<=dw.vel_max;v+=resolution_velocity){
        for(double w=dw.omega_min;w<=dw.omega_max;w+=resolution_omega){
            State state=[0.0,0.0,0.0,current_state.velocity,current_state.omega];
            std::vector<State> traj;
            for(double t=0;t<=PREDICT_TIME;t+=DT){
                roomba_motion(state,v,w);
                traj.push_back(state);
            }
            trajectories.push_back(traj);

            double cost_heading=calc_cost_heading(traj.back());
            double cost_velocity=calc_cost_velocity(v);
            double cost_obstacle=calc_cost_obstacle(traj);
            double cost_sum=COST_HEADING_GAIN*cost_heading+COST_VELOCITY_GAIN*cost_velocity+COST_OBSTACLE_GAIN*cost_obstacle;

            if(cost_sum>cost_min){
                cost_max=cost_sum;
                best_traj=traj;
            }
        }
    }
    return 0;
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
    return 0;
}

double DynamicWindowApproach::calc_cost_heading(State traj_last_state)
{
    double angle_to_goal=atan2(local_goal.y-traj_last_state.y,local_goal.x-traj_last_state.x);
    double angle_diff=angle_to_goal-traj_last_state.yaw;
    if(angle_diff>M_PI) angle_diff-=2*M_PI;
    else if(angle_diff<-M_PI) angle_diff+=2*M_PI;
    angle_diff=std::abs(angle_diff);

    return std::max(angle_diff,0,0);
}

double DynamicWindowApproach::calc_cost_velocity(double velocity)
{
    return std::max(local_goal.velocity-velocity,0.0);
}

double DynamicWindowApproach::calc_cost_obstacle(std::vector<State> traj)
{
    double dist_min=1e3; //最も近くなる時の距離
    for(auto& state : traj){
        for(auto& obs : obs_list){
            double dist=sqrt(pow(state.x-obs[0],2)+pow(state.y-obs[1],2));
            if(dist<dist_min) dist_min=dist;
        }
    }
    return 1/dist_min; //距離が小さくなるほどコストが大きくなる
}

void DynamicWindowApproach::scan_to_obs()
{
    double angle=scan.angle_min;
    for(auto r : scan.ranges){
        double x=r*cos(angle);
        double y=r*sin(angle);
        std::vector<double> obs_state={x,y};
        obs_list.push_back(obs_state);
        angle+=scan.angle_increment;
}

void DynamicWindowApproach::process()
{
    calc_dynamic_window();
    calc_trajectory();
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"local_path_planner");
    DynamicWindowApproach dwa;
    dwa.process();
    return 0;
}
