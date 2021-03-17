#include<Localizer/Localizer.h>

Localizer::Localizer():private_nh("~")
{
    map_sub = nh.subscribe("/map", 100, &Localizer::map_callback, this);
    laser_sub = nh.subscribe("/scan", 100, &Localizer::laser_callback, this);
    odometry_sub = nh.subscribe("/roomba/odometry", 100, &Localizer::odometry_callback, this);

    mcl_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mcl_pose", 100);
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser = *msg;
}
void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;
}

void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odometry = *msg;
}
void Localizer::process()
{
    ;
}
Localizer::Particle::Particle()
{
    ;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Localizer");
    Localizer localizer;
    localizer.process();
    return 0;
}
