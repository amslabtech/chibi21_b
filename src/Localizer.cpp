#include<Localizer/Localizer.h>

Localizer::Localizer():private_nh("~")
{
    private_nh.getParam("hz", hz);
    private_nh.getParam("particle_number", particle_number);
    private_nh.getParam("init_x", init_x);
    private_nh.getParam("init_y", init_y);
    private_nh.getParam("init_yaw", init_yaw);
    private_nh.getParam("init_x_sigma", init_x_sigma);
    private_nh.getParam("init_y_sigma", init_y_sigma);
    private_nh.getParam("init_yaw_sigma", init_y_sigma);

    map_sub = nh.subscribe("/map", 100, &Localizer::map_callback, this);
    laser_sub = nh.subscribe("/scan", 100, &Localizer::laser_callback, this);
    odometry_sub = nh.subscribe("/roomba/odometry", 100, &Localizer::odometry_callback, this);

    mcl_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mcl_pose", 100);
    p_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/p_pose_array", 100);
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser = *msg;
}
void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;
    map_get_ok = true;
    ROS_INFO("map get");
    for(int i=0; i<particle_number; i++){
        Particle p = make_particle();
        ROS_INFO("make particle");
    }
    create_p_pose_array_from_p_array(p_array);
}

void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odometry = *msg;
}

Localizer::Particle Localizer::make_particle()
{
    Particle p(this);
    p_array.push_back(p);
    return p;
}

double Localizer::gaussian(double mu, double sigma)
{
    std::random_device seed;
    std::mt19937 engine(seed());
    std::normal_distribution<> dist(mu, sigma);
    return dist(engine);
}

void Localizer::create_p_pose_array_from_p_array(std::vector<Particle> p_array)
{
    p_pose_array.poses.clear();
    p_pose_array.header.frame_id = "map";
    for(int i=0; i < p_array.size(); i++){
        p_pose_array.poses.push_back(p_array.at(i).p_pose.pose);
    }
}

void Localizer::process()
{
    ros::Rate rate(hz);
    while(ros::ok()){

        p_pose_array_pub.publish(p_pose_array);
        ROS_INFO("publish p_pose_array");
        ros::spinOnce();
        rate.sleep();
    }
}
Localizer::Particle::Particle(Localizer* localizer):private_nh("~")
{
    mcl = localizer;
    p_pose.header.frame_id = "map";
    set_p(mcl->init_x, mcl->init_y, mcl->init_yaw, mcl->init_x_sigma, mcl->init_y_sigma, mcl->init_yaw_sigma);


}

int Localizer::Particle::p_pose_to_map_index(geometry_msgs::PoseStamped p_pose)
{
    double x = p_pose.pose.position.x;
    double y = p_pose.pose.position.y;
    int index_x = int((x - mcl->map.info.origin.position.x) / mcl->map.info.resolution);
    int index_y = int((y - mcl->map.info.origin.position.y) / mcl->map.info.resolution);
    return index_x + index_y * mcl->map.info.width;
}

void Localizer::Particle::set_p(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma)
{
    do{
        p_pose.pose.position.x = mcl->gaussian(x, x_sigma);
        p_pose.pose.position.y = mcl->gaussian(y, y_sigma);
        quaternionTFToMsg(tf::createQuaternionFromYaw(mcl->gaussian(yaw,yaw_sigma)),p_pose.pose.orientation);
    }while(mcl->map.data[p_pose_to_map_index(p_pose)] != 0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Localizer");
    Localizer localizer;
    localizer.process();
    return 0;
}
