#include<Localizer/Localizer.h>
std::random_device seed;
Localizer::Localizer():private_nh("~")
{
    private_nh.getParam("hz", hz);
    private_nh.getParam("particle_number", particle_number);
    private_nh.getParam("init_x", init_x);
    private_nh.getParam("init_y", init_y);
    private_nh.getParam("init_yaw", init_yaw);
    private_nh.getParam("init_x_sigma", init_x_sigma);
    private_nh.getParam("init_y_sigma", init_y_sigma);
    private_nh.getParam("init_yaw_sigma", init_yaw_sigma);
    private_nh.getParam("move_noise_ratio", move_noise_ratio);
    private_nh.getParam("search_range", search_range);
    private_nh.getParam("laser_noise", laser_noise);
    private_nh.getParam("step_count", step_count);
    private_nh.getParam("alpha_slow_th", alpha_slow_th);
    private_nh.getParam("alpha_fast_th", alpha_fast_th);
    private_nh.getParam("reset_x_sigma", reset_x_sigma);
    private_nh.getParam("reset_y_sigma", reset_y_sigma);
    private_nh.getParam("reset_yaw_sigma", reset_yaw_sigma);


    map_sub = nh.subscribe("/map", 100, &Localizer::map_callback, this);
    laser_sub = nh.subscribe("/scan", 100, &Localizer::laser_callback, this);
    odometry_sub = nh.subscribe("/roomba/odometry", 100, &Localizer::odometry_callback, this);

    mcl_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mcl_pose", 100);
    p_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/p_pose_array", 100);

    mcl_pose.pose.position.x = 0.0;
    mcl_pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), mcl_pose.pose.orientation);
    mcl_pose.header.frame_id = "map";
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser = *msg;
    if(map_get_ok){
        observation_update();
    }

}
void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;
    map_get_ok = true;
    ROS_INFO("map get");
    for(int i=0; i<particle_number; i++){
        Particle p = make_particle();
    }
    ROS_INFO("make particles");
    //create_p_pose_array_from_p_array(p_array);
}

void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(map_get_ok){
        previous_odometry = current_odometry;
        current_odometry = *msg;
        if(!odometry_get_ok){previous_odometry = current_odometry;}
        motion_update();
        estimate_pose();
        odometry_get_ok = true;
    }
}

Localizer::Particle Localizer::make_particle()
{
    Particle p(this);
    p_array.push_back(p);
    return p;
}

double Localizer::gaussian(double mu, double sigma)
{
    std::mt19937 engine(seed());
    std::normal_distribution<> dist(mu, sigma);
    return dist(engine);
}
double Localizer::gaussian(double mu, double sigma, double x)
{
    double ans = exp(- std::pow((mu - x), 2) / std::pow(sigma, 2) / 2.0) / sqrt(2.0 * M_PI * std::pow(sigma, 2));
    return ans;
}


void Localizer::create_p_pose_array_from_p_array(std::vector<Particle> p_array)
{
    p_pose_array.poses.clear();
    p_pose_array.header.frame_id = "map";
    for(int i=0; i < p_array.size(); i++){
        p_pose_array.poses.push_back(p_array.at(i).p_pose.pose);
    }
}

void Localizer::motion_update()
{
    double dx = current_odometry.pose.pose.position.x - previous_odometry.pose.pose.position.x;
    double dy = current_odometry.pose.pose.position.y - previous_odometry.pose.pose.position.y;
    double current_yaw = create_yaw_from_msg(current_odometry.pose.pose.orientation);
    double previous_yaw = create_yaw_from_msg(previous_odometry.pose.pose.orientation);
    double dyaw = substract_yawA_from_yawB(previous_yaw, current_yaw);
    double dtrans = sqrt(dx*dx + dy*dy);
    double drot1 = adjust_yaw(atan2(dy, dx) - previous_yaw);
    double drot2 = adjust_yaw(dyaw - drot1);


    for(int i=0; i<p_array.size(); i++){
        p_array.at(i).p_move(dtrans, drot1, drot2);
    }
}

int Localizer::xy_to_map_index(double x, double y)
{
    int index_x = int((x - map.info.origin.position.x) / map.info.resolution);
    int index_y = int((y - map.info.origin.position.y) / map.info.resolution);
    return index_x + index_y * map.info.width;
}

bool Localizer::map_range_check(double x, double y)
{
    double x_first = map.info.origin.position.x;
    double y_first = map.info.origin.position.y;
    double x_last = x_first + map.info.width * map.info.resolution;
    double y_last = y_first + map.info.height * map.info.resolution;

    if(x_first <= x && x <= x_last && y_first <= y && y <= y_last){return true;}
    else{return false;}
}

double Localizer::dist_from_p_to_wall(double x, double y, double yaw)
{
    double x_start = x;
    double y_start = y;
    double search_step = map.info.resolution;
    double dist_from_start = search_step;

    for(dist_from_start; dist_from_start <= search_range; dist_from_start += search_step){
        double x_now = x_start + dist_from_start * cos(yaw);
        double y_now = y_start + dist_from_start * sin(yaw);
        bool flag = map_range_check(x_now, y_now);
        if(flag){
            int map_index = xy_to_map_index(x_now, y_now);
            if(map.data[map_index] == 100){
                return dist_from_start;
            }
            if(map.data[map_index] == -1){
                return search_range;
            }
        }
        else{
            return search_range;
        }
    }
    return search_range;
}

void Localizer::normalize_w()
{
    alpha = 0;
    for(int i=0; i<p_array.size(); i++){
        alpha += p_array.at(i).w;
    }
    for(int i=0; i<p_array.size(); i++){
        p_array.at(i).w /= alpha;\
    }
}

void Localizer::resampling_particle()
{
    normalize_w();
    std::default_random_engine engine(seed());
    std::uniform_real_distribution<> random(0.0, 1.0);

    double r = random(engine);
    int index = 0;
    double reset_ratio = 1 - (alpha_fast / alpha_slow);
    //  std::cout << "reset_ratio " << reset_ratio << std::endl;
    std::vector<Particle> p_array_after_resampling;
    while(p_array_after_resampling.size() < p_array.size()){
        r += random(engine) / p_array.size();
        if(random(engine) > reset_ratio){
            //r += random(engine) / p_array.size();
            while(r > p_array.at(index).w){
                r -= p_array.at(index).w;
                index = (index + 1) % p_array.size();
            }
            p_array_after_resampling.push_back(p_array.at(index));
            p_array_after_resampling.back().w = 1.0 / p_array.size();
        }
        else{
            Particle p = make_particle();
            p.set_p(mcl_pose.pose.position.x, mcl_pose.pose.position.y, create_yaw_from_msg(mcl_pose.pose.orientation), reset_x_sigma, reset_y_sigma, reset_yaw_sigma);
            p.w = 1.0 / (double)p_array.size();
            p_array_after_resampling.push_back(p);
        }
    }
    p_array = p_array_after_resampling;
}
void Localizer::observation_update()
{
    for(int i=0; i<p_array.size(); i++){
        double w = p_array.at(i).calc_w();
    }
    estimate_pose();
    if(alpha_slow == 0){
        alpha_slow = alpha;
    }
    else{
        alpha_slow += alpha_slow_th * (alpha - alpha_slow);
    }
    if(alpha_fast == 0){
        alpha_fast = alpha;
    }
    else{
        alpha_fast += alpha_fast_th * (alpha - alpha_fast);
    }
    /*
    for(int i=0; i<p_array.size(); i++){
        std::cout << "p[" << i << "].w" << p_array.at(i).w << std::endl;
    }
    */
    resampling_particle();

}

void Localizer::estimate_pose()
{
    normalize_w();
    double x = 0;
    double y = 0;
    double yaw = 0;
    for(int i=0; i<p_array.size(); i++){
        x += p_array.at(i).p_pose.pose.position.x * p_array.at(i).w;
        y += p_array.at(i).p_pose.pose.position.y * p_array.at(i).w;
        yaw += create_yaw_from_msg(p_array.at(i).p_pose.pose.orientation) * p_array.at(i).w;
    }
    yaw = adjust_yaw(yaw);

    mcl_pose.pose.position.x = x;
    mcl_pose.pose.position.y = y;
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), mcl_pose.pose.orientation);
}
void Localizer::process()
{
    ros::Rate rate(hz);
    while(ros::ok()){
        create_p_pose_array_from_p_array(p_array);
        p_pose_array_pub.publish(p_pose_array);
        mcl_pose_pub.publish(mcl_pose);
        //ROS_INFO("publish p_pose_array");
        ros::spinOnce();
        rate.sleep();
    }
}
Localizer::Particle::Particle(Localizer* localizer):private_nh("~")
{
    mcl = localizer;
    p_pose.header.frame_id = "map";
    set_p(mcl->init_x, mcl->init_y, mcl->init_yaw, mcl->init_x_sigma, mcl->init_y_sigma, mcl->init_yaw_sigma);
    w = 1.0 / (double)mcl->particle_number;
}

void Localizer::Particle::set_p(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma)
{
    do{
        p_pose.pose.position.x = mcl->gaussian(x, x_sigma);
        p_pose.pose.position.y = mcl->gaussian(y, y_sigma);
        quaternionTFToMsg(tf::createQuaternionFromYaw(mcl->gaussian(yaw,yaw_sigma)),p_pose.pose.orientation);
    }while(mcl->map.data[mcl->xy_to_map_index(p_pose.pose.position.x, p_pose.pose.position.y)] != 0);
}

double Localizer::create_yaw_from_msg(geometry_msgs::Quaternion q)
{
    double roll, pitch, yaw;
    tf::Quaternion quaternion(q.x, q.y, q.z, q.w);
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    return yaw;
}

double Localizer::substract_yawA_from_yawB(double yawA, double yawB)
{
    double dyaw = yawB - yawA;
    dyaw = adjust_yaw(dyaw);

    return dyaw;
}

void Localizer::Particle::p_move(double dtrans, double drot1, double drot2)
{
    dtrans += mcl->gaussian(0.0, dtrans * mcl->move_noise_ratio);
    drot1 += mcl->gaussian(0.0, drot1 * mcl->move_noise_ratio);
    drot2 += mcl->gaussian(0.0, drot2 * mcl->move_noise_ratio);

    double yaw = mcl->create_yaw_from_msg(p_pose.pose.orientation);
    p_pose.pose.position.x += dtrans * cos(mcl->adjust_yaw(yaw + drot1));
    p_pose.pose.position.y += dtrans * sin(mcl->adjust_yaw(yaw + drot1));
    quaternionTFToMsg(tf::createQuaternionFromYaw(mcl->adjust_yaw(yaw + drot1 + drot2)), p_pose.pose.orientation);

}

double Localizer::Particle::calc_w()
{
    for(int i=0; i<mcl->laser.ranges.size(); i+=mcl->step_count){
        double angle = i * mcl->laser.angle_increment + mcl->laser.angle_min;
        double dist_to_wall = mcl->dist_from_p_to_wall(p_pose.pose.position.x, p_pose.pose.position.y, mcl->create_yaw_from_msg(p_pose.pose.orientation) + angle);
        w += mcl->gaussian(mcl->laser.ranges[i], mcl->laser_noise, dist_to_wall);
    }
    return w;
}

double  Localizer::adjust_yaw(double yaw)
{
    if(yaw > M_PI){yaw -= 2*M_PI;}
    if(yaw < -M_PI){yaw += 2*M_PI;}

    return yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Localizer");
    Localizer localizer;
    localizer.process();
    return 0;
}
