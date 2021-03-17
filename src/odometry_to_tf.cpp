#include<Localizer/Localizer.h>

int main()
{
    ros::init(argc, argv, "localizer");
    Particle localizer;
    localizer.process();
    return 0;
}
