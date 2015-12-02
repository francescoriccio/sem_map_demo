#include "dotDetector.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "DotDetector");
    dotDetector dd;
    dd.start();

    return 0;
}
