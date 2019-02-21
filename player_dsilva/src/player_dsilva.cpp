#include <ros/ros.h>
#include <iostream>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "player_dsilva");

  ros::NodeHandle nh;

  for (int i = 0; i < 10; i++)
  {
    std::cout << i << std::endl;
  }
  return 0;
}
