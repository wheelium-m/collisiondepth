#include <icra_2013_experiments/nav_tests.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "nav_tests", ros::init_options::AnonymousName);
  NavTests nt;

  if(!nt.getParams())
  {
    ROS_ERROR("Failed to get all required params from param server.");
    return -1;
  }
  nt.printLocations();
  nt.printExperiments();
  sleep(2);

  if(!nt.runTests())
  {
    ROS_ERROR("Failed to run all of the tests.");
    return -1;
  }

  return 0;
}


