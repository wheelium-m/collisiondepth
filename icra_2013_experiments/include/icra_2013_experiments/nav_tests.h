#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <iterator>
#include <list>
#include <map>
#include <tf/tf.h>
#include <pviz/pviz.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef struct
{
  std::vector<double> langles;
  std::vector<double> rangles;
  BodyPose body;
} RobotPose;

typedef struct
{
  std::string name;
  std::string goal;
  RobotPose start;
} Experiment;


class NavTests
{
  public:

    NavTests();
    ~NavTests(){};

    bool getParams();
    bool getLocations();
    bool getExperiments();
    
    void printLocations();
    void printExperiments();

    bool runTests();

    void rpyToQuat(double roll, double pitch, double yaw, double &qx, double &qy, double &qz, double &qw);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    std::map<std::string, Experiment> exp_map_;
    std::map<std::string, std::vector<double> > loc_map_;

    tf::TransformListener tf_;

    RobotPose start_pose_;
    RobotPose current_pose_;

    MoveBaseClient *mbc_;

    bool sendNavGoal(std::string name);
    void getBasePose(double &x, double &y, double &theta, geometry_msgs::Quaternion &quat);
  
};

