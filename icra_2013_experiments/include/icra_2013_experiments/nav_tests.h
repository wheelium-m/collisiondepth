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
#include <icra_2013_experiments/arm.h>
#include <icra_2013_experiments/torso.h>
#include <sbpl_3dnav_planner/sbpl_3dnav_planner.h>

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
  std::string start;
  std::string config;
} Experiment;


class NavTests
{
  public:

    NavTests();
    ~NavTests();

    bool getParams();
    bool getLocations();
    bool getExperiments();
    bool getConfigurations();

    void printLocations();
    void printExperiments();

    bool runTests();
    bool goToRobotConfiguration(std::vector<double> &rangles, std::vector<double> &langles, double torso);

    void visualizeAllConfigurations();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    std::map<std::string, Experiment> exp_map_;
    std::map<std::string, RobotPose> config_map_;
    std::map<std::string, std::vector<double> > loc_map_;

    tf::TransformListener tf_;

    PViz pviz_;

    Arm *larm_;
    Arm *rarm_;
    Torso torso_;

    MoveBaseClient *mbc_;
    sbpl_3dnav_planner::Sbpl3DNavPlanner *planner_;

    bool callPlanner(std::string start, std::string goal);
    bool sendNavGoal(std::string name);
    void getBasePose(double &x, double &y, double &theta, geometry_msgs::Quaternion &quat);
    void rpyToQuat(double roll, double pitch, double yaw, double &qx, double &qy, double &qz, double &qw);
    void bodyPoseToPoseStamped(const BodyPose &bp, std::string frame_id, geometry_msgs::PoseStamped &ps);
    void printRobotPose(RobotPose &pose, std::string name);

};

