#include <icra_2013_experiments/nav_tests.h>


NavTests::NavTests() : ph_("~")
{
  mbc_ = new MoveBaseClient("move_base", true);

  while(!mbc_->waitForServer(ros::Duration(5.0)))
    ROS_INFO("[exp] Waiting for the move_base action server to come up.");

  larm_ = new Arm("left");
  rarm_ = new Arm("right");
}

NavTests::~NavTests()
{
  if(mbc_)
    delete mbc_;
  if(larm_)
    delete larm_;
  if(rarm_)
    delete rarm_;
}

bool NavTests::getParams()
{
  if(!getLocations() || !getExperiments() || !getConfigurations())
    return false;
  return true;
}

bool NavTests::getLocations()
{
  XmlRpc::XmlRpcValue loc_list;
  geometry_msgs::Pose p;
  std::string name;
  std::string loc_name = "locations";

  if(!ph_.hasParam(loc_name))
  {
    ROS_WARN("[exp] No list of locations found on the param server.");
    return false;
  }
  ph_.getParam(loc_name, loc_list);

  if(loc_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("[exp] Location list is not an array. Something is wrong...exiting.");
    return false;
  }

  if(loc_list.size() == 0)
  {
    ROS_ERROR("[exp] List of locations is empty.");
    return false;
  }

  for(int i = 0; i < loc_list.size(); ++i)
  {
    if(!loc_list[i].hasMember("name"))
    {
      ROS_ERROR("Each location must have a name.");
      return false;
    }
    name = std::string(loc_list[i]["name"]);
    std::stringstream ss(loc_list[i]["pose"]);
    std::string p;
    while(ss >> p)
      loc_map_[name].push_back(atof(p.c_str()));
  }

  ROS_INFO("[exp] Successfully fetched %d locations from param server.", int(loc_list.size()));
  return true;
}

bool NavTests::getConfigurations()
{
  XmlRpc::XmlRpcValue config_list;
  geometry_msgs::Pose p;
  std::string name;
  std::string loc_name = "configurations";

  if(!ph_.hasParam(loc_name))
  {
    ROS_WARN("[exp] No list of robot configurations found on the param server.");
    return false;
  }
  ph_.getParam(loc_name, config_list);

  if(config_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("[exp] Configuration list is not an array. Something is wrong...exiting.");
    return false;
  }

  if(config_list.size() == 0)
  {
    ROS_ERROR("[exp] List of configurations is empty.");
    return false;
  }

  for(int i = 0; i < config_list.size(); ++i)
  {
    if(!config_list[i].hasMember("name"))
    {
      ROS_ERROR("[exp] Each robot configuration must have a name.");
      return false;
    }
    name = std::string(config_list[i]["name"]);
    std::stringstream ss(config_list[i]["rangles"]);
    std::string p;
    config_map_[name].rangles.clear();
    config_map_[name].langles.clear();
    while(ss >> p)
      config_map_[name].rangles.push_back(atof(p.c_str()));
    std::stringstream ss1(config_list[i]["langles"]);
    while(ss >> p)
      config_map_[name].langles.push_back(atof(p.c_str()));
    config_map_[name].body.z = double(config_list[i]["torso"]);
  }

  ROS_INFO("[exp] Successfully fetched %d locations from param server.", int(config_list.size()));
  return true;
}

bool NavTests::getExperiments()
{
  XmlRpc::XmlRpcValue exp_list;
  Experiment e;
  std::string exp_name = "experiments";
  XmlRpc::XmlRpcValue plist;
  std::string p;

  if(!ph_.hasParam(exp_name))
  {
    ROS_WARN("No list of experiments found on the param server.");
    return false;
  }
  ph_.getParam(exp_name, exp_list);

  if(exp_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("Experiment list is not an array. Something is wrong...exiting.");
    return false;
  }

  if(exp_list.size() == 0)
  {
    ROS_ERROR("List of experiments is empty.");
    return false;
  }

  for(int i = 0; i < exp_list.size(); ++i)
  {
    if(!exp_list[i].hasMember("name"))
    {
      ROS_ERROR("Each experiment must have a name.");
      return false;
    }
    e.name = std::string(exp_list[i]["name"]);

    if(!exp_list[i].hasMember("goal"))
    {
      ROS_ERROR("Each experiment must have a goal....duh.");
      return false;
    }
    e.goal = std::string(exp_list[i]["goal"]);
    e.start = std::string(exp_list[i]["start"]);
/*
    if(exp_list[i].hasMember("start"))
    {
      e.start.rangles.clear();
      e.start.langles.clear();
      std::vector<double> bpose;
      plist = exp_list[i]["start"]["right"];
      std::stringstream ss(plist);
      while(ss >> p)
        e.start.rangles.push_back(atof(p.c_str()));

      plist = exp_list[i]["start"]["left"];
      std::stringstream ss1(plist);
      while(ss1 >> p)
        e.start.langles.push_back(atof(p.c_str()));
    
      e.start.body.z = double(exp_list[i]["start"]["spine"]);
    }
*/
    ROS_INFO("Adding experiment: %s", e.name.c_str());
    exp_map_[e.name] = e;
  }

  return true;
}

void NavTests::printLocations()
{
  if(loc_map_.begin() == loc_map_.end())
  {
    ROS_ERROR("[exp] No locations found.");
    return;
  }
  for(std::map<std::string,std::vector<double> >::const_iterator iter = loc_map_.begin(); iter != loc_map_.end(); ++iter)
  {
    ROS_INFO("name: %s", iter->first.c_str());
    ROS_INFO("  x: % 0.3f  y: % 0.3f  theta: %0.3f", iter->second.at(0), iter->second.at(1), iter->second.at(2));
  }
}

void NavTests::printExperiments()
{
  if(exp_map_.begin() == exp_map_.end())
  {
    ROS_ERROR("[exp] No experiments found.");
    return;
  }
  for(std::map<std::string,Experiment>::iterator iter = exp_map_.begin(); iter != exp_map_.end(); ++iter)
  {
    int p = std::distance(exp_map_.begin(), iter);
    ROS_INFO("------------------------------");
    ROS_INFO("[%d]  name: %s", p, iter->second.name.c_str());
    ROS_INFO("[%d]  goal: %s", p, iter->second.goal.c_str());
    ROS_INFO("[%d] start: %s", p, iter->second.start.c_str());
    /*
    ROS_INFO("[%d]   right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", p, iter->second.start.rangles[0], iter->second.start.rangles[1], iter->second.start.rangles[2], iter->second.start.rangles[3], iter->second.start.rangles[4], iter->second.start.rangles[5], iter->second.start.rangles[6]);
    ROS_INFO("[%d]    left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", p, iter->second.start.langles[0], iter->second.start.langles[1], iter->second.start.langles[2], iter->second.start.langles[3], iter->second.start.langles[4], iter->second.start.langles[5], iter->second.start.langles[6]);
    ROS_INFO("[%d]   torso: % 0.3f", p, iter->second.start.body.z);
    */
  }
}

bool NavTests::sendNavGoal(std::string name)
{
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = loc_map_[exp_map_[name].goal].at(0);
  goal.target_pose.pose.position.y = loc_map_[exp_map_[name].goal].at(1);
  goal.target_pose.pose.position.z = 0;
  rpyToQuat(0, 0, loc_map_[exp_map_[name].goal].at(2), goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);

  ROS_INFO("[exp] Sending goal");
  mbc_->sendGoal(goal);

  mbc_->waitForResult();

  if(mbc_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("[exp] Achieved goal %s.", name.c_str());
  else
  {
    ROS_INFO("[exp] Failed to plan to %s", name.c_str());
    return false;
  }
  return true;
}

void NavTests::rpyToQuat(double roll, double pitch, double yaw, double &qx, double &qy, double &qz, double &qw)
{
  tf::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  qx = quat.getX();
  qy = quat.getY();
  qz = quat.getZ();
  qw = quat.getW();
}

bool NavTests::runTests()
{
  geometry_msgs::Quaternion quat;
  double bx, by, btheta;

  for(std::map<std::string,Experiment>::iterator iter = exp_map_.begin(); iter != exp_map_.end(); ++iter)
  {
    printf("************** %s **************\n", iter->first.c_str());
    
    getBasePose(bx, by, btheta, quat);
    ROS_INFO("[exp] start_base: %0.3f %0.3f %0.3frad", bx, by, btheta);
    ROS_INFO("[exp] Going to start configuration.");
    config_map_[iter->second.start].body.x = bx;
    config_map_[iter->second.start].body.y = by;
    config_map_[iter->second.start].body.theta = btheta;
    printRobotPose(config_map_[iter->second.start], "start");
    if(!goToRobotConfiguration(config_map_[iter->second.start].rangles, config_map_[iter->second.start].langles, config_map_[iter->second.start].body.z))
      return false;
    sleep(2);

    ROS_INFO("[exp] Sending goal...");
    if(!sendNavGoal(iter->first))
      return false;
  }
  return true;
}

void NavTests::getBasePose(double &x, double &y, double &theta, geometry_msgs::Quaternion &quat)
{
  tf::StampedTransform transf;

  try
  {
    tf_.lookupTransform("map", "base_footprint", ros::Time(0), transf);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("***********  Is there a map? The map-robot transform failed. (%s)", ex.what());
  }

  x = transf.getOrigin().x();
  y = transf.getOrigin().y();
  theta = tf::getYaw(transf.getRotation());
  tf::quaternionTFToMsg(transf.getRotation(), quat);
}

bool NavTests::goToRobotConfiguration(std::vector<double> &rangles, std::vector<double> &langles, double torso)
{
  if(langles.size() < 7 || rangles.size() < 7)
  {
    ROS_ERROR("[exp] Failed to go to robot configuration. Arm angles are incomplete. (left: %d  right: %d)", int(langles.size()), int(rangles.size()));
    return false;
  }

  larm_->sendArmToConfiguration(langles, 1.0);  
  rarm_->sendArmToConfiguration(rangles, 1.0);
  return true;
}

void NavTests::printRobotPose(RobotPose &pose, std::string name)
{
  if(pose.rangles.size() < 7 || pose.langles.size() < 7)
  {
    ROS_ERROR("[exp] Trying to print RobotPose but size of rangles = %d and size of langles = %d.", int(pose.rangles.size()), int(pose.langles.size()));
    return;
  }
  ROS_INFO("[%s] right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", name.c_str(), pose.rangles[0], pose.rangles[1], pose.rangles[2], pose.rangles[3], pose.rangles[4], pose.rangles[5], pose.rangles[6]);
  ROS_INFO("[%s]  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", name.c_str(), pose.langles[0], pose.langles[1], pose.langles[2], pose.langles[3], pose.langles[4], pose.langles[5], pose.langles[6]);
  ROS_INFO("[%s]  base: % 0.3f % 0.3f % 0.3f", name.c_str(), pose.body.x, pose.body.y, pose.body.theta);
  ROS_INFO("[%s] torso: % 0.3f", name.c_str(), pose.body.z);
}


