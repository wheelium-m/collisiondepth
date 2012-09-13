#include <simplecontrollers/arm.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <time.h>

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

Arm::Arm(std::string arm_name) {
  reference_frame_ = "base_footprint";

  arm_name_ = arm_name;

  if (arm_name_.compare("left") == 0) {
    ik_service_name_ = "/pr2_left_arm_kinematics_sushi/get_ik";
    constraint_ik_service_name_ = "/pr2_left_arm_kinematics/get_constraint_aware_ik";
    joint_names_.push_back("l_shoulder_pan_joint");
    joint_names_.push_back("l_shoulder_lift_joint");
    joint_names_.push_back("l_upper_arm_roll_joint");
    joint_names_.push_back("l_elbow_flex_joint");
    joint_names_.push_back("l_forearm_roll_joint");
    joint_names_.push_back("l_wrist_flex_joint");
    joint_names_.push_back("l_wrist_roll_joint");
    traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action",
        true);
  } else {
    ik_service_name_ = "/pr2_right_arm_kinematics_sushi/get_ik";
    constraint_ik_service_name_ = "/pr2_right_arm_kinematics/get_constraint_aware_ik";
    joint_names_.push_back("r_shoulder_pan_joint");
    joint_names_.push_back("r_shoulder_lift_joint");
    joint_names_.push_back("r_upper_arm_roll_joint");
    joint_names_.push_back("r_elbow_flex_joint");
    joint_names_.push_back("r_forearm_roll_joint");
    joint_names_.push_back("r_wrist_flex_joint");
    joint_names_.push_back("r_wrist_roll_joint");
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action",
        true);
  }

  while (!traj_client_->waitForServer(ros::Duration(5.0)))
    ROS_INFO("Waiting for the joint_trajectory_action server");

  ROS_INFO("[arm] Initialized.");
}

Arm::~Arm() {
  delete traj_client_;
}

void Arm::sendArmToConfiguration(double configuration[7], double move_time) {
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.header.seq = 0;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.header.frame_id = reference_frame_;

  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(7);

  for (unsigned int i = 0; i < 7; i++) {
    goal.trajectory.points[0].positions[i] = configuration[i];
    goal.trajectory.joint_names.push_back(joint_names_[i]);
  }

  goal.trajectory.points[0].velocities.resize(7);
//  for (size_t j = 0; j < 7; ++j)
//    goal.trajectory.points[0].velocities[j] = 1;

  goal.trajectory.points[0].time_from_start = ros::Duration(move_time);

  double start_time = ros::Time::now().toSec();
  traj_client_->sendGoal(goal);
  ROS_DEBUG(
      "sending goal to controller took %f seconds (no feedback here)", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}

void Arm::sendArmToConfigurations(
    std::vector<std::vector<double> > &configurations,
    std::vector<double> move_times) {
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.header.seq = 0;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.header.frame_id = reference_frame_;

  goal.trajectory.points.resize(configurations.size());

  for (unsigned int i = 0; i < 7; i++)
    goal.trajectory.joint_names.push_back(joint_names_[i]);

  for (unsigned int i = 0; i < configurations.size(); i++) {
    goal.trajectory.points[i].time_from_start = ros::Duration(move_times[i]);

    goal.trajectory.points[i].positions.resize(7);
    for (unsigned int j = 0; j < 7; j++) {
      goal.trajectory.points[i].positions[j] = configurations[i][j];
    }

    goal.trajectory.points[i].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
      goal.trajectory.points[i].velocities[j] = 0.0000001;
  }

  double start_time = ros::Time::now().toSec();
  traj_client_->sendGoal(goal);
  ROS_DEBUG(
      "sending goal to controller took %f seconds.", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}

/*
 void Arm::sendArmToPose(double pose[6])
 {
 conductor::ExecuteCartesianIKTrajectory::Request request;
 conductor::ExecuteCartesianIKTrajectory::Response response;

 btQuaternion quaternion;
 geometry_msgs::Quaternion quaternion_msg;

 ros::ServiceClient client = nh_.serviceClient<conductor::ExecuteCartesianIKTrajectory>("ik_trajectory", true);

 request.header.frame_id = reference_frame_;
 request.header.stamp = ros::Time::now();

 request.poses.resize(1);
 request.poses[0].position.x = pose[0];
 request.poses[0].position.y = pose[1];
 request.poses[0].position.z = pose[2];

 quaternion.setRPY(pose[3],pose[4],pose[5]);
 tf::quaternionTFToMsg(quaternion, quaternion_msg);

 request.poses[0].orientation = quaternion_msg;

 if(client.call(request,response))
 {
 if(response.success)
 ROS_DEBUG("successfully went to pose");
 else
 ROS_ERROR("wtf bitch. can't go to pose");
 }

 }
 */

/**
 * Takes a pose in the base_footprint frame.
 */
bool Arm::sendArmToPose(double pose[6], double move_time) {
  btQuaternion quaternion;
  geometry_msgs::Quaternion quaternion_msg;
  quaternion.setRPY(pose[3], pose[4], pose[5]);
  tf::quaternionTFToMsg(quaternion, quaternion_msg);

  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = pose[0];
  pose_msg.position.y = pose[1];
  pose_msg.position.z = pose[2];
  pose_msg.orientation = quaternion_msg;

  std::vector<double> jnt_pos(7, 0), solution(7, 0);

  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state;
  if(arm_name_.compare("right_arm") == 0)
    state = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("r_arm_controller/state", nh_, ros::Duration(2));
  else
    state = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("l_arm_controller/state", nh_, ros::Duration(2));

  if(state->actual.positions.size() == jnt_pos.size())
    ROS_INFO("[arm] Received the joint angles from the arm controller topic.");

  for(size_t i = 0; i < jnt_pos.size(); ++i)
    jnt_pos[i] = state->actual.positions[i];

  double start_time = ros::Time::now().toSec();
  if (computeIK(pose_msg, jnt_pos, solution)) {
    ROS_INFO("computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
  } else
    return false;

  double configuration[7];

  for (int i = 0; i < 7; i++)
    configuration[i] = solution[i];

  sendArmToConfiguration(configuration, move_time);
  return true;
}

bool Arm::sendArmToPose(double pose[6], double move_time, bool constraint_aware) {
  btQuaternion quaternion;
  geometry_msgs::Quaternion quaternion_msg;
  quaternion.setRPY(pose[3], pose[4], pose[5]);
  tf::quaternionTFToMsg(quaternion, quaternion_msg);

  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = pose[0];
  pose_msg.position.y = pose[1];
  pose_msg.position.z = pose[2];
  pose_msg.orientation = quaternion_msg;

  std::vector<double> jnt_pos(7, 0), solution(7, 0);

  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state;
  if(arm_name_.compare("right_arm") == 0)
    state = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("r_arm_controller/state", nh_, ros::Duration(2));
  else
    state = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("l_arm_controller/state", nh_, ros::Duration(2));

  if(state->actual.positions.size() == jnt_pos.size())
    ROS_INFO("[arm] Received the joint angles fromthe arm controller topic.");

  for(size_t i = 0; i < jnt_pos.size(); ++i)
    jnt_pos[i] = state->actual.positions[i];

  double start_time = ros::Time::now().toSec();
  if(constraint_aware)
  {
    if (computeConstraintAwareIK(pose_msg, jnt_pos, solution)) {
      ROS_INFO("[arm] Computed constraint aware IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
    } else
      return false;
  }
  else
  {
    if (computeIK(pose_msg, jnt_pos, solution)) {
      ROS_INFO("[arm] Computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
    } else
      return false;
  }

  double configuration[7];
  for (int i = 0; i < 7; i++)
    configuration[i] = solution[i];

  sendArmToConfiguration(configuration, move_time);
  return true;
}

void Arm::sendArmToPoses(std::vector<std::vector<double> > &poses,
    std::vector<double> move_times) {
  geometry_msgs::Pose pose_msg;
  btQuaternion quaternion;
  geometry_msgs::Quaternion quaternion_msg;
  std::vector<std::vector<double> > configurations(poses.size(),
      std::vector<double>(7, 0));

  std::vector<double> jnt_pos(7, 0), solution(7, 0);

  for (unsigned int i = 0; i < poses.size(); i++) {
    quaternion.setRPY(poses[i][3], poses[i][4], poses[i][5]);
    tf::quaternionTFToMsg(quaternion, quaternion_msg);

    pose_msg.position.x = poses[i][0];
    pose_msg.position.y = poses[i][1];
    pose_msg.position.z = poses[i][2];
    pose_msg.orientation = quaternion_msg;

    double start_time = ros::Time::now().toSec();
    if (computeIK(pose_msg, jnt_pos, configurations[i]))
      ROS_DEBUG(
          "[sendArmToPoses] Computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
    else
      return;
  }

  sendArmToConfigurations(configurations, move_times);
}

bool Arm::computeIK(const geometry_msgs::Pose &pose,
    std::vector<double> jnt_pos, std::vector<double> &solution) {
  kinematics_msgs::GetPositionIK::Request request;
  kinematics_msgs::GetPositionIK::Response response;
  if (arm_name_.compare("left") == 0)
    request.ik_request.ik_link_name = "l_wrist_roll_link";
  else
    request.ik_request.ik_link_name = "r_wrist_roll_link";

  request.ik_request.pose_stamped.pose = pose;
  request.ik_request.pose_stamped.header.stamp = ros::Time();
  request.ik_request.pose_stamped.header.frame_id = reference_frame_;
  request.timeout = ros::Duration(2.0);

  request.ik_request.ik_seed_state.joint_state.header.stamp = ros::Time();
  request.ik_request.ik_seed_state.joint_state.header.frame_id =
      reference_frame_;
  request.ik_request.ik_seed_state.joint_state.name = joint_names_;
  request.ik_request.ik_seed_state.joint_state.position.clear();

  ROS_INFO("[arm] xyz: %f %f %f   frame: %s", pose.position.x, pose.position.y, pose.position.z, request.ik_request.pose_stamped.header.frame_id.c_str());
  for (int j = 0; j < 7; ++j)
    request.ik_request.ik_seed_state.joint_state.position.push_back(jnt_pos[j]);

  ros::service::waitForService(ik_service_name_);
  ros::ServiceClient client = nh_.serviceClient<kinematics_msgs::GetPositionIK>(ik_service_name_, true);

  if (client.call(request, response)) {
    ROS_DEBUG("Obtained IK solution");
    if (response.error_code.val == response.error_code.SUCCESS) {
      for (unsigned int i = 0; i < response.solution.joint_state.name.size();
          i++) {
        solution[i] = response.solution.joint_state.position[i];
        ROS_DEBUG(
            "Joint: %s %f", response.solution.joint_state.name[i].c_str(), response.solution.joint_state.position[i]);
      }
    } else {
      ROS_ERROR(
          "Inverse kinematics failed for %s. (error code: %d)", request.ik_request.ik_link_name.c_str(), response.error_code.val);
      return false;
    }

    ROS_DEBUG("IK Solution");
    for (unsigned int i = 0; i < solution.size(); ++i)
      ROS_DEBUG("%i: %f", i, solution[i]);
  } else {
    ROS_ERROR("IK service failed");
    return false;
  }
  ROS_INFO("[arm] Successfully received an IK solution.");
  return true;
}

bool Arm::computeConstraintAwareIK(const geometry_msgs::Pose &pose,
    std::vector<double> jnt_pos, std::vector<double> &solution) {
  kinematics_msgs::GetConstraintAwarePositionIK::Request request;
  kinematics_msgs::GetConstraintAwarePositionIK::Response response;
  if (arm_name_.compare("left") == 0)
    request.ik_request.ik_link_name = "l_wrist_roll_link";
  else
    request.ik_request.ik_link_name = "r_wrist_roll_link";

  request.ik_request.pose_stamped.pose = pose;
  request.ik_request.pose_stamped.header.stamp = ros::Time();
  request.ik_request.pose_stamped.header.frame_id = reference_frame_;
  request.timeout = ros::Duration(2.0);

  request.ik_request.ik_seed_state.joint_state.header.stamp = ros::Time();
  request.ik_request.ik_seed_state.joint_state.header.frame_id = reference_frame_;
  request.ik_request.ik_seed_state.joint_state.name = joint_names_;
  request.ik_request.ik_seed_state.joint_state.position.clear();

  ROS_INFO("[arm] xyz: %f %f %f   frame: %s  (constraint aware)", pose.position.x, pose.position.y, pose.position.z, request.ik_request.pose_stamped.header.frame_id.c_str());
  for (int j = 0; j < 7; ++j)
    request.ik_request.ik_seed_state.joint_state.position.push_back(jnt_pos[j]);

  ros::service::waitForService(constraint_ik_service_name_);
  ros::ServiceClient client = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(constraint_ik_service_name_, true);

  if (client.call(request, response)) {
    ROS_DEBUG("Obtained IK solution");
    if (response.error_code.val == response.error_code.SUCCESS) {
      for (unsigned int i = 0; i < response.solution.joint_state.name.size(); i++) {
        solution[i] = response.solution.joint_state.position[i];
        ROS_DEBUG("Joint: %s %f", response.solution.joint_state.name[i].c_str(), response.solution.joint_state.position[i]);
      }
    } else {
      ROS_ERROR("Inverse kinematics failed for %s. (error code: %d)", request.ik_request.ik_link_name.c_str(), response.error_code.val);
      return false;
    }

    ROS_DEBUG("IK Solution");
    for (unsigned int i = 0; i < solution.size(); ++i)
      ROS_DEBUG("%i: %f", i, solution[i]);
  } else {
    ROS_ERROR("[arm] IK service failed");
    return false;
  }

  ROS_INFO("[arm] Successfully received a constraint aware IK solution.");
  return true;
}

bool Arm::setAllowedContacts(std::string object_id){

  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  ros::ServiceClient get_planning_scene_client =nh_.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

  arm_navigation_msgs::GetPlanningScene::Request req;
  arm_navigation_msgs::GetPlanningScene::Response res;
  ROS_INFO("[arm] Setting the allowed contacts...");
  arm_navigation_msgs::AllowedContactSpecification ac;
  ac.name = "table area";
  ac.shape.type = arm_navigation_msgs::Shape::BOX;
  ac.shape.dimensions.resize(3);
  ac.shape.dimensions[0] = 8.0;
  ac.shape.dimensions[1] = 8.0;
  ac.shape.dimensions[2] = 2.0;
  ac.pose_stamped.header.frame_id = "/map";
  ac.pose_stamped.header.stamp = ros::Time::now();
  ac.pose_stamped.pose.position.x = 0;;
  ac.pose_stamped.pose.position.y = 0;;
  ac.pose_stamped.pose.position.z = 1.0;;
  ac.pose_stamped.pose.orientation.w = 1.0;;
  ac.penetration_depth = 10000.0;
  ac.link_names.resize(2);
  ac.link_names[1] = "collision_map";

  req.operations.collision_operations.resize(1);
  req.operations.collision_operations[0].operation = arm_navigation_msgs::CollisionOperation::DISABLE;
  req.operations.collision_operations[0].object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
  if(arm_name_.compare("right") == 0){  
    ac.link_names[0] = "r_gripper_r_finger_link";
    req.planning_scene_diff.allowed_contacts.push_back(ac);
    ac.link_names[0] = "r_gripper_l_finger_link";
    req.planning_scene_diff.allowed_contacts.push_back(ac);
    ac.link_names[0] = "r_gripper_r_finger_tip_link";
    req.planning_scene_diff.allowed_contacts.push_back(ac);
    ac.link_names[0] = "r_gripper_l_finger_tip_link";
    req.planning_scene_diff.allowed_contacts.push_back(ac);
    ac.link_names[0] = "r_gripper_palm_link";
    req.planning_scene_diff.allowed_contacts.push_back(ac);
    req.operations.collision_operations[0].object1 = "r_end_effector";
  }
  else{
    ac.link_names[0] = "l_gripper_r_finger_link";
    req.planning_scene_diff.allowed_contacts.push_back(ac);
    ac.link_names[0] = "l_gripper_l_finger_link";
    req.planning_scene_diff.allowed_contacts.push_back(ac);
    ac.link_names[0] = "l_gripper_r_finger_tip_link";
    req.planning_scene_diff.allowed_contacts.push_back(ac);
    ac.link_names[0] = "l_gripper_l_finger_tip_link";
    req.planning_scene_diff.allowed_contacts.push_back(ac);
    ac.link_names[0] = "l_gripper_palm_link";
    req.planning_scene_diff.allowed_contacts.push_back(ac);
    req.operations.collision_operations[0].object1 = "l_end_effector";
  }

  if(!object_id.empty())
  {
    ROS_ERROR("[arm] Disabling collisions for %s in addition to the gripper.", object_id.c_str());
    ac.link_names[0] = object_id;
    req.planning_scene_diff.allowed_contacts.push_back(ac);

    arm_navigation_msgs::CollisionOperation object_op;
    object_op.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
    object_op.object1 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    object_op.object2 = object_id;
    req.operations.collision_operations.push_back(object_op);
  }

  ROS_INFO("[arm] Allowing collision operations  between %s and %s.", req.operations.collision_operations[0].object1.c_str(), req.operations.collision_operations[0].object2.c_str());
  for(size_t i = 0; i < req.planning_scene_diff.allowed_contacts.size(); ++i)
    ROS_INFO("[arm] Allowing collisions between %s and %s.", req.planning_scene_diff.allowed_contacts[i].link_names[0].c_str(), req.planning_scene_diff.allowed_contacts[i].link_names[1].c_str());

  if(!get_planning_scene_client.call(req, res)) {
    ROS_WARN("[arm] Can't get planning scene");
    return false;
  }
  return true;
}

