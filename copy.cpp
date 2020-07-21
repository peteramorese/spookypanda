#include <pluginlib/class_loader.h>
#include <ros/ros.h>


// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <boost/scoped_ptr.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


float dot(float vec1[3], float vec2[3]); 
void cross(float *x, float *y, float *z, float vec1[3], float vec2[3]);
void openGrip(trajectory_msgs::JointTrajectory& handstate);
void closeGrip(trajectory_msgs::JointTrajectory& handstate); 
void pickup(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::CollisionObject> colObjIns); 
void place(moveit::planning_interface::MoveGroupInterface& group); 

const float pi=M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pleasework");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
  move_group.setPlanningTime(40);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

/* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");


  


   namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  ROS_INFO_NAMED("pleasework", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("pleasework", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("pleasework", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));








/////////////////////////////////////////////////////////////////////////////////////////////////////

//LOADING A PLANNER
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

//from tutorial

if (!node_handle.getParam("planning_plugin", planner_plugin_name))
  ROS_FATAL_STREAM("Could not find planner plugin name");
try
{
  planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
      "moveit_core", "planning_interface::PlannerManager"));
}
catch (pluginlib::PluginlibException& ex)
{
  ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
}
try 
{ 
  planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
  if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
    ROS_FATAL_STREAM("Could not initialize planner instance");
  ROS_INFO_STREAM("Using planner '" << planner_instance->getDescription() << "'");
}

catch (pluginlib::PluginlibException& ex)
{
  const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
  std::stringstream ss;
  for (std::size_t i = 0; i < classes.size(); ++i)
    ss << classes[i] << " ";
  ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                       << "Available plugins: " << ss.str());
}

move_group.setPlannerId("RRTstarkConfigDefault");
/////////////////////////////////////////////////////////////////////////////////////////////////////




 
  int Nobj=4;
  std::vector<moveit_msgs::CollisionObject> colObj;
  colObj.resize(Nobj);

  for (int i=0; i<Nobj; i++) {
  colObj[i].header.frame_id = move_group.getPlanningFrame(); //table, pillar1, pillar2, thing1
  }
  



  colObj[0].id = "table1";
  colObj[1].id = "pillar1";
  colObj[2].id = "pillar2";
  colObj[3].id = "thing1";
  std::vector<std::string> colObjIDs;
  for (int i=0; i<Nobj; i++){
	  colObjIDs.push_back(colObj[i].id);
  }

  // Pillar
  colObj[1].primitives.resize(1);
  colObj[1].primitives[0].type = colObj[1].primitives[0].CYLINDER;
  colObj[1].primitives[0].dimensions.resize(2);
  colObj[1].primitives[0].dimensions[0] = 2.5; //height
  colObj[1].primitives[0].dimensions[1] = 0.1; //radius
  colObj[2].primitives.resize(1);
  colObj[2].primitives[0].type = colObj[2].primitives[0].CYLINDER;
  colObj[2].primitives[0].dimensions.resize(2);
  colObj[2].primitives[0].dimensions[0] = 2.5; //height
  colObj[2].primitives[0].dimensions[1] = 0.1; //radius

  // Table
  colObj[0].primitives.resize(1);
  colObj[0].primitives[0].type = colObj[0].primitives[0].BOX;
  colObj[0].primitives[0].dimensions.resize(3);
  colObj[0].primitives[0].dimensions[0] = 2; 
  colObj[0].primitives[0].dimensions[1] = 2;
  colObj[0].primitives[0].dimensions[2] = .1;

  // Thing1 
  colObj[3].primitives.resize(1);
  colObj[3].primitives[0].type = colObj[3].primitives[0].BOX;
  colObj[3].primitives[0].dimensions.resize(3);
  colObj[3].primitives[0].dimensions[0] = .05; 
  colObj[3].primitives[0].dimensions[1] = .05;
  colObj[3].primitives[0].dimensions[2] = 1;

  // Positions

  colObj[0].primitive_poses.resize(1);
  colObj[0].primitive_poses[0].orientation.w = 1; //table
  colObj[0].primitive_poses[0].position.x = 0;
  colObj[0].primitive_poses[0].position.y = 0;
  colObj[0].primitive_poses[0].position.z = -.05;

  colObj[1].primitive_poses.resize(1);
  colObj[1].primitive_poses[0].orientation.w = 1; //pillar1
  colObj[1].primitive_poses[0].position.x = -.5;
  colObj[1].primitive_poses[0].position.y = 0;
  colObj[1].primitive_poses[0].position.z = 1.25;

  colObj[2].primitive_poses.resize(1);
  colObj[2].primitive_poses[0].orientation.w = 1; //pillar2
  colObj[2].primitive_poses[0].position.x = 0;
  colObj[2].primitive_poses[0].position.y = -.5;
  colObj[2].primitive_poses[0].position.z = 1.25;

  colObj[3].primitive_poses.resize(1);
  colObj[3].primitive_poses[0].orientation.w = 1; //thing1
  colObj[3].primitive_poses[0].position.x = .4;
  colObj[3].primitive_poses[0].position.y = .4;
  colObj[3].primitive_poses[0].position.z = .5;

  for (int i=0; i<Nobj; i++) {
  colObj[i].operation = colObj[i].ADD; //table, pillar1, pillar2, thing1
  }


  planning_scene_interface.addCollisionObjects(colObj);

  visual_tools.prompt("done initializing workspace. press next to init pose");

  int numpos = 3;
  float posvec [numpos+1][7]={{1,0,0,0, .5, 0, .5}, {1,0,0,0, -.5, .5, .5}, {0,1,0,0, -.5, -.5, .5}, {0,0,1,0, .5, -.5, .5}};

// SET INITIAL POSE
  move_group.setEndEffectorLink("panda_link8");
  geometry_msgs::Pose initpose;
  tf2::Quaternion initorient;
  initorient.setRPY(-pi/2, -pi/4, -pi/2);
  initpose.orientation = tf2::toMsg(initorient);

  initpose.position.x = .4;
  initpose.position.y = 0;
  initpose.position.z = .5;
  move_group.setPoseTarget(initpose);
  move_group.setPlanningTime(10.0);
  moveit::planning_interface::MoveGroupInterface::Plan initplan;
  move_group.plan(initplan);
  move_group.execute(initplan);

  visual_tools.prompt("done initializing. press to embark! :)");
/*
  for (int i=1; i<=numpos; i++){
	//robot_state::RobotState start_state(*move_group.getCurrentState());
  	geometry_msgs::Pose mypose;
  	mypose.orientation.x = posvec[i][0];
  	mypose.orientation.y = posvec[i][1];
  	mypose.orientation.z = posvec[i][2];
  	mypose.orientation.w = posvec[i][3];
  	mypose.position.x = posvec[i][4];
  	mypose.position.y = posvec[i][5];
  	mypose.position.z = posvec[i][6];
	ROS_INFO_NAMED("pleasework", "moving to x=%.2f, y=%.2f, z=%.2f.", posvec[i][4], posvec[i][5], posvec[i][6]);
	ROS_INFO_NAMED("pleasework", "goal orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f.", posvec[i][0
], posvec[i][1], posvec[i][2], posvec[i][3]);
	move_group.setPoseTarget(mypose);
	moveit::planning_interface::MoveGroupInterface::Plan theplan;
	move_group.plan(theplan);
	//move_group.move();
	move_group.execute(theplan);

  	
	bool success = (move_group.plan(theplan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  	ROS_INFO_NAMED("pleasework", "did I move around the pillar? %s", success ? "yes" : "no");
  	visual_tools.prompt("Press for next position");

  }
  
 */ 
  ros::WallDuration(1.0).sleep();
  pickup(move_group, colObj);
  ros::WallDuration(1.0).sleep();
  place(move_group);
  geometry_msgs::Pose currentObjPoses;
  planning_scene_interface.getObjectPoses(colObjIDs);
//  ROS_INFO_NAMED("pleasework","Position of Thing1: x=%.2f, y=%.2f, z=%.2f",currentObjPoses[3].position.x, currentObjPoses[3].position.y, currentObjPoses[3].position.z);

  ROS_INFO_NAMED("pleasework","Target x: %.2f y: %.2f z: %.2f", colObj[3].primitive_poses[0].position.x-.085, colObj[3].primitive_poses[0].position.y, colObj[3].primitive_poses[0].position.z);

//////////////////////////////////// SHUTDOWN //////////////////////////////////////
  move_group.setPoseTarget(initpose);
  move_group.setPlanningTime(10.0);
  move_group.plan(initplan);
  move_group.execute(initplan); //move panda to init pose

  planning_scene_interface.removeCollisionObjects(colObjIDs); 
  ros::shutdown();
////////////////////////////////////////////////////////////////////////////////////
  return 0;
}

float dot(float *A, float *B) {
	float vec1[3]={*(A),*(A+1),*(A+2)};
	float vec2[3]={*(B),*(B+1),*(B+2)};
	float dotproduct = vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2];	
	
	
	ROS_INFO_NAMED("pleasework","v1 %f, %f, %f",vec1[0], vec1[1], vec1[2]);
	ROS_INFO_NAMED("pleasework","v2 %f, %f, %f",vec2[0], vec2[1], vec2[2]);
	return dotproduct;
}

void cross(float *x, float *y, float *z, float *A, float *B) {
	float vec1[3]={*(A),*(A+1),*(A+2)};
	float vec2[3]={*(B),*(B+1),*(B+2)};
	
	float crossproduct[3];
	*x=vec1[1]*vec2[2]-vec1[2]*vec2[1];
	*y=vec1[2]*vec2[0]-vec1[0]*vec2[2];
	*z=vec1[0]*vec2[1]-vec1[1]*vec2[0];
}


void openGrip(trajectory_msgs::JointTrajectory& handstate) {
	handstate.joint_names.resize(2);
	handstate.joint_names[0]="panda_finger_joint1";
	handstate.joint_names[1]="panda_finger_joint2";
	handstate.points.resize(1);
	handstate.points[0].positions.resize(2);
	handstate.points[0].positions[0]=0.04;
	handstate.points[0].positions[1]=0.04;
	handstate.points[0].time_from_start=ros::Duration(.5);
}

void closeGrip(trajectory_msgs::JointTrajectory& handstate) {
	handstate.joint_names.resize(2);
	handstate.joint_names[0]="panda_finger_joint1";
	handstate.joint_names[1]="panda_finger_joint2";
	handstate.points.resize(1);
	handstate.points[0].positions.resize(2);
	handstate.points[0].positions[0]=0.025; //.04 for open
	handstate.points[0].positions[1]=0.025;terface.addCollisionObjects(colObj);
	handstate.points[0].time_from_start=ros::Duration(.5);
}

void pickup(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::CollisionObject> colObjIns) {
	std::vector<moveit_msgs::Grasp> grasps;
	grasps.resize(1);
	grasps[0].grasp_pose.header.frame_id = "panda_link0";
	tf2::Quaternion orient;
	orient.setRPY(-pi/2, -pi/4, -pi/2);
	grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orient);
	grasps[0].grasp_pose.pose.position.x = colObjIns[3].primitive_poses[0].position.x-.09;//.09
	grasps[0].grasp_pose.pose.position.y = colObjIns[3].primitive_poses[0].position.y;
	grasps[0].grasp_pose.pose.position.z = colObjIns[3].primitive_poses[0].position.z;
	ROS_INFO_NAMED("pleasework","Target x: %.2f y: %.2f z: %.2f", colObjIns[3].primitive_poses[0].position.x-.085, colObjIns[3].primitive_poses[0].position.y, colObjIns[3].primitive_poses[0].position.z);
	// This defines the pre_grasp_posture
	grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
	grasps[0].pre_grasp_approach.direction.vector.x = 1.0; //unit in direction of approach
	grasps[0].pre_grasp_approach.direction.vector.y = 0.0;
	grasps[0].pre_grasp_approach.direction.vector.z = 0.0;
	grasps[0].pre_grasp_approach.min_distance = .095; //values copied from tutorial
	grasps[0].pre_grasp_approach.desired_distance = .115;  

	grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
	grasps[0].post_grasp_retreat.direction.vector.x = 0.0;
	grasps[0].post_grasp_retreat.direction.vector.y = 0.0;
	grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
	grasps[0].post_grasp_retreat.min_distance = .1;
	grasps[0].post_grasp_retreat.desired_distance = .25;

	openGrip(grasps[0].pre_grasp_posture); //Open grip in the approach
	closeGrip(grasps[0].grasp_posture);

	move_group.setSupportSurfaceName("table1");
	move_group.pick("thing1", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group) {
	std::vector<moveit_msgs::PlaceLocation> placeloc;
	placeloc.resize(1);
	placeloc[0].place_pose.header.frame_id = "panda_link0";
	tf2::Quaternion orient;
	orient.setRPY(0, 0, pi);
	placeloc[0].place_pose.pose.orientation = tf2::toMsg(orient);
	placeloc[0].place_pose.pose.position.x = -.4; //this marks the center of the object
	placeloc[0].place_pose.pose.position.y = .4;
	placeloc[0].place_pose.pose.position.z = .5;
	
	placeloc[0].pre_place_approach.direction.header.frame_id = "panda_link0";
	placeloc[0].pre_place_approach.direction.vector.x = 0.0; //unit in direction of approach
	placeloc[0].pre_place_approach.direction.vector.y = 0.0;
	placeloc[0].pre_place_approach.direction.vector.z = -1.0;
	placeloc[0].pre_place_approach.min_distance = .095; //values copied from tutorial
	placeloc[0].pre_place_approach.desired_distance = .115;  

	placeloc[0].post_place_retreat.direction.header.frame_id = "panda_link0";
	placeloc[0].post_place_retreat.direction.vector.x = 1.0;
	placeloc[0].post_place_retreat.direction.vector.y = 0.0;
	placeloc[0].post_place_retreat.direction.vector.z = 0.0;
	placeloc[0].post_place_retreat.min_distance = .1;
	placeloc[0].post_place_retreat.desired_distance = .25;
	
	openGrip(placeloc[0].post_place_posture); //Open grip in the approach

	move_group.setSupportSurfaceName("table1");
	move_group.place("thing1", placeloc);
	ROS_INFO_NAMED("pleasework","you finished placing");
}

