#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <string>

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


const float pi=M_PI;

struct objStruc {
	std::string objID;
	std::string supportsurface;
	float posx;
	float posy;
	float posz;
	float approachx;
	float approachy;
	float approachz;
	float retreatx;
	float retreaty;
	float retreatz;
	float roll;
	float pitch;
	float yaw;
	float qx;
	float qy;
	float qz;
	float qw;
	float gripwidth;
	float setback;
} pickstruc, placestruc;

struct nodestruct {
	char action;
	int eeLoc;
	int grpObj;
	std::vector<int> objLoc;
};

void str2struct(std::string instring, nodestruct& instruct) {
	instruct.action = instring[0];
	int whatdo = 0;
	std::vector<std::string> tempstr;
	std::string tempstr2;
	for (int i=2; i<instring.length(); i++) {
		if (instring[i]=='_') {
			tempstr.push_back(tempstr2);
			tempstr2.clear();
		} else {
			std::string tempstr3(1,instring[i]);
			tempstr2.append(tempstr3);
		}
	}	
	instruct.eeLoc = std::stoi(tempstr[0],nullptr,0);
	instruct.grpObj = std::stoi(tempstr[1],nullptr,0);
	for (int i=2; i<tempstr.size(); i++) {
		instruct.objLoc.push_back(std::stoi(tempstr[i],nullptr,0));
	}
}


int pow(int num, int exp); 
float powf(float num, float exp);
float dot(float vec1[3], float vec2[3]); 
void cross(float *x, float *y, float *z, float vec1[3], float vec2[3]);
void openGrip(trajectory_msgs::JointTrajectory& handstate);
void closeGrip(trajectory_msgs::JointTrajectory& handstate, float width); 



//moveit::planning_interface::MoveItErrorCode planPick(const std::string& object, const std::vector<moveit_msgs::Grasp>& grasps, moveit::planning_interface::MoveGroupInterface::Plan& plan);
bool pickup(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::CollisionObject> colObjIns, objStruc objS); 
bool place(moveit::planning_interface::MoveGroupInterface& group, objStruc objS); 




int main(int argc, char** argv)
{
	ros::init(argc, argv, "wombatdemo");
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
	ROS_INFO_NAMED("wombatdemo", "Planning frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("wombatdemo", "End effector link: %s", move_group.getEndEffectorLink().c_str());
	ROS_INFO_NAMED("wombatdemo", "Available Planning Groups:");
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

	move_group.setPlannerId("RRTkConfigDefault");
	/////////////////////////////////////////////////////////////////////////////////////////////////////




	int Nobj=5;
	std::vector<moveit_msgs::CollisionObject> colObj;
	colObj.resize(Nobj);

	for (int i=0; i<Nobj; i++) {
		colObj[i].header.frame_id = move_group.getPlanningFrame();
	}




	colObj[0].id = "table1";
	colObj[1].id = "table2";

	int Nsurfaces = 2;
	// Table
	for (int i=0; i<Nsurfaces; i++) {
	colObj[i].primitives.resize(1);
	colObj[i].primitives[0].type = colObj[0].primitives[0].BOX;
	colObj[i].primitives[0].dimensions.resize(3);
	colObj[i].primitives[0].dimensions[0] = .3; 
	colObj[i].primitives[0].dimensions[1] = .5;
	colObj[i].primitives[0].dimensions[2] = .05;
	}
	// Things 
	float thingWidth = .04;
	float thingHeight = .2;
	float thingLength = .04;
	int Nthings = Nobj-Nsurfaces; // "2" for table1 and table2

	for (int i=Nsurfaces; i<Nobj-1; i++) {
		colObj[i].primitives.resize(1);
		colObj[i].primitives[0].type = colObj[3].primitives[0].CYLINDER;
		colObj[i].primitives[0].dimensions.resize(2);
		colObj[i].primitives[0].dimensions[0] = thingHeight;
		colObj[i].primitives[0].dimensions[1] = thingWidth/2;
	}

	colObj[Nobj-1].primitives.resize(1);
	colObj[Nobj-1].primitives[0].type = colObj[3].primitives[0].BOX;
	colObj[Nobj-1].primitives[0].dimensions.resize(3);
	colObj[Nobj-1].primitives[0].dimensions[0] = thingLength; 
	colObj[Nobj-1].primitives[0].dimensions[1] = thingWidth;
	colObj[Nobj-1].primitives[0].dimensions[2] = thingHeight;


	// Positions of tables
	float objPosArr [Nobj-Nthings][4] = {{1,.2,.4,-.025}, {1,-.2,-.4,-.025}};

	for (int i=0; i<(Nobj-Nthings); i++) {
		colObj[i].primitive_poses.resize(1);
		colObj[i].primitive_poses[0].orientation.w = objPosArr[i][0];
		colObj[i].primitive_poses[0].position.x = objPosArr[i][1];
		colObj[i].primitive_poses[0].position.y = objPosArr[i][2];
		colObj[i].primitive_poses[0].position.z = objPosArr[i][3];
		colObj[i].operation = colObj[i].ADD; 
	}


	///////////////////////// Object State & Positions //////////////////////////////////

	int Nsubstate = 5;
	float tol = .0008;
	float pullback = .085 + thingLength/2;
	float ymove = .25;
	// x, y, z, ax, ay, az, rx, ry, rz, r, p, y, objw(1)
	// Vertical index of these arrays defines states
	/*double state_objPick [Nsubstate][13] {
	  { .8,-.3,thingHeight/2, 1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, -pi/2, 1}, 
	  { .8,  0,thingHeight/2, 1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, -pi/2, 1}, 
	  { .8, .3,thingHeight/2, 1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, -pi/2, 1}, 
	  { .4, .6,thingHeight/2, 0, 1.0, 0, 0, 0, 1.0, -pi/2, -pi/4, 0, 1}, 
	  {-.7,-thingHeight/2, thingHeight/2+tol, -1.0, 0, 0, 0, 0, 1.0, 0, 0, pi, 1}, 
	  {-.7, thingHeight/2, thingHeight/2+tol, -1.0, 0, 0, 0, 0, 1.0, 0, 0, pi, 1}, 
	  {-.7, 0, thingHeight+thingWidth/2+4*tol, -1.0, 0, 0, 0, 0, 1.0, pi/2, 0, pi, 1},
	  {-.7, 0, thingHeight+thingWidth+thingHeight/2+6*tol, -1.0, 0, 0, 0, 0, 1.0, 0, 0, pi, 1},
	  {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
	  };*/
	double state_objPick [Nsubstate][13] {
		{ .2, .3,thingHeight/2, 1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, -pi/2, 1}, 
			{ .2, .5,thingHeight/2, 1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, -pi/2, 1},
			{ -.2,-.5,thingHeight/2+tol, -1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, pi/2, 1},
			{-.2,-.3,thingHeight/2+tol, -1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, pi/2, 1},
			{-.2,-.4,thingHeight+thingWidth/2+4*tol, -1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, pi/2, 1}
	};


	/*double state_objPlace [Nsubstate][12] {
	  { .8,-.3,thingHeight/2, 0, 0, -1.0, -1.0, 0, 0, -pi/2, -pi/4, -pi/2}, 
	  { .8,  0,thingHeight/2, 0, 0, -1.0, -1.0, 0, 0, -pi/2, -pi/4, -pi/2}, 
	  { .8, .3,thingHeight/2, 0, 0, -1.0, -1.0, 0, 0, -pi/2, -pi/4, -pi/2},
	  { .4, .6,thingHeight/2, 0, 0, -1.0, -1.0, 0, 0, -pi/2, -pi/4, -pi/2},
	  {-.6,-thingHeight/2+ymove, thingHeight/2+tol, 0, 0, -1.0, 1, 0, 0, 0, 0, pi}, 
	  {-.6, thingHeight/2+ymove, thingHeight/2+tol, 0, 0, -1.0, 1, 0, 0, 0, 0, pi}, 
	  {-.6, ymove, thingHeight+thingWidth/2+2*tol, -1.0, 0, 0, 1, 0, 0, pi/2, 0, pi},
	  {-.6, ymove, thingHeight+thingWidth+thingHeight/2+3*tol, 0, 0, -1.0, 1, 0, 0, 0, 0, pi/2},
	  {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
	  };*/

	// x, y, z, ax, ay, az, rx, ry, rz, {q1, q2, q3}, q4
	double state_objPlace [Nsubstate][13] {
		{ .2,.3,thingHeight/2, 0, 0, -1.0, -1.0, 0, 0, 0, 0, 0, 0}, 
			{ .2,.5,thingHeight/2, 0, 0, -1.0, -1.0, 0, 0, 0, 0, 0, 0}, 
			{-.2,-.5, thingHeight/2+tol, 0, 0, -1.0, 1, 0, 0, 0, 0, sin(pi/2), cos(pi/2)}, 
			{-.2,-.3, thingHeight/2+tol, 0, 0, -1.0, 1, 0, 0, 0, 0, sin(pi/2), cos(pi/2)}, 
			{-.2,-.4, thingHeight+thingWidth/2+4*tol, 0, 0, -1.0, 1, 0, 0, 0, -1/sqrt(2) , 1/sqrt(2), 0}
	};

	// DEMONSTRATION #1
	//std::vector<std::string> stateSequence = {"m_0_0_2_4_1_", "m_2_0_2_4_1_", "g_2_0_2_4_1_", "h_2_1_0_4_1_", "h_0_1_0_4_1_", "h_3_1_0_4_1_", "p_3_1_0_4_1_", "m_3_0_3_4_1_", "m_0_0_3_4_1_", "m_1_0_3_4_1_", "g_1_0_3_4_1_", "h_1_3_3_4_0_", "h_0_3_3_4_0_", "h_5_3_3_4_0_", "p_5_3_3_4_0_", "m_5_0_3_4_5_"};

	// DEMONSTRATION #2
	std::vector<std::string> stateSequence = {"m_0_0_2_4_1_", "m_4_0_2_4_1_", "g_4_0_2_4_1_", "h_4_2_2_0_1_", "h_0_2_2_0_1_", "h_3_2_2_0_1_", "p_3_2_2_0_1_", "m_3_0_2_3_1_", "m_0_0_2_3_1_", "m_2_0_2_3_1_", "g_2_0_2_3_1_", "h_2_1_0_3_1_", "h_0_1_0_3_1_", "h_4_1_0_3_1_", "p_4_1_0_3_1_", "m_4_0_4_3_1_", "m_0_0_4_3_1_", "m_1_0_4_3_1_", "g_1_0_4_3_1_", "h_1_3_4_3_0_", "h_0_3_4_3_0_", "h_5_3_4_3_0_", "p_5_3_4_3_0_", "m_5_0_4_3_5_"};

	// DEMONSTRATION #3
	//std::vector<std::string> stateSequence = {"m_0_0_2_4_1_", "m_2_0_2_4_1_", "g_2_0_2_4_1_", "h_2_1_0_4_1_", "h_0_1_0_4_1_", "h_3_1_0_4_1_", "p_3_1_0_4_1_", "m_3_0_3_4_1_", "m_0_0_3_4_1_", "m_4_0_3_4_1_", "g_4_0_3_4_1_", "h_4_2_3_0_1_", "h_0_2_3_0_1_", "h_2_2_3_0_1_", "p_2_2_3_0_1_", "m_2_0_3_2_1_", "m_0_0_3_2_1_", "m_3_0_3_2_1_", "g_3_0_3_2_1_", "h_3_1_0_2_1_", "h_0_1_0_2_1_", "h_4_1_0_2_1_", "p_4_1_0_2_1_", "m_4_0_4_2_1_", "m_0_0_4_2_1_", "m_2_0_4_2_1_", "g_2_0_4_2_1_", "h_2_2_4_0_1_", "h_0_2_4_0_1_", "h_3_2_4_0_1_", "p_3_2_4_0_1_", "m_3_0_4_3_1_", "m_0_0_4_3_1_", "m_1_0_4_3_1_", "g_1_0_4_3_1_", "h_1_3_4_3_0_", "h_0_3_4_3_0_", "h_5_3_4_3_0_", "p_5_3_4_3_0_", "m_5_0_4_3_5_"};
	
	nodestruct initnode;
	str2struct(stateSequence[0], initnode);
	std::cout << "InitNode action:" << initnode.action  << std::endl;
	std::cout << "InitNode eeLoc:" << initnode.eeLoc  << std::endl;
	std::cout << "InitNode grpObj:" << initnode.grpObj  << std::endl;
	for (int i=0; i<initnode.objLoc.size(); i++) {
		std::cout << "  InitNode objLoc:" << initnode.objLoc[i]  << std::endl;
	}
	if (initnode.objLoc.size() != Nthings) {
		ROS_ERROR_NAMED("wombatdemo","Number of objects mismatch");
	}



	// Positions of things
	for (int i=Nsurfaces; i<Nobj; i++) {
		int ind = i - Nsurfaces;
		colObj[i].id = "thing" + std::to_string(ind+1);
		colObj[i].primitive_poses.resize(1);
		colObj[i].primitive_poses[0].orientation.w = 1; //state_objPick[initnode.objLoc[i]-1][12];
		colObj[i].primitive_poses[0].position.x = state_objPick[initnode.objLoc[ind]-1][0];
		colObj[i].primitive_poses[0].position.y = state_objPick[initnode.objLoc[ind]-1][1];
		colObj[i].primitive_poses[0].position.z = state_objPick[initnode.objLoc[ind]-1][2];
		colObj[i].operation = colObj[i].ADD; 
	}
	// Change the position of the object to the position the eef must arrive at
	for (int i=0; i<Nsubstate; i++) {
		for (int ii=0; ii<3; ii++) {
			if (state_objPick[i][ii+3]==1.0) {
				state_objPick[i][ii]=state_objPick[i][ii] - pullback;
			} else if (state_objPick[i][ii+3]==-1.0) {		
				state_objPick[i][ii]=state_objPick[i][ii] + pullback;
			}
			ROS_INFO_NAMED("wombatdemo"," %.1f",state_objPick[i][ii]);

		}
	}


	std::vector<std::string> colObjIDs;
	for (int i=0; i<Nobj; i++){
		colObjIDs.push_back(colObj[i].id);
	}

	planning_scene_interface.addCollisionObjects(colObj);
	
	visual_tools.prompt("Press to embark");

	/////////////////////////////// Plan and Execute ////////////////////////////////////
	std::vector<std::string> supportsurfacevec;
	supportsurfacevec = {"table1", "table1", "table2", "table2", "table2"};
	int Ntasks = stateSequence.size();
	for (int i=0; i<Ntasks; i++) {
		nodestruct node;
		str2struct(stateSequence[i], node);
		std::cout << "Node action:" << node.action  << std::endl;
		std::cout << "Node eeLoc:" << node.eeLoc  << std::endl;
		std::cout << "Node grpObj:" << node.grpObj  << std::endl;
		for (int i=0; i<node.objLoc.size(); i++) {
			std::cout << "  Node objLoc:" << node.objLoc[i]  << std::endl;
		}
		int currObj;
		bool success;
		move_group.setPlanningTime(20.0);

		switch (node.action) {
			case 'g' : {
				for (int i=0; i<node.objLoc.size(); i++) {
					if (node.objLoc[i] == node.eeLoc) {
						currObj = i+Nobj-Nthings;
					} 
				}
				int a = node.eeLoc-1;	
				pickstruc.objID = colObjIDs[currObj];
				pickstruc.posx = state_objPick[a][0];
				pickstruc.posy = state_objPick[a][1];
				pickstruc.posz = state_objPick[a][2];
				pickstruc.approachx = state_objPick[a][3];
				pickstruc.approachy = state_objPick[a][4];
				pickstruc.approachz = state_objPick[a][5];
				pickstruc.retreatx = state_objPick[a][6];
				pickstruc.retreaty = state_objPick[a][7];
				pickstruc.retreatz = state_objPick[a][8];
				pickstruc.roll = state_objPick[a][9];
				pickstruc.pitch = state_objPick[a][10];
				pickstruc.yaw = state_objPick[a][11];
				std::cout<<"CURRENTLY WORKING ACTION: grasp" << std::endl;
				std::cout<<" location:"<<a+1 << std::endl;
				std::cout<<"  moving to x:" << pickstruc.posx << std::endl;
				std::cout<<"  moving to x:" << pickstruc.posy << std::endl;
				std::cout<<"  moving to x:" << pickstruc.posz << std::endl;

				std::cout<<"  moving to r:" << pickstruc.roll << std::endl;
				std::cout<<"  moving to p:" << pickstruc.pitch << std::endl;
				std::cout<<"  moving to y:" << pickstruc.yaw << std::endl;

				pickstruc.gripwidth = thingWidth;
				pickstruc.setback = 0; //thingLength/2;
				pickstruc.supportsurface = supportsurfacevec[a] ;

				ros::WallDuration(1.0).sleep();
				success = pickup(move_group, colObj, pickstruc);
				ros::WallDuration(1.0).sleep();
				if (!success) {
					ROS_ERROR_NAMED("wombatdemo","Planning Failed in pickup");
					ROS_BREAK();
				}
				break;
				}
			case 'p' : {
				move_group.setEndEffectorLink("panda_link8");
				currObj = node.grpObj + Nobj - Nthings-1;
				placestruc.objID = colObjIDs[currObj];
				//std::map<std::string, geometry_msgs::Pose> cPose;
				//cPose = planning_scene_interface.getObjectPoses(colObjIDs);
				int b = node.eeLoc-1;
				placestruc.posx = state_objPlace[b][0];
				placestruc.posy = state_objPlace[b][1];
				placestruc.posz = state_objPlace[b][2];
				placestruc.approachx = state_objPlace[b][3];
				placestruc.approachy = state_objPlace[b][4];
				placestruc.approachz = state_objPlace[b][5];
				placestruc.retreatx = state_objPlace[b][6];
				placestruc.retreaty = state_objPlace[b][7];
				placestruc.retreatz = state_objPlace[b][8];
				//placestruc.roll = state_objPlace[b][9];
				//placestruc.pitch = state_objPlace[b][10];
				//placestruc.yaw = state_objPlace[b][11];
				placestruc.qx = state_objPlace[b][9];
				placestruc.qy = state_objPlace[b][10];
				placestruc.qz = state_objPlace[b][11];
				placestruc.qw = state_objPlace[b][12];
				placestruc.gripwidth = thingWidth;
				placestruc.setback = 0; //thingLength/2;
				placestruc.supportsurface = supportsurfacevec[b];
				std::cout<<"CURRENTLY WORKING ACTION: place" << std::endl;
				std::cout<<" location:"<<b+1 << std::endl;
				std::cout<<"  moving to x:" << placestruc.posx << std::endl;
				std::cout<<"  moving to x:" << placestruc.posy << std::endl;
				std::cout<<"  moving to x:" << placestruc.posz << std::endl;

				std::cout<<"  moving to qx:" << placestruc.qx << std::endl;
				std::cout<<"  moving to qy:" << placestruc.qy << std::endl;
				std::cout<<"  moving to qz:" << placestruc.qz << std::endl;
				std::cout<<"  moving to qw:" << placestruc.qw << std::endl;

				ros::WallDuration(1.0).sleep();
				success = place(move_group, placestruc);
				ros::WallDuration(1.0).sleep();
				if (!success) {
					move_group.detachObject(placestruc.objID);
					ROS_ERROR_NAMED("wombatdemo","Planning Failed when placing");
					ROS_BREAK();
				}
				break;
				}
			default : 
				if (node.eeLoc != 0) {
					move_group.setEndEffectorLink("panda_link8");
					geometry_msgs::Pose movepose;
					int a = node.eeLoc-1;	
					tf2::Quaternion moveorient;
					if (node.action == 'm') {
						moveorient.setRPY(state_objPick[a][9], state_objPick[a][10], state_objPick[a][11]);
						movepose.orientation = tf2::toMsg(moveorient);
						movepose.position.x = state_objPick[a][0];
						movepose.position.y = state_objPick[a][1];
						movepose.position.z = state_objPick[a][2]+.4;
					} else {
						moveorient.setRPY(state_objPlace[a][9], state_objPlace[a][10], state_objPlace[a][11]);
						movepose.orientation = tf2::toMsg(moveorient);
						movepose.position.x = state_objPlace[a][0];
						movepose.position.y = state_objPlace[a][1];
						movepose.position.z = state_objPlace[a][2]+.4;
					}
					std::cout<<"CURRENTLY WORKING ACTION: move or hold" << std::endl;
					std::cout<< node.action << std::endl;
					std::cout<<" location:"<<a+1 << std::endl;
					std::cout<<"  moving to x:" << movepose.position.x << std::endl;
					std::cout<<"  moving to y:" << movepose.position.y << std::endl;
					std::cout<<"  moving to z:" << movepose.position.z << std::endl;

					move_group.setPoseTarget(movepose);
					moveit::planning_interface::MoveGroupInterface::Plan moveplan;
					move_group.plan(moveplan);
					move_group.execute(moveplan);
				}
		}

	}

//////////////////////////////////// SHUTDOWN //////////////////////////////////////
/*
   move_group.setPoseTarget(initpose);
   move_group.setPlanningTime(10.0);
   move_group.plan(initplan);
   move_group.execute(initplan); //move panda to init pose
   */
planning_scene_interface.removeCollisionObjects(colObjIDs); 
ros::shutdown();

////////////////////////////////////////////////////////////////////////////////////

return 0;
}

int pow(int num, int exp) {
	if (exp==0){
		return 1;
	} else {
		int base = num;
		for (int i=1; i<exp; i++){
			base = base*num;
		}
		return base;
	}
}

float powf(float num, float exp) {
	if (exp==0){
		return 1;
	} else {
		float base = num;
		for (int i=1; i<exp; i++){
			base = base*num;
		}
		return base;
	}
}







float dot(float *A, float *B) {
	float vec1[3]={*(A),*(A+1),*(A+2)};
	float vec2[3]={*(B),*(B+1),*(B+2)};
	float dotproduct = vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2];	


	ROS_INFO_NAMED("wombatdemo","v1 %f, %f, %f",vec1[0], vec1[1], vec1[2]);
	ROS_INFO_NAMED("wombatdemo","v2 %f, %f, %f",vec2[0], vec2[1], vec2[2]);
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

void closeGrip(trajectory_msgs::JointTrajectory& handstate, float width) {
	handstate.joint_names.resize(2);
	handstate.joint_names[0]="panda_finger_joint1";
	handstate.joint_names[1]="panda_finger_joint2";
	handstate.points.resize(1);
	handstate.points[0].positions.resize(2);
	handstate.points[0].positions[0]=width/2; //.04 for open, .025 for close
	handstate.points[0].positions[1]=width/2;
	handstate.points[0].time_from_start=ros::Duration(.5);
}

bool pickup(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::CollisionObject> colObjIns, objStruc objS) {	
	std::vector<moveit_msgs::Grasp> grasps;
	grasps.resize(1);
	grasps[0].grasp_pose.header.frame_id = "panda_link0";
	tf2::Quaternion orient;
	//orient.setRPY(-pi/2, -pi/4, -pi/2);
	orient.setRPY(objS.roll, objS.pitch, objS.yaw);
	grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orient);
	grasps[0].grasp_pose.pose.position.x = objS.posx-objS.setback;//.09
	//grasps[0].grasp_pose.pose.position.y = colObjIns[3].primitive_poses[0].position.y;
	grasps[0].grasp_pose.pose.position.y = objS.posy;
	grasps[0].grasp_pose.pose.position.z = objS.posz;
	ROS_INFO_NAMED("wombatdemo","Target Pickup x: %.2f y: %.2f z: %.2f", objS.posx-objS.setback, objS.posy, objS.posz);
	// This defines the pre_grasp_posture
	grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
	grasps[0].pre_grasp_approach.direction.vector.x = objS.approachx; //unit in direction of approach
	grasps[0].pre_grasp_approach.direction.vector.y = objS.approachy;
	grasps[0].pre_grasp_approach.direction.vector.z = objS.approachz;
	grasps[0].pre_grasp_approach.min_distance = .05;//.095; //values copied from tutorial
	grasps[0].pre_grasp_approach.desired_distance = .08; //.115;  

	grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
	grasps[0].post_grasp_retreat.direction.vector.x = objS.retreatx;
	grasps[0].post_grasp_retreat.direction.vector.y = objS.retreaty;
	grasps[0].post_grasp_retreat.direction.vector.z = objS.retreatz;
	grasps[0].post_grasp_retreat.min_distance = .05;//.1;
	grasps[0].post_grasp_retreat.desired_distance = .08;//.25;

	openGrip(grasps[0].pre_grasp_posture); //Open grip in the approach
	closeGrip(grasps[0].grasp_posture, objS.gripwidth);

	move_group.setSupportSurfaceName(objS.supportsurface);

	moveit::planning_interface::MoveGroupInterface::Plan testplan;

	bool success;
	success = (move_group.pick(objS.objID, grasps)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("Pickup plan: %s", success ? "Success" : "Failed");
	return success;
}

bool place(moveit::planning_interface::MoveGroupInterface& move_group, objStruc objS) {
	std::vector<moveit_msgs::PlaceLocation> placeloc;
	placeloc.resize(1);
	placeloc[0].place_pose.header.frame_id = "panda_link0";
	//tf2::Quaternion orient;
	//orient.setRPY(objS.roll, objS.pitch, objS.yaw);	
	//placeloc[0].place_pose.pose.orientation = tf2::toMsg(orient);
	placeloc[0].place_pose.pose.orientation.x = objS.qx; 
	placeloc[0].place_pose.pose.orientation.y = objS.qy;
	placeloc[0].place_pose.pose.orientation.z = objS.qz;
	placeloc[0].place_pose.pose.orientation.w = objS.qw;
	std::cout<< placeloc[0].place_pose.pose.orientation.x<< std::endl; 
	std::cout<< placeloc[0].place_pose.pose.orientation.y<< std::endl; 
	std::cout<< placeloc[0].place_pose.pose.orientation.z<< std::endl; 
	std::cout<< placeloc[0].place_pose.pose.orientation.w<< std::endl; 
	placeloc[0].place_pose.pose.position.x = objS.posx; //this marks the center of the object
	placeloc[0].place_pose.pose.position.y = objS.posy;
	placeloc[0].place_pose.pose.position.z = objS.posz;
	ROS_INFO_NAMED("wombatdemo","Target Place x: %.2f y: %.2f z: %.2f", objS.posx, objS.posy, objS.posz);
	
	placeloc[0].pre_place_approach.direction.header.frame_id = "panda_link0";
	placeloc[0].pre_place_approach.direction.vector.x = objS.approachx; //unit in direction of approach
	placeloc[0].pre_place_approach.direction.vector.y = objS.approachy;
	placeloc[0].pre_place_approach.direction.vector.z = objS.approachz;
	placeloc[0].pre_place_approach.min_distance = .05;//.095; //values copied from tutorial
	placeloc[0].pre_place_approach.desired_distance = .08;//.115;  

	placeloc[0].post_place_retreat.direction.header.frame_id = "panda_link0";
	placeloc[0].post_place_retreat.direction.vector.x = objS.retreatx;
	placeloc[0].post_place_retreat.direction.vector.y = objS.retreaty;
	placeloc[0].post_place_retreat.direction.vector.z = objS.retreatz;
	placeloc[0].post_place_retreat.min_distance = .05;//.1;
	placeloc[0].post_place_retreat.desired_distance = .08;//.25;

	openGrip(placeloc[0].post_place_posture); //Open grip in the approach
	move_group.setSupportSurfaceName(objS.supportsurface);

	bool success;
	success = (move_group.place(objS.objID, placeloc)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("Pickup plan: %s", success ? "Success" : "Failed");
	return success;
}

