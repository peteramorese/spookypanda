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
	float gripwidth;
	float setback;
} pickstruc, placestruc;

int pow(int num, int exp); 
float dot(float vec1[3], float vec2[3]); 
void cross(float *x, float *y, float *z, float vec1[3], float vec2[3]);
void openGrip(trajectory_msgs::JointTrajectory& handstate);
void closeGrip(trajectory_msgs::JointTrajectory& handstate, float width); 



void pickup(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::CollisionObject> colObjIns, objStruc objS); 
void place(moveit::planning_interface::MoveGroupInterface& group, objStruc objS); 


// Vim trash
// "gg =G" auto indent



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





	int Nobj=5;
	std::vector<moveit_msgs::CollisionObject> colObj;
	colObj.resize(Nobj);

	for (int i=0; i<Nobj; i++) {
		colObj[i].header.frame_id = move_group.getPlanningFrame(); //table, pillar1, pillar2, thing1
	}




	colObj[0].id = "table1";
	colObj[1].id = "pillar1";
	colObj[2].id = "thing1";
	colObj[3].id = "thing2";
	colObj[4].id = "thing3";
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
	/*
	   colObj[2].primitives.resize(1);
	   colObj[2].primitives[0].type = colObj[2].primitives[0].CYLINDER;
	   colObj[2].primitives[0].dimensions.resize(2);
	   colObj[2].primitives[0].dimensions[0] = 2.5; //height
	   colObj[2].primitives[0].dimensions[1] = 0.1; //radius
	   */

	// Table
	colObj[0].primitives.resize(1);
	colObj[0].primitives[0].type = colObj[0].primitives[0].BOX;
	colObj[0].primitives[0].dimensions.resize(3);
	colObj[0].primitives[0].dimensions[0] = 2; 
	colObj[0].primitives[0].dimensions[1] = 2;
	colObj[0].primitives[0].dimensions[2] = .1;

	// Things 
	float thingWidth = .04;
	float thingHeight = .2;
	float thingLength = .04;
	int Nthings = 3;

	for (int i=2; i<Nthings+2; i++){
		colObj[i].primitives.resize(1);
		colObj[i].primitives[0].type = colObj[3].primitives[0].BOX;
		colObj[i].primitives[0].dimensions.resize(3);
		colObj[i].primitives[0].dimensions[0] = thingLength; 
		colObj[i].primitives[0].dimensions[1] = thingWidth;
		colObj[i].primitives[0].dimensions[2] = thingHeight;
	}

	// Positions
	float objPosArr [Nobj][4] = {{1,0,0,-.05}, {1,1.2,1.2,1.25}, {1,.8,-.3,thingHeight/2}, {1,.8,0,thingHeight/2}, {1,.8,.3,thingHeight/2}};
	//table, pillar1, thing1, thing2, thing3
	for (int i=0; i<Nobj; i++) {
		colObj[i].primitive_poses.resize(1);
		colObj[i].primitive_poses[0].orientation.w = objPosArr[i][0];
		colObj[i].primitive_poses[0].position.x = objPosArr[i][1];
		colObj[i].primitive_poses[0].position.y = objPosArr[i][2];
		colObj[i].primitive_poses[0].position.z = objPosArr[i][3];
		colObj[i].operation = colObj[i].ADD; //table, pillar1, pillar2, thing1
	}


	planning_scene_interface.addCollisionObjects(colObj);

	/////////////////////////////// Graph Math //////////////////////////////////////////

	int Nsubstate = 6;
	int initstateNum, goalstateNum;
	std::vector<int> initstate = {1,1,1,0,0,0};
	std::vector<int> goalstate = {0,0,0,1,1,1};	

	// CREATE VECTOR CONTAINING ALL STATES
	std::vector<int> stateMat;
	int Nstate = 0;
	std::vector<int> tempVec = {0,0,0,0,0,0};
	for (int j=0; j<(pow(2,Nsubstate)-1); j++) {
		int dec = j;
		int a = 0;
		//ROS_INFO_NAMED("pleasework","Im still alive: %d, %d", j, pow(2,Nsubstate)-1);
		//tempVec.clear();
		for (int i=0; i<(Nsubstate); i++){
			if ((dec-pow(2,Nsubstate-1-i))>=0) {
				dec=dec-pow(2,Nsubstate-1-i);
				tempVec[i]=1;
				a++;

				//ROS_INFO_NAMED("pleasework","Hello here is dec: %d, with j=%d", dec, j);
			} else {
				tempVec[i]=0;
			}
		}
		if (a==Nthings) {
			Nstate++; // "1" start indexing
			ROS_INFO_NAMED("pleasework","\n\nState %d (%d) was added and converted to:", Nstate, j); 
			for (int ii=0; ii<(Nsubstate); ii++){
				stateMat.push_back(tempVec[ii]);
				ROS_INFO_NAMED("pleasework","%d", tempVec[ii]);
			}
			if (tempVec==initstate) {
				initstateNum=Nstate;
				ROS_INFO_NAMED("pleasework", "This is the initial state");
			} else if (tempVec==goalstate) {
				goalstateNum=Nstate;
				ROS_INFO_NAMED("pleasework", "This is the goal state");
			}
		}
	}
	// Here is how to index the first element in the state tuple: stateMat[Nsubstate*(ind-1)]
	// States are named numerically by this index

	// PROPAGATE ALL EDGES
	std::vector<int> edgeMat;
	std::vector<float> edgeLength;
	int Nedge = 0;
	// Edges defined by a two element tuple where elements represent connected states/nodes
	// Edit the indexing to include self-loop edges or directional edges
	for (int i=2; i<=Nstate; i++) {
		for (int ii=1; ii<i; ii++) {
			int diff = 0;
			for (int iii=0; iii<Nsubstate; iii++) {
				diff=diff+abs(stateMat[Nsubstate*(ii-1)+iii]-stateMat[Nsubstate*(i-1)+iii]);
				//ROS_INFO_NAMED("pleasework","diff=%d",diff);
			}
			if (diff==2) {
				edgeMat.push_back(i);
				edgeMat.push_back(ii);
				edgeLength.push_back(1); // Use this to propagate edge lengths/weights
				Nedge++;
				ROS_INFO_NAMED("pleasework","Edge connecting state %d and %d",i,ii);
			}
		}
	}

	// FIND THE SHORTEST PATH (Dijkstra's Algorithm)
	float distSP[Nstate];
	bool done = false;
	int currentSP = initstateNum; // "1" start indexing
	ROS_INFO_NAMED("pleasework","currentSP = %d ", currentSP);
	bool visit[Nstate];
	float minVal = -1.0;
	int minInd;	
	// Assign distance to all nodes, init node is zero all other nodes are infinity (-1.0)
	for (int i=0; i<Nstate; i++) {
		visit[i] = false;
		if ((i+1)==initstateNum) {
			distSP[i]=0;	
		} else {
			distSP[i]=-1.0;
		}
	}
	while (!done) {
		for (int ii=0; ii<Nedge; ii++) {
			if (edgeMat[2*ii]==currentSP) {
				if (!visit[edgeMat[2*ii+1]-1]) {
					float a = distSP[edgeMat[2*ii+1]-1]; // Current distance value of neighbor node
					float b = edgeLength[ii] + distSP[currentSP-1]; // Current distance plus length of edge
					if ((a==-1.0)||(a > b)) {
						distSP[edgeMat[2*ii+1]-1] = b;
					} 
					ROS_INFO_NAMED("pleasework"," distance element %d was changed to %.1f", edgeMat[2*ii+1], b);
				}
			} else if (edgeMat[2*ii+1]==currentSP) {
				if (!visit[edgeMat[2*ii]-1]) {
					float a = distSP[edgeMat[2*ii]-1];
					float b = edgeLength[ii] + distSP[currentSP-1]; 
					if ((a==-1.0)||(a > b)) {
						distSP[edgeMat[2*ii]-1] = b;
					}
					ROS_INFO_NAMED("pleasework"," distance element %d was changed to %.1f", edgeMat[2*ii], b);
				}
			}
		}
		visit[currentSP-1] = true;
		minVal = -1.0;
		ROS_INFO_NAMED("pleasework"," current node: %d ", currentSP);
		for (int iii = 0; iii<Nstate; iii++) {
			if (!visit[iii]) {
				if (minVal==-1.0) {
					minVal = distSP[iii];
					minInd = iii;
				} else if ((distSP[iii] < minVal)&&(distSP[iii] >= 0)) {
					minVal = distSP[iii];
					minInd = iii;
				}
				ROS_INFO_NAMED("pleasework","Unvisitied, minInd= %d", minInd);
			}
		}
		currentSP = minInd+1;
		if (minInd+1 == goalstateNum) {
			done = true;
			ROS_INFO_NAMED("pleasework","Minimum Distance = %.1f",minVal);
		}
	}


	/////////////////////////////////////////////////////////////////////////////////////


	visual_tools.prompt("done initializing workspace. press next to init pose");

	// SET INITIAL POSE
	move_group.setEndEffectorLink("panda_link8");
	geometry_msgs::Pose initpose;
	tf2::Quaternion initorient;
	initorient.setRPY(-pi/2, -pi/4, -pi/2);
	initpose.orientation = tf2::toMsg(initorient);

	initpose.position.x = .4;
	initpose.position.y = 0;
	initpose.position.z = .15;
	move_group.setPoseTarget(initpose);
	move_group.setPlanningTime(15.0);
	moveit::planning_interface::MoveGroupInterface::Plan initplan;
	move_group.plan(initplan);
	move_group.execute(initplan);

	visual_tools.prompt("done initializing. press to embark! :)");

	///////////////////////////// Pick and Place Struct /////////////////////////////////
	/*
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
	   float gripwidth;
	   float setback; 
	   pickstruc, placestruc;
	   */

	pickstruc.objID = colObj[3].id;
	pickstruc.supportsurface = "table1";
	pickstruc.posx = colObj[3].primitive_poses[0].position.x-.075;
	pickstruc.posy = colObj[3].primitive_poses[0].position.y;
	pickstruc.posz = colObj[3].primitive_poses[0].position.z;
	pickstruc.approachx = 1.0;
	pickstruc.approachy = 0;
	pickstruc.approachz = 0;
	pickstruc.retreatx = 0;
	pickstruc.retreaty = 0;
	pickstruc.retreatz = 1.0;
	pickstruc.roll = -pi/2;
	pickstruc.pitch = -pi/4;
	pickstruc.yaw = -pi/2;
	pickstruc.gripwidth = thingWidth;
	pickstruc.setback = thingLength/2;

	placestruc.objID = colObj[3].id;
	placestruc.supportsurface = "table1";
	placestruc.posx = -.6;
	placestruc.posy = 0;
	placestruc.posz = thingHeight/2;
	placestruc.approachx = 0;
	placestruc.approachy = 0;
	placestruc.approachz = -1.0;
	placestruc.retreatx = 1.0;
	placestruc.retreaty = 0;
	placestruc.retreatz = 0;
	placestruc.roll = 0;
	placestruc.pitch = 0;
	placestruc.yaw = pi;
	placestruc.gripwidth = thingWidth;
	placestruc.setback = thingLength/2;

	/////////////////////////////////////////////////////////////////////////////////////
	int Ntasks = 3;
	int task_objOrder [Ntasks] = {1,2,3};
	double task_objPick [Ntasks][12] {{colObj[2].primitive_poses[0].position.x-.075, colObj[2].primitive_poses[0].position.y, colObj[2].primitive_poses[0].position.z, 1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, -pi/2}, {colObj[3].primitive_poses[0].position.x-.075, colObj[3].primitive_poses[0].position.y, colObj[3].primitive_poses[0].position.z, 1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, -pi/2}, {colObj[4].primitive_poses[0].position.x-.075, colObj[4].primitive_poses[0].position.y, colObj[4].primitive_poses[0].position.z, 1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, -pi/2}};
	double task_objPlace [Ntasks][12] {{0, -.4, thingHeight/2+.0005, 0, 0, -1, 1, 0, 0, 0, 0, pi}, {0, .4, thingHeight/2+.0005, 0, 0, -1, 1, 0, 0, 0, 0, pi}, {0, 0, thingHeight+thingWidth/2+.005, -1, 0, 0, 1, 0, 0, pi/2, 0, pi}};



	for (int i=0; i<Ntasks; i++) {
		int obj = 1+task_objOrder [i];
		pickstruc.objID = colObjIDs[obj];
		pickstruc.posx = task_objPick [i][0];
		pickstruc.posy = task_objPick [i][1];
		pickstruc.posz = task_objPick [i][2];
		pickstruc.approachx = task_objPick [i][3];
		pickstruc.approachy = task_objPick [i][4];
		pickstruc.approachz = task_objPick [i][5];
		pickstruc.retreatx = task_objPick [i][6];
		pickstruc.retreaty = task_objPick [i][7];
		pickstruc.retreatz = task_objPick [i][8];
		pickstruc.roll = task_objPick [i][9];
		pickstruc.pitch = task_objPick [i][10];
		pickstruc.yaw = task_objPick [i][11];

		placestruc.objID = colObjIDs[obj];
		placestruc.posx = task_objPlace [i][0];
		placestruc.posy = task_objPlace [i][1];
		placestruc.posz = task_objPlace [i][2];
		placestruc.approachx = task_objPlace [i][3];
		placestruc.approachy = task_objPlace [i][4];
		placestruc.approachz = task_objPlace [i][5];
		placestruc.retreatx = task_objPlace [i][6];
		placestruc.retreaty = task_objPlace [i][7];
		placestruc.retreatz = task_objPlace [i][8];
		placestruc.roll = task_objPlace [i][9];
		placestruc.pitch = task_objPlace [i][10];
		placestruc.yaw = task_objPlace [i][11];

		move_group.setPlanningTime(20.0);
		ros::WallDuration(1.0).sleep();
		pickup(move_group, colObj, pickstruc);
		ros::WallDuration(1.0).sleep();
		place(move_group, placestruc);
		geometry_msgs::Pose currentObjPoses;
		planning_scene_interface.getObjectPoses(colObjIDs);
		//	ROS_INFO_NAMED("pleasework","Target x: %.2f y: %.2f z: %.2f", colObj[3].primitive_poses[0].position.x-.085, colObj[3].primitive_poses[0].position.y, colObj[3].primitive_poses[0].position.z);

	}



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

void pickup(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::CollisionObject> colObjIns, objStruc objS) {	
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
	ROS_INFO_NAMED("pleasework","Target x: %.2f y: %.2f z: %.2f", objS.posx-objS.setback, objS.posy, objS.posz);
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
	move_group.pick(objS.objID, grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group, objStruc objS) {
	std::vector<moveit_msgs::PlaceLocation> placeloc;
	placeloc.resize(1);
	placeloc[0].place_pose.header.frame_id = "panda_link0";
	tf2::Quaternion orient;
	//orient.setRPY(0, 0, pi);
	orient.setRPY(objS.roll, objS.pitch, objS.yaw);	
	placeloc[0].place_pose.pose.orientation = tf2::toMsg(orient);
	placeloc[0].place_pose.pose.position.x = objS.posx; //this marks the center of the object
	placeloc[0].place_pose.pose.position.y = objS.posy;
	placeloc[0].place_pose.pose.position.z = objS.posz;

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
	move_group.place(objS.objID, placeloc);
	ROS_INFO_NAMED("pleasework","you finished placing");
}

