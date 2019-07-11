// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace visualization_msgs;

const std::string fixed_frame_id = "base_link";
static const std::string PLANNING_GROUP = "crane_arm";


void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of crane */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gbase_to_lg";
  posture.joint_names[1] = "gbase_to_rg";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0; //from urdf upper value in the respectful joint
  posture.points[0].positions[1] = 0; //from urdf upper value in the respectful joint
  posture.points[0].time_from_start = ros::Duration(1.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of crane */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gbase_to_lg";
  posture.joint_names[1] = "gbase_to_rg";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.125/* 0.125 */; //move from finger's origins so that distance between fingers will be equal to width of picking object
  posture.points[0].positions[1] = 0.125/* 0.125 */; 
  posture.points[0].time_from_start = ros::Duration(1.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group,moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    ROS_INFO_STREAM("PICKING");

    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    // This is the pose of link7 (fake-empty link)
    grasps[0].grasp_pose.header.frame_id = fixed_frame_id;
    tf2::Quaternion orientation;
    //RPY - (Roll-x Pitch-y Yaw-z)
    //THIS IS ORIENTATION OF GRIPPER
    orientation.setRPY(M_PI, 0.0 , M_PI / 2); // third value should depend on the third value of picking pipe, so gripper can grab the pipe
    double  height_pipe, height_finger, height_base, some_space;

    height_pipe   = 0.5;
    height_finger = 1.0; //from urdf rgripper/lgripper z value in geometry
    height_base   = 0.5; //from urdf gripper_base z value in geometry
    some_space    = 0.1; //just assumpton to be 0.1
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = -12.0;   // center_of_pipe_x
    grasps[0].grasp_pose.pose.position.y = 0.0;   // center_of_pipe_y
    grasps[0].grasp_pose.pose.position.z = 0.45 - 0.5*height_pipe + height_finger + height_base + some_space;

    // Setting pre-grasp approach
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = fixed_frame_id;
    /* Direction is set as negative z axis */
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0; // in our case it approaches in z direction
    grasps[0].pre_grasp_approach.min_distance = height_pipe*0.5;     // some distance shorter than the next. May be half of height of pipe + some gap. So that fingers do not touch the ground
    grasps[0].pre_grasp_approach.desired_distance = height_pipe*0.5+some_space; 

    // Setting post-grasp retreat
    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = fixed_frame_id;
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = height_pipe*0.5;
    grasps[0].post_grasp_retreat.desired_distance = height_pipe*0.5+some_space;

    // Setting posture of eef before grasp
    openGripper(grasps[0].pre_grasp_posture);

    // Setting posture of eef during grasp
    closedGripper(grasps[0].grasp_posture);


  std::map<std::string, moveit_msgs::AttachedCollisionObject> att_obs = planning_scene_interface.getAttachedObjects();
  ROS_INFO_STREAM("------ATTACHED:");
  for (auto att_ob : att_obs)
    ROS_INFO_STREAM(att_ob.first);
  ROS_INFO_STREAM("------COLLISION:");
  ros::V_string col_names = planning_scene_interface.getKnownObjectNames();
  for (auto col_name : col_names)
    ROS_INFO_STREAM(col_name);

    move_group.setSupportSurfaceName("cage_floor");
    move_group.pick("pipe", grasps);
    
}

void place(moveit::planning_interface::MoveGroupInterface& group,moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  ROS_INFO_STREAM("PLACING");
    

  // Create a vector of placings to be attempted.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  //  place_location.resize(2);
  place_location.resize(1);

  // Setting place location pose
  place_location[0].place_pose.header.frame_id = fixed_frame_id;
  tf2::Quaternion orientation, orientation2;
  orientation.setRPY(M_PI/2.0, 0.0 , 0.0); //this was the rpy when the pipe was created
  // THIS IS THE DESIRED ORIENTATION OF PIPE(NOT GRIPPER)
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);


  /* While placing it is the exact location of the center of the object. */
  double height_pipe   = 0.5;

  place_location[0].place_pose.pose.position.x = 5.0;
  place_location[0].place_pose.pose.position.y = 0.0;
  place_location[0].place_pose.pose.position.z = 0.5+0.5*height_pipe/* 1.65 */; //height of table + 1/2 height of object

  ROS_INFO_STREAM("Place location ("<<place_location[0].place_pose.pose.position.x<<" , "<<place_location[0].place_pose.pose.position.y<<" , "<<place_location[0].place_pose.pose.position.z<<")");

  // Setting pre-place approach
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = fixed_frame_id;
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.25; // 
  place_location[0].pre_place_approach.desired_distance = 0.35; //

  // Setting post-grasp retreat
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = fixed_frame_id;
  /* Direction is set as positive z axis */
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.25;
  place_location[0].post_place_retreat.desired_distance = 0.35;

  // Setting posture of eef after placing object
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("feed_table");
  
  /* check which objects are attached and which are collision */
  std::map<std::string, moveit_msgs::AttachedCollisionObject> att_obs = planning_scene_interface.getAttachedObjects();
  ROS_INFO_STREAM("------ATTACHED:");
  for (auto att_ob : att_obs)
    ROS_INFO_STREAM(att_ob.first);
  ROS_INFO_STREAM("------COLLISION:");
  ros::V_string col_names = planning_scene_interface.getKnownObjectNames();
  for (auto col_name : col_names)
    ROS_INFO_STREAM(col_name);

// Call place to place the object using the place locations given.
  group.place("pipe", place_location);

}



void buildBasicScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    /* clear all old objects */
    planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);
    /* ground */
    collision_objects[0].id = "ground";
    collision_objects[0].header.frame_id = fixed_frame_id;
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0]=50;
    collision_objects[0].primitives[0].dimensions[1]=50;
    collision_objects[0].primitives[0].dimensions[2]=0.2;
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x=0.0;
    collision_objects[0].primitive_poses[0].position.y=0.0;
    collision_objects[0].primitive_poses[0].position.z=-1.0;
    collision_objects[0].operation = collision_objects[0].ADD;

    /* feed table */   
    collision_objects[1]=collision_objects[0];
    collision_objects[1].id = "feed_table";
    collision_objects[1].primitives[0].dimensions[0]=2;
    collision_objects[1].primitives[0].dimensions[1]=20;
    collision_objects[1].primitives[0].dimensions[2]=0.5;
    collision_objects[1].primitive_poses[0].position.x=5.0;
    collision_objects[1].primitive_poses[0].position.y=0.0;
    collision_objects[1].primitive_poses[0].position.z=0.25;

    /* cage floor */ 
    collision_objects[2]=collision_objects[1];
    collision_objects[2].id = "cage_floor";
    double const cage_Wx = 10.0, cage_Ly=20.0, cage_Hz=0.2, cage_X=-10.0, cage_Y=0.0, cage_Z=0.1;
    collision_objects[2].primitives[0].dimensions[0]=cage_Wx;
    collision_objects[2].primitives[0].dimensions[1]=cage_Ly;
    collision_objects[2].primitives[0].dimensions[2]=cage_Hz;
    collision_objects[2].primitive_poses[0].position.x=cage_X;
    collision_objects[2].primitive_poses[0].position.y=cage_Y;
    collision_objects[2].primitive_poses[0].position.z=cage_Z;

    /* pipe */
    double const pipe_l = 15.0, pipe_r = 0.25;
    collision_objects[3].header.frame_id = fixed_frame_id;
    collision_objects[3].id = "pipe";
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].CYLINDER;
    collision_objects[3].primitives[0].dimensions.resize(2);
    collision_objects[3].primitives[0].dimensions[0] = 15.0; //length
    collision_objects[3].primitives[0].dimensions[1] = 0.25; //radius

    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = -12.0;
    collision_objects[3].primitive_poses[0].position.y = cage_Y;
    collision_objects[3].primitive_poses[0].position.z = cage_Z+cage_Hz/2.0+pipe_r;
    tf2::Quaternion orientation;
    orientation.setRPY(M_PI/2.0, 0.0 , 0.0); //RPY - (Roll-x Pitch-y Yaw-z)
    collision_objects[3].primitive_poses[0].orientation = tf2::toMsg(orientation);
    collision_objects[3].operation = collision_objects[3].ADD;

    /* adding pipes */
    /* add to planning scene */
    planning_scene_interface.applyCollisionObjects(collision_objects);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
  group.setPlanningTime(45.0);


    for (auto i = 0 ; i< 10; i++)
    {
        buildBasicScene(planning_scene_interface);
        ros::WallDuration(1.0).sleep();

        pick(group,planning_scene_interface);
        ros::WallDuration(1.0).sleep();

        place(group,planning_scene_interface);
        ros::WallDuration(1.0).sleep();
    }


  ros::waitForShutdown();
  return 0;
}
