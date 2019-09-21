#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#define PI 3.14159

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/Constraints.h>
#include <visualization_msgs/Marker.h>

std::vector<geometry_msgs::Pose> create_manipulation_poses(double retreat_dis,
		double approach_dis,const tf::Transform &target_tf);

std::vector<geometry_msgs::Pose> transform_from_tcp_to_wrist(tf::Transform tcp_to_wrist_tf,
		const std::vector<geometry_msgs::Pose> tcp_poses);

std::ostream& operator<<(std::ostream& os, const tf::Vector3 vec);
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point pt);

moveit_msgs::Constraints create_path_orientation_constraints(const geometry_msgs::Pose &goal_pose,
		float x_tolerance,float y_tolerance,float z_tolerance,std::string link_name);


namespace IRIM
{
  class pick_and_place_config
  {
  public:

    // Used parameter
    std::string ARM_GROUP_NAME;  // MoveIt Planning Group associated with the robot arm
    std::string END_EFFECTOR_LINK;  // Moveit End Effector
    std::string WORLD_FRAME_ID;  // Frame name for the fixed world reference frame
    std::string BACE_FRAME_ID;  // Frame name for the fixed world reference frame
    std::string TARGET_NAME;
    std::string SAFEPOINT_NAME;
    std::string VERTICAL_SAFEPOINT_NAME;
    std::string PLACEPOINT_NAME;
    std::string HOME_POSE_NAME;  // Named pose for robot Home position 
    std::string WAIT_POSE_NAME;  // Named pose for robot WAIT position 
    double RETREAT_DISTANCE;     // Distance to back away from pick/place pose after grasp/release
    double APPROACH_DISTANCE;    // Distance to stand off from pick/place pose before grasp/release
    double ACCELERATE_SCALAR_FACTOR;   // The factor of your scalar accelerate
    double VELOCITY_SCALAR_FACTOR;     // The factor of your scalar velocity
    double ATTEMPT_TIMES;              // The attempt times
    double ALLOW_TIME;
    std::string MOTION_PLAN_SERVICE; // service for requesting moveit for a motion plan
    tf::Transform BOX_PLACE_TF;  // Transform from the WORLD frame to the PLACE location

    pick_and_place_config()
    {
      ARM_GROUP_NAME  = "manipulator";
      END_EFFECTOR_LINK = "ee_link";
      WORLD_FRAME_ID = "world_frame";
      BACE_FRAME_ID = "base_link"; 
      TARGET_NAME = "target_position";
      SAFEPOINT_NAME = "safe_point";
      VERTICAL_SAFEPOINT_NAME = "vertical_safe_point";
      PLACEPOINT_NAME = "place";
      HOME_POSE_NAME  = "home";
      WAIT_POSE_NAME  = "wait";
      MOTION_PLAN_SERVICE = "plan_kinematic_path";
      ACCELERATE_SCALAR_FACTOR = 1;
      VELOCITY_SCALAR_FACTOR = 1;
      RETREAT_DISTANCE  = 0.05;
      APPROACH_DISTANCE = 0.05;
      ALLOW_TIME = 2;
      ATTEMPT_TIMES = 10;
      BOX_PLACE_TF  = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0,0,0));
    }

  };
};

#endif /* PICK_AND_PLACE_UTILITIES_H_ */