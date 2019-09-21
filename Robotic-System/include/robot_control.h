#ifndef PICK_AND_PLACE_H_
#define PICK_AND_PLACE_H_

#include "device_controler.h"
#include "robot_config.h"
#include "obstable_manager.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/shape_operations.h>


// =============================== aliases ===============================
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlaningScenceInterfacePtr;
typedef boost::shared_ptr<IRIM::pick_and_place_config> ConfigPtr;
typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
typedef boost::shared_ptr<IRIM::Obstable_Manager> ObsManaPtr;

namespace IRIM
{
	class PickAndPlace
	{
	public:	
		PickAndPlace()
		{
			ptrNodeHandle = NodeHandlePtr(new ros::NodeHandle());
			ptrConfig = ConfigPtr(new IRIM::pick_and_place_config());
            ptrPlanScenceInterface = PlaningScenceInterfacePtr(new moveit::planning_interface::PlanningSceneInterface());
			ptrMovegroup = MoveGroupPtr(new moveit::planning_interface::MoveGroup(ptrConfig->ARM_GROUP_NAME));
            motion_plan_client = ptrNodeHandle->serviceClient<moveit_msgs::GetMotionPlan>(ptrConfig->MOTION_PLAN_SERVICE);
			sleep(1);
			UR_pub = ptrNodeHandle->advertise<std_msgs::String>("/ur_driver/URScript",20);	
			position_pub = ptrNodeHandle->advertise<std_msgs::String>("/robot_current_position", 20);	   
			sleep(1);
			continue_flag = false;
			position_msg.data = "lower";
		};

		MoveGroupPtr ptrMovegroup;
		ConfigPtr ptrConfig;
		NodeHandlePtr ptrNodeHandle;
        PlaningScenceInterfacePtr ptrPlanScenceInterface;
		ros::ServiceClient motion_plan_client;
		ros::Publisher UR_pub;
		ros::Publisher position_pub;

        std_msgs::String position_msg; 

		IRIM::VALE_CONTROLER vale_controler;
		IRIM::LIFT_CONTROLER lift_controler;

		void robot_init();
		bool create_motion_plan(const geometry_msgs::Pose &pose_target,
        const moveit_msgs::RobotState &start_robot_state, moveit::planning_interface::MoveGroupInterface::Plan &plan);
		tf::StampedTransform listen_to_transform();
		void cartesian_move(geometry_msgs::Pose &current_pose ,geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface::Plan &plan);
		void save_transform(std::string object_type, tf::StampedTransform &tranform);
		void read_offset_info(string object_type, double &x_offset,  double &y_offset, double &z_offset);
		void desk_save_transform(tf::StampedTransform &tranform);
		void add_collision_high_point();
		void add_collision_middle_point();
		void add_collision_low_point();
		void move_to_home();
		void move_to_home_UR();

		void move_to_AGV();

		void move_to_home_UR_no_wait();
		void home_to_safepoint_UR();
		void move_to_safepoint();
		void move_to_placepoint(std::string placepoint, bool is_vertical);

		void placepoint_to_safepoint_UR();

		void retreat_to_safepoint_UR();
		
		void pick();
		void UR_pick(std::string placepoint);
		void UR_pick_special();
		void move_to_approch_point();

		void safepoint_approach_UR();

		void manipulator_impact();

		void move_to_low_placepoint(bool is_vertical);
		void move_to_middle_placepoint(bool is_vertical);
		void move_to_high_placepoint(bool is_vertical);
		bool get_continue_flag(){return continue_flag;};
		bool is_at_home_position();

		geometry_msgs::Pose pose_target;
		bool continue_flag;
		void add_offset(std::string object_type);
		void add_offset_origin(std::string object_type);
		
	};

	class VERTICAL_PickAndPlace: public PickAndPlace
	{
		public:

		void save_transform(std::string object_type, tf::StampedTransform &tranform);
		void desk_save_transform(tf::StampedTransform &tranform);
		void move_to_safepoint();
		void pick();
		void UR_pick();
		void UR_pick_special();
		void home_to_v_safepoint_UR();
		void move_to_approch_point();
		void v_safepoint_approach_UR();
		void v_retreat_to_safepoint_UR();
		void placepoint_to_v_safepoint_UR();
	};

	
};

#endif /* PICK_AND_PLACE_H_ */