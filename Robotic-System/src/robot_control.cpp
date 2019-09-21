#include "robot_control.h"
#include "offset.h"
#include "device_controler.h"
#include <opencv2/core/core.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <fstream>
#include <string>


namespace IRIM
{

	void PickAndPlace::robot_init()
    {
        ROS_INFO("ROBOT INIT");
        ptrMovegroup->setMaxAccelerationScalingFactor (ptrConfig->ACCELERATE_SCALAR_FACTOR);
        ptrMovegroup->setMaxVelocityScalingFactor(ptrConfig->VELOCITY_SCALAR_FACTOR);
        ptrMovegroup->setNumPlanningAttempts(ptrConfig->ATTEMPT_TIMES);
        ptrMovegroup->setPlanningTime(ptrConfig->ALLOW_TIME);
        ptrMovegroup->setEndEffectorLink(ptrConfig->END_EFFECTOR_LINK);
    };

    bool PickAndPlace::create_motion_plan(const geometry_msgs::Pose &pose_target, const moveit_msgs::RobotState &start_robot_state, moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        std::vector<double> position_tolerances(3,0.01f);
        std::vector<double> orientation_tolerances(3,0.01f);
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "world";
        p.pose = pose_target;
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(ptrConfig->END_EFFECTOR_LINK,p,position_tolerances,
                                                orientation_tolerances);
        moveit_msgs::GetMotionPlan motion_plan;
        moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
        moveit_msgs::MotionPlanResponse &res = motion_plan.response.motion_plan_response;
        req.start_state = start_robot_state;
        req.start_state.is_diff = false;
        req.group_name = ptrConfig->ARM_GROUP_NAME;
        req.goal_constraints.push_back(pose_goal);
        req.allowed_planning_time = 3.0f;
        req.num_planning_attempts = 10;

        bool success = false;
        if(motion_plan_client.call(motion_plan))
        {
            plan.start_state_ = res.trajectory_start;
            plan.trajectory_ = res.trajectory;
            success = true;
        }
        return success;
    }

    void PickAndPlace::move_to_home()
    {
        ROS_INFO("ROBOT MOVE TO HOME");
        ptrMovegroup->setStartState(*ptrMovegroup->getCurrentState());
        ptrMovegroup->setNamedTarget(ptrConfig->HOME_POSE_NAME);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        ptrMovegroup->plan(plan);
        ptrMovegroup->execute(plan);
    }

    void PickAndPlace::move_to_home_UR()
    {
        ROS_INFO("ROBOT MOVE TO HOME");
        std_msgs::String msg_home;
        //msg_home.data = "movej(p[-0.16,-0.09793,0.46128,1.9490,-0.0615,2.5272],0.3,2)";
        msg_home.data = "movej([-1.6246679464923304, -1.3621934095965784 , -1.878751579915182, -1.4857099691974085 ,1.5331093072891235, 1.2118388414382935],0.5,3)";
        UR_pub.publish(msg_home);
        sleep(5);
    }

    void PickAndPlace::move_to_AGV()
    {
        ROS_INFO("ROBOT MOVE TO AGV");
        std_msgs::String msg_agv;
        //msg_home.data = "movej(p[-0.16,-0.09793,0.46128,1.9490,-0.0615,2.5272],0.3,2)";
        msg_agv.data = "movej([-1.2111318747149866, -2.619988743458883, -0.5727618376361292, -0.44539004961122686, 1.5552667379379272, 1.2116594314575195],0.5,3)";
        UR_pub.publish(msg_agv);
        sleep(6);
        vale_controler.undo_it();
    }


    void PickAndPlace::move_to_home_UR_no_wait()
    {
        ROS_INFO("ROBOT MOVE TO HOME");
        std_msgs::String msg_home;
        //msg_home.data = "movej(p[-0.16,-0.09793,0.46128,1.9490,-0.0615,2.5272],0.3,2)";
        msg_home.data = "movej([-1.6246679464923304, -1.3621934095965784 , -1.878751579915182, -1.4857099691974085 ,1.5331093072891235, 1.2118388414382935],0.5,3)";
        UR_pub.publish(msg_home);
        sleep(0.5);
    }

    void PickAndPlace::home_to_safepoint_UR()
    {
        ROS_INFO("UR HOME TO SAFEPOINT");
        std_msgs::String msg_home_to_safepoint;
        msg_home_to_safepoint.data = "movej([-2.645116154347555, -1.5689762274371546 , -2.314948383961813 ,-0.7765196005450647, 1.6100679636001587, 0.6629928350448608],0.5,3)";
        UR_pub.publish(msg_home_to_safepoint);
        sleep(3.5);
    }

    void PickAndPlace::placepoint_to_safepoint_UR()
    {
        ROS_INFO("UR HOME TO SAFEPOINT");
        std_msgs::String msg_home_to_safepoint;
        msg_home_to_safepoint.data = "movej([-2.645116154347555, -1.5689762274371546 , -2.314948383961813 ,-0.7765196005450647, 1.6100679636001587, 0.6629928350448608],0.5,3)";
        UR_pub.publish(msg_home_to_safepoint);
    // We shorten the pause duration here
        sleep(4);
    }

    void PickAndPlace::safepoint_approach_UR()
    {
        ROS_INFO("UR SAFEPOINT TO APPROACH");
        geometry_msgs::Pose approch_point = pose_target;
        approch_point.position.x -= ptrConfig->APPROACH_DISTANCE;

        cout << "The UR position is (" << approch_point.position.x *-1 <<", " << std::to_string(approch_point.position.y* -1) << "," << std::to_string(approch_point.position.z* 1) << ")";
        string out_string = "movel(p[" + std::to_string( approch_point.position.x *-1) +","+ std::to_string(approch_point.position.y* -1) +","+ std::to_string(approch_point.position.z) + ",2.2394, -0.2495, -2.2941],0.5,3)";
        
        std_msgs::String msg_ur_approach;
        msg_ur_approach.data = out_string;
        UR_pub.publish(msg_ur_approach);
        //sleep(2);
    }

    void VERTICAL_PickAndPlace::v_safepoint_approach_UR()
    {
        ROS_INFO("UR APPROACH TO TARGET POSITION");
        geometry_msgs::Pose approch_point = pose_target;
        approch_point.position.x -= ptrConfig->APPROACH_DISTANCE;

        cout << "The UR position is (" << approch_point.position.x *-1 <<", " << std::to_string(approch_point.position.y* -1) << "," << std::to_string(approch_point.position.z* 1) << ")";
        string out_string = "movel(p[" + std::to_string( approch_point.position.x *-1) +","+ std::to_string(approch_point.position.y* -1) +","+ std::to_string(approch_point.position.z) + ",0.0153, 3.1589, 0.1015],0.5,3)";
        
        std_msgs::String msg_ur_approach;
        msg_ur_approach.data = out_string;
        UR_pub.publish(msg_ur_approach);
        //sleep(2);
    }

    void VERTICAL_PickAndPlace::v_retreat_to_safepoint_UR()
    {
        ROS_INFO("UR RETREAT TO SAFEPOINT");
        std_msgs::String msg_retreat_to_safepoint;
        msg_retreat_to_safepoint.data = "movej([-2.645116154347555,-1.5689762274371546,-2.314948383961813,-0.7765196005450647,1.6100679636001587,0.6629928350448608],0.9,2)";
        UR_pub.publish(msg_retreat_to_safepoint);
        sleep(4);
    }

    void PickAndPlace::retreat_to_safepoint_UR()
    {
        ROS_INFO("UR RETREAT TO SAFEPOINT");
        std_msgs::String msg_retreat_to_safepoint;
        //msg_retreat_to_safepoint.data = "movej([-2.645116154347555,-1.5689762274371546,-2.314948383961813,-0.7765196005450647,1.6100679636001587,0.6629928350448608],0.9,2)";
        msg_retreat_to_safepoint.data = "movej([-2.664550844823019, -1.7695110479937952, -2.301850144063131, -0.711677376423971, 1.5434269905090332, 0.4423557221889496],0.9,3)";
        UR_pub.publish(msg_retreat_to_safepoint);
        sleep(1.5);
    }

    void VERTICAL_PickAndPlace::home_to_v_safepoint_UR()
    {
        ROS_INFO("UR HOME TO VERTICAL SAFEPOINT");
        std_msgs::String msg_home_to_v_safepoint;
        msg_home_to_v_safepoint.data = "movej([-2.794180695210592, -1.8688538710223597 , -2.363633696232931 ,-2.031231705342428, -1.208346192036764, -0.06831533113588506],2,4)";
        UR_pub.publish(msg_home_to_v_safepoint);
        sleep(3.5);
    }

    void VERTICAL_PickAndPlace::placepoint_to_v_safepoint_UR()
    {
        ROS_INFO("UR HOME TO VERTICAL SAFEPOINT");
        std_msgs::String msg_home_to_v_safepoint;
        msg_home_to_v_safepoint.data = "movej([-2.794180695210592, -1.8688538710223597 , -2.363633696232931 ,-2.031231705342428, -1.208346192036764, -0.06831533113588506],2,4)";
        UR_pub.publish(msg_home_to_v_safepoint);
    // We shorten the pause duration here
        sleep(3.5);
    }

    void PickAndPlace::move_to_safepoint()
    {
        sleep(1);        
        ptrMovegroup->setStartState(*ptrMovegroup->getCurrentState());
        ROS_INFO("ROBOT MOVE TO SAFE POSITION");  
        ptrMovegroup->setNamedTarget(ptrConfig->SAFEPOINT_NAME);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        ptrMovegroup->plan(plan);
        ptrMovegroup->execute(plan);
    }

    void PickAndPlace::manipulator_impact()
    {
        ROS_INFO("UR MOVE TO 0");
        std_msgs::String msg_to_0;
        msg_to_0.data = "movej([-3.1716352144824427, -1.4446561972247522 , -1.0771716276751917 ,0.43171608448028564, 1.5690597295761108, 1.1802091598510742],0.5,3)";
        UR_pub.publish(msg_to_0);
        sleep(6);

        ROS_INFO("UR MOVE TO 1");
        std_msgs::String msg_to_1;
        msg_to_1.data = "movej([-3.208642546330587, -1.8804782072650355, -1.8680761496173304, 1.5377432107925415, 1.5692754983901978, 1.1801971197128296],0.5,3)";
        UR_pub.publish(msg_to_1);
        sleep(4);

        ROS_INFO("UR MOVE TO 2");
        std_msgs::String msg_to_2;
        msg_to_2.data = "movej([-3.208606545125143, -1.8805020491229456, -1.8680880705462855, -0.0985186735736292, 1.5692875385284424, 1.180245041847229],0.5,3)";
        UR_pub.publish(msg_to_2);
        sleep(2.5);

    };


    void PickAndPlace::move_to_low_placepoint(bool is_vertical)
    {
        ROS_INFO("LOW PLACE ");
        std_msgs::String msg_guodu;
        std_msgs::String msg_place;
        msg_guodu.data = "movel(p[-0.4,-0.35,0.15,1.6401,0.9513,-1.6235],0.5,3)";
        msg_place.data = "movel(p[0.25,-0.45,0.15,1.6401,0.9513,-1.6235],0.5,3)";
        UR_pub.publish(msg_guodu);
        //sleep(2.2);
        if(is_vertical)
        {
            //sleep(1.5);
            sleep(2);
        }
        else
        {
            //sleep(3);
            sleep(3.5);
        }

        UR_pub.publish(msg_place);
        sleep(2);
        vale_controler.undo_it();
        /*
        ptrMovegroup->setStartState(*ptrMovegroup->getCurrentState());
        ROS_INFO("ROBOT MOVE TO VERITICAL PLACE POSITION");  
        ptrMovegroup->setNamedTarget(ptrConfig->PLACEPOINT_NAME);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        ptrMovegroup->plan(plan);
        ptrMovegroup->execute(plan);
        vale_controler.undo_it();
        sleep(2);
*/

        /*geometry_msgs::Pose target_pose;
        target_pose.orientation.w = 0.707;
        target_pose.orientation.x = 0;
        target_pose.orientation.y = 0.707;
        target_pose.orientation.z = 0;

        target_pose.position.x = 0.6;
        target_pose.position.y = -0.09;
        target_pose.position.z = 0.04;
        ptrMovegroup->setPoseTarget(target_pose);
        ptrMovegroup->move();
        ROS_INFO("ROBOT MOVE TO PLACE POSITION");
        moveit_msgs::Constraints endEffector_constraints;
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "ee_link";   
        ocm.header.frame_id = ptrConfig->BACE_FRAME_ID;
        ocm.orientation.w = 1;
        ocm.absolute_x_axis_tolerance = 0.5;
        ocm.absolute_y_axis_tolerance = 0.5;
        ocm.absolute_z_axis_tolerance = 2*3.14;
        ocm.weight = 1.0;
        endEffector_constraints.orientation_constraints.push_back(ocm);
        ptrMovegroup->setPathConstraints(endEffector_constraints);
        target_pose.position.x = 0.59;
        target_pose.position.y = -0.32;
        target_pose.position.z = 0.03;
        ptrMovegroup->setPoseTarget(target_pose);
        //ptrMovegroup->setNamedTarget(ptrConfig->PLACEPOINT_NAME);
        ptrMovegroup->move();
        ptrMovegroup->clearPathConstraints();
        sleep(2);
        */
    }

	void PickAndPlace::move_to_middle_placepoint(bool is_vertical)
    {
        //high
        ROS_INFO("MIDDLE PLACE ");
        std_msgs::String msg_guodu;
        std_msgs::String msg_place;
        //sleep(0.5);
        msg_guodu.data = "movel(p[-0.4,-0.35,-0.15,1.6401,0.9513,-1.6235],0.5,3)";
        msg_place.data = "movel(p[0.25,-0.45,-0.15,1.6401,0.9513,-1.6235],0.5,3)";
        UR_pub.publish(msg_guodu);
        if(is_vertical)
        {
            sleep(2);
        }
        else
        {
            sleep(3);
        }
        UR_pub.publish(msg_place);
        sleep(2);
        vale_controler.undo_it();

    };

	void PickAndPlace::move_to_high_placepoint(bool is_vertical)
    {
        ROS_INFO("HIGH PLACE ");
        std_msgs::String msg_guodu;
        std_msgs::String msg_place;
        //sleep(0.5);        
        msg_guodu.data = "movel(p[-0.4,-0.35,-0.30,1.6401,0.9513,-1.6235],0.5,3)";
        msg_place.data = "movel(p[0.25,-0.45,-0.30,1.6401,0.9513,-1.6235],0.5,3)";
        UR_pub.publish(msg_guodu);
        if(is_vertical)
        {
            sleep(2);
        }
        else
        {
            sleep(3);
        }
        UR_pub.publish(msg_place);
        sleep(2);
        vale_controler.undo_it();

    };

    void PickAndPlace::move_to_placepoint(std::string placeposition, bool is_vertical)
    {
        if(!strcmp(placeposition.c_str(), "low"))
        {
            move_to_low_placepoint(is_vertical);
        }
        else if(!strcmp(placeposition.c_str(), "middle"))
        {
            move_to_middle_placepoint(is_vertical);
        }
        else if(!strcmp(placeposition.c_str(), "high"))
        {
            move_to_high_placepoint(is_vertical);
        }
    };


    void PickAndPlace::save_transform(std::string object_type, tf::StampedTransform &tranform)
    {

        ROS_INFO("SAVE TRANSFORM ");        
        std::cin.clear();        
        pose_target.position.x = tranform.getOrigin().x() + 0.051;
        pose_target.position.y = tranform.getOrigin().y() - 0.005;
        pose_target.position.z = tranform.getOrigin().z() + 0.065;
        std::cout << "The position we chose is ";
        std::cout << "(" << pose_target.position.x << ", " << pose_target.position.y << ", " << pose_target.position.z << ")" << std::endl;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,PI/2,0), pose_target.orientation);
        add_offset(object_type);
        std::cout << "Do you want to continue? press 'y' and 'u' will abort this result " << std::endl;
        char temp;
        temp = 'y';
        //std::cin >> temp;
        //std::cin.clear();
        continue_flag = (temp == 'y')?true:false;
    }


    void PickAndPlace::move_to_approch_point()
    {
        sleep(1);
        
        ROS_INFO("MOVE TO APPROACH POINT ");                        
        moveit_msgs::RobotState robot_state;
        robot_state::RobotStatePtr current_state = ptrMovegroup->getCurrentState();
        robot_state::robotStateToRobotStateMsg(*current_state, robot_state);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        geometry_msgs::Pose approch_point = pose_target;
        approch_point.position.z += ptrConfig->APPROACH_DISTANCE;
        create_motion_plan(approch_point, robot_state, plan);
        ptrMovegroup->execute(plan);
    }

    void PickAndPlace::pick()
    {
        ROS_INFO("PICK ");    
        sleep(2);
        IRIM::Obstable_Manager obs_manager(*ptrNodeHandle);
        //obs_manager.remove_obstacle();                      
        moveit_msgs::RobotState robot_state;
        robot_state::RobotStatePtr current_state= ptrMovegroup->getCurrentState();
        robot_state::robotStateToRobotStateMsg(*current_state,robot_state);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        geometry_msgs::Pose retreat_point = pose_target;
        retreat_point.position.z += ptrConfig->RETREAT_DISTANCE;
        vale_controler.do_it();
        create_motion_plan(pose_target, robot_state, plan);


        ptrMovegroup->execute(plan);

        sleep(2);
        
        ROS_INFO("RETREAT "); 
        current_state= ptrMovegroup->getCurrentState();
        robot_state::robotStateToRobotStateMsg(*current_state,robot_state);
        create_motion_plan(retreat_point, robot_state, plan);
        ptrMovegroup->execute(plan);
        sleep(1);
        //obs_manager.add_obstacle();  
    }

    void PickAndPlace::add_collision_high_point()
    {
        ROS_INFO("ADD COLLISION ");


        std::vector<std::string> objects_id;
        objects_id.push_back("Containter");
        objects_id.push_back("VERTICALBAN");
        objects_id.push_back("VERTICALBAN2");
        ptrPlanScenceInterface->removeCollisionObjects(objects_id);
        sleep(0.5);

        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = ptrMovegroup->getPlanningFrame();

        collision_object.id = "box1";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.3;
        primitive.dimensions[1] = 0.5;
        primitive.dimensions[2] = 0.3;

        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0;
        box_pose.position.y =  0;
        box_pose.position.z =  -0.15;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);

        // INSERT Left Back WALL  
        moveit_msgs::CollisionObject collision_object_BW;
        collision_object_BW.id = "BackWall";
        collision_object_BW.header.frame_id = ptrMovegroup->getPlanningFrame();
        
        primitive.dimensions[0] = 0.06;
        primitive.dimensions[1] = 0.18;
        primitive.dimensions[2] = 0.6;


        box_pose.orientation.w = 1.0;
        box_pose.position.x =  -0.2;
        box_pose.position.y =  0;
        box_pose.position.z =  0.3;
        collision_object_BW.primitives.push_back(primitive);
        collision_object_BW.primitive_poses.push_back(box_pose);
        collision_object_BW.operation = collision_object_BW.ADD;

        collision_objects.push_back(collision_object_BW);


        // INSERT CONTAINTER   
        /*moveit_msgs::CollisionObject collision_object_CTER;
        collision_object_CTER.id = "Containter";
        collision_object_CTER.header.frame_id = ptrMovegroup->getPlanningFrame();
        
        primitive.dimensions[0] = 0.47;
        primitive.dimensions[1] = 0.38;
        primitive.dimensions[2] = 0.17;


        box_pose.orientation.w = 1.0;
        box_pose.position.x =  -0.23;
        box_pose.position.y =  0.43;
        box_pose.position.z =  0.15;
        collision_object_CTER.primitives.push_back(primitive);
        collision_object_CTER.primitive_poses.push_back(box_pose);
        collision_object_CTER.operation = collision_object_CTER.ADD;

        collision_objects.push_back(collision_object_CTER);*/

         // INSERT VERTICALBAN2
        moveit_msgs::CollisionObject collision_object_VERTICALBAN2;
        collision_object_VERTICALBAN2.id = "VERTICALBAN2";
        collision_object_VERTICALBAN2.header.frame_id = ptrMovegroup->getPlanningFrame();

        primitive.dimensions[0] = 0.6;
        primitive.dimensions[1] = 1.8;
        primitive.dimensions[2] = 0.5;

        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0.978;
        box_pose.position.y =  0;
        //box_pose.position.z =  -0.54;        
        box_pose.position.z =  -0.332;
        collision_object_VERTICALBAN2.primitives.push_back(primitive);
        collision_object_VERTICALBAN2.primitive_poses.push_back(box_pose);
        collision_object_VERTICALBAN2.operation = collision_object_VERTICALBAN2.ADD;

        collision_objects.push_back(collision_object_VERTICALBAN2);


        // INSERT cell
        moveit_msgs::CollisionObject collision_object_cell;
        collision_object_cell.id = "cell";
        collision_object_cell.header.frame_id = ptrMovegroup->getPlanningFrame();

        primitive.dimensions[0] = 2;
        primitive.dimensions[1] = 2;
        primitive.dimensions[2] = 0.2;

        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0;
        box_pose.position.y =  0;        
        box_pose.position.z =  0.7;
        collision_object_cell.primitives.push_back(primitive);
        collision_object_cell.primitive_poses.push_back(box_pose);
        collision_object_cell.operation = collision_object_cell.ADD;

        collision_objects.push_back(collision_object_cell);

        ptrPlanScenceInterface->addCollisionObjects(collision_objects);

        sleep(0.5);        

    }

   void PickAndPlace::add_collision_low_point()
    {
        ROS_INFO("ADD COLLISION ");
        std::vector<std::string> objects_id;
        objects_id.push_back("Containter");
        objects_id.push_back("VERTICALBAN");
        objects_id.push_back("VERTICALBAN2");
        ptrPlanScenceInterface->removeCollisionObjects(objects_id);
        sleep(0.5);

        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = ptrMovegroup->getPlanningFrame();

        collision_object.id = "Base_base";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.3;
        primitive.dimensions[1] = 0.4;
        primitive.dimensions[2] = 0.3;

        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0;
        box_pose.position.y =  0;
        box_pose.position.z =  -0.15;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);

        // INSERT Left Back WALL  
        moveit_msgs::CollisionObject collision_object_BW;
        collision_object_BW.id = "BackWall";
        collision_object_BW.header.frame_id = ptrMovegroup->getPlanningFrame();
        
        primitive.dimensions[0] = 0.06;
        primitive.dimensions[1] = 0.18;
        primitive.dimensions[2] = 0.6;


        box_pose.orientation.w = 1.0;
        box_pose.position.x =  -0.2;
        box_pose.position.y =  0;
        box_pose.position.z =  0.3;
        collision_object_BW.primitives.push_back(primitive);
        collision_object_BW.primitive_poses.push_back(box_pose);
        collision_object_BW.operation = collision_object_BW.ADD;

        collision_objects.push_back(collision_object_BW);


        // INSERT CONTAINTER   
        moveit_msgs::CollisionObject collision_object_CTER;
        collision_object_CTER.id = "Containter";
        collision_object_CTER.header.frame_id = ptrMovegroup->getPlanningFrame();
        
        primitive.dimensions[0] = 0.47;
        primitive.dimensions[1] = 0.38;
        primitive.dimensions[2] = 0.17;


        box_pose.orientation.w = 1.0;
        box_pose.position.x =  -0.08;
        box_pose.position.y =  0.43;
        box_pose.position.z =  0.0;
        collision_object_CTER.primitives.push_back(primitive);
        collision_object_CTER.primitive_poses.push_back(box_pose);
        collision_object_CTER.operation = collision_object_CTER.ADD;

        collision_objects.push_back(collision_object_CTER);

        // INSERT VERTICALBAN
        moveit_msgs::CollisionObject collision_object_VERTICALBAN;
        collision_object_VERTICALBAN.id = "VERTICALBAN";
        collision_object_VERTICALBAN.header.frame_id = ptrMovegroup->getPlanningFrame();

        primitive.dimensions[0] = 0.6;
        primitive.dimensions[1] = 1.8;
        primitive.dimensions[2] = 0.5;

        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0.94;
        box_pose.position.y =  0;
        //box_pose.position.z =  0.43;
        box_pose.position.z =  0.44;
        collision_object_VERTICALBAN.primitives.push_back(primitive);
        collision_object_VERTICALBAN.primitive_poses.push_back(box_pose);
        collision_object_VERTICALBAN.operation = collision_object_VERTICALBAN.ADD;

        collision_objects.push_back(collision_object_VERTICALBAN);

         // INSERT VERTICALBAN2
        moveit_msgs::CollisionObject collision_object_VERTICALBAN2;
        collision_object_VERTICALBAN2.id = "VERTICALBAN2";
        collision_object_VERTICALBAN2.header.frame_id = ptrMovegroup->getPlanningFrame();

        primitive.dimensions[0] = 0.6;
        primitive.dimensions[1] = 1.8;
        primitive.dimensions[2] = 0.5;

        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0.94;
        box_pose.position.y =  0;
        //box_pose.position.z =  -0.54;        
        box_pose.position.z =  -0.59;
        collision_object_VERTICALBAN2.primitives.push_back(primitive);
        collision_object_VERTICALBAN2.primitive_poses.push_back(box_pose);
        collision_object_VERTICALBAN2.operation = collision_object_VERTICALBAN2.ADD;

        collision_objects.push_back(collision_object_VERTICALBAN2);


        // INSERT cell
        moveit_msgs::CollisionObject collision_object_cell;
        collision_object_cell.id = "cell";
        collision_object_cell.header.frame_id = ptrMovegroup->getPlanningFrame();

        primitive.dimensions[0] = 2;
        primitive.dimensions[1] = 2;
        primitive.dimensions[2] = 0.2;

        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0;
        box_pose.position.y =  0;        
        box_pose.position.z =  0.7;
        collision_object_cell.primitives.push_back(primitive);
        collision_object_cell.primitive_poses.push_back(box_pose);
        collision_object_cell.operation = collision_object_cell.ADD;

        collision_objects.push_back(collision_object_cell);

        ptrPlanScenceInterface->addCollisionObjects(collision_objects);

        sleep(0.5);        

    }


   void PickAndPlace::add_collision_middle_point()
    {
        ROS_INFO("ADD COLLISION ");
        std::vector<std::string> objects_id;
        objects_id.push_back("Containter");
        objects_id.push_back("VERTICALBAN");
        objects_id.push_back("VERTICALBAN2");
        ptrPlanScenceInterface->removeCollisionObjects(objects_id);
        sleep(0.5);

        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = ptrMovegroup->getPlanningFrame();

        collision_object.id = "box1";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.3;
        primitive.dimensions[1] = 0.5;
        primitive.dimensions[2] = 0.3;

        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0;
        box_pose.position.y =  0;
        box_pose.position.z =  -0.15;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);

        // INSERT Left Back WALL  
        moveit_msgs::CollisionObject collision_object_BW;
        collision_object_BW.id = "BackWall";
        collision_object_BW.header.frame_id = ptrMovegroup->getPlanningFrame();
        
        primitive.dimensions[0] = 0.06;
        primitive.dimensions[1] = 0.18;
        primitive.dimensions[2] = 0.6;


        box_pose.orientation.w = 1.0;
        box_pose.position.x =  -0.2;
        box_pose.position.y =  0;
        box_pose.position.z =  0.3;
        collision_object_BW.primitives.push_back(primitive);
        collision_object_BW.primitive_poses.push_back(box_pose);
        collision_object_BW.operation = collision_object_BW.ADD;

        collision_objects.push_back(collision_object_BW);

        // INSERT CONTAINTER   
        moveit_msgs::CollisionObject collision_object_CTER;
        collision_object_CTER.id = "Containter";
        collision_object_CTER.header.frame_id = ptrMovegroup->getPlanningFrame();
        
        primitive.dimensions[0] = 0.47;
        primitive.dimensions[1] = 0.38;
        primitive.dimensions[2] = 0.17;


        box_pose.orientation.w = 1.0;
        box_pose.position.x =  -0.23;
        box_pose.position.y =  0.43;
        box_pose.position.z =  -0.40;
        collision_object_CTER.primitives.push_back(primitive);
        collision_object_CTER.primitive_poses.push_back(box_pose);
        collision_object_CTER.operation = collision_object_CTER.ADD;

        collision_objects.push_back(collision_object_CTER);

         // INSERT VERTICALBAN2
        moveit_msgs::CollisionObject collision_object_VERTICALBAN2;
        collision_object_VERTICALBAN2.id = "VERTICALBAN2";
        collision_object_VERTICALBAN2.header.frame_id = ptrMovegroup->getPlanningFrame();

        primitive.dimensions[0] = 0.6;
        primitive.dimensions[1] = 1.8;
        primitive.dimensions[2] = 0.5;

        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0.95;
        box_pose.position.y =  0;
        //box_pose.position.z =  -0.54;        
        box_pose.position.z =  -0.58;
        collision_object_VERTICALBAN2.primitives.push_back(primitive);
        collision_object_VERTICALBAN2.primitive_poses.push_back(box_pose);
        collision_object_VERTICALBAN2.operation = collision_object_VERTICALBAN2.ADD;

        collision_objects.push_back(collision_object_VERTICALBAN2);


        // INSERT cell
        moveit_msgs::CollisionObject collision_object_cell;
        collision_object_cell.id = "cell";
        collision_object_cell.header.frame_id = ptrMovegroup->getPlanningFrame();

        primitive.dimensions[0] = 2;
        primitive.dimensions[1] = 2;
        primitive.dimensions[2] = 0.2;

        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0;
        box_pose.position.y =  0;        
        box_pose.position.z =  0.7;
        collision_object_cell.primitives.push_back(primitive);
        collision_object_cell.primitive_poses.push_back(box_pose);
        collision_object_cell.operation = collision_object_cell.ADD;

        collision_objects.push_back(collision_object_cell);

        ptrPlanScenceInterface->addCollisionObjects(collision_objects);

        sleep(0.5);        

    }


    void VERTICAL_PickAndPlace::save_transform(std::string object_type, tf::StampedTransform &tranform)
    {
        ROS_INFO("SAVE TRANSFORM ");        
        std::cin.clear();        
        pose_target.position.x = tranform.getOrigin().x() + 0.02;
        pose_target.position.y = tranform.getOrigin().y();
        pose_target.position.z = tranform.getOrigin().z() + 0.005;

        std::cout << "The position we chose is ";
        std::cout << "(" << pose_target.position.x << ", " << pose_target.position.y << ", " << pose_target.position.z << ")" << std::endl;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(PI,0,0), pose_target.orientation);

        add_offset(object_type);
        std::cout << "Do you want to continue? press 'y' and 'u' will abort this result " << std::endl;
        char temp;
        temp = 'y';
        //std::cin >> temp;
        //std::cin.clear();
        continue_flag = (temp == 'y')?true:false;
    }

    std::vector<std::string> string_split(std::string str, std::string pattern)
    {
        std::vector<std::string> string_ret;
        if(pattern.empty()) return string_ret;
        size_t start = 0;
        size_t index = str.find_first_of(pattern, 0);
        while(index!=str.npos)
        {
            if(start !=index) string_ret.push_back(str.substr(start, index-start));
            start = index +1;
            index = str.find_first_of(pattern, start);            

        }
        if(!str.substr(start).empty()) string_ret.push_back(str.substr(start));
        return string_ret;
    }

    void PickAndPlace::read_offset_info(string object_type, double &x_offset,  double &y_offset, double &z_offset)
    {
        std::string file_name = "/home/neousys/Desktop/IRIM_EVENTUAL/src/ro_control/cfg/" + object_type + ".txt";
        std::vector<std::string> sub_string;
        std::string pattern = ";";
        std::ifstream inFile;
        std::string temp;        
        std::string sLine;

        //
        inFile.open(file_name.c_str(), std::ios::in);
        if(inFile.fail())
        {
            fprintf(stderr, "file %s open error\n", file_name.c_str());
        }

        std::getline(inFile, sLine);
        sub_string = string_split(sLine, pattern);
        if(sub_string.size() != 3)
        {
            x_offset = 0;
            y_offset = 0;
            z_offset = 0;
        }
        else
        {
            temp = sub_string[0].substr(11, sub_string[0].size()-1);
            x_offset = std::atof(temp.c_str());

            temp = sub_string[1].substr(11, sub_string[1].size()-1);
            y_offset = std::atof(temp.c_str());
      
            temp = sub_string[2].substr(11, sub_string[2].size()-1);
            z_offset = std::atof(temp.c_str());
        }
     
        inFile.close();

    }

    void PickAndPlace::add_offset(std::string object_type)
    {

        double x_offset = 0.0;
        double y_offset = 0.0;
        double z_offset = 0.0;
        
        //if(!strcmp(object_type.c_str(), "french fries")){ read_offset_info(object_type, x_offset,  y_offset, z_offset);}
        read_offset_info(object_type, x_offset,  y_offset, z_offset);

        
        IRIM::OffSolver offsolver;
        string file_name = "/home/neousys/Desktop/IRIM_EVENTUAL/src/ro_control/offset/" + object_type + ".txt";
        if(offsolver.read_from_file(file_name))
        {
            pair<double,double> offset =  offsolver.give_the_offset(pose_target.position.y);
            x_offset += offset.first;
            y_offset += offset.second;
            
        };

        pose_target.position.x += x_offset;
        pose_target.position.y += y_offset;
        pose_target.position.z += z_offset;

        cout << x_offset << endl;
        cout << y_offset << endl;
        cout << z_offset << endl;

    }

    void PickAndPlace::add_offset_origin(std::string object_type)
    {
        double x_offset = 0.0;
        double y_offset = 0.0;
        double z_offset = 0.0;
        
        //if(!strcmp(object_type.c_str(), "french fries")){ read_offset_info(object_type, x_offset,  y_offset, z_offset);}
        read_offset_info(object_type, x_offset,  y_offset, z_offset);

        pose_target.position.x += x_offset;
        pose_target.position.y += y_offset;
        pose_target.position.z += z_offset;

        cout << x_offset << endl;
        cout << y_offset << endl;
        cout << z_offset << endl;

    }


    tf::StampedTransform PickAndPlace::listen_to_transform()
    {
        tf::StampedTransform transform;
        bool continue_flag = false;
        static tf::TransformListener listener(ros::Duration(10));
        sleep(1);
        // while(!continue_flag)
        // {
            //position_pub.publish(position_msg);  

        listener.waitForTransform("base_link", "target_position", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_link", "target_position", ros::Time(0), transform);

        continue_flag = true;

        return transform;
    }
        


    void PickAndPlace::desk_save_transform(tf::StampedTransform &tranform)
    {
        ROS_INFO("SAVE TRANSFORM ");        
        std::cin.clear();        
        pose_target.position.x = tranform.getOrigin().x() - 0.029;
        pose_target.position.y = tranform.getOrigin().y() - 0.01;
        pose_target.position.z = tranform.getOrigin().z() + 0.03;
        std::cout << "The position we chose is ";
        std::cout << "(" << pose_target.position.x << ", " << pose_target.position.y << ", " << pose_target.position.z << ")" << std::endl;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,PI/2,0), pose_target.orientation);
        std::cout << "Do you want to continue? press 'y' and 'u' will abort this result " << std::endl;
        char temp;
        //temp = 'y';
        std::cin >> temp;
        std::cin.clear();
        continue_flag = (temp == 'y')?true:false;
    }

    void VERTICAL_PickAndPlace::desk_save_transform(tf::StampedTransform &tranform)
    {
        ROS_INFO("SAVE TRANSFORM ");        
        std::cin.clear();        
        pose_target.position.x = tranform.getOrigin().x() + 0.02;
        pose_target.position.y = tranform.getOrigin().y() - 0.014;
        pose_target.position.z = tranform.getOrigin().z() - 0.014;
        std::cout << "The position we chose is ";
        std::cout << "(" << pose_target.position.x << ", " << pose_target.position.y << ", " << pose_target.position.z << ")" << std::endl;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,0), pose_target.orientation);
        std::cout << "Do you want to continue? press 'y' and 'u' will abort this result " << std::endl;
        char temp;
        //temp = 'y';
        std::cin >> temp;
        std::cin.clear();
        continue_flag = (temp == 'y')?true:false;
    }


    void VERTICAL_PickAndPlace::move_to_safepoint()
    {
        sleep(1);
        ptrMovegroup->setStartState(*ptrMovegroup->getCurrentState());
        ROS_INFO("ROBOT MOVE TO VERTICAL SAFE POSITION");     
        ptrMovegroup->setNamedTarget(ptrConfig->VERTICAL_SAFEPOINT_NAME);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        ptrMovegroup->plan(plan);
        ptrMovegroup->execute(plan);

    }

    void VERTICAL_PickAndPlace::move_to_approch_point()
    {
        sleep(1);
        ROS_INFO("MOVE TO VERTICAL APPROACH POINT ");                     
        moveit_msgs::RobotState robot_state;
        robot_state::RobotStatePtr current_state = ptrMovegroup->getCurrentState();
        robot_state::robotStateToRobotStateMsg(*current_state, robot_state);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        geometry_msgs::Pose approch_point = pose_target;
        approch_point.position.x -= ptrConfig->APPROACH_DISTANCE;
        create_motion_plan(approch_point, robot_state, plan);
        ptrMovegroup->execute(plan);
    }

    void VERTICAL_PickAndPlace::pick()
    {
        ROS_INFO("VERTICAL PICK ");  
        //IRIM::Obstable_Manager obs_manager(*ptrNodeHandle);
        //obs_manager.remove_obstacle();                      
        moveit_msgs::RobotState robot_state;
        sleep(1);
        robot_state::RobotStatePtr current_state= ptrMovegroup->getCurrentState();
        robot_state::robotStateToRobotStateMsg(*current_state,robot_state);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        geometry_msgs::Pose retreat_point = pose_target;
        retreat_point.position.x -= ptrConfig->RETREAT_DISTANCE;
        retreat_point.position.z += ptrConfig->RETREAT_DISTANCE;

        vale_controler.do_it();
        create_motion_plan(pose_target, robot_state, plan);

        ptrMovegroup->execute(plan);

        sleep(1);

        ROS_INFO("VERTICAL RETREAT "); 
        current_state= ptrMovegroup->getCurrentState();
        robot_state::robotStateToRobotStateMsg(*current_state,robot_state);
        /*moveit_msgs::Constraints endEffector_constraints;
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "ee_link";   
        ocm.header.frame_id = ptrConfig->BACE_FRAME_ID;
        ocm.orientation.w = 1;
        ocm.absolute_x_axis_tolerance = 0.1;
        ocm.absolute_y_axis_tolerance = 0.1;
        ocm.absolute_z_axis_tolerance = 2*3.14;
        ocm.weight = 1.0;
        endEffector_constraints.orientation_constraints.push_back(ocm);
        ptrMovegroup->setPathConstraints(endEffector_constraints);
        ptrMovegroup->setPoseTarget(retreat_point, ptrConfig->END_EFFECTOR_LINK);
        ptrMovegroup->plan(plan);*/
        create_motion_plan(retreat_point, robot_state, plan);
        //ptrMovegroup->clearPathConstraints();
        //cartesian_move(pose_target, retreat_point, plan);
        ptrMovegroup->execute(plan);
        //obs_manager.add_obstacle();  
    }

    void PickAndPlace::cartesian_move(geometry_msgs::Pose &current_pose, geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        using namespace std;
        ROS_INFO("CARTESIAN MOVE"); 
        const double numsize = 5;
        vector<geometry_msgs::Pose> waypoints;
        moveit::core::RobotStatePtr kinematic_state;
        kinematic_state = ptrMovegroup->getCurrentState();
        waypoints.resize(numsize);
        waypoints.at(0) = current_pose;
        double x_incresment = (target_pose.position.x - current_pose.position.x)/ (numsize-1);
        double y_incresment = (target_pose.position.y - current_pose.position.y)/ (numsize-1);
        double z_incresment = (target_pose.position.z - current_pose.position.z)/ (numsize-1);
        for(int i = 1; i < numsize-1; i++)
        {
            waypoints.at(i).position.x = waypoints.at(i-1).position.x + x_incresment;
            waypoints.at(i).position.y = waypoints.at(i-1).position.y + y_incresment;
            waypoints.at(i).position.z = waypoints.at(i-1).position.z + z_incresment;
        }
        double CART_STEP_SIZE_ = 0.01;
        double CART_JUMP_THRESH_ = 0.1;
        bool AVOID_COLLISIONS_ = false;
        moveit_msgs::RobotTrajectory trajectory_;
        ptrMovegroup->computeCartesianPath(waypoints,CART_STEP_SIZE_,CART_JUMP_THRESH_,trajectory_,AVOID_COLLISIONS_);
        robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(), ptrConfig->ARM_GROUP_NAME);
        rt.setRobotTrajectoryMsg(*kinematic_state, trajectory_);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        iptp.computeTimeStamps(rt);
        rt.getRobotTrajectoryMsg(trajectory_);
        plan.trajectory_ = trajectory_;
    } 

    void VERTICAL_PickAndPlace::UR_pick()
    {
        ROS_INFO("VERTICAL PICK ");  
        std_msgs::String pick_position;
        std_msgs::String retreat_position;
        vale_controler.do_it();
        pick_position.data = "movel(pose_add(get_actual_tcp_pose(),p[-0.05,0,0,0,0,0]),0.1,0.5)";
        //retreat_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0.1,0,0.1,0,0,0]),0.3,0.5)";
        retreat_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0.1,0,0.2,0,0,0]),0.3,0.5)";
        UR_pub.publish(pick_position);
        sleep(3);
        UR_pub.publish(retreat_position);
        sleep(2);
    }

    void VERTICAL_PickAndPlace::UR_pick_special()
    {
        ROS_INFO("VERTICAL PICK ");  
        std_msgs::String pick_position;
        std_msgs::String retreat_position;
        vale_controler.do_it();
        pick_position.data = "movel(pose_add(get_actual_tcp_pose(),p[-0.1,0,0,0,0,0]),0.1,1)";
        //retreat_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0.15,0,0.1,0,0,0]),0.3,1)";
        retreat_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0.15,0,0.2,0,0,0]),0.3,1)";
        UR_pub.publish(pick_position);
        sleep(3);
        UR_pub.publish(retreat_position);
        sleep(2.5);
    }

    void PickAndPlace::UR_pick(std::string placeposition)
    {
        ROS_INFO("VERTICAL PICK ");  
        std_msgs::String pick_position;
        std_msgs::String retreat_position;
        vale_controler.do_it();
        pick_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0,0,-0.04,0,0,0]),0.2,1)";
        retreat_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0.15,0,0.13,0,0,0]),0.2,1)";

        if(!strcmp(placeposition.c_str(), "low"))
        {
            pick_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0,0,-0.04,0,0,0]),0.2,1)";
            retreat_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0.15,0,0.13,0,0,0]),0.2,1)";
        }
        else if(!strcmp(placeposition.c_str(), "middle"))
        {
            pick_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0,0,-0.03,0,0,0]),0.2,1)";
            retreat_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0.15,0,0.15,0,0,0]),0.2,1)";
        }
        else if(!strcmp(placeposition.c_str(), "high"))
        {
            pick_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0,0,-0.03,0,0,0]),0.2,1)";
            retreat_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0.15,0,0.17,0,0,0]),0.2,1)";
        }

        UR_pub.publish(pick_position);
        sleep(2);
        UR_pub.publish(retreat_position);
        sleep(2);

    }

    void PickAndPlace::UR_pick_special()
    {
        ROS_INFO("VERTICAL PICK ");  
        std_msgs::String pick_position;
        std_msgs::String retreat_position;
        vale_controler.do_it();
        pick_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0,0,-0.09,0,0,0]),0.2,1)";
        //retreat_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0.15,0,0.14,0,0,0]),0.2,1)";
        retreat_position.data = "movel(pose_add(get_actual_tcp_pose(),p[0.15,0,0.19,0,0,0]),0.2,1)";
        UR_pub.publish(pick_position);
        sleep(3);
        UR_pub.publish(retreat_position);
        sleep(2);

    }

    bool PickAndPlace::is_at_home_position()
    {
        //-1.775
        //-1.624
        const double upper_bonud = -1.62;
        const double lower_bonud = -1.63;
        
        std::vector<double> joint_value = ptrMovegroup->getCurrentJointValues();
        if(joint_value[0] > lower_bonud && joint_value[0] < upper_bonud)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
}   
