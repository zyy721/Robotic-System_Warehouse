#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ro_control/taskAction.h>
#include "robot_config.h"
#include "robot_control.h"
#include "data_Input.h"
#include "decision_maker.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>

typedef actionlib::SimpleActionServer<ro_control::taskAction> taskServer;

#define BIG_UPPER_BOUND 0.64
using namespace moveit_msgs;


std::string FILE_NAME = "/home/neousys/Desktop/IRIM_EVENTUAL/src/object_detection/tasklists/grasped.txt";
std::string OVER_RANGE_FILE_NAME = "/home/neousys/Desktop/IRIM_EVENTUAL/src/object_detection/tasklists/overrange.txt";
std::string IROS_FILE_NAME = "/home/neousys/Desktop/IRIM_EVENTUAL/src/object_detection/tasklists/iros.txt";


typedef boost::shared_ptr<IRIM::VERTICAL_PickAndPlace> VPickAndPlacePtr;
typedef boost::shared_ptr<IRIM::PickAndPlace> PickAndPlacePtr;
typedef boost::shared_ptr<IRIM::Chassisc_Control> Chassisc_ControlPtr;


void object_class_capture(const std_msgs::String::ConstPtr& msg);
void empty_the_string();
bool is_too_far_to_pick(tf::StampedTransform transform, std::string object_type);
void clear_the_file(std::string FILE_NAME);
void the_main_pick_loop(std::string placeposition, std::string picking_object);
void inquiry_start();

std::string object_type;
std::string object_type_save;
VPickAndPlacePtr ptrVerticalPickandPlace;
PickAndPlacePtr ptrPickandPlace;
Chassisc_ControlPtr ptrChassics_controler;

// IRIM::VERTICAL_PickAndPlace vpickandplace;
// IRIM::PickAndPlace pickandplace;
int continue_flag = 0;


class TASK_RECEIVER
{
public:
  taskServer as_; 

  std::string action_name_;
  ro_control::taskFeedback feedback_;
  ro_control::taskResult result_;
  IRIM::LIFT_CONTROLER lift_controler;
  IRIM::MOTOR_CONTROLER motor_controler;
    

  int robot_current_position;
  string lift_current_position;

  TASK_RECEIVER(std::string name, ros::NodeHandle &nh_) : as_(nh_, name, boost::bind(& TASK_RECEIVER::executeCB, this, _1), false), action_name_(name)
  {
    ROS_INFO("Start a Task Receiver ");
    
    as_.start();       
    ptrChassics_controler = Chassisc_ControlPtr(new IRIM::Chassisc_Control("control")); 
    ptrPickandPlace = PickAndPlacePtr(new IRIM::PickAndPlace());
    ptrVerticalPickandPlace = VPickAndPlacePtr(new IRIM::VERTICAL_PickAndPlace());
    ptrPickandPlace->robot_init();

    robot_current_position  = 23;
    ptrPickandPlace->move_to_home_UR();
    
    lift_controler.position_low();

    lift_current_position ="low";
  }

  ~TASK_RECEIVER(void)
  {
  }

  void executeCB(const ro_control::taskGoalConstPtr &goal)
  {
    // helper variables
    vector<int> execute_tour = goal->tour;
    string picking_position = goal->lift_pos;
    string object_name = goal->pick_target;
    int goal_vertex = goal->vertex_index;
    clear_the_file(IROS_FILE_NAME);
    clear_the_file(OVER_RANGE_FILE_NAME);
    clear_the_file(FILE_NAME);
    

    std::ofstream ofresult(IROS_FILE_NAME,ios::app);
    ofresult << object_name;
    ofresult.close();
    //robot_current_position = execute_tour.at(0);
       //moving 
    if(robot_current_position != goal_vertex && picking_position.compare(lift_current_position))
    {
        bool success = true;
        //attention ! The last tour is executed without picking
        ROS_INFO("Moving while lift and chasis");
        for(int i = 1; i <execute_tour.size(); i++)
        {
            ptrChassics_controler->move_to_position(to_string(execute_tour.at(i)));
            feedback_.current_index = execute_tour.at(i);
            robot_current_position = execute_tour.at(i);
            as_.publishFeedback(feedback_);
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            } 
        }
        lift_controler.lift_moving(picking_position);
        lift_current_position = picking_position;
        the_main_pick_loop(picking_position, object_name);
        
        if(success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            
            result_.Done = success;
            as_.setSucceeded(result_);           
            // set the action state to succeeded
        }else
        {
            ROS_INFO("%s: Aborted", action_name_.c_str());
            //set the action state to aborted
            as_.setAborted(result_);
        }       
    }
    else if (robot_current_position == goal_vertex && picking_position.compare(lift_current_position))
    {
        bool success = true;
        ROS_INFO("Moving lift ");
        lift_controler.lift_moving(picking_position);
        lift_current_position = picking_position;
        the_main_pick_loop(picking_position, object_name);
        
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        result_.Done = success;
        as_.setSucceeded(result_);   

    }
    else if (robot_current_position != goal_vertex && !picking_position.compare(lift_current_position))
    {
        bool success = true;
        
        ROS_INFO("Moving chasis");
        for(int i = 1; i <execute_tour.size(); i++)
        {
            ptrChassics_controler->move_to_position(to_string(execute_tour.at(i)));
            feedback_.current_index = execute_tour.at(i);
            robot_current_position = execute_tour.at(i);

            as_.publishFeedback(feedback_);
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            } 
        }
        the_main_pick_loop(picking_position, object_name);
        
        if(success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            result_.Done = success;
            as_.setSucceeded(result_);            
            // set the action state to succeeded
        }else
        {
            ROS_INFO("%s: Aborted", action_name_.c_str());
            //set the action state to aborted
            as_.setAborted(result_);
        }

    } 
    else
    {
        ROS_INFO("Just Picking");
        bool success = true;
        
        the_main_pick_loop(picking_position, object_name);
        
        if(success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            result_.Done = success;
            as_.setSucceeded(result_);            
            // set the action state to succeeded
        }else
        {
            ROS_INFO("%s: Aborted", action_name_.c_str());
            //set the action state to aborted
            as_.setAborted(result_);
        }
        
    }
    ptrPickandPlace->move_to_home_UR();
    
    clear_the_file(IROS_FILE_NAME);
    ROS_INFO("milestone");
  }

};


int main(int argc, char**argv)
{
    ros::init(argc, argv, "irim");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh_;
    
    static ros::Subscriber sub_2D = nh_.subscribe("/yolo_class", 1, object_class_capture);

    TASK_RECEIVER task_receiver("execute_task", nh_);
    while(ros::ok())
    {
        sleep(1);
    }
}

void object_class_capture(const std_msgs::String::ConstPtr& msg)
{
    object_type = msg->data.c_str();
}

void empty_the_string()
{
    object_type = "empty";
    object_type_save = "empty"; 
    continue_flag--;  

}

void clear_the_file(std::string FILE_NAME)
{
    std::ofstream ofresult(FILE_NAME);
    ofresult << "";
    ofresult.close();
}

void the_main_pick_loop(std::string placeposition, std::string picking_object)
{  
    std::ofstream ofresult(FILE_NAME,ios::app);
    
    while(ros::ok())
    { 
        continue_flag++;
        IRIM::DeepLearningDM decisionMaker;
        tf::StampedTransform transform = ptrPickandPlace->listen_to_transform();

        if(continue_flag > 2)
        {
            continue_flag = 0;
            break;
        } 
        if( !strcmp(object_type.c_str(), "empty")){cout << "***empty!***" << endl; continue;}
        else{ object_type_save = object_type;}

        if(is_too_far_to_pick(transform, object_type_save))
        {
            std::ofstream over_range_ofresult(OVER_RANGE_FILE_NAME,ios::app);
            over_range_ofresult << object_type_save << endl;
            over_range_ofresult.close();
            
            continue_flag--; 
            sleep(1);
            continue;
        }

        if(decisionMaker.is_vertical_plan(object_type_save))
        {
            ofresult << object_type_save << endl; 
            ptrPickandPlace->save_transform(object_type_save, transform);
            if(!ptrPickandPlace->get_continue_flag()) continue;
            //#4 change the return time from placeposition to safepoint
            if(!ptrPickandPlace->is_at_home_position())//如果机器人的base的位置在某个范围内，认为机器人在placepoint的位置
            {
                ptrPickandPlace->placepoint_to_safepoint_UR();
            }
            else
            {
                ptrPickandPlace->home_to_safepoint_UR();
            }
            ptrPickandPlace->safepoint_approach_UR();
            //ptrPickandPlace->UR_pick();
            //ptrPickandPlace->retreat_to_safepoint_UR();
            if(!strcmp(object_type_save.c_str(), "cup"))
            {
                ptrPickandPlace->UR_pick_special();
            }
            else
            {
                ptrPickandPlace->UR_pick(placeposition);
            }
            ptrPickandPlace->move_to_placepoint(placeposition,false);


            ptrPickandPlace->continue_flag = false;
                       
            empty_the_string();
        }
        else
        {
            ofresult << object_type_save << endl; 
            ptrVerticalPickandPlace->save_transform(object_type_save, transform);
            if(!ptrVerticalPickandPlace->get_continue_flag()) continue;
            //#4 change the return time from placeposition to safepoint
            if(!ptrVerticalPickandPlace->is_at_home_position())//如果机器人的base的位置在某个范围内，认为机器人在placepoint的位置
            {
                ptrVerticalPickandPlace->placepoint_to_v_safepoint_UR();
            }
            else
            {
                ptrVerticalPickandPlace->home_to_v_safepoint_UR();
            }
            ptrVerticalPickandPlace->v_safepoint_approach_UR();
            if(!strcmp(object_type_save.c_str(), "cola") || !strcmp(object_type_save.c_str(), "shampoo"))
            {
                ptrVerticalPickandPlace->UR_pick_special();
            }
            else
            {
                ptrVerticalPickandPlace->UR_pick();
            }
            ptrVerticalPickandPlace->v_retreat_to_safepoint_UR();
            ptrVerticalPickandPlace->move_to_placepoint(placeposition,true);

            
            ptrVerticalPickandPlace->continue_flag = false;
                          
            empty_the_string();         
        }

    }
    ofresult.close();
    clear_the_file(OVER_RANGE_FILE_NAME);
}
   
void inquiry_start()
{
    char temp;
    cout << "Ready to start?" << endl;
    cout << "Enter 'y' or 'Y' to start the picking! " << endl;        
    cin >> temp;
    while(temp != 'y' && temp != 'Y')
    {
        cin >> temp;
    }
    clear_the_file(FILE_NAME);
    clear_the_file(OVER_RANGE_FILE_NAME);
}

bool is_too_far_to_pick(tf::StampedTransform transform, std::string object_type)
{
    double sum = 0;
    sum = transform.getOrigin().x()*transform.getOrigin().x() + transform.getOrigin().y()*transform.getOrigin().y() + transform.getOrigin().z()*transform.getOrigin().z();
    if(BIG_UPPER_BOUND < sum){ return true;}
    if(transform.getOrigin().x() < 0.3){ return true;}    
    //if((transform.getOrigin().ythe_main_pick_loop() < -0.4) && (!is_vertical)){ return true;}
    if((!strcmp(object_type.c_str(), "cup")) && (transform.getOrigin().y() < -0.392635) && (transform.getOrigin().y() > 0.158307)){ return true;}
    if((!strcmp(object_type.c_str(), "cola")) && (transform.getOrigin().y() < -0.374959) && (transform.getOrigin().y() > 0.0761038)){ return true;}
    if((!strcmp(object_type.c_str(), "shampoo")) && (transform.getOrigin().y() < -0.385136) && (transform.getOrigin().y() > 0.0859363)){ return true;}
    if((!strcmp(object_type.c_str(), "toothbrush")) && (transform.getOrigin().y() < -0.449456) && (transform.getOrigin().y() > 0.100057)){ return true;}
    if((!strcmp(object_type.c_str(), "orange")) && (transform.getOrigin().y() < -0.444994) && (transform.getOrigin().y() > 0.113043)){ return true;}
    if((!strcmp(object_type.c_str(), "oreo")) && (transform.getOrigin().y() < -0.445808) && (transform.getOrigin().y() > 0.112582)){ return true;}
    if((!strcmp(object_type.c_str(), "spray")) && (transform.getOrigin().y() < -0.344083) && (transform.getOrigin().y() > 0.0796991)){ return true;}
    if((!strcmp(object_type.c_str(), "french fries")) && (transform.getOrigin().y() < -0.443171) && (transform.getOrigin().y() > 0.108857)){ return true;}
    if((!strcmp(object_type.c_str(), "tissue")) && (transform.getOrigin().y() < -0.434845) && (transform.getOrigin().y() > 0.107003)){ return true;}
    if((!strcmp(object_type.c_str(), "chips")) && (transform.getOrigin().y() < -0.436209) && (transform.getOrigin().y() > 0.0775457)){ return true;}
    else {return false;}
}
