#include "robot_config.h"
#include "robot_control.h"
#include "data_Input.h"
#include "decision_maker.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include "Task.h"

#define BIG_UPPER_BOUND 0.64
using namespace moveit_msgs;


std::string FILE_NAME = "/home/neousys/Desktop/IRIM_EVENTUAL/src/object_detection/tasklists/grasped.txt";
std::string OVER_RANGE_FILE_NAME = "/home/neousys/Desktop/IRIM_EVENTUAL/src/object_detection/tasklists/overrange.txt";

typedef boost::shared_ptr<IRIM::VERTICAL_PickAndPlace> VPickAndPlacePtr;
typedef boost::shared_ptr<IRIM::PickAndPlace> PickAndPlacePtr;

void object_class_capture(const std_msgs::String::ConstPtr& msg);
void empty_the_string();
bool is_too_far_to_pick(tf::StampedTransform transform, std::string object_type);
void clear_the_file(std::string FILE_NAME);
void the_main_pick_loop(std::string placeposition);
void inquiry_start();

std::string object_type;
std::string object_type_save;
VPickAndPlacePtr ptrVerticalPickandPlace;
PickAndPlacePtr ptrPickandPlace;
// IRIM::VERTICAL_PickAndPlace vpickandplace;
// IRIM::PickAndPlace pickandplace;
int continue_flag = 0;

//if_receive a task, execute it.
//You also need to define a golbal variant to tackle the repeat picking situation


int main(int argc, char**argv)
{
    ros::init(argc, argv, "irim");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle;
    static ros::Subscriber sub_2D = node_handle.subscribe("/yolo_class", 1, object_class_capture);

    ptrPickandPlace = PickAndPlacePtr(new IRIM::PickAndPlace());
    ptrVerticalPickandPlace = VPickAndPlacePtr(new IRIM::VERTICAL_PickAndPlace());

    IRIM::LIFT_CONTROLER lift_controler;
    IRIM::MOTOR_CONTROLER motor_controler;
    IRIM::Chassisc_Control chassic_controller("control");

    //ptrPickandPlace->add_collision_low_point();   
    ptrPickandPlace->robot_init();

    ptrPickandPlace->move_to_home_UR();
    lift_controler.position_low();
    chassic_controller.move_to_position("0");
    //chassic_controller.move_to_position_while_lift("0","low");
    lift_controler.position_middle();
    

    inquiry_start();
    chassic_controller.move_to_position("1");
    the_main_pick_loop("middle");
    ptrPickandPlace->move_to_home_UR_no_wait();
    sleep(1);
    //lift_controler.position_low();
    chassic_controller.move_to_position_while_lift("2","low");
    the_main_pick_loop("low");

    lift_controler.position_high();
    the_main_pick_loop("high");

    chassic_controller.move_to_position("3");
    the_main_pick_loop("high");
    ptrPickandPlace->move_to_home_UR_no_wait();
    sleep(2);

    lift_controler.position_low();    
    the_main_pick_loop("low");
    ptrPickandPlace->move_to_home_UR_no_wait();

    chassic_controller.move_to_position_while_lift("4","middle");
    the_main_pick_loop("middle");
    ptrPickandPlace->move_to_home_UR_no_wait();
    sleep(1);

    chassic_controller.move_to_position_while_lift("5","low");

    motor_controler.do_it();
    motor_controler.undo_it();

    clear_the_file(FILE_NAME);    
    return 0;
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

void the_main_pick_loop(std::string placeposition)
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
    //if((transform.getOrigin().y() < -0.4) && (!is_vertical)){ return true;}
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
