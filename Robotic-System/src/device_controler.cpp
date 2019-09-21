#include "device_controler.h"

namespace IRIM
{
    void Chassisc_Control::robot_init()
    {
        ac.waitForServer(); 
        sleep(1);
    };

    void Chassisc_Control::move_to_position(const string& name)
    {
        ac.sendGoal(goal_yeilder(name, false));
        ac.waitForResult(ros::Duration(300.0));
        while(ros::ok())
        {
            ROS_INFO("Action finished: %s",ac.getState().toString().c_str());
            if(!strcmp(ac.getState().toString().c_str(), "SUCCEEDED"))
            {
                ROS_INFO("Action finished: %s",ac.getState().toString().c_str());
                break;
            };
        }
    };

    void Chassisc_Control::execute_a_path(vector<int> path)
    {
        for(int i = 1;i < path.size();i++)
        {
            move_to_position(std::to_string(path.at(i)));
        }
    };


    void Chassisc_Control::move_to_position_while_lift(const string& name, const string &lifter_position)
{
    std::string sent_message;
    ros::Rate loop_rate(5); 
    
    ac.sendGoal(goal_yeilder(name, false));
    cout << "send_to_DiPan" << endl;
    sleep(0.2);
    //ac.waitForResult(ros::Duration(300.0));

    if(!strcmp(lifter_position.c_str(), "high")){sent_message = "02?;";}
    else if(!strcmp(lifter_position.c_str(), "middle")){sent_message = "01?;";}
    else if(!strcmp(lifter_position.c_str(), "low")){sent_message = "00?;";}

    serial::Serial serial_publisher;

    try
    {
        serial_publisher.setPort("/dev/ttyUSB0");
        serial_publisher.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_publisher.setTimeout(to);
        serial_publisher.open();
        //ptr_ser_buffer = new unsigned char[25];
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
    }
    sleep(1);

    serial_publisher.write(sent_message);
    cout << "send_to_Lifter" << endl;
    
    std::string temp_return_str = "FAILED";
    bool serial_success_flag = false;

    while(ros::ok())
    {
        temp_return_str = ac.getState().toString();        
        ROS_INFO("Action finished: %s",temp_return_str.c_str());

        if(!strcmp(temp_return_str.c_str(), "SUCCEEDED")){}//如果成功了一次，我们就不保存机器人的状态了
        else//如果目前还没有等到成功，我们就把当前的状态存起来
        {
            temp_return_str = ac.getState().toString();
        }

        if(serial_publisher.available())
        {
            ROS_INFO("Lift task completed");      
            serial_publisher.read(100);
            sleep(1);
            serial_publisher.read(100);  
            serial_success_flag = true;
        }

        if(!strcmp(temp_return_str.c_str(), "SUCCEEDED") && serial_success_flag)
        {
            ROS_INFO("Lift task complete and arrive at the destination.");
            break;
        };
        loop_rate.sleep(); 
    }
};
    ro_control::controlGoal Chassisc_Control::goal_yeilder(const string stationID, bool station_flag)
    {
        ro_control::controlGoal goal;
        goal.stationID = stationID;
        goal.is_station = station_flag;
        return goal;
    };
    
    void serial_publisher::serial_init()
    {
    try
    {
        this->setPort("/dev/ttyUSB0");
        this->setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        this->setTimeout(to);
        this->open();
        //ptr_ser_buffer = new unsigned char[25];
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
    }
    }

    void serial_publisher::send_message()
    {
        this->write(ptr_ser_buffer, 3);
    }

    void serial_publisher::send_message_string(const string str_input)
    {
        this->write(str_input);
        ros::Rate loop_rate(5); 
        while(ros::ok())
        {
            if(this->available())
            { 
                ROS_INFO("Task completed");      
                this->read(100);
                sleep(1);
                this->read(100);                
                break;
            }
            loop_rate.sleep(); 
        }

    };

    void VALE_CONTROLER::do_it()
    {
        ROS_INFO("VALE_OPEN ");   
        // std::cout << "press 'c' to open vale " << std::endl;
        // char temp_char;
        // std::cin.clear(); 
        // std::cin >> temp_char;
        // std::cin.clear();
  /*      ptr_ser_buffer[0] = 0x11; //吸盘开启
        ptr_ser_buffer[1] = 0x0d;
        ptr_ser_buffer[2] = 0x0a;*/
        string VALE_OPEN = "11?;";
        send_message_string(VALE_OPEN);
    }

    void VALE_CONTROLER::undo_it()
    {
        // std::cout << "Do you want to continue? press 'y' and 'u' will abort this result " << std::endl;
        // char temp;
        // std::cin.clear(); 
        // std::cin >> temp;
        // std::cin.clear();
        ROS_INFO("VALE_CLOSE "); 
    /*    ptr_ser_buffer[0] = 0x10; //吸盘吸合
        ptr_ser_buffer[1] = 0x0d;
        ptr_ser_buffer[2] = 0x0a;*/
        string VALE_CLOSE = "10?;";        
        send_message_string(VALE_CLOSE);
        // std::cout << "Do you want to continue? press 'y' and 'u' will abort this result " << std::endl;
        // std::cin.clear(); 
        // std::cin >> temp;
        // std::cin.clear();
    }

    void LIFT_CONTROLER::do_it()
    {
        // ROS_INFO("LOW_POINT "); 
        // /*        
        // ptr_ser_buffer[0] = 0x00; //低点
        // ptr_ser_buffer[1] = 0x0d;
        // ptr_ser_buffer[2] = 0x0a;*/
        // string LOW_POINT = "00?;";        
        // send_message_string(LOW_POINT);
        // std::cout << "press 'c' to move to low position " << std::endl;
        // char temp_char;
        // std::cin.clear(); 
        // std::cin >> temp_char; 
        // std::cin.clear();  
    }
    void LIFT_CONTROLER::undo_it()
    {
//         ROS_INFO("HIGH_POINT ");
// /*        
//         ptr_ser_buffer[0] = 0x01; //高点
//         ptr_ser_buffer[1] = 0x0d;
//         ptr_ser_buffer[2] = 0x0a;*/
//         string HIGH_POINT = "01?;";                
//         send_message_string(HIGH_POINT);
//         std::cout << "had move to high position, press 'c' to continue" << std::endl;
//         char temp_char;
//         std::cin.clear(); 
//         std::cin >> temp_char; 
//         std::cin.clear();  
    }


    void LIFT_CONTROLER::init()
    {
        ROS_INFO("LIFT INIT");                 
      /*  ptr_ser_buffer[0] = 0x00; //低点
        ptr_ser_buffer[1] = 0x0d;
        ptr_ser_buffer[2] = 0x0a;*/
        string init = "07?;";                        
        send_message_string(init);  
        // std::cout << "had move to low position, press 'c' to continue" << std::endl;
        // char temp_char;
        // std::cin.clear(); 
        // std::cin >> temp_char; 
        // std::cin.clear(); 
    }


    void LIFT_CONTROLER::lift_moving(string lift_pos)
    {
        if(!lift_pos.compare("low"))position_low();
        if(!lift_pos.compare("middle"))position_middle();
        if(!lift_pos.compare("high"))position_high();
        if(!lift_pos.compare("Return"))position_lowest();
        
    }

    void LIFT_CONTROLER::position_low()
    {
        ROS_INFO("LOW_POINT ");                 
      /*  ptr_ser_buffer[0] = 0x00; //低点
        ptr_ser_buffer[1] = 0x0d;
        ptr_ser_buffer[2] = 0x0a;*/
        string LOW_POINT = "00?;";                        
        send_message_string(LOW_POINT);  
                sleep(10);
        // std::cout << "had move to low position, press 'c' to continue" << std::endl;
        // char temp_char;
        // std::cin.clear(); 
        // std::cin >> temp_char; 
        // std::cin.clear(); 
    }


    void LIFT_CONTROLER::position_lowest()
    {
        ROS_INFO("LOW_POINT ");                 
      /*  ptr_ser_buffer[0] = 0x00; //低点
        ptr_ser_buffer[1] = 0x0d;
        ptr_ser_buffer[2] = 0x0a;*/
        string LOW_POINT = "09?;";                        
        send_message_string(LOW_POINT);  
                sleep(10);
        // std::cout << "had move to low position, press 'c' to continue" << std::endl;
        // char temp_char;
        // std::cin.clear(); 
        // std::cin >> temp_char; 
        // std::cin.clear(); 
    }

    void LIFT_CONTROLER::position_middle()
    {
        ROS_INFO("MIDDLE_POINT ");
        /*        
        ptr_ser_buffer[0] = 0x01; //中点
        ptr_ser_buffer[1] = 0x0d;
        ptr_ser_buffer[2] = 0x0a;*/
        string MIDDLE_POINT = "01?;";                                
        send_message_string(MIDDLE_POINT);     
                sleep(10);
        // std::cout << "had move to middle position, press 'c' to continue" << std::endl;
        // char temp_char;
        // std::cin.clear(); 
        // std::cin >> temp_char;   
        // std::cin.clear();
    }
    void LIFT_CONTROLER::position_high()
    {
        ROS_INFO("HIGH_POINT ");
        /*        
        ptr_ser_buffer[0] = 0x02; //高点
        ptr_ser_buffer[1] = 0x0d;
        ptr_ser_buffer[2] = 0x0a;*/
        string HIGH_POINT = "02?;";                                        
        send_message_string(HIGH_POINT);     
                sleep(10); 
        // std::cout << "had move to high position, press 'c' to continue" << std::endl;
        // char temp_char;
        // std::cin.clear(); 
        // std::cin >> temp_char;  
        // std::cin.clear(); 
    }

    void MOTOR_CONTROLER::undo_it()
    {
        ROS_INFO("MOTOR_DOWN "); 
        /*
        ptr_ser_buffer[0] = 0x10; //推杆电机返回
        ptr_ser_buffer[1] = 0x0d;
        ptr_ser_buffer[2] = 0x0a;*/
        string MOTOR_DOWN = "20?;";                                                
        send_message_string(MOTOR_DOWN);
        // std::cout << "Do you want to continue? press 'y' and 'u' will abort this result " << std::endl;
        // std::cin.clear(); 
        // std::cin >> temp;
        // std::cin.clear();
    }

    void MOTOR_CONTROLER::do_it()
    {
        ROS_INFO("MOTOR_LIFT ");         
        string MOTOR_LIFT = "21?;";                                                        
        send_message_string(MOTOR_LIFT);
    }

}
