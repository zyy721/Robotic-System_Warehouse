#ifndef DEVICE_CONTROL_H
#define DEVICE_CONTROL_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <serial/serial.h>
#include <std_msgs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <ro_control/controlAction.h>

using namespace std;

typedef actionlib::SimpleActionClient<ro_control::controlAction> Client;

namespace IRIM
{
   class serial_publisher: private serial::Serial
  {
  public:
    serial_publisher()
    {
      serial_init();
      sleep(1);
    }
    /*~serial_publisher()
    {
      delete []ptr_ser_buffer;
    }*/
    virtual void do_it() = 0;
    virtual void undo_it() = 0;
    void send_message();
    void send_message_string(const string);
    
    //members
    unsigned char* ptr_ser_buffer;
  private:
    void serial_init();
    
  };

  class VALE_CONTROLER: public serial_publisher
  {
  public:
    virtual void do_it();
    virtual void undo_it();
  };

  class LIFT_CONTROLER: public serial_publisher
  {
  public:
    LIFT_CONTROLER()
    {
       //init();
    }
    virtual void do_it();
    virtual void undo_it();
    void init();
    void position_low();
    void position_middle();
    void position_high();
    void position_lowest();
    void lift_moving(string lift_pos);

    
  };

  class MOTOR_CONTROLER: public serial_publisher
  {
  public:
    virtual void do_it();
    virtual void undo_it();
  };


  class Chassisc_Control
  {
  public:
      Chassisc_Control(const string name): ac(_nh, name, true)
      {
          sleep(1);
          robot_init();
      }
      void move_to_position(const string& name);
      void move_to_position_while_lift(const string& name, const string &lifter_position);
      void execute_a_path(vector<int> path);
  private:
      ros::NodeHandle _nh;
      Client ac;

      ro_control::controlGoal goal_yeilder(const string stationID, bool station_flag);
      void robot_init();
  };


}

#endif /* DEVICE_CONTROL_H */


