// Generated by gencpp from file ro_control/RoiList.msg
// DO NOT EDIT!


#ifndef RO_CONTROL_MESSAGE_ROILIST_H
#define RO_CONTROL_MESSAGE_ROILIST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/RegionOfInterest.h>

namespace ro_control
{
template <class ContainerAllocator>
struct RoiList_
{
  typedef RoiList_<ContainerAllocator> Type;

  RoiList_()
    : roi_list()  {
    }
  RoiList_(const ContainerAllocator& _alloc)
    : roi_list(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::sensor_msgs::RegionOfInterest_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::RegionOfInterest_<ContainerAllocator> >::other >  _roi_list_type;
  _roi_list_type roi_list;





  typedef boost::shared_ptr< ::ro_control::RoiList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ro_control::RoiList_<ContainerAllocator> const> ConstPtr;

}; // struct RoiList_

typedef ::ro_control::RoiList_<std::allocator<void> > RoiList;

typedef boost::shared_ptr< ::ro_control::RoiList > RoiListPtr;
typedef boost::shared_ptr< ::ro_control::RoiList const> RoiListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ro_control::RoiList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ro_control::RoiList_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ro_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'ro_control': ['/home/neousys/Desktop/IRIM_EVENTUAL/devel/share/ro_control/msg', '/home/neousys/Desktop/IRIM_EVENTUAL/src/ro_control/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ro_control::RoiList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ro_control::RoiList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ro_control::RoiList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ro_control::RoiList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ro_control::RoiList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ro_control::RoiList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ro_control::RoiList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "02f9dca59ab31139796b538f72a25688";
  }

  static const char* value(const ::ro_control::RoiList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x02f9dca59ab31139ULL;
  static const uint64_t static_value2 = 0x796b538f72a25688ULL;
};

template<class ContainerAllocator>
struct DataType< ::ro_control::RoiList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ro_control/RoiList";
  }

  static const char* value(const ::ro_control::RoiList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ro_control::RoiList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_msgs/RegionOfInterest[] roi_list\n\
================================================================================\n\
MSG: sensor_msgs/RegionOfInterest\n\
# This message is used to specify a region of interest within an image.\n\
#\n\
# When used to specify the ROI setting of the camera when the image was\n\
# taken, the height and width fields should either match the height and\n\
# width fields for the associated image; or height = width = 0\n\
# indicates that the full resolution image was captured.\n\
\n\
uint32 x_offset  # Leftmost pixel of the ROI\n\
                 # (0 if the ROI includes the left edge of the image)\n\
uint32 y_offset  # Topmost pixel of the ROI\n\
                 # (0 if the ROI includes the top edge of the image)\n\
uint32 height    # Height of ROI\n\
uint32 width     # Width of ROI\n\
\n\
# True if a distinct rectified ROI should be calculated from the \"raw\"\n\
# ROI in this message. Typically this should be False if the full image\n\
# is captured (ROI not used), and True if a subwindow is captured (ROI\n\
# used).\n\
bool do_rectify\n\
";
  }

  static const char* value(const ::ro_control::RoiList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ro_control::RoiList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.roi_list);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RoiList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ro_control::RoiList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ro_control::RoiList_<ContainerAllocator>& v)
  {
    s << indent << "roi_list[]" << std::endl;
    for (size_t i = 0; i < v.roi_list.size(); ++i)
    {
      s << indent << "  roi_list[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::sensor_msgs::RegionOfInterest_<ContainerAllocator> >::stream(s, indent + "    ", v.roi_list[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RO_CONTROL_MESSAGE_ROILIST_H
