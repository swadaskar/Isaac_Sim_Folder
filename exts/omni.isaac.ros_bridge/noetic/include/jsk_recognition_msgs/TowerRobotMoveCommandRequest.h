// Generated by gencpp from file jsk_recognition_msgs/TowerRobotMoveCommandRequest.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_TOWERROBOTMOVECOMMANDREQUEST_H
#define JSK_RECOGNITION_MSGS_MESSAGE_TOWERROBOTMOVECOMMANDREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct TowerRobotMoveCommandRequest_
{
  typedef TowerRobotMoveCommandRequest_<ContainerAllocator> Type;

  TowerRobotMoveCommandRequest_()
    : robot_index(0)
    , plate_index(0)
    , from_tower(0)
    , to_tower(0)
    , option_command(0)  {
    }
  TowerRobotMoveCommandRequest_(const ContainerAllocator& _alloc)
    : robot_index(0)
    , plate_index(0)
    , from_tower(0)
    , to_tower(0)
    , option_command(0)  {
  (void)_alloc;
    }



   typedef int32_t _robot_index_type;
  _robot_index_type robot_index;

   typedef int32_t _plate_index_type;
  _plate_index_type plate_index;

   typedef int32_t _from_tower_type;
  _from_tower_type from_tower;

   typedef int32_t _to_tower_type;
  _to_tower_type to_tower;

   typedef int32_t _option_command_type;
  _option_command_type option_command;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(ROBOT1)
  #undef ROBOT1
#endif
#if defined(_WIN32) && defined(ROBOT2)
  #undef ROBOT2
#endif
#if defined(_WIN32) && defined(ROBOT3)
  #undef ROBOT3
#endif
#if defined(_WIN32) && defined(PLATE_SMALL)
  #undef PLATE_SMALL
#endif
#if defined(_WIN32) && defined(PLATE_MIDDLE)
  #undef PLATE_MIDDLE
#endif
#if defined(_WIN32) && defined(PLATE_LARGE)
  #undef PLATE_LARGE
#endif
#if defined(_WIN32) && defined(TOWER_LOWEST)
  #undef TOWER_LOWEST
#endif
#if defined(_WIN32) && defined(TOWER_MIDDLE)
  #undef TOWER_MIDDLE
#endif
#if defined(_WIN32) && defined(TOWER_HIGHEST)
  #undef TOWER_HIGHEST
#endif
#if defined(_WIN32) && defined(TOWER_LOWEST2)
  #undef TOWER_LOWEST2
#endif
#if defined(_WIN32) && defined(OPTION_NONE)
  #undef OPTION_NONE
#endif
#if defined(_WIN32) && defined(OPTION_MOVE_INITIAL)
  #undef OPTION_MOVE_INITIAL
#endif

  enum {
    ROBOT1 = 1,
    ROBOT2 = 2,
    ROBOT3 = 3,
    PLATE_SMALL = 1,
    PLATE_MIDDLE = 2,
    PLATE_LARGE = 3,
    TOWER_LOWEST = 1,
    TOWER_MIDDLE = 2,
    TOWER_HIGHEST = 3,
    TOWER_LOWEST2 = 1,
    OPTION_NONE = 0,
    OPTION_MOVE_INITIAL = 1,
  };


  typedef boost::shared_ptr< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TowerRobotMoveCommandRequest_

typedef ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<std::allocator<void> > TowerRobotMoveCommandRequest;

typedef boost::shared_ptr< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest > TowerRobotMoveCommandRequestPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest const> TowerRobotMoveCommandRequestConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator2> & rhs)
{
  return lhs.robot_index == rhs.robot_index &&
    lhs.plate_index == rhs.plate_index &&
    lhs.from_tower == rhs.from_tower &&
    lhs.to_tower == rhs.to_tower &&
    lhs.option_command == rhs.option_command;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aadba056bdce0494569ab50ecd2ec90c";
  }

  static const char* value(const ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaadba056bdce0494ULL;
  static const uint64_t static_value2 = 0x569ab50ecd2ec90cULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/TowerRobotMoveCommandRequest";
  }

  static const char* value(const ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# a service to move robot with tower index\n"
"byte ROBOT1=1\n"
"byte ROBOT2=2\n"
"byte ROBOT3=3\n"
"\n"
"byte PLATE_SMALL=1\n"
"byte PLATE_MIDDLE=2\n"
"byte PLATE_LARGE=3\n"
"\n"
"byte TOWER_LOWEST=1\n"
"byte TOWER_MIDDLE=2\n"
"byte TOWER_HIGHEST=3\n"
"byte TOWER_LOWEST2=1\n"
"\n"
"byte OPTION_NONE=0\n"
"byte OPTION_MOVE_INITIAL=1\n"
"\n"
"int32 robot_index\n"
"int32 plate_index\n"
"int32 from_tower\n"
"int32 to_tower\n"
"int32 option_command\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.robot_index);
      stream.next(m.plate_index);
      stream.next(m.from_tower);
      stream.next(m.to_tower);
      stream.next(m.option_command);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TowerRobotMoveCommandRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::TowerRobotMoveCommandRequest_<ContainerAllocator>& v)
  {
    s << indent << "robot_index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.robot_index);
    s << indent << "plate_index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.plate_index);
    s << indent << "from_tower: ";
    Printer<int32_t>::stream(s, indent + "  ", v.from_tower);
    s << indent << "to_tower: ";
    Printer<int32_t>::stream(s, indent + "  ", v.to_tower);
    s << indent << "option_command: ";
    Printer<int32_t>::stream(s, indent + "  ", v.option_command);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_TOWERROBOTMOVECOMMANDREQUEST_H
