// Generated by gencpp from file cortex_control/CortexCommandAck.msg
// DO NOT EDIT!


#ifndef CORTEX_CONTROL_MESSAGE_CORTEXCOMMANDACK_H
#define CORTEX_CONTROL_MESSAGE_CORTEXCOMMANDACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cortex_control
{
template <class ContainerAllocator>
struct CortexCommandAck_
{
  typedef CortexCommandAck_<ContainerAllocator> Type;

  CortexCommandAck_()
    : cortex_command_time()
    , cortex_command_id(0)
    , time_offset()  {
    }
  CortexCommandAck_(const ContainerAllocator& _alloc)
    : cortex_command_time()
    , cortex_command_id(0)
    , time_offset()  {
  (void)_alloc;
    }



   typedef ros::Time _cortex_command_time_type;
  _cortex_command_time_type cortex_command_time;

   typedef int64_t _cortex_command_id_type;
  _cortex_command_id_type cortex_command_id;

   typedef ros::Duration _time_offset_type;
  _time_offset_type time_offset;





  typedef boost::shared_ptr< ::cortex_control::CortexCommandAck_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cortex_control::CortexCommandAck_<ContainerAllocator> const> ConstPtr;

}; // struct CortexCommandAck_

typedef ::cortex_control::CortexCommandAck_<std::allocator<void> > CortexCommandAck;

typedef boost::shared_ptr< ::cortex_control::CortexCommandAck > CortexCommandAckPtr;
typedef boost::shared_ptr< ::cortex_control::CortexCommandAck const> CortexCommandAckConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cortex_control::CortexCommandAck_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cortex_control::CortexCommandAck_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cortex_control::CortexCommandAck_<ContainerAllocator1> & lhs, const ::cortex_control::CortexCommandAck_<ContainerAllocator2> & rhs)
{
  return lhs.cortex_command_time == rhs.cortex_command_time &&
    lhs.cortex_command_id == rhs.cortex_command_id &&
    lhs.time_offset == rhs.time_offset;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cortex_control::CortexCommandAck_<ContainerAllocator1> & lhs, const ::cortex_control::CortexCommandAck_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cortex_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cortex_control::CortexCommandAck_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cortex_control::CortexCommandAck_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cortex_control::CortexCommandAck_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cortex_control::CortexCommandAck_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cortex_control::CortexCommandAck_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cortex_control::CortexCommandAck_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cortex_control::CortexCommandAck_<ContainerAllocator> >
{
  static const char* value()
  {
    return "81e929ba8529002a2be583a8c6600952";
  }

  static const char* value(const ::cortex_control::CortexCommandAck_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x81e929ba8529002aULL;
  static const uint64_t static_value2 = 0x2be583a8c6600952ULL;
};

template<class ContainerAllocator>
struct DataType< ::cortex_control::CortexCommandAck_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cortex_control/CortexCommandAck";
  }

  static const char* value(const ::cortex_control::CortexCommandAck_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cortex_control::CortexCommandAck_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Command time stamp of the latest Cortex command received.\n"
"time cortex_command_time\n"
"\n"
"# ID of the latest command received.\n"
"int64 cortex_command_id\n"
"\n"
"# If there is a slight accumulative clock rate difference between the Cortex\n"
"# commander and the low-level controller, the time offset gives how much\n"
"# further ahead the controller's clock is from the Cortex commander's clock (note\n"
"# it can be negative). So synchronizing the clocks would entail\n"
"#\n"
"#    <cortex_commander_time_synced> = <cortex_commander_time> + time_offset\n"
"duration time_offset\n"
;
  }

  static const char* value(const ::cortex_control::CortexCommandAck_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cortex_control::CortexCommandAck_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.cortex_command_time);
      stream.next(m.cortex_command_id);
      stream.next(m.time_offset);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CortexCommandAck_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cortex_control::CortexCommandAck_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cortex_control::CortexCommandAck_<ContainerAllocator>& v)
  {
    s << indent << "cortex_command_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.cortex_command_time);
    s << indent << "cortex_command_id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.cortex_command_id);
    s << indent << "time_offset: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.time_offset);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CORTEX_CONTROL_MESSAGE_CORTEXCOMMANDACK_H
