// Generated by gencpp from file jsk_recognition_msgs/ContactSensor.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_CONTACTSENSOR_H
#define JSK_RECOGNITION_MSGS_MESSAGE_CONTACTSENSOR_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct ContactSensor_
{
  typedef ContactSensor_<ContainerAllocator> Type;

  ContactSensor_()
    : header()
    , contact(false)
    , link_name()  {
    }
  ContactSensor_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , contact(false)
    , link_name(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _contact_type;
  _contact_type contact;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _link_name_type;
  _link_name_type link_name;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> const> ConstPtr;

}; // struct ContactSensor_

typedef ::jsk_recognition_msgs::ContactSensor_<std::allocator<void> > ContactSensor;

typedef boost::shared_ptr< ::jsk_recognition_msgs::ContactSensor > ContactSensorPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::ContactSensor const> ContactSensorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.contact == rhs.contact &&
    lhs.link_name == rhs.link_name;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "364b2b952a51d85dfa877e334264e361";
  }

  static const char* value(const ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x364b2b952a51d85dULL;
  static const uint64_t static_value2 = 0xfa877e334264e361ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/ContactSensor";
  }

  static const char* value(const ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Header\n"
"Header header\n"
"\n"
"# Whether sensor detects contact or not\n"
"bool contact\n"
"\n"
"string link_name\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.contact);
      stream.next(m.link_name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ContactSensor_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::ContactSensor_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "contact: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.contact);
    s << indent << "link_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.link_name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_CONTACTSENSOR_H
