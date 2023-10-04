// Generated by gencpp from file jsk_recognition_msgs/ICPAlignWithBoxResponse.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_ICPALIGNWITHBOXRESPONSE_H
#define JSK_RECOGNITION_MSGS_MESSAGE_ICPALIGNWITHBOXRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <jsk_recognition_msgs/ICPResult.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct ICPAlignWithBoxResponse_
{
  typedef ICPAlignWithBoxResponse_<ContainerAllocator> Type;

  ICPAlignWithBoxResponse_()
    : result()  {
    }
  ICPAlignWithBoxResponse_(const ContainerAllocator& _alloc)
    : result(_alloc)  {
  (void)_alloc;
    }



   typedef  ::jsk_recognition_msgs::ICPResult_<ContainerAllocator>  _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ICPAlignWithBoxResponse_

typedef ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<std::allocator<void> > ICPAlignWithBoxResponse;

typedef boost::shared_ptr< ::jsk_recognition_msgs::ICPAlignWithBoxResponse > ICPAlignWithBoxResponsePtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::ICPAlignWithBoxResponse const> ICPAlignWithBoxResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a511a876c2be142caffd78741c68e4cf";
  }

  static const char* value(const ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa511a876c2be142cULL;
  static const uint64_t static_value2 = 0xaffd78741c68e4cfULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/ICPAlignWithBoxResponse";
  }

  static const char* value(const ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/ICPResult result\n"
"\n"
"\n"
"================================================================================\n"
"MSG: jsk_recognition_msgs/ICPResult\n"
"Header header\n"
"geometry_msgs/Pose pose\n"
"string name\n"
"float64 score\n"
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
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ICPAlignWithBoxResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::ICPAlignWithBoxResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    s << std::endl;
    Printer< ::jsk_recognition_msgs::ICPResult_<ContainerAllocator> >::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_ICPALIGNWITHBOXRESPONSE_H
