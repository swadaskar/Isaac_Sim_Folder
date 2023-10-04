// Generated by gencpp from file jsk_recognition_msgs/PeoplePoseArray.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_PEOPLEPOSEARRAY_H
#define JSK_RECOGNITION_MSGS_MESSAGE_PEOPLEPOSEARRAY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <jsk_recognition_msgs/PeoplePose.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct PeoplePoseArray_
{
  typedef PeoplePoseArray_<ContainerAllocator> Type;

  PeoplePoseArray_()
    : header()
    , poses()  {
    }
  PeoplePoseArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , poses(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::jsk_recognition_msgs::PeoplePose_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::jsk_recognition_msgs::PeoplePose_<ContainerAllocator> >> _poses_type;
  _poses_type poses;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> const> ConstPtr;

}; // struct PeoplePoseArray_

typedef ::jsk_recognition_msgs::PeoplePoseArray_<std::allocator<void> > PeoplePoseArray;

typedef boost::shared_ptr< ::jsk_recognition_msgs::PeoplePoseArray > PeoplePoseArrayPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::PeoplePoseArray const> PeoplePoseArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.poses == rhs.poses;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "57d49e8e639421734a0ce15bfde9d80d";
  }

  static const char* value(const ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x57d49e8e63942173ULL;
  static const uint64_t static_value2 = 0x4a0ce15bfde9d80dULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/PeoplePoseArray";
  }

  static const char* value(const ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"jsk_recognition_msgs/PeoplePose[] poses\n"
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
"\n"
"================================================================================\n"
"MSG: jsk_recognition_msgs/PeoplePose\n"
"string[] limb_names\n"
"geometry_msgs/Pose[] poses\n"
"float32[] scores\n"
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

  static const char* value(const ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.poses);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PeoplePoseArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::PeoplePoseArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "poses[]" << std::endl;
    for (size_t i = 0; i < v.poses.size(); ++i)
    {
      s << indent << "  poses[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::jsk_recognition_msgs::PeoplePose_<ContainerAllocator> >::stream(s, indent + "    ", v.poses[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_PEOPLEPOSEARRAY_H
