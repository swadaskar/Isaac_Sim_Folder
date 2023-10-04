// Generated by gencpp from file jsk_recognition_msgs/SegmentStamped.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_SEGMENTSTAMPED_H
#define JSK_RECOGNITION_MSGS_MESSAGE_SEGMENTSTAMPED_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <jsk_recognition_msgs/Segment.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct SegmentStamped_
{
  typedef SegmentStamped_<ContainerAllocator> Type;

  SegmentStamped_()
    : header()
    , segment()  {
    }
  SegmentStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , segment(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::jsk_recognition_msgs::Segment_<ContainerAllocator>  _segment_type;
  _segment_type segment;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> const> ConstPtr;

}; // struct SegmentStamped_

typedef ::jsk_recognition_msgs::SegmentStamped_<std::allocator<void> > SegmentStamped;

typedef boost::shared_ptr< ::jsk_recognition_msgs::SegmentStamped > SegmentStampedPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::SegmentStamped const> SegmentStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.segment == rhs.segment;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1f2fbdf9b9a242110bee5312e7718d1f";
  }

  static const char* value(const ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1f2fbdf9b9a24211ULL;
  static const uint64_t static_value2 = 0x0bee5312e7718d1fULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/SegmentStamped";
  }

  static const char* value(const ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"Segment segment\n"
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
"MSG: jsk_recognition_msgs/Segment\n"
"geometry_msgs/Point start_point\n"
"geometry_msgs/Point end_point\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.segment);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SegmentStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::SegmentStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "segment: ";
    s << std::endl;
    Printer< ::jsk_recognition_msgs::Segment_<ContainerAllocator> >::stream(s, indent + "  ", v.segment);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_SEGMENTSTAMPED_H
