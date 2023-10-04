// Generated by gencpp from file jsk_recognition_msgs/SimpleOccupancyGridArray.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_SIMPLEOCCUPANCYGRIDARRAY_H
#define JSK_RECOGNITION_MSGS_MESSAGE_SIMPLEOCCUPANCYGRIDARRAY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <jsk_recognition_msgs/SimpleOccupancyGrid.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct SimpleOccupancyGridArray_
{
  typedef SimpleOccupancyGridArray_<ContainerAllocator> Type;

  SimpleOccupancyGridArray_()
    : header()
    , grids()  {
    }
  SimpleOccupancyGridArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , grids(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> >> _grids_type;
  _grids_type grids;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> const> ConstPtr;

}; // struct SimpleOccupancyGridArray_

typedef ::jsk_recognition_msgs::SimpleOccupancyGridArray_<std::allocator<void> > SimpleOccupancyGridArray;

typedef boost::shared_ptr< ::jsk_recognition_msgs::SimpleOccupancyGridArray > SimpleOccupancyGridArrayPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::SimpleOccupancyGridArray const> SimpleOccupancyGridArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.grids == rhs.grids;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e448b458270a6ec465d66169c4180932";
  }

  static const char* value(const ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe448b458270a6ec4ULL;
  static const uint64_t static_value2 = 0x65d66169c4180932ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/SimpleOccupancyGridArray";
  }

  static const char* value(const ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"SimpleOccupancyGrid[] grids\n"
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
"MSG: jsk_recognition_msgs/SimpleOccupancyGrid\n"
"Header header\n"
"# plane coefficients\n"
"float32[4] coefficients\n"
"# cells\n"
"float32 resolution\n"
"geometry_msgs/Point[] cells\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.grids);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SimpleOccupancyGridArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::SimpleOccupancyGridArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "grids[]" << std::endl;
    for (size_t i = 0; i < v.grids.size(); ++i)
    {
      s << indent << "  grids[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> >::stream(s, indent + "    ", v.grids[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_SIMPLEOCCUPANCYGRIDARRAY_H
