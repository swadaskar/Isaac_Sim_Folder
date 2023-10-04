// Generated by gencpp from file jsk_recognition_msgs/SimpleOccupancyGrid.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_SIMPLEOCCUPANCYGRID_H
#define JSK_RECOGNITION_MSGS_MESSAGE_SIMPLEOCCUPANCYGRID_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct SimpleOccupancyGrid_
{
  typedef SimpleOccupancyGrid_<ContainerAllocator> Type;

  SimpleOccupancyGrid_()
    : header()
    , coefficients()
    , resolution(0.0)
    , cells()  {
      coefficients.assign(0.0);
  }
  SimpleOccupancyGrid_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , coefficients()
    , resolution(0.0)
    , cells(_alloc)  {
  (void)_alloc;
      coefficients.assign(0.0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array<float, 4>  _coefficients_type;
  _coefficients_type coefficients;

   typedef float _resolution_type;
  _resolution_type resolution;

   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::Point_<ContainerAllocator> >> _cells_type;
  _cells_type cells;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> const> ConstPtr;

}; // struct SimpleOccupancyGrid_

typedef ::jsk_recognition_msgs::SimpleOccupancyGrid_<std::allocator<void> > SimpleOccupancyGrid;

typedef boost::shared_ptr< ::jsk_recognition_msgs::SimpleOccupancyGrid > SimpleOccupancyGridPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::SimpleOccupancyGrid const> SimpleOccupancyGridConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.coefficients == rhs.coefficients &&
    lhs.resolution == rhs.resolution &&
    lhs.cells == rhs.cells;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "25fb4ce5a31aab052ba1250fcdda9da7";
  }

  static const char* value(const ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x25fb4ce5a31aab05ULL;
  static const uint64_t static_value2 = 0x2ba1250fcdda9da7ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/SimpleOccupancyGrid";
  }

  static const char* value(const ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"# plane coefficients\n"
"float32[4] coefficients\n"
"# cells\n"
"float32 resolution\n"
"geometry_msgs/Point[] cells\n"
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
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.coefficients);
      stream.next(m.resolution);
      stream.next(m.cells);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SimpleOccupancyGrid_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::SimpleOccupancyGrid_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "coefficients[]" << std::endl;
    for (size_t i = 0; i < v.coefficients.size(); ++i)
    {
      s << indent << "  coefficients[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.coefficients[i]);
    }
    s << indent << "resolution: ";
    Printer<float>::stream(s, indent + "  ", v.resolution);
    s << indent << "cells[]" << std::endl;
    for (size_t i = 0; i < v.cells.size(); ++i)
    {
      s << indent << "  cells[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.cells[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_SIMPLEOCCUPANCYGRID_H
