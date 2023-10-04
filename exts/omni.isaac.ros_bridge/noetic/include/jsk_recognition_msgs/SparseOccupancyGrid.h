// Generated by gencpp from file jsk_recognition_msgs/SparseOccupancyGrid.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_SPARSEOCCUPANCYGRID_H
#define JSK_RECOGNITION_MSGS_MESSAGE_SPARSEOCCUPANCYGRID_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <jsk_recognition_msgs/SparseOccupancyGridColumn.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct SparseOccupancyGrid_
{
  typedef SparseOccupancyGrid_<ContainerAllocator> Type;

  SparseOccupancyGrid_()
    : header()
    , origin_pose()
    , resolution(0.0)
    , columns()  {
    }
  SparseOccupancyGrid_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , origin_pose(_alloc)
    , resolution(0.0)
    , columns(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _origin_pose_type;
  _origin_pose_type origin_pose;

   typedef float _resolution_type;
  _resolution_type resolution;

   typedef std::vector< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> >> _columns_type;
  _columns_type columns;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> const> ConstPtr;

}; // struct SparseOccupancyGrid_

typedef ::jsk_recognition_msgs::SparseOccupancyGrid_<std::allocator<void> > SparseOccupancyGrid;

typedef boost::shared_ptr< ::jsk_recognition_msgs::SparseOccupancyGrid > SparseOccupancyGridPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::SparseOccupancyGrid const> SparseOccupancyGridConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.origin_pose == rhs.origin_pose &&
    lhs.resolution == rhs.resolution &&
    lhs.columns == rhs.columns;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "497269ddab6058d0d4860f25dc49448f";
  }

  static const char* value(const ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x497269ddab6058d0ULL;
  static const uint64_t static_value2 = 0xd4860f25dc49448fULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/SparseOccupancyGrid";
  }

  static const char* value(const ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"geometry_msgs/Pose origin_pose\n"
"float32 resolution\n"
"\n"
"SparseOccupancyGridColumn[] columns\n"
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
"\n"
"================================================================================\n"
"MSG: jsk_recognition_msgs/SparseOccupancyGridColumn\n"
"int32 column_index\n"
"SparseOccupancyGridCell[] cells\n"
"\n"
"================================================================================\n"
"MSG: jsk_recognition_msgs/SparseOccupancyGridCell\n"
"int32 row_index\n"
"float32 value\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.origin_pose);
      stream.next(m.resolution);
      stream.next(m.columns);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SparseOccupancyGrid_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::SparseOccupancyGrid_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "origin_pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.origin_pose);
    s << indent << "resolution: ";
    Printer<float>::stream(s, indent + "  ", v.resolution);
    s << indent << "columns[]" << std::endl;
    for (size_t i = 0; i < v.columns.size(); ++i)
    {
      s << indent << "  columns[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> >::stream(s, indent + "    ", v.columns[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_SPARSEOCCUPANCYGRID_H
