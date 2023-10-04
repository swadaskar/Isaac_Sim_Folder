// Generated by gencpp from file jsk_recognition_msgs/ClusterPointIndices.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_CLUSTERPOINTINDICES_H
#define JSK_RECOGNITION_MSGS_MESSAGE_CLUSTERPOINTINDICES_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <pcl_msgs/PointIndices.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct ClusterPointIndices_
{
  typedef ClusterPointIndices_<ContainerAllocator> Type;

  ClusterPointIndices_()
    : header()
    , cluster_indices()  {
    }
  ClusterPointIndices_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , cluster_indices(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::pcl_msgs::PointIndices_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::pcl_msgs::PointIndices_<ContainerAllocator> >> _cluster_indices_type;
  _cluster_indices_type cluster_indices;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> const> ConstPtr;

}; // struct ClusterPointIndices_

typedef ::jsk_recognition_msgs::ClusterPointIndices_<std::allocator<void> > ClusterPointIndices;

typedef boost::shared_ptr< ::jsk_recognition_msgs::ClusterPointIndices > ClusterPointIndicesPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::ClusterPointIndices const> ClusterPointIndicesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.cluster_indices == rhs.cluster_indices;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d43e94ea5e491effac7685a42b7b9d14";
  }

  static const char* value(const ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd43e94ea5e491effULL;
  static const uint64_t static_value2 = 0xac7685a42b7b9d14ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/ClusterPointIndices";
  }

  static const char* value(const ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ClusterPointIndices is used to represent segmentation result.\n"
"# Simply put, ClusterPointIndices is a list of PointIndices.\n"
"Header header\n"
"pcl_msgs/PointIndices[] cluster_indices\n"
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
"MSG: pcl_msgs/PointIndices\n"
"Header header\n"
"int32[] indices\n"
"\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.cluster_indices);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ClusterPointIndices_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::ClusterPointIndices_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "cluster_indices[]" << std::endl;
    for (size_t i = 0; i < v.cluster_indices.size(); ++i)
    {
      s << indent << "  cluster_indices[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pcl_msgs::PointIndices_<ContainerAllocator> >::stream(s, indent + "    ", v.cluster_indices[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_CLUSTERPOINTINDICES_H
