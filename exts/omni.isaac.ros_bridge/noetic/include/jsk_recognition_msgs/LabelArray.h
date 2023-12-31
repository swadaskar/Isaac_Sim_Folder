// Generated by gencpp from file jsk_recognition_msgs/LabelArray.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_LABELARRAY_H
#define JSK_RECOGNITION_MSGS_MESSAGE_LABELARRAY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <jsk_recognition_msgs/Label.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct LabelArray_
{
  typedef LabelArray_<ContainerAllocator> Type;

  LabelArray_()
    : header()
    , labels()  {
    }
  LabelArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , labels(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::jsk_recognition_msgs::Label_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::jsk_recognition_msgs::Label_<ContainerAllocator> >> _labels_type;
  _labels_type labels;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> const> ConstPtr;

}; // struct LabelArray_

typedef ::jsk_recognition_msgs::LabelArray_<std::allocator<void> > LabelArray;

typedef boost::shared_ptr< ::jsk_recognition_msgs::LabelArray > LabelArrayPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::LabelArray const> LabelArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::LabelArray_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::LabelArray_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.labels == rhs.labels;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::LabelArray_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::LabelArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8cdb9aed89bee725ff5ad76b2986927d";
  }

  static const char* value(const ::jsk_recognition_msgs::LabelArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8cdb9aed89bee725ULL;
  static const uint64_t static_value2 = 0xff5ad76b2986927dULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/LabelArray";
  }

  static const char* value(const ::jsk_recognition_msgs::LabelArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"Label[] labels\n"
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
"MSG: jsk_recognition_msgs/Label\n"
"int32 id\n"
"string name\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::LabelArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.labels);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LabelArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::LabelArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::LabelArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "labels[]" << std::endl;
    for (size_t i = 0; i < v.labels.size(); ++i)
    {
      s << indent << "  labels[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::jsk_recognition_msgs::Label_<ContainerAllocator> >::stream(s, indent + "    ", v.labels[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_LABELARRAY_H
