// Generated by gencpp from file jsk_recognition_msgs/ModelCoefficientsArray.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_MODELCOEFFICIENTSARRAY_H
#define JSK_RECOGNITION_MSGS_MESSAGE_MODELCOEFFICIENTSARRAY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <pcl_msgs/ModelCoefficients.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct ModelCoefficientsArray_
{
  typedef ModelCoefficientsArray_<ContainerAllocator> Type;

  ModelCoefficientsArray_()
    : header()
    , coefficients()  {
    }
  ModelCoefficientsArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , coefficients(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::pcl_msgs::ModelCoefficients_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::pcl_msgs::ModelCoefficients_<ContainerAllocator> >> _coefficients_type;
  _coefficients_type coefficients;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> const> ConstPtr;

}; // struct ModelCoefficientsArray_

typedef ::jsk_recognition_msgs::ModelCoefficientsArray_<std::allocator<void> > ModelCoefficientsArray;

typedef boost::shared_ptr< ::jsk_recognition_msgs::ModelCoefficientsArray > ModelCoefficientsArrayPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::ModelCoefficientsArray const> ModelCoefficientsArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.coefficients == rhs.coefficients;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "059efee897c3f4ae027a493e30c4c26b";
  }

  static const char* value(const ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x059efee897c3f4aeULL;
  static const uint64_t static_value2 = 0x027a493e30c4c26bULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/ModelCoefficientsArray";
  }

  static const char* value(const ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ModelCoefficientsArray is used to represent coefficients of model\n"
"# for each segmented clusters.\n"
"# Simply put, ModelCoefficientsArray is a list of ModelCoefficients.\n"
"Header header\n"
"pcl_msgs/ModelCoefficients[] coefficients\n"
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
"MSG: pcl_msgs/ModelCoefficients\n"
"Header header\n"
"float32[] values\n"
"\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.coefficients);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ModelCoefficientsArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::ModelCoefficientsArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "coefficients[]" << std::endl;
    for (size_t i = 0; i < v.coefficients.size(); ++i)
    {
      s << indent << "  coefficients[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pcl_msgs::ModelCoefficients_<ContainerAllocator> >::stream(s, indent + "    ", v.coefficients[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_MODELCOEFFICIENTSARRAY_H
