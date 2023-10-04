// Generated by gencpp from file jsk_recognition_msgs/NonMaximumSuppressionResponse.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_NONMAXIMUMSUPPRESSIONRESPONSE_H
#define JSK_RECOGNITION_MSGS_MESSAGE_NONMAXIMUMSUPPRESSIONRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <jsk_recognition_msgs/Rect.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct NonMaximumSuppressionResponse_
{
  typedef NonMaximumSuppressionResponse_<ContainerAllocator> Type;

  NonMaximumSuppressionResponse_()
    : bbox()
    , bbox_count(0)  {
    }
  NonMaximumSuppressionResponse_(const ContainerAllocator& _alloc)
    : bbox(_alloc)
    , bbox_count(0)  {
  (void)_alloc;
    }



   typedef std::vector< ::jsk_recognition_msgs::Rect_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::jsk_recognition_msgs::Rect_<ContainerAllocator> >> _bbox_type;
  _bbox_type bbox;

   typedef int64_t _bbox_count_type;
  _bbox_count_type bbox_count;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct NonMaximumSuppressionResponse_

typedef ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<std::allocator<void> > NonMaximumSuppressionResponse;

typedef boost::shared_ptr< ::jsk_recognition_msgs::NonMaximumSuppressionResponse > NonMaximumSuppressionResponsePtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::NonMaximumSuppressionResponse const> NonMaximumSuppressionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator2> & rhs)
{
  return lhs.bbox == rhs.bbox &&
    lhs.bbox_count == rhs.bbox_count;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8db21435e67f6d13fc94ccbd355f30f1";
  }

  static const char* value(const ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8db21435e67f6d13ULL;
  static const uint64_t static_value2 = 0xfc94ccbd355f30f1ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/NonMaximumSuppressionResponse";
  }

  static const char* value(const ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/Rect[] bbox\n"
"int64 bbox_count\n"
"\n"
"\n"
"================================================================================\n"
"MSG: jsk_recognition_msgs/Rect\n"
"int32 x\n"
"int32 y\n"
"int32 width\n"
"int32 height\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.bbox);
      stream.next(m.bbox_count);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NonMaximumSuppressionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::NonMaximumSuppressionResponse_<ContainerAllocator>& v)
  {
    s << indent << "bbox[]" << std::endl;
    for (size_t i = 0; i < v.bbox.size(); ++i)
    {
      s << indent << "  bbox[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::jsk_recognition_msgs::Rect_<ContainerAllocator> >::stream(s, indent + "    ", v.bbox[i]);
    }
    s << indent << "bbox_count: ";
    Printer<int64_t>::stream(s, indent + "  ", v.bbox_count);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_NONMAXIMUMSUPPRESSIONRESPONSE_H
