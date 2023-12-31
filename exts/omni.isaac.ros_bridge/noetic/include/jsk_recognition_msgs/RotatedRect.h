// Generated by gencpp from file jsk_recognition_msgs/RotatedRect.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_ROTATEDRECT_H
#define JSK_RECOGNITION_MSGS_MESSAGE_ROTATEDRECT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct RotatedRect_
{
  typedef RotatedRect_<ContainerAllocator> Type;

  RotatedRect_()
    : x(0.0)
    , y(0.0)
    , width(0.0)
    , height(0.0)
    , angle(0.0)  {
    }
  RotatedRect_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , width(0.0)
    , height(0.0)
    , angle(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _width_type;
  _width_type width;

   typedef double _height_type;
  _height_type height;

   typedef double _angle_type;
  _angle_type angle;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> const> ConstPtr;

}; // struct RotatedRect_

typedef ::jsk_recognition_msgs::RotatedRect_<std::allocator<void> > RotatedRect;

typedef boost::shared_ptr< ::jsk_recognition_msgs::RotatedRect > RotatedRectPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::RotatedRect const> RotatedRectConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.width == rhs.width &&
    lhs.height == rhs.height &&
    lhs.angle == rhs.angle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e970c93bbd35a570f7d9acc8228e9280";
  }

  static const char* value(const ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe970c93bbd35a570ULL;
  static const uint64_t static_value2 = 0xf7d9acc8228e9280ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/RotatedRect";
  }

  static const char* value(const ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"float64 width\n"
"float64 height\n"
"float64 angle # degree\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.width);
      stream.next(m.height);
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RotatedRect_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::RotatedRect_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "width: ";
    Printer<double>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<double>::stream(s, indent + "  ", v.height);
    s << indent << "angle: ";
    Printer<double>::stream(s, indent + "  ", v.angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_ROTATEDRECT_H
