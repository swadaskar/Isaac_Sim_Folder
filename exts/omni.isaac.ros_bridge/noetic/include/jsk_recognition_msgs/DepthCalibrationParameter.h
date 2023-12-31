// Generated by gencpp from file jsk_recognition_msgs/DepthCalibrationParameter.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_DEPTHCALIBRATIONPARAMETER_H
#define JSK_RECOGNITION_MSGS_MESSAGE_DEPTHCALIBRATIONPARAMETER_H


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
struct DepthCalibrationParameter_
{
  typedef DepthCalibrationParameter_<ContainerAllocator> Type;

  DepthCalibrationParameter_()
    : coefficients2()
    , coefficients1()
    , coefficients0()
    , use_abs(false)  {
    }
  DepthCalibrationParameter_(const ContainerAllocator& _alloc)
    : coefficients2(_alloc)
    , coefficients1(_alloc)
    , coefficients0(_alloc)
    , use_abs(false)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _coefficients2_type;
  _coefficients2_type coefficients2;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _coefficients1_type;
  _coefficients1_type coefficients1;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _coefficients0_type;
  _coefficients0_type coefficients0;

   typedef uint8_t _use_abs_type;
  _use_abs_type use_abs;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> const> ConstPtr;

}; // struct DepthCalibrationParameter_

typedef ::jsk_recognition_msgs::DepthCalibrationParameter_<std::allocator<void> > DepthCalibrationParameter;

typedef boost::shared_ptr< ::jsk_recognition_msgs::DepthCalibrationParameter > DepthCalibrationParameterPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::DepthCalibrationParameter const> DepthCalibrationParameterConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator2> & rhs)
{
  return lhs.coefficients2 == rhs.coefficients2 &&
    lhs.coefficients1 == rhs.coefficients1 &&
    lhs.coefficients0 == rhs.coefficients0 &&
    lhs.use_abs == rhs.use_abs;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d8318983ee0a76ad66ecf4b504350888";
  }

  static const char* value(const ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd8318983ee0a76adULL;
  static const uint64_t static_value2 = 0x66ecf4b504350888ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/DepthCalibrationParameter";
  }

  static const char* value(const ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# each vector stands for C(u, v)\n"
"# C(u, v) = a_0 * u^2 + a_1 * u + a_2 * v^2 + a_3 * v + a_4\n"
"float64[] coefficients2\n"
"float64[] coefficients1\n"
"float64[] coefficients0\n"
"bool use_abs\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.coefficients2);
      stream.next(m.coefficients1);
      stream.next(m.coefficients0);
      stream.next(m.use_abs);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DepthCalibrationParameter_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::DepthCalibrationParameter_<ContainerAllocator>& v)
  {
    s << indent << "coefficients2[]" << std::endl;
    for (size_t i = 0; i < v.coefficients2.size(); ++i)
    {
      s << indent << "  coefficients2[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.coefficients2[i]);
    }
    s << indent << "coefficients1[]" << std::endl;
    for (size_t i = 0; i < v.coefficients1.size(); ++i)
    {
      s << indent << "  coefficients1[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.coefficients1[i]);
    }
    s << indent << "coefficients0[]" << std::endl;
    for (size_t i = 0; i < v.coefficients0.size(); ++i)
    {
      s << indent << "  coefficients0[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.coefficients0[i]);
    }
    s << indent << "use_abs: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_abs);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_DEPTHCALIBRATIONPARAMETER_H
