// Generated by gencpp from file cortex_control/MsgServiceResponse.msg
// DO NOT EDIT!


#ifndef CORTEX_CONTROL_MESSAGE_MSGSERVICERESPONSE_H
#define CORTEX_CONTROL_MESSAGE_MSGSERVICERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cortex_control
{
template <class ContainerAllocator>
struct MsgServiceResponse_
{
  typedef MsgServiceResponse_<ContainerAllocator> Type;

  MsgServiceResponse_()
    : result(0)
    , error_str()  {
    }
  MsgServiceResponse_(const ContainerAllocator& _alloc)
    : result(0)
    , error_str(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _result_type;
  _result_type result;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _error_str_type;
  _error_str_type error_str;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(SUCCESS)
  #undef SUCCESS
#endif
#if defined(_WIN32) && defined(FAILURE)
  #undef FAILURE
#endif

  enum {
    SUCCESS = 0u,
    FAILURE = 1u,
  };


  typedef boost::shared_ptr< ::cortex_control::MsgServiceResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cortex_control::MsgServiceResponse_<ContainerAllocator> const> ConstPtr;

}; // struct MsgServiceResponse_

typedef ::cortex_control::MsgServiceResponse_<std::allocator<void> > MsgServiceResponse;

typedef boost::shared_ptr< ::cortex_control::MsgServiceResponse > MsgServiceResponsePtr;
typedef boost::shared_ptr< ::cortex_control::MsgServiceResponse const> MsgServiceResponseConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cortex_control::MsgServiceResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cortex_control::MsgServiceResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cortex_control::MsgServiceResponse_<ContainerAllocator1> & lhs, const ::cortex_control::MsgServiceResponse_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result &&
    lhs.error_str == rhs.error_str;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cortex_control::MsgServiceResponse_<ContainerAllocator1> & lhs, const ::cortex_control::MsgServiceResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cortex_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cortex_control::MsgServiceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cortex_control::MsgServiceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cortex_control::MsgServiceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cortex_control::MsgServiceResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cortex_control::MsgServiceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cortex_control::MsgServiceResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cortex_control::MsgServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "475e1bc2772fc07032175b4dd5d6ce13";
  }

  static const char* value(const ::cortex_control::MsgServiceResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x475e1bc2772fc070ULL;
  static const uint64_t static_value2 = 0x32175b4dd5d6ce13ULL;
};

template<class ContainerAllocator>
struct DataType< ::cortex_control::MsgServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cortex_control/MsgServiceResponse";
  }

  static const char* value(const ::cortex_control::MsgServiceResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cortex_control::MsgServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 SUCCESS=0\n"
"uint8 FAILURE=1\n"
"uint8 result\n"
"string error_str\n"
"\n"
;
  }

  static const char* value(const ::cortex_control::MsgServiceResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cortex_control::MsgServiceResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
      stream.next(m.error_str);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MsgServiceResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cortex_control::MsgServiceResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cortex_control::MsgServiceResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.result);
    s << indent << "error_str: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.error_str);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CORTEX_CONTROL_MESSAGE_MSGSERVICERESPONSE_H
