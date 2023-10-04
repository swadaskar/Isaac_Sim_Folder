// Generated by gencpp from file jsk_recognition_msgs/Object.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_OBJECT_H
#define JSK_RECOGNITION_MSGS_MESSAGE_OBJECT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Vector3.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct Object_
{
  typedef Object_<ContainerAllocator> Type;

  Object_()
    : id(0)
    , name()
    , class_id(0)
    , class_name()
    , image_resources()
    , mesh_resource()
    , weight(0.0)
    , dimensions()  {
    }
  Object_(const ContainerAllocator& _alloc)
    : id(0)
    , name(_alloc)
    , class_id(0)
    , class_name(_alloc)
    , image_resources(_alloc)
    , mesh_resource(_alloc)
    , weight(0.0)
    , dimensions(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _id_type;
  _id_type id;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _name_type;
  _name_type name;

   typedef int32_t _class_id_type;
  _class_id_type class_id;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _class_name_type;
  _class_name_type class_name;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _image_resources_type;
  _image_resources_type image_resources;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _mesh_resource_type;
  _mesh_resource_type mesh_resource;

   typedef float _weight_type;
  _weight_type weight;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _dimensions_type;
  _dimensions_type dimensions;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::Object_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::Object_<ContainerAllocator> const> ConstPtr;

}; // struct Object_

typedef ::jsk_recognition_msgs::Object_<std::allocator<void> > Object;

typedef boost::shared_ptr< ::jsk_recognition_msgs::Object > ObjectPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::Object const> ObjectConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::Object_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::Object_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::Object_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::Object_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.name == rhs.name &&
    lhs.class_id == rhs.class_id &&
    lhs.class_name == rhs.class_name &&
    lhs.image_resources == rhs.image_resources &&
    lhs.mesh_resource == rhs.mesh_resource &&
    lhs.weight == rhs.weight &&
    lhs.dimensions == rhs.dimensions;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::Object_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::Object_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::Object_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::Object_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::Object_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::Object_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::Object_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::Object_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "57c53e712043df0244b0482d0447adee";
  }

  static const char* value(const ::jsk_recognition_msgs::Object_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x57c53e712043df02ULL;
  static const uint64_t static_value2 = 0x44b0482d0447adeeULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/Object";
  }

  static const char* value(const ::jsk_recognition_msgs::Object_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# object instant info (ex. oreo_cookie)\n"
"int32 id  # object id\n"
"string name  # object name\n"
"\n"
"# object class info (ex. snack)\n"
"int32 class_id\n"
"string class_name\n"
"\n"
"string[] image_resources  # image urls\n"
"string mesh_resource  # mesh file url\n"
"\n"
"float32 weight  # weight [kg]\n"
"geometry_msgs/Vector3 dimensions  # bounding box [m]\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::Object_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::Object_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.name);
      stream.next(m.class_id);
      stream.next(m.class_name);
      stream.next(m.image_resources);
      stream.next(m.mesh_resource);
      stream.next(m.weight);
      stream.next(m.dimensions);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Object_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::Object_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::Object_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.name);
    s << indent << "class_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.class_id);
    s << indent << "class_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.class_name);
    s << indent << "image_resources[]" << std::endl;
    for (size_t i = 0; i < v.image_resources.size(); ++i)
    {
      s << indent << "  image_resources[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.image_resources[i]);
    }
    s << indent << "mesh_resource: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.mesh_resource);
    s << indent << "weight: ";
    Printer<float>::stream(s, indent + "  ", v.weight);
    s << indent << "dimensions: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.dimensions);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_OBJECT_H
