// Generated by gencpp from file jsk_recognition_msgs/SparseOccupancyGridColumn.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_SPARSEOCCUPANCYGRIDCOLUMN_H
#define JSK_RECOGNITION_MSGS_MESSAGE_SPARSEOCCUPANCYGRIDCOLUMN_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <jsk_recognition_msgs/SparseOccupancyGridCell.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct SparseOccupancyGridColumn_
{
  typedef SparseOccupancyGridColumn_<ContainerAllocator> Type;

  SparseOccupancyGridColumn_()
    : column_index(0)
    , cells()  {
    }
  SparseOccupancyGridColumn_(const ContainerAllocator& _alloc)
    : column_index(0)
    , cells(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _column_index_type;
  _column_index_type column_index;

   typedef std::vector< ::jsk_recognition_msgs::SparseOccupancyGridCell_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::jsk_recognition_msgs::SparseOccupancyGridCell_<ContainerAllocator> >> _cells_type;
  _cells_type cells;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> const> ConstPtr;

}; // struct SparseOccupancyGridColumn_

typedef ::jsk_recognition_msgs::SparseOccupancyGridColumn_<std::allocator<void> > SparseOccupancyGridColumn;

typedef boost::shared_ptr< ::jsk_recognition_msgs::SparseOccupancyGridColumn > SparseOccupancyGridColumnPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::SparseOccupancyGridColumn const> SparseOccupancyGridColumnConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator2> & rhs)
{
  return lhs.column_index == rhs.column_index &&
    lhs.cells == rhs.cells;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator1> & lhs, const ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "55074b193e722d5ead092ffe27f06522";
  }

  static const char* value(const ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x55074b193e722d5eULL;
  static const uint64_t static_value2 = 0xad092ffe27f06522ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/SparseOccupancyGridColumn";
  }

  static const char* value(const ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 column_index\n"
"SparseOccupancyGridCell[] cells\n"
"\n"
"================================================================================\n"
"MSG: jsk_recognition_msgs/SparseOccupancyGridCell\n"
"int32 row_index\n"
"float32 value\n"
;
  }

  static const char* value(const ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.column_index);
      stream.next(m.cells);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SparseOccupancyGridColumn_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::SparseOccupancyGridColumn_<ContainerAllocator>& v)
  {
    s << indent << "column_index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.column_index);
    s << indent << "cells[]" << std::endl;
    for (size_t i = 0; i < v.cells.size(); ++i)
    {
      s << indent << "  cells[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::jsk_recognition_msgs::SparseOccupancyGridCell_<ContainerAllocator> >::stream(s, indent + "    ", v.cells[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_SPARSEOCCUPANCYGRIDCOLUMN_H
