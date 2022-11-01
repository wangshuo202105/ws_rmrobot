// Generated by gencpp from file rm_msgs/Set_Force_Position.msg
// DO NOT EDIT!


#ifndef RM_MSGS_MESSAGE_SET_FORCE_POSITION_H
#define RM_MSGS_MESSAGE_SET_FORCE_POSITION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rm_msgs
{
template <class ContainerAllocator>
struct Set_Force_Position_
{
  typedef Set_Force_Position_<ContainerAllocator> Type;

  Set_Force_Position_()
    : sensor(0)
    , mode(0)
    , N(0)
    , load(0)  {
    }
  Set_Force_Position_(const ContainerAllocator& _alloc)
    : sensor(0)
    , mode(0)
    , N(0)
    , load(0)  {
  (void)_alloc;
    }



   typedef uint8_t _sensor_type;
  _sensor_type sensor;

   typedef uint8_t _mode_type;
  _mode_type mode;

   typedef int16_t _N_type;
  _N_type N;

   typedef uint8_t _load_type;
  _load_type load;





  typedef boost::shared_ptr< ::rm_msgs::Set_Force_Position_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rm_msgs::Set_Force_Position_<ContainerAllocator> const> ConstPtr;

}; // struct Set_Force_Position_

typedef ::rm_msgs::Set_Force_Position_<std::allocator<void> > Set_Force_Position;

typedef boost::shared_ptr< ::rm_msgs::Set_Force_Position > Set_Force_PositionPtr;
typedef boost::shared_ptr< ::rm_msgs::Set_Force_Position const> Set_Force_PositionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rm_msgs::Set_Force_Position_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rm_msgs::Set_Force_Position_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rm_msgs::Set_Force_Position_<ContainerAllocator1> & lhs, const ::rm_msgs::Set_Force_Position_<ContainerAllocator2> & rhs)
{
  return lhs.sensor == rhs.sensor &&
    lhs.mode == rhs.mode &&
    lhs.N == rhs.N &&
    lhs.load == rhs.load;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rm_msgs::Set_Force_Position_<ContainerAllocator1> & lhs, const ::rm_msgs::Set_Force_Position_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rm_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::rm_msgs::Set_Force_Position_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rm_msgs::Set_Force_Position_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rm_msgs::Set_Force_Position_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rm_msgs::Set_Force_Position_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rm_msgs::Set_Force_Position_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rm_msgs::Set_Force_Position_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rm_msgs::Set_Force_Position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "53a9f39d19bfb21fb905a51b86a97f5b";
  }

  static const char* value(const ::rm_msgs::Set_Force_Position_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x53a9f39d19bfb21fULL;
  static const uint64_t static_value2 = 0xb905a51b86a97f5bULL;
};

template<class ContainerAllocator>
struct DataType< ::rm_msgs::Set_Force_Position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rm_msgs/Set_Force_Position";
  }

  static const char* value(const ::rm_msgs::Set_Force_Position_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rm_msgs::Set_Force_Position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 sensor\n"
"uint8 mode\n"
"int16 N\n"
"uint8 load\n"
;
  }

  static const char* value(const ::rm_msgs::Set_Force_Position_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rm_msgs::Set_Force_Position_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sensor);
      stream.next(m.mode);
      stream.next(m.N);
      stream.next(m.load);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Set_Force_Position_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rm_msgs::Set_Force_Position_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rm_msgs::Set_Force_Position_<ContainerAllocator>& v)
  {
    s << indent << "sensor: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sensor);
    s << indent << "mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mode);
    s << indent << "N: ";
    Printer<int16_t>::stream(s, indent + "  ", v.N);
    s << indent << "load: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.load);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RM_MSGS_MESSAGE_SET_FORCE_POSITION_H