// Generated by gencpp from file franka_aic/JointTorqueComparison.msg
// DO NOT EDIT!


#ifndef FRANKA_AIC_MESSAGE_JOINTTORQUECOMPARISON_H
#define FRANKA_AIC_MESSAGE_JOINTTORQUECOMPARISON_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace franka_aic
{
template <class ContainerAllocator>
struct JointTorqueComparison_
{
  typedef JointTorqueComparison_<ContainerAllocator> Type;

  JointTorqueComparison_()
    : tau_error()
    , tau_commanded()
    , tau_measured()
    , root_mean_square_error(0.0)  {
      tau_error.assign(0.0);

      tau_commanded.assign(0.0);

      tau_measured.assign(0.0);
  }
  JointTorqueComparison_(const ContainerAllocator& _alloc)
    : tau_error()
    , tau_commanded()
    , tau_measured()
    , root_mean_square_error(0.0)  {
  (void)_alloc;
      tau_error.assign(0.0);

      tau_commanded.assign(0.0);

      tau_measured.assign(0.0);
  }



   typedef boost::array<double, 7>  _tau_error_type;
  _tau_error_type tau_error;

   typedef boost::array<double, 7>  _tau_commanded_type;
  _tau_commanded_type tau_commanded;

   typedef boost::array<double, 7>  _tau_measured_type;
  _tau_measured_type tau_measured;

   typedef double _root_mean_square_error_type;
  _root_mean_square_error_type root_mean_square_error;





  typedef boost::shared_ptr< ::franka_aic::JointTorqueComparison_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::franka_aic::JointTorqueComparison_<ContainerAllocator> const> ConstPtr;

}; // struct JointTorqueComparison_

typedef ::franka_aic::JointTorqueComparison_<std::allocator<void> > JointTorqueComparison;

typedef boost::shared_ptr< ::franka_aic::JointTorqueComparison > JointTorqueComparisonPtr;
typedef boost::shared_ptr< ::franka_aic::JointTorqueComparison const> JointTorqueComparisonConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::franka_aic::JointTorqueComparison_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::franka_aic::JointTorqueComparison_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::franka_aic::JointTorqueComparison_<ContainerAllocator1> & lhs, const ::franka_aic::JointTorqueComparison_<ContainerAllocator2> & rhs)
{
  return lhs.tau_error == rhs.tau_error &&
    lhs.tau_commanded == rhs.tau_commanded &&
    lhs.tau_measured == rhs.tau_measured &&
    lhs.root_mean_square_error == rhs.root_mean_square_error;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::franka_aic::JointTorqueComparison_<ContainerAllocator1> & lhs, const ::franka_aic::JointTorqueComparison_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace franka_aic

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::franka_aic::JointTorqueComparison_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_aic::JointTorqueComparison_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::franka_aic::JointTorqueComparison_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::franka_aic::JointTorqueComparison_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_aic::JointTorqueComparison_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_aic::JointTorqueComparison_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::franka_aic::JointTorqueComparison_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6c09db90263c92a2e4e4d736f67bc033";
  }

  static const char* value(const ::franka_aic::JointTorqueComparison_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6c09db90263c92a2ULL;
  static const uint64_t static_value2 = 0xe4e4d736f67bc033ULL;
};

template<class ContainerAllocator>
struct DataType< ::franka_aic::JointTorqueComparison_<ContainerAllocator> >
{
  static const char* value()
  {
    return "franka_aic/JointTorqueComparison";
  }

  static const char* value(const ::franka_aic::JointTorqueComparison_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::franka_aic::JointTorqueComparison_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[7] tau_error\n"
"float64[7] tau_commanded\n"
"float64[7] tau_measured\n"
"float64 root_mean_square_error\n"
;
  }

  static const char* value(const ::franka_aic::JointTorqueComparison_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::franka_aic::JointTorqueComparison_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.tau_error);
      stream.next(m.tau_commanded);
      stream.next(m.tau_measured);
      stream.next(m.root_mean_square_error);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointTorqueComparison_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::franka_aic::JointTorqueComparison_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::franka_aic::JointTorqueComparison_<ContainerAllocator>& v)
  {
    s << indent << "tau_error[]" << std::endl;
    for (size_t i = 0; i < v.tau_error.size(); ++i)
    {
      s << indent << "  tau_error[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.tau_error[i]);
    }
    s << indent << "tau_commanded[]" << std::endl;
    for (size_t i = 0; i < v.tau_commanded.size(); ++i)
    {
      s << indent << "  tau_commanded[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.tau_commanded[i]);
    }
    s << indent << "tau_measured[]" << std::endl;
    for (size_t i = 0; i < v.tau_measured.size(); ++i)
    {
      s << indent << "  tau_measured[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.tau_measured[i]);
    }
    s << indent << "root_mean_square_error: ";
    Printer<double>::stream(s, indent + "  ", v.root_mean_square_error);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FRANKA_AIC_MESSAGE_JOINTTORQUECOMPARISON_H
