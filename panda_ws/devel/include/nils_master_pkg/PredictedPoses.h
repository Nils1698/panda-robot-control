// Generated by gencpp from file nils_master_pkg/PredictedPoses.msg
// DO NOT EDIT!


#ifndef NILS_MASTER_PKG_MESSAGE_PREDICTEDPOSES_H
#define NILS_MASTER_PKG_MESSAGE_PREDICTEDPOSES_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <std_msgs/MultiArrayLayout.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace nils_master_pkg
{
template <class ContainerAllocator>
struct PredictedPoses_
{
  typedef PredictedPoses_<ContainerAllocator> Type;

  PredictedPoses_()
    : header()
    , layout()
    , poses()  {
    }
  PredictedPoses_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , layout(_alloc)
    , poses(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::std_msgs::MultiArrayLayout_<ContainerAllocator>  _layout_type;
  _layout_type layout;

   typedef std::vector< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> >> _poses_type;
  _poses_type poses;





  typedef boost::shared_ptr< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> const> ConstPtr;

}; // struct PredictedPoses_

typedef ::nils_master_pkg::PredictedPoses_<std::allocator<void> > PredictedPoses;

typedef boost::shared_ptr< ::nils_master_pkg::PredictedPoses > PredictedPosesPtr;
typedef boost::shared_ptr< ::nils_master_pkg::PredictedPoses const> PredictedPosesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nils_master_pkg::PredictedPoses_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::nils_master_pkg::PredictedPoses_<ContainerAllocator1> & lhs, const ::nils_master_pkg::PredictedPoses_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.layout == rhs.layout &&
    lhs.poses == rhs.poses;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::nils_master_pkg::PredictedPoses_<ContainerAllocator1> & lhs, const ::nils_master_pkg::PredictedPoses_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace nils_master_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> >
{
  static const char* value()
  {
    return "235f45d8599c5db0d0ef8cb792832391";
  }

  static const char* value(const ::nils_master_pkg::PredictedPoses_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x235f45d8599c5db0ULL;
  static const uint64_t static_value2 = 0xd0ef8cb792832391ULL;
};

template<class ContainerAllocator>
struct DataType< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nils_master_pkg/PredictedPoses";
  }

  static const char* value(const ::nils_master_pkg::PredictedPoses_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"std_msgs/MultiArrayLayout layout\n"
"geometry_msgs/PoseWithCovariance[] poses\n"
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
"MSG: std_msgs/MultiArrayLayout\n"
"# The multiarray declares a generic multi-dimensional array of a\n"
"# particular data type.  Dimensions are ordered from outer most\n"
"# to inner most.\n"
"\n"
"MultiArrayDimension[] dim # Array of dimension properties\n"
"uint32 data_offset        # padding elements at front of data\n"
"\n"
"# Accessors should ALWAYS be written in terms of dimension stride\n"
"# and specified outer-most dimension first.\n"
"# \n"
"# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n"
"#\n"
"# A standard, 3-channel 640x480 image with interleaved color channels\n"
"# would be specified as:\n"
"#\n"
"# dim[0].label  = \"height\"\n"
"# dim[0].size   = 480\n"
"# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n"
"# dim[1].label  = \"width\"\n"
"# dim[1].size   = 640\n"
"# dim[1].stride = 3*640 = 1920\n"
"# dim[2].label  = \"channel\"\n"
"# dim[2].size   = 3\n"
"# dim[2].stride = 3\n"
"#\n"
"# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/MultiArrayDimension\n"
"string label   # label of given dimension\n"
"uint32 size    # size of given dimension (in type units)\n"
"uint32 stride  # stride of given dimension\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseWithCovariance\n"
"# This represents a pose in free space with uncertainty.\n"
"\n"
"Pose pose\n"
"\n"
"# Row-major representation of the 6x6 covariance matrix\n"
"# The orientation parameters use a fixed-axis representation.\n"
"# In order, the parameters are:\n"
"# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n"
"float64[36] covariance\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::nils_master_pkg::PredictedPoses_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.layout);
      stream.next(m.poses);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PredictedPoses_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nils_master_pkg::PredictedPoses_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nils_master_pkg::PredictedPoses_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "layout: ";
    s << std::endl;
    Printer< ::std_msgs::MultiArrayLayout_<ContainerAllocator> >::stream(s, indent + "  ", v.layout);
    s << indent << "poses[]" << std::endl;
    for (size_t i = 0; i < v.poses.size(); ++i)
    {
      s << indent << "  poses[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> >::stream(s, indent + "    ", v.poses[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // NILS_MASTER_PKG_MESSAGE_PREDICTEDPOSES_H
