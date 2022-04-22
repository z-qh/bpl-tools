// Generated by gencpp from file imu_gnss/gps_navi_msg.msg
// DO NOT EDIT!


#ifndef IMU_GNSS_MESSAGE_GPS_NAVI_MSG_H
#define IMU_GNSS_MESSAGE_GPS_NAVI_MSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace imu_gnss
{
template <class ContainerAllocator>
struct gps_navi_msg_
{
  typedef gps_navi_msg_<ContainerAllocator> Type;

  gps_navi_msg_()
    : header()
    , head()
    , InsWorkStatus()
    , PosStatus()
    , latitude(0.0)
    , longitude(0.0)
    , elevation(0.0)
    , qifu(0.0)
    , northspeed(0.0)
    , eastspeed(0.0)
    , skyspeed(0.0)
    , rollAngle(0.0)
    , pitchAngle(0.0)
    , yawAngle(0.0)
    , beiyong1(0.0)
    , beiyong2(0.0)  {
    }
  gps_navi_msg_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , head(_alloc)
    , InsWorkStatus(_alloc)
    , PosStatus(_alloc)
    , latitude(0.0)
    , longitude(0.0)
    , elevation(0.0)
    , qifu(0.0)
    , northspeed(0.0)
    , eastspeed(0.0)
    , skyspeed(0.0)
    , rollAngle(0.0)
    , pitchAngle(0.0)
    , yawAngle(0.0)
    , beiyong1(0.0)
    , beiyong2(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _head_type;
  _head_type head;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _InsWorkStatus_type;
  _InsWorkStatus_type InsWorkStatus;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _PosStatus_type;
  _PosStatus_type PosStatus;

   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef double _elevation_type;
  _elevation_type elevation;

   typedef double _qifu_type;
  _qifu_type qifu;

   typedef double _northspeed_type;
  _northspeed_type northspeed;

   typedef double _eastspeed_type;
  _eastspeed_type eastspeed;

   typedef double _skyspeed_type;
  _skyspeed_type skyspeed;

   typedef double _rollAngle_type;
  _rollAngle_type rollAngle;

   typedef double _pitchAngle_type;
  _pitchAngle_type pitchAngle;

   typedef double _yawAngle_type;
  _yawAngle_type yawAngle;

   typedef double _beiyong1_type;
  _beiyong1_type beiyong1;

   typedef double _beiyong2_type;
  _beiyong2_type beiyong2;





  typedef boost::shared_ptr< ::imu_gnss::gps_navi_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::imu_gnss::gps_navi_msg_<ContainerAllocator> const> ConstPtr;

}; // struct gps_navi_msg_

typedef ::imu_gnss::gps_navi_msg_<std::allocator<void> > gps_navi_msg;

typedef boost::shared_ptr< ::imu_gnss::gps_navi_msg > gps_navi_msgPtr;
typedef boost::shared_ptr< ::imu_gnss::gps_navi_msg const> gps_navi_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::imu_gnss::gps_navi_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::imu_gnss::gps_navi_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::imu_gnss::gps_navi_msg_<ContainerAllocator1> & lhs, const ::imu_gnss::gps_navi_msg_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.head == rhs.head &&
    lhs.InsWorkStatus == rhs.InsWorkStatus &&
    lhs.PosStatus == rhs.PosStatus &&
    lhs.latitude == rhs.latitude &&
    lhs.longitude == rhs.longitude &&
    lhs.elevation == rhs.elevation &&
    lhs.qifu == rhs.qifu &&
    lhs.northspeed == rhs.northspeed &&
    lhs.eastspeed == rhs.eastspeed &&
    lhs.skyspeed == rhs.skyspeed &&
    lhs.rollAngle == rhs.rollAngle &&
    lhs.pitchAngle == rhs.pitchAngle &&
    lhs.yawAngle == rhs.yawAngle &&
    lhs.beiyong1 == rhs.beiyong1 &&
    lhs.beiyong2 == rhs.beiyong2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::imu_gnss::gps_navi_msg_<ContainerAllocator1> & lhs, const ::imu_gnss::gps_navi_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace imu_gnss

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::imu_gnss::gps_navi_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::imu_gnss::gps_navi_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::imu_gnss::gps_navi_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::imu_gnss::gps_navi_msg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::imu_gnss::gps_navi_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::imu_gnss::gps_navi_msg_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::imu_gnss::gps_navi_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "43097861a26f8ace927b45fc1f7e2291";
  }

  static const char* value(const ::imu_gnss::gps_navi_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x43097861a26f8aceULL;
  static const uint64_t static_value2 = 0x927b45fc1f7e2291ULL;
};

template<class ContainerAllocator>
struct DataType< ::imu_gnss::gps_navi_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "imu_gnss/gps_navi_msg";
  }

  static const char* value(const ::imu_gnss::gps_navi_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::imu_gnss::gps_navi_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"string head\n"
"\n"
"string InsWorkStatus		\n"
"string PosStatus\n"
"		\n"
"float64 latitude			\n"
"float64 longitude		\n"
"float64 elevation		\n"
"float64 qifu\n"
"\n"
"float64 northspeed		\n"
"float64 eastspeed			\n"
"float64 skyspeed				\n"
"\n"
"float64 rollAngle				\n"
"float64 pitchAngle			\n"
"float64 yawAngle\n"
"\n"
"float64 beiyong1\n"
"float64 beiyong2\n"
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
;
  }

  static const char* value(const ::imu_gnss::gps_navi_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::imu_gnss::gps_navi_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.head);
      stream.next(m.InsWorkStatus);
      stream.next(m.PosStatus);
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.elevation);
      stream.next(m.qifu);
      stream.next(m.northspeed);
      stream.next(m.eastspeed);
      stream.next(m.skyspeed);
      stream.next(m.rollAngle);
      stream.next(m.pitchAngle);
      stream.next(m.yawAngle);
      stream.next(m.beiyong1);
      stream.next(m.beiyong2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct gps_navi_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::imu_gnss::gps_navi_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::imu_gnss::gps_navi_msg_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "head: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.head);
    s << indent << "InsWorkStatus: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.InsWorkStatus);
    s << indent << "PosStatus: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.PosStatus);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "elevation: ";
    Printer<double>::stream(s, indent + "  ", v.elevation);
    s << indent << "qifu: ";
    Printer<double>::stream(s, indent + "  ", v.qifu);
    s << indent << "northspeed: ";
    Printer<double>::stream(s, indent + "  ", v.northspeed);
    s << indent << "eastspeed: ";
    Printer<double>::stream(s, indent + "  ", v.eastspeed);
    s << indent << "skyspeed: ";
    Printer<double>::stream(s, indent + "  ", v.skyspeed);
    s << indent << "rollAngle: ";
    Printer<double>::stream(s, indent + "  ", v.rollAngle);
    s << indent << "pitchAngle: ";
    Printer<double>::stream(s, indent + "  ", v.pitchAngle);
    s << indent << "yawAngle: ";
    Printer<double>::stream(s, indent + "  ", v.yawAngle);
    s << indent << "beiyong1: ";
    Printer<double>::stream(s, indent + "  ", v.beiyong1);
    s << indent << "beiyong2: ";
    Printer<double>::stream(s, indent + "  ", v.beiyong2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IMU_GNSS_MESSAGE_GPS_NAVI_MSG_H
