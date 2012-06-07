/* Auto-generated by genmsg_cpp for file /home/david/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/srv/RequestPlaneID.srv */
#ifndef AU_UAV_ROS_SERVICE_REQUESTPLANEID_H
#define AU_UAV_ROS_SERVICE_REQUESTPLANEID_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace AU_UAV_ROS
{
template <class ContainerAllocator>
struct RequestPlaneIDRequest_ {
  typedef RequestPlaneIDRequest_<ContainerAllocator> Type;

  RequestPlaneIDRequest_()
  : requestedID(0)
  {
  }

  RequestPlaneIDRequest_(const ContainerAllocator& _alloc)
  : requestedID(0)
  {
  }

  typedef int16_t _requestedID_type;
  int16_t requestedID;


  typedef boost::shared_ptr< ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RequestPlaneIDRequest
typedef  ::AU_UAV_ROS::RequestPlaneIDRequest_<std::allocator<void> > RequestPlaneIDRequest;

typedef boost::shared_ptr< ::AU_UAV_ROS::RequestPlaneIDRequest> RequestPlaneIDRequestPtr;
typedef boost::shared_ptr< ::AU_UAV_ROS::RequestPlaneIDRequest const> RequestPlaneIDRequestConstPtr;


template <class ContainerAllocator>
struct RequestPlaneIDResponse_ {
  typedef RequestPlaneIDResponse_<ContainerAllocator> Type;

  RequestPlaneIDResponse_()
  : planeID(0)
  {
  }

  RequestPlaneIDResponse_(const ContainerAllocator& _alloc)
  : planeID(0)
  {
  }

  typedef int16_t _planeID_type;
  int16_t planeID;


  typedef boost::shared_ptr< ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RequestPlaneIDResponse
typedef  ::AU_UAV_ROS::RequestPlaneIDResponse_<std::allocator<void> > RequestPlaneIDResponse;

typedef boost::shared_ptr< ::AU_UAV_ROS::RequestPlaneIDResponse> RequestPlaneIDResponsePtr;
typedef boost::shared_ptr< ::AU_UAV_ROS::RequestPlaneIDResponse const> RequestPlaneIDResponseConstPtr;

struct RequestPlaneID
{

typedef RequestPlaneIDRequest Request;
typedef RequestPlaneIDResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct RequestPlaneID
} // namespace AU_UAV_ROS

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5be8a994a877237ddffcaee9593ab72a";
  }

  static const char* value(const  ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5be8a994a877237dULL;
  static const uint64_t static_value2 = 0xdffcaee9593ab72aULL;
};

template<class ContainerAllocator>
struct DataType< ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "AU_UAV_ROS/RequestPlaneIDRequest";
  }

  static const char* value(const  ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int16 requestedID\n\
\n\
";
  }

  static const char* value(const  ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "416bfeb9353c15e349b0805ccac980c0";
  }

  static const char* value(const  ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x416bfeb9353c15e3ULL;
  static const uint64_t static_value2 = 0x49b0805ccac980c0ULL;
};

template<class ContainerAllocator>
struct DataType< ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "AU_UAV_ROS/RequestPlaneIDResponse";
  }

  static const char* value(const  ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int16 planeID\n\
\n\
\n\
";
  }

  static const char* value(const  ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.requestedID);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RequestPlaneIDRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.planeID);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RequestPlaneIDResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<AU_UAV_ROS::RequestPlaneID> {
  static const char* value() 
  {
    return "58e09e6bef3056cafdf42d800c5df65f";
  }

  static const char* value(const AU_UAV_ROS::RequestPlaneID&) { return value(); } 
};

template<>
struct DataType<AU_UAV_ROS::RequestPlaneID> {
  static const char* value() 
  {
    return "AU_UAV_ROS/RequestPlaneID";
  }

  static const char* value(const AU_UAV_ROS::RequestPlaneID&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "58e09e6bef3056cafdf42d800c5df65f";
  }

  static const char* value(const AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "AU_UAV_ROS/RequestPlaneID";
  }

  static const char* value(const AU_UAV_ROS::RequestPlaneIDRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "58e09e6bef3056cafdf42d800c5df65f";
  }

  static const char* value(const AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "AU_UAV_ROS/RequestPlaneID";
  }

  static const char* value(const AU_UAV_ROS::RequestPlaneIDResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // AU_UAV_ROS_SERVICE_REQUESTPLANEID_H

