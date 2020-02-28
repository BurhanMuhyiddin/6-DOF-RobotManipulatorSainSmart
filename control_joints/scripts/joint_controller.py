import rospy
from sensor_msgs.msg import JointState

class JointController :
    def __init__(self):
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(1)

    def publish_joint_states(self):
        while not rospy.is_shutdown():
            new_msg = JointState()
            new_msg.name = ['base_waist', 'waist_forearm', 'forearm_forearmRotation', 
                                        'forearmRotation_wrist', 'wrist_wristRotation']
            new_msg.position = [0.0, 0.025, 0.35, -0.03, 0.0245]
            new_msg.velocity = []
            new_msg.effort = []
            rospy.loginfo("New message sent...")
            self.pub.publish(new_msg)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joint_controller')
    jc1 = JointController()
    jc1.publish_joint_states()




    ###
    // Generated by gencpp from file mrm_description/Floats_array.msg
// DO NOT EDIT!


#ifndef MRM_DESCRIPTION_MESSAGE_FLOATS_ARRAY_H
#define MRM_DESCRIPTION_MESSAGE_FLOATS_ARRAY_H

#include <ros/service_traits.h>


#include <mrm_description/Floats_arrayRequest.h>
#include <mrm_description/Floats_arrayResponse.h>


namespace mrm_description
{

struct Floats_array
{

typedef Floats_arrayRequest Request;
typedef Floats_arrayResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Floats_array
} // namespace mrm_description


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::mrm_description::Floats_array > {
  static const char* value()
  {
    return "086e0a6631d243c06d9ccfa024bfe376";
  }

  static const char* value(const ::mrm_description::Floats_array&) { return value(); }
};

template<>
struct DataType< ::mrm_description::Floats_array > {
  static const char* value()
  {
    return "mrm_description/Floats_array";
  }

  static const char* value(const ::mrm_description::Floats_array&) { return value(); }
};


// service_traits::MD5Sum< ::mrm_description::Floats_arrayRequest> should match 
// service_traits::MD5Sum< ::mrm_description::Floats_array > 
template<>
struct MD5Sum< ::mrm_description::Floats_arrayRequest>
{
  static const char* value()
  {
    return MD5Sum< ::mrm_description::Floats_array >::value();
  }
  static const char* value(const ::mrm_description::Floats_arrayRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::mrm_description::Floats_arrayRequest> should match 
// service_traits::DataType< ::mrm_description::Floats_array > 
template<>
struct DataType< ::mrm_description::Floats_arrayRequest>
{
  static const char* value()
  {
    return DataType< ::mrm_description::Floats_array >::value();
  }
  static const char* value(const ::mrm_description::Floats_arrayRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::mrm_description::Floats_arrayResponse> should match 
// service_traits::MD5Sum< ::mrm_description::Floats_array > 
template<>
struct MD5Sum< ::mrm_description::Floats_arrayResponse>
{
  static const char* value()
  {
    return MD5Sum< ::mrm_description::Floats_array >::value();
  }
  static const char* value(const ::mrm_description::Floats_arrayResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::mrm_description::Floats_arrayResponse> should match 
// service_traits::DataType< ::mrm_description::Floats_array > 
template<>
struct DataType< ::mrm_description::Floats_arrayResponse>
{
  static const char* value()
  {
    return DataType< ::mrm_description::Floats_array >::value();
  }
  static const char* value(const ::mrm_description::Floats_arrayResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MRM_DESCRIPTION_MESSAGE_FLOATS_ARRAY_H
