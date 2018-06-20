#ifndef ARUCO_IMAGE_H
#define ARUCO_IMAGE_H

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>

//------------------------------------------------------------------------------
namespace aruco_ros {
//------------------------------------------------------------------------------

struct ArucoImage : public sensor_msgs::Image {

  ArucoImage()
  : error_code(0) {}

  // For whatever reasons that someone smarter than me could probably understand, these are public static
  // methods that DO exist in the base class (sensor_msgs::Image) but can't be seen by its derived classes
  // (maybe something to do with the ROS_DEPRECATED macro). So for the time being, I just hacked them into
  // the derived class, copied verbatim.
  static const std::string __s_getDataType() { return "sensor_msgs/Image"; }
  static const std::string __s_getMD5Sum() { return "060021388200f6f0f447d0fcd9c64743"; }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, height);
    ros::serialization::deserialize(stream, width);
    ros::serialization::deserialize(stream, encoding);
    ros::serialization::deserialize(stream, is_bigendian);
    ros::serialization::deserialize(stream, step);
    ros::serialization::deserialize(stream, error_code);  // Added since does not exist in base
    ros::serialization::deserialize(stream, data);
    return stream.getData();
  }

  int error_code;
};

typedef boost::shared_ptr<ArucoImage> ArucoImagePtr;
typedef boost::shared_ptr<ArucoImage const> ArucoImageConstPtr;

struct CvArucoImage : public cv_bridge::CvImage {

  ArucoImagePtr toArucoImageMsg() const {
    ArucoImagePtr ptr = boost::make_shared<ArucoImage>();
    toArucoImageMsg(*ptr);
    return ptr;
  }

  void toArucoImageMsg(ArucoImage& aruco_ros_image) const {
    toImageMsg(aruco_ros_image);
    aruco_ros_image.error_code = error_code;
  }

  int error_code;
};

typedef boost::shared_ptr<CvArucoImage> CvArucoImagePtr;
typedef boost::shared_ptr<CvArucoImage const> CvArucoImageConstPtr;

//------------------------------------------------------------------------------
}
//------------------------------------------------------------------------------

#endif // ARUCO_IMAGE_H