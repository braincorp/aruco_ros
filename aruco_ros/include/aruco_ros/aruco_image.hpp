#ifndef ARUCO_IMAGE_H
#define ARUCO_IMAGE_H

#include <aruco/aruco.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>

//------------------------------------------------------------------------------
namespace aruco_ros {
//------------------------------------------------------------------------------

struct ArucoImage : public sensor_msgs::Image {
  friend class CvArucoImage;

  ArucoImage()
  : marker_corners(4, std::vector<float>(2))
  , marker_id(-1)
  , error_code(0) {}

  static const std::string __s_getDataType() { return "sensor_msgs/Image"; }
  static const std::string __s_getMD5Sum() { return "060021388200f6f0f447d0fcd9c64743"; }

  virtual uint8_t* deserialize(uint8_t *read_ptr);
  aruco::Marker getMarker() const;

  int error_code;
  int marker_id;

private:
  void setMarker(const aruco::Marker marker);

  std::vector<std::vector<float> > marker_corners;
};
typedef boost::shared_ptr<ArucoImage> ArucoImagePtr;
typedef boost::shared_ptr<ArucoImage const> ArucoImageConstPtr;

struct CvArucoImage : public cv_bridge::CvImage {

  ArucoImagePtr toArucoImageMsg() const;
  void toArucoImageMsg(ArucoImage& aruco_ros_image) const;

  int error_code;
  aruco::Marker marker;
};
typedef boost::shared_ptr<CvArucoImage> CvArucoImagePtr;
typedef boost::shared_ptr<CvArucoImage const> CvArucoImageConstPtr;

//------------------------------------------------------------------------------
}
//------------------------------------------------------------------------------

#endif // ARUCO_IMAGE_H