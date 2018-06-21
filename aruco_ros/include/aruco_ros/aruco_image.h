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
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, height);
    ros::serialization::deserialize(stream, width);
    ros::serialization::deserialize(stream, encoding);
    ros::serialization::deserialize(stream, is_bigendian);
    ros::serialization::deserialize(stream, step);
    ros::serialization::deserialize(stream, marker_corners);
    ros::serialization::deserialize(stream, marker_id);
    ros::serialization::deserialize(stream, error_code);
    ros::serialization::deserialize(stream, data);
    return stream.getData();
  }

  aruco::Marker getMarker() const {
    std::vector<cv::Point2f> point2fVec(marker_corners.size());
    for (int i = 0; i < marker_corners.size(); ++i) {
      std::vector<float> marker_corner = marker_corners[i];
      point2fVec[i] = cv::Point2f(marker_corner[0], marker_corner[1]);
    }
    return aruco::Marker(point2fVec, marker_id);
  }

  int error_code;
  int marker_id;

private:
  std::vector<std::vector<float> > marker_corners;

  void setMarker(const aruco::Marker marker) {
    marker_id = marker.id;
    for (int i = 0; i < marker_corners.size(); ++i) {
      marker_corners[i][0] = marker[i].x;
      marker_corners[i][1] = marker[i].y;
    }
  }
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
    aruco_ros_image.setMarker(marker);
  }

  int error_code;
  aruco::Marker marker;
};
typedef boost::shared_ptr<CvArucoImage> CvArucoImagePtr;
typedef boost::shared_ptr<CvArucoImage const> CvArucoImageConstPtr;

//------------------------------------------------------------------------------
}
//------------------------------------------------------------------------------

#endif // ARUCO_IMAGE_H