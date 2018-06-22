#include <aruco_ros/aruco_image.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

//------------------------------------------------------------------------------
namespace aruco_ros {
//------------------------------------------------------------------------------

uint8_t*
ArucoImage::deserialize(uint8_t *read_ptr) {
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

aruco::Marker
ArucoImage::getMarker() const {
  std::vector<cv::Point2f> point2fVec(marker_corners.size());
  for (int i = 0; i < marker_corners.size(); ++i) {
    std::vector<float> marker_corner = marker_corners[i];
    point2fVec[i] = cv::Point2f(marker_corner[0], marker_corner[1]);
  }
  return aruco::Marker(point2fVec, marker_id);
}

void
ArucoImage::setMarker(const aruco::Marker marker) {
  marker_id = marker.id;
  for (int i = 0; i < marker_corners.size(); ++i) {
    marker_corners[i][0] = 1; //marker[i].x;
    marker_corners[i][1] = 1; //marker[i].y;
  }
}

ArucoImagePtr
CvArucoImage::toArucoImageMsg() const {
  ArucoImagePtr ptr = boost::make_shared<ArucoImage>();
  toArucoImageMsg(*ptr);
  return ptr;
}

void
CvArucoImage::toArucoImageMsg(ArucoImage& aruco_ros_image) const {
  toImageMsg(aruco_ros_image);
  aruco_ros_image.error_code = error_code;
  aruco_ros_image.setMarker(marker);
}

//------------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
