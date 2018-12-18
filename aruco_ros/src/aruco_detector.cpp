/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
/**
* @file simple_single.cpp
* @author Bence Magyar
* @date June 2012
* @version 0.1
* @brief ROS version of the example named "simple" in the Aruco software package.
*/

#include <iostream>
#include <aruco/aruco.h>
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/Corner.h>
#include <aruco/cvdrawingutils.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace aruco;

class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  MarkerDetector mDetector;
  vector<Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher transform_pub;
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;

  double yaw_tolerance;
  double roll_tolerance;
  double pitch_tolerance;
  double max_distance;
  double min_distance;
  double expected_yaw;  // Use to detect code mounted upside down or twisted
  double expected_roll;  // Use to detect steep angle of approach
  double expected_pitch;  // Use to detect steep angle of approach

  bool overlay_bounding_box;
  bool overlay_error_message;
  bool is_camera_rotated;

  cv::Point position;

  double marker_size;

  int marker_id[101]; // Hardcoded, max number of aruco code that can be detected is 101
  int num_markers_in_list; //

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf::TransformListener _tfListener;

public:
  ArucoSimple()
    : cam_info_received(false),
      nh("~"),
      it(nh),
      position(0,30)
  {
    // Subscriber to image and camera info
    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    pose_pub = nh.advertise<aruco_msgs::Marker>("pose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);

    // ROS Param from node. Set default of 10 compatibles Aruco codes
    nh.param<double>("marker_size", marker_size, 0.10);
    nh.param<int>("num_markers", num_markers_in_list, 101);
    nh.param<int>("marker_id_0", marker_id[0], 500);
    nh.param<int>("marker_id_1", marker_id[1], 614);
    nh.param<int>("marker_id_2", marker_id[2], 590);
    nh.param<int>("marker_id_3", marker_id[3], 750);
    nh.param<int>("marker_id_4", marker_id[4], 625);
    nh.param<int>("marker_id_5", marker_id[5], 582);
    nh.param<int>("marker_id_6", marker_id[6], 785);
    nh.param<int>("marker_id_7", marker_id[7], 798);
    nh.param<int>("marker_id_8", marker_id[8], 825);
    nh.param<int>("marker_id_9", marker_id[9], 921);
    nh.param<int>("marker_id_10", marker_id[10], 945);
    nh.param<int>("marker_id_11", marker_id[11], 225);
    nh.param<int>("marker_id_12", marker_id[12], 368);
    nh.param<int>("marker_id_13", marker_id[13], 494);
    nh.param<int>("marker_id_14", marker_id[14], 577);
    nh.param<int>("marker_id_15", marker_id[15], 681);
    nh.param<int>("marker_id_16", marker_id[16], 711);
    nh.param<int>("marker_id_17", marker_id[17], 846);
    nh.param<int>("marker_id_18", marker_id[18], 988);
    nh.param<int>("marker_id_19", marker_id[19], 147);
    nh.param<int>("marker_id_20", marker_id[20], 298);
    nh.param<int>("marker_id_21", marker_id[21], 331);
    nh.param<int>("marker_id_22", marker_id[22], 478);
    nh.param<int>("marker_id_23", marker_id[23], 533);
    nh.param<int>("marker_id_24", marker_id[24], 674);
    nh.param<int>("marker_id_25", marker_id[25], 739);
    nh.param<int>("marker_id_26", marker_id[26], 857);
    nh.param<int>("marker_id_27", marker_id[27], 935);
    nh.param<int>("marker_id_28", marker_id[28], 82);
    nh.param<int>("marker_id_29", marker_id[29], 285);
    nh.param<int>("marker_id_30", marker_id[30], 301);
    nh.param<int>("marker_id_31", marker_id[31], 433);
    nh.param<int>("marker_id_32", marker_id[32], 550);
    nh.param<int>("marker_id_33", marker_id[33], 652);
    nh.param<int>("marker_id_34", marker_id[34], 791);
    nh.param<int>("marker_id_35", marker_id[35], 883);
    nh.param<int>("marker_id_36", marker_id[36], 863);
    nh.param<int>("marker_id_37", marker_id[37], 177);
    nh.param<int>("marker_id_38", marker_id[38], 256);
    nh.param<int>("marker_id_39", marker_id[39], 310);
    nh.param<int>("marker_id_40", marker_id[40], 483);
    nh.param<int>("marker_id_41", marker_id[41], 510);
    nh.param<int>("marker_id_42", marker_id[42], 895);
    nh.param<int>("marker_id_43", marker_id[43], 770);
    nh.param<int>("marker_id_44", marker_id[44], 802);
    nh.param<int>("marker_id_45", marker_id[45], 976);
    nh.param<int>("marker_id_46", marker_id[46], 181);
    nh.param<int>("marker_id_47", marker_id[47], 102);
    nh.param<int>("marker_id_48", marker_id[48], 387);
    nh.param<int>("marker_id_49", marker_id[49], 451);
    nh.param<int>("marker_id_50", marker_id[50], 98);
    nh.param<int>("marker_id_51", marker_id[51], 640);
    nh.param<int>("marker_id_52", marker_id[52], 768);
    nh.param<int>("marker_id_53", marker_id[53], 41);
    nh.param<int>("marker_id_54", marker_id[54], 960);
    nh.param<int>("marker_id_55", marker_id[55], 132);
    nh.param<int>("marker_id_56", marker_id[56], 202);
    nh.param<int>("marker_id_57", marker_id[57], 349);
    nh.param<int>("marker_id_58", marker_id[58], 418);
    nh.param<int>("marker_id_59", marker_id[59], 522);
    nh.param<int>("marker_id_60", marker_id[60], 668);
    nh.param<int>("marker_id_61", marker_id[61], 722);
    nh.param<int>("marker_id_62", marker_id[62], 833);
    nh.param<int>("marker_id_63", marker_id[63], 954);
    nh.param<int>("marker_id_64", marker_id[64], 126);
    nh.param<int>("marker_id_65", marker_id[65], 260);
    nh.param<int>("marker_id_66", marker_id[66], 355);
    nh.param<int>("marker_id_67", marker_id[67], 112);
    nh.param<int>("marker_id_68", marker_id[68], 509);
    nh.param<int>("marker_id_69", marker_id[69], 631);
    nh.param<int>("marker_id_70", marker_id[70], 777);
    nh.param<int>("marker_id_71", marker_id[71], 812);
    nh.param<int>("marker_id_72", marker_id[72], 910);
    nh.param<int>("marker_id_73", marker_id[73], 163);
    nh.param<int>("marker_id_74", marker_id[74], 219);
    nh.param<int>("marker_id_75", marker_id[75], 372);
    nh.param<int>("marker_id_76", marker_id[76], 404);
    nh.param<int>("marker_id_77", marker_id[77], 569);
    nh.param<int>("marker_id_78", marker_id[78], 27);
    nh.param<int>("marker_id_79", marker_id[79], 702);
    nh.param<int>("marker_id_80", marker_id[80], 877);
    nh.param<int>("marker_id_81", marker_id[81], 901);
    nh.param<int>("marker_id_82", marker_id[82], 154);
    nh.param<int>("marker_id_83", marker_id[83], 277);
    nh.param<int>("marker_id_84", marker_id[84], 327);
    nh.param<int>("marker_id_85", marker_id[85], 427);
    nh.param<int>("marker_id_86", marker_id[86], 59);
    nh.param<int>("marker_id_87", marker_id[87], 601);
    nh.param<int>("marker_id_88", marker_id[88], 78);
    nh.param<int>("marker_id_89", marker_id[89], 247);
    nh.param<int>("marker_id_90", marker_id[90], 398);
    nh.param<int>("marker_id_91", marker_id[91], 448);
    nh.param<int>("marker_id_92", marker_id[92], 544);
    nh.param<int>("marker_id_93", marker_id[93], 698);
    nh.param<int>("marker_id_94", marker_id[94], 743);
    nh.param<int>("marker_id_95", marker_id[95], 826);
    nh.param<int>("marker_id_96", marker_id[96], 998);
    nh.param<int>("marker_id_97", marker_id[97], 199);
    nh.param<int>("marker_id_98", marker_id[98], 235);
    nh.param<int>("marker_id_99", marker_id[99], 39);
    nh.param<int>("marker_id_100", marker_id[100], 462);

    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);

    //Parameters for detecting erroneous conditions
    nh.param<double>("min_distance", min_distance, 0.5);
    nh.param<double>("max_distance", max_distance, 2.);
    nh.param<double>("yaw_tolerance", yaw_tolerance, M_PI/12.);
    nh.param<double>("roll_tolerance", roll_tolerance, M_PI/8.);
    nh.param<double>("pitch_tolerance", pitch_tolerance, M_PI/8.);
    nh.param<double>("expected_yaw", expected_yaw, -M_PI/2);
    nh.param<double>("expected_roll", expected_roll, -M_PI/2);
    nh.param<double>("expected_pitch", expected_pitch, 0.);

    nh.param<bool>("overlay_bounding_box", overlay_bounding_box, true);
    nh.param<bool>("overlay_error_message", overlay_error_message, true);

    nh.param<bool>("is_camera_rotated", is_camera_rotated, false);

    ROS_ASSERT(camera_frame != "" && marker_frame != "");
    //ROS_ASSERT(num_markers_in_list <= 11);

    if ( reference_frame.empty() )
      reference_frame = camera_frame;

    ROS_INFO("Aruco node started with marker size of %f m and %d markers to track", marker_size, num_markers_in_list);// marker id to track: %d %d %d %d %d %d %d %d %d %d %d",
//             marker_size, marker_id[0], marker_id[1], marker_id[2], marker_id[3], marker_id[4], marker_id[5],
//             marker_id[6], marker_id[7], marker_id[8], marker_id[9], marker_id[10]);
    ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
             reference_frame.c_str(), marker_frame.c_str());
  }

  bool getTransform(const std::string& refFrame,
                    const std::string& childFrame,
                    tf::StampedTransform& transform)
  {
    std::string errMsg;

    if ( !_tfListener.waitForTransform(refFrame,
                                       childFrame,
                                       ros::Time(0),
                                       ros::Duration(0.5),
                                       ros::Duration(0.01),
                                       &errMsg)
         )
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        _tfListener.lookupTransform( refFrame, childFrame,
                                     ros::Time(0),                  //get latest available
                                     transform);
      }
      catch ( const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }

    }
    return true;
  }


  bool is_marker_id_in_list(int id){
    for (unsigned i = 0; i < num_markers_in_list; ++i){
      if (marker_id[i] == id){
        return true;
      }
    }
    return false;
  }

  std::string get_name_from_id(int id){
    char str[100];

    for (unsigned i = 0; i < num_markers_in_list; ++i){
      if (marker_id[i] == id){
        sprintf(str, "Home %d", i);
        return std::string(str);
      }
    }
    return "Unknown code";
  }

  int get_home_from_id(int id){
    char str[100];

    for (unsigned i = 0; i < num_markers_in_list; ++i){
      if (marker_id[i] == id){
        return i;
      }
    }
    return 0;
  }

  /**
  * process_marker: Publish information about marker (pose and tf)
  * input Marker
  */
  void process_marker(Marker& marker, ros::Time& curr_stamp){
    tf::Transform transform = aruco_ros::arucoMarker2Tf(marker);
    aruco_msgs::Marker arucoMsg;
    double roll, pitch, yaw;
    unsigned int error_condition = 0;
    std::string error_message = "";
    static tf::TransformBroadcaster br;
    bool hasTransformToReference;

    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

    // Get TF between camera and reference frame
    tf::StampedTransform cameraToReference;
    cameraToReference.setIdentity();

    if (reference_frame == camera_frame) {
      hasTransformToReference = true;
    } else {
      hasTransformToReference = getTransform(reference_frame, camera_frame, cameraToReference);
    }

    // Test for erroneous conditions
    // Note that order is important for error message
    // All error codes will be forwarded, so this is important
    // only if error messages are overlayed on the camera image
    if (abs(remainder (roll - expected_roll, 2*M_PI)) > roll_tolerance){
      error_message = aruco_msgs::Marker::ANGLE_TOO_STEEP_MESSAGE;
      error_condition = aruco_msgs::Marker::ANGLE_TOO_STEEP;
    }
    else if (abs(remainder (pitch - expected_pitch, 2*M_PI)) > pitch_tolerance){
      error_message = aruco_msgs::Marker::CODE_NOT_FLAT_MESSAGE;
      error_condition = aruco_msgs::Marker::CODE_NOT_FLAT;
    }
    else if (abs(remainder(yaw - expected_yaw, 2*M_PI))  > yaw_tolerance){
      error_message = aruco_msgs::Marker::CODE_TWISTED_MESSAGE;
      error_condition = aruco_msgs::Marker::CODE_TWISTED;
    }
    else if (abs(remainder(yaw - expected_yaw, 2*M_PI))  > M_PI/2.){
      error_message = aruco_msgs::Marker::CODE_UPSIDE_DOWN_MESSAGE;
      error_condition = aruco_msgs::Marker::CODE_UPSIDE_DOWN;
    }
    else if (marker.getDistanceFromCamera() < min_distance){
      error_message = aruco_msgs::Marker::TOO_CLOSE_MESSAGE;
      error_condition = aruco_msgs::Marker::TOO_CLOSE;
    }
    else if (marker.getDistanceFromCamera() > max_distance){
      error_message = aruco_msgs::Marker::TOO_FAR_MESSAGE;
      error_condition = aruco_msgs::Marker::TOO_FAR;
    }
    else if (!hasTransformToReference) {
      error_message = aruco_msgs::Marker::NO_TRANSFORM_MESSAGE;
      error_condition = aruco_msgs::Marker::NO_TRANSFORM;
    }

    if (overlay_bounding_box) {
      // Only overlay error message on the image when at least one error condition has been met
      // In this condition, also draw a red rectangle around the code
      if (error_condition > 0){
        marker.draw(inImage,cv::Scalar(255, 0, 0), 2, false);
        if (overlay_error_message){
          cv::putText(inImage, error_message.c_str(), position, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,0,0,255), 2);
        }
      } else {
        // Otherwise, draw a green rectangle around the detected code (Green = success)
        marker.draw(inImage,cv::Scalar(0, 255, 0), 4, false, get_name_from_id(marker.id));
      }
    }

    // Get total TF between aruco code and reference frame, and broadcast it
    transform =
      static_cast<tf::Transform>(cameraToReference)
      * static_cast<tf::Transform>(rightToLeft)
      * transform;

    tf::StampedTransform stampedTransform(transform, curr_stamp,
                                          reference_frame, marker_frame);
    br.sendTransform(stampedTransform);
    geometry_msgs::TransformStamped transformMsg;
    tf::transformStampedTFToMsg(stampedTransform, transformMsg);
    transform_pub.publish(transformMsg);

    // Populate and publish Aruco Marker msg
    // We publish the home id, home code, all error codes (binary mask) and the main error message
    // We also publish the pose of the aruco code
    geometry_msgs::PoseWithCovariance poseMsg;
    tf::poseTFToMsg(transform, poseMsg.pose);

    arucoMsg.header.frame_id = reference_frame;
    arucoMsg.header.stamp = curr_stamp;
    arucoMsg.id = marker.id;
    arucoMsg.home_id = get_home_from_id(marker.id);
    arucoMsg.error_code = error_condition;
    arucoMsg.error_message = error_message;
    arucoMsg.pose = poseMsg;

    // Include the corner points of the marker, so we an reconstruct its position in the UI
    assert(marker.size() == 4);
    for (int i = 0; i < 4; ++i) {
      aruco_msgs::Corner corner;
      corner.x = marker[i].x;
      corner.y = marker[i].y;
      arucoMsg.corners.push_back(corner);
    }
    pose_pub.publish(arucoMsg);
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    ros::Time curr_stamp(ros::Time::now());
    if(cam_info_received)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        if (is_camera_rotated) {
          cv::rotate(inImage, inImage, cv::ROTATE_90_CLOCKWISE);
        }

        //detection results will go into "markers"
        markers.clear();
        //Ok, let's detect
        mDetector.detect(inImage, markers, camParam, marker_size, false);
        //for each marker, draw info and its boundaries in the image

        if (markers.size() == 1 && is_marker_id_in_list(markers[0].id)){
          // Only process markers if there is only one known marker in FOV
          // only publishing the selected markers
          process_marker(markers[0], curr_stamp);
        }else if (markers.size() > 1){
          // If multiple aruco code have been detected, return the error message

          for (int i=0; overlay_bounding_box && i<markers.size(); ++i) {
            markers[i].draw(inImage,cv::Scalar(255, 0, 0), 2, false);
          }

          aruco_msgs::Marker arucoMsg;
          // Include the corner points of all markers, so we an reconstruct their positions in the UI
          for (int i = 0; i < markers.size(); ++i) {
            for (int j = 0; j < 4; ++j) {
              aruco_msgs::Corner corner;
              corner.x = markers[i][j].x;
              corner.y = markers[i][j].y;
              arucoMsg.corners.push_back(corner);
            }
          }
          arucoMsg.header.frame_id = reference_frame;
          arucoMsg.header.stamp = curr_stamp;
          arucoMsg.error_code = aruco_msgs::Marker::MORE_THAN_ONE_CODE;
          arucoMsg.error_message = aruco_msgs::Marker::MORE_THAN_ONE_CODE_MESSAGE;
          pose_pub.publish(arucoMsg);

          if (overlay_error_message){
            cv::putText(inImage, aruco_msgs::Marker::MORE_THAN_ONE_CODE_MESSAGE, position, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,0,0,255), 2);
          }
        }

        if(image_pub.getNumSubscribers() > 0)
        {
          //show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if(debug_pub.getNumSubscribers() > 0)
        {
          //show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(
        tf::Vector3(
            -msg.P[3]/msg.P[0],
            -msg.P[7]/msg.P[5],
            0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();
  }
};


int main(int argc,char **argv)
{
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;

  ros::spin();
}
