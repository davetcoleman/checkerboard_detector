/*
 *  Checkerboard detector node for ROS
 *  Copyright (C) 2010
 *     David Feil-Seifer
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tf_conversions/tf_eigen.h>
//#include <Eigen/Core>
#include <Eigen/Geometry>

tf::TransformBroadcaster* tf_broadcaster_;
tf::TransformListener* tf_listener_;
std::string checkerboard_frame;

#define RTOD(a) ((a) * 180.0 / M_PI )

int nx = 4;
int ny = 6;

cv::Mat getObjPoints( int x, int y, double size )
{
  bool xodd = x % 2 == 1; bool yodd = x % 2 == 1;

  std::vector<cv::Point3f> corners;

  for( int i = 0; i < y; i++ )
  {
    float ycoord = size*(-y/2+i);
    if( !yodd ) ycoord += size/2.0;

    for( int j = 0; j < x; j++ )
    {
      float xcoord = size*(-x/2+j);
      if( !xodd ) xcoord += size/2.0;
      cv::Point3f p;
      p.x = xcoord;
      p.y = ycoord;
      p.z = 0;
      corners.push_back(p);
    }
  }
  cv::Mat ret(cv::Size(x*y,1), CV_32FC3, (void*)&corners[0]);
  return ret;
}


void imageCallback( const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& infomsg )
{
  ROS_DEBUG_STREAM_NAMED("imageCallback","recieved image");

  cv::Mat img;
  try
  {
    cv_bridge::CvImagePtr input_bridge = cv_bridge::toCvCopy(msg, "bgr8");
    img = input_bridge->image;

    // Copy the image
    cv::Mat oldimg = img;

    ROS_DEBUG_STREAM_NAMED("imageCallback","findChessboardCorners");

    std::vector<cv::Point2f> corners;
    if( cv::findChessboardCorners(img, cvSize(nx,ny), corners ) )
    {
      cv::Mat cornersmat( cv::Size(corners.size(), 1), CV_32FC2, (void*)&corners[0] );
      cv::drawChessboardCorners(img, cvSize(nx,ny), cornersmat, true );
      std::vector<cv::Point2f> imgpts;
      double fR3[3], fT3[3];
      cv::Mat rvec(3, 1, CV_64FC1, fR3);
      cv::Mat tvec(3, 1, CV_64FC1, fT3);

      image_geometry::PinholeCameraModel pcam;
      pcam.fromCameraInfo(infomsg);
      double square_size = 0.108; // this might not be what i think it is. was originally .108
      cv::Mat objpts = getObjPoints(nx,ny,square_size);

      cv::solvePnP( objpts, cornersmat, pcam.intrinsicMatrix(), pcam.distortionCoeffs(), rvec, tvec );

      tf::Transform checkerboard_tf;
      checkerboard_tf.setOrigin( tf::Vector3(fT3[0], fT3[1], fT3[2] ) );
      double rx = fR3[0] + M_PI, ry = fR3[1], rz = fR3[2];
      ROS_INFO( "tx: (%0.2f,%0.2f,%0.2f) rx: (%0.2f,%0.2f,%0.2f)", fT3[0],fT3[1], fT3[2], fR3[0],fR3[1],fR3[2]);
      tf::Quaternion quat;
      quat.setRPY(rx, ry, rz);
      checkerboard_tf.setRotation( quat );

      geometry_msgs::Transform pose_msg;
      tf::transformTFToMsg(checkerboard_tf, pose_msg);

      ROS_INFO_STREAM_NAMED("imageCallback","Publishing tf from " << msg->header.frame_id << " to " << checkerboard_frame << " pose:\n" << pose_msg);

      if (pose_msg.translation.x > 10 || pose_msg.translation.x > 10 || pose_msg.translation.x > 10)
      {
        ROS_ERROR_STREAM_NAMED("temp","ignoring transform because translation too large");
      }
      else
      {

        // Convert from camera_rgb_optical_frame to camera_link
        std::string camera_link = "/camera_link"; // TODO move this
        tf::StampedTransform camera_transform;
        try
        {
          ROS_INFO_STREAM_NAMED("temp","waiting for tf transform lookup");
          tf_listener_->lookupTransform(camera_link, msg->header.frame_id, ros::Time(0), camera_transform);
        }
        catch (tf::TransformException& ex)
        {
          ROS_WARN("TF exception:\n%s", ex.what());
          return;
        }

        tf::transformTFToMsg(camera_transform, pose_msg);
        ROS_INFO_STREAM_NAMED("temp","Camera transform from " << camera_link << " to " << msg->header.frame_id << " is: \n" << pose_msg);

        // Transform checkerboard_tf to camera_link
        Eigen::Affine3d camera_transform_eigen; 
        Eigen::Affine3d checkerboard_frame_eigen;
        tf::transformTFToEigen(camera_transform, camera_transform_eigen);
        tf::transformTFToEigen(checkerboard_tf, checkerboard_frame_eigen);

        checkerboard_frame_eigen = Eigen::Affine3d (Eigen::Translation3d (
                                     0.241228,
                                     0.362018,
                                     4.97308))*
          Eigen::Affine3d (Eigen::Quaterniond(
              0.988663,
              -0.0230351,
              -0.148242,
              -0.00630734
            ));

        tf::poseEigenToTF(checkerboard_frame_eigen * camera_transform_eigen.inverse(), checkerboard_tf);

        tf_broadcaster_->sendTransform(tf::StampedTransform(
            checkerboard_tf,
            msg->header.stamp,
            checkerboard_frame,     // parent frame
            camera_link //msg->header.frame_id  // child frame
          ));
      }
    }
    else
    {
      ROS_WARN_STREAM_NAMED("imageCallback","checkerboard not found");
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // bundle up image with checkerboard dots drawn for export
  cv::imshow("corners", img );
  cv::waitKey(10);
}

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "checkerboard" );
  ROS_INFO_STREAM_NAMED("main","Checkerboard detection starting");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  tf_broadcaster_ = new tf::TransformBroadcaster();
  tf_listener_ = new tf::TransformListener();

  cvNamedWindow( "corners", 1 );
  nh_priv.param("child_frame_id", checkerboard_frame, std::string("/checkerboard"));

  image_transport::ImageTransport it(nh);
  image_transport::CameraSubscriber image_sub = it.subscribeCamera("/camera/rgb/image_rect_color", 1, imageCallback );
  ros::spin();
  return 0;

}
