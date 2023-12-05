///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include "stereo_sync.hpp"

#ifndef NDEBUG
#include <ros/console.h>
#endif

#include <chrono>

namespace zed_nodelets
{
StereoSyncNodelet::StereoSyncNodelet()
{
}

StereoSyncNodelet::~StereoSyncNodelet()
{
  if (mApproxStereoSync)
    delete mApproxStereoSync;

  if (mExactStereoSync)
    delete mExactStereoSync;
}

void StereoSyncNodelet::onInit()
{
  // Node handlers
  mNh = getNodeHandle();
  mNhP = getPrivateNodeHandle();

#ifndef NDEBUG
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif

  NODELET_INFO("********** Starting nodelet '%s' **********", getName().c_str());

  readParameters();

  //mPubRaw = mNhP.advertise<zed_interfaces::RGBDSensors>("rgbd_sens", 1);
  image_transport::ImageTransport it(mNhP);
  mleftImagePub = it.advertiseCamera("left/image_rect", 1);
  mrightImagePub = it.advertiseCamera("right/image_rect", 1);
  NODELET_INFO_STREAM("Advertised on topic " << mleftImagePub.getTopic());
  NODELET_INFO_STREAM("Advertised on topic " << mrightImagePub.getTopic());

  if (mUseApproxSync)
  {
    NODELET_DEBUG("Using Approximate Time sync");

      mApproxStereoSync = new message_filters::Synchronizer<ApproxStereoSyncPolicy>(
          ApproxStereoSyncPolicy(mQueueSize), mSubLeftGrayImage, mSubRightGrayImage, mSubLeftGrayCamInfo, mSubRightGrayCamInfo);
      mApproxStereoSync->registerCallback(
          boost::bind(&StereoSyncNodelet::callbackStereo, this, _1, _2, _3, _4));

      NODELET_DEBUG("Left gray + Right gray Sync");
  }
  else
  {
    NODELET_DEBUG("Using Exact Time sync");

      mExactStereoSync = new message_filters::Synchronizer<ExactStereoSyncPolicy>(
          ExactStereoSyncPolicy(mQueueSize), mSubLeftGrayImage, mSubRightGrayImage, mSubLeftGrayCamInfo, mSubRightGrayCamInfo);
      mExactStereoSync->registerCallback(
          boost::bind(&StereoSyncNodelet::callbackStereo, this, _1, _2, _3, _4));

      NODELET_DEBUG("Left gray + Right gray Sync");
  }
    
  // Create remappings
  ros::NodeHandle left_hn(mNh, mZedNodeletName + "/left");
  ros::NodeHandle right_nh(mNh, mZedNodeletName + "/right");
  ros::NodeHandle left_pnh(mNhP, mZedNodeletName + "/left");
  ros::NodeHandle right_pnh(mNhP, mZedNodeletName + "/right");

  image_transport::ImageTransport left_it(left_hn);
  image_transport::ImageTransport right_it(right_nh);

  image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), left_pnh);
  image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), right_pnh);

  mSubLeftGrayImage.subscribe(left_it, left_hn.resolveName("image_rect_gray"), 1, hintsRgb);
  mSubRightGrayImage.subscribe(right_it, right_nh.resolveName("image_rect_gray"), 1, hintsDepth);
  mSubLeftGrayCamInfo.subscribe(left_hn, "camera_info", 1);
  mSubRightGrayCamInfo.subscribe(right_nh, "camera_info", 1);

  NODELET_INFO_STREAM(" * Subscribed to topic: " << mSubLeftGrayImage.getTopic().c_str());
  NODELET_INFO_STREAM(" * Subscribed to topic: " << mSubLeftGrayCamInfo.getTopic().c_str());
  NODELET_INFO_STREAM(" * Subscribed to topic: " << mSubRightGrayImage.getTopic().c_str());
  NODELET_INFO_STREAM(" * Subscribed to topic: " << mSubRightGrayCamInfo.getTopic().c_str());

}

void StereoSyncNodelet::readParameters()
{
  NODELET_INFO("*** PARAMETERS [%s]***", getName().c_str());

  mNhP.getParam("zed_nodelet_name", mZedNodeletName);
  mNhP.getParam("approx_sync", mUseApproxSync);
  mNhP.getParam("queue_size", mQueueSize);
  mNhP.getParam("sub_imu", mUseImu);
  mNhP.getParam("sub_mag", mUseMag);

  NODELET_INFO(" * zed_nodelet_name -> %s", mZedNodeletName.c_str());
  NODELET_INFO(" * approx_sync -> %s", mUseApproxSync ? "true" : "false");
  NODELET_INFO(" * queue_size  -> %d", mQueueSize);
  NODELET_INFO(" * sub_imu -> %s", mUseImu ? "true" : "false");
  NODELET_INFO(" * sub_mag -> %s", mUseMag ? "true" : "false");
}


void StereoSyncNodelet::callbackStereo(const sensor_msgs::ImageConstPtr &left,
                                          const sensor_msgs::ImageConstPtr &right,
                                          const sensor_msgs::CameraInfoConstPtr &leftCameraInfo,
                                          const sensor_msgs::CameraInfoConstPtr &rightCameraInfo)
{
  // ----> Frequency calculation
  static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

  double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
  last_time = now;

  double freq = 1e6 / elapsed_usec;
  NODELET_DEBUG("Freq: %.2f", freq);
  // <---- Frequency calculation

  double leftStamp = left->header.stamp.toSec();
  double rightStamp = right->header.stamp.toSec();
  double leftInfoStamp = leftCameraInfo->header.stamp.toSec();
  double rightInfoStamp = rightCameraInfo->header.stamp.toSec();

  double diff = leftStamp - rightStamp;

  NODELET_DEBUG("Callback Stereo - Left TS: %.9f - Right TS: %.9f - Diff:  %.9f", leftStamp, rightStamp, diff);

  uint32_t subraw = mleftImagePub.getNumSubscribers() + mrightImagePub.getNumSubscribers();

  if (subraw == 0)
  {
    return;
  }

  //zed_interfaces::RGBDSensorsPtr outSyncMsg = boost::make_shared<zed_interfaces::RGBDSensors>();

  //outSyncMsg->header.frame_id = rgb->header.frame_id;
  //outSyncMsg->header.stamp = rgb->header.stamp;
  //outSyncMsg->rgbCameraInfo = *rgbCameraInfo;
  //outSyncMsg->depthCameraInfo = *depthCameraInfo;

  //outSyncMsg->rgb = *rgb;
  //outSyncMsg->depth = *depth;
  //outSyncMsg->imu = *imu;
  //outSyncMsg->mag = *mag;

  //mPubRaw.publish(outSyncMsg);


  sensor_msgs::CameraInfoPtr ciLeft(new sensor_msgs::CameraInfo());
  ciLeft->header.frame_id = left->header.frame_id;
  ciLeft->header.stamp = left->header.stamp;
  ciLeft->header.seq = left->header.seq;
  ciLeft->width= left->width;
  ciLeft->height= left->height;

  sensor_msgs::CameraInfoPtr ciRight(new sensor_msgs::CameraInfo());
  ciRight->header.frame_id = right->header.frame_id;
  ciRight->header.stamp = right->header.stamp;
  ciRight->header.seq = right->header.seq;
  ciRight->width= right->width;
  ciRight->height= right->height;

  mleftImagePub.publish(*left, *ciLeft);
  mrightImagePub.publish(*right, *ciRight);


  if (leftStamp != right->header.stamp.toSec())
  {
    NODELET_ERROR("Input stamps changed between the beginning and the end of the callback! Make "
                  "sure the node publishing the topics doesn't override the same data after publishing them. A "
                  "solution is to use this node within another nodelet manager. ");
    NODELET_ERROR("Stamps: "
                  "left=%f->%f right=%f->%f",
                  leftStamp, left->header.stamp.toSec(), rightStamp, right->header.stamp.toSec());
  }
}

}  // namespace zed_nodelets
