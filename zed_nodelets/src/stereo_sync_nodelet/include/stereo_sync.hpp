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

#ifndef RGBD_SENSOR_SYNC_HPP
#define RGBD_SENSOR_SYNC_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>


namespace zed_nodelets
{
class StereoSyncNodelet : public nodelet::Nodelet
{
public:
  StereoSyncNodelet();
  virtual ~StereoSyncNodelet();

protected:
  /*! \brief Initialization function called by the Nodelet base class
   */
  virtual void onInit();

  /*! \brief Reads parameters from the param server
   */
  void readParameters();

  /*! \brief Callback for gray stereo images synchronization
   */
  void callbackStereo(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right,
                    const sensor_msgs::CameraInfoConstPtr& leftCameraInfo,
                    const sensor_msgs::CameraInfoConstPtr& rightCameraInfo);


private:
  // Node handlers
  ros::NodeHandle mNh;   // Node handler
  ros::NodeHandle mNhP;  // Private Node handler

  // Publishers
  image_transport::CameraPublisher mleftImagePub;
  image_transport::CameraPublisher mrightImagePub;

  // Subscribers
  image_transport::SubscriberFilter mSubLeftGrayImage;
  image_transport::SubscriberFilter mSubRightGrayImage;
  message_filters::Subscriber<sensor_msgs::CameraInfo> mSubLeftGrayCamInfo;
  message_filters::Subscriber<sensor_msgs::CameraInfo> mSubRightGrayCamInfo;
  

  // Approx sync policies
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
      ApproxStereoSyncPolicy;
 
  message_filters::Synchronizer<ApproxStereoSyncPolicy>* mApproxStereoSync = nullptr;

  // Exact sync policies
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                    sensor_msgs::CameraInfo>
      ExactStereoSyncPolicy;
  message_filters::Synchronizer<ExactStereoSyncPolicy>* mExactStereoSync = nullptr;

  // Params
  std::string mZedNodeletName = "zed_node";
  bool mUseApproxSync = true;
  bool mUseImu = true;
  bool mUseMag = true;
  int mQueueSize = 50;
};

}  // namespace zed_nodelets

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_nodelets::StereoSyncNodelet, nodelet::Nodelet)

#endif  // RGBD_SENSOR_SYNC_HPP
