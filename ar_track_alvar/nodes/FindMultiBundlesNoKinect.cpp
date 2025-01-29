/*
  Software License Agreement (BSD License)

  Copyright (c) 2012, Scott Niekum
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
  * Neither the name of the Willow Garage nor the names of its
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  author: Scott Niekum
*/


#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"
#include "ar_track_alvar/Shared.h"
#include "ar_track_alvar/SizedMarkerDetector.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarVisibleMarker.h>
#include <ar_track_alvar_msgs/AlvarVisibleMarkers.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>

#include <thread>

using namespace alvar;
using namespace std;

Camera *cam;
cv_bridge::CvImagePtr cv_ptr_;
image_transport::Subscriber cam_sub_;

ros::Publisher arMarkerPub_;
ros::Publisher arVisibleMarkerPub_;
ros::Publisher rvizMarkerPub_;

ar_track_alvar_msgs::AlvarMarkers arPoseMarkers_;
ar_track_alvar_msgs::AlvarVisibleMarkers arPoseVisibleMarkers_;

tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
std::vector<SizedMarkerDetector*> marker_detectors;

std::string output_frame;

void processMarkerDetector(SizedMarkerDetector* marker_detector, cv::Mat& ipl_image, Camera* cam, const std_msgs::Header& header, 
                           const std::string& output_frame, const tf::StampedTransform& cam_to_output)
{
  marker_detector->GetMultiMarkerPoses(ipl_image, cam);
  marker_detector->computeSeenMarkers();
  marker_detector->createMarkersMsgs(header, output_frame, cam_to_output);
}

//Callback to handle getting video frames and processing them
void getCapCallback (const sensor_msgs::ImageConstPtr& image_msg)
{
  //If we've already gotten the cam info, then go ahead
  if(cam->getCamInfo_)
  {
    try
    {
      //Get the transformation from the Camera to the output frame for this image capture
      tf::StampedTransform cam_to_output;
      try
      {
      	tf_listener->waitForTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, ros::Duration(1.0));
      	tf_listener->lookupTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, cam_to_output);
      }
      catch (tf::TransformException& ex)
      {
	      ROS_ERROR("%s",ex.what());
      }

      arPoseMarkers_.markers.clear();
      arPoseVisibleMarkers_.markers.clear();

      //Convert the image
      cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

      //Get the estimated pose of the main markers by using all the markers in each bundle

      // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
      // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
      // do this conversion here -jbinney
      cv::Mat ipl_image = cv_ptr_->image;

      std::vector<std::thread> threads;
      for (auto* marker_detector : marker_detectors)
      {
        threads.emplace_back(processMarkerDetector, 
                  marker_detector, std::ref(ipl_image), std::ref(cam),
                  std::cref(image_msg->header), std::cref(output_frame), std::cref(cam_to_output));
      }

      for (auto& thread : threads)
          if (thread.joinable())
              thread.join();
      
      /*for(auto& marker_detector : marker_detectors)
      {
        marker_detector->GetMultiMarkerPoses(ipl_image, cam);
        marker_detector->computeSeenMarkers();

        marker_detector->createMarkersMsgs(image_msg->header, output_frame, cam_to_output);
      }*/

      for(auto& marker_detector : marker_detectors)
      {
        for(auto& marker : marker_detector->rviz_markers_)
          rvizMarkerPub_.publish(marker);

        for(auto& transform : marker_detector->transforms_)
          tf_broadcaster->sendTransform(transform);

        for(auto& marker : marker_detector->ar_pose_markers_)
          arPoseMarkers_.markers.emplace_back(marker);

        for(auto& marker : marker_detector->ar_visible_markers_)
          arPoseVisibleMarkers_.markers.emplace_back(marker);
      }

      //Publish the marker messages
      arPoseMarkers_.header.stamp = image_msg->header.stamp;
      arPoseVisibleMarkers_.header.stamp = image_msg->header.stamp;
      arMarkerPub_.publish (arPoseMarkers_);
      arVisibleMarkerPub_.publish (arPoseVisibleMarkers_);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
    }
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "marker_detect");
  ros::NodeHandle n;

  if(argc < 8){
    std::cout << std::endl;
    cout << "Not enough arguments provided." << endl;
    cout << "Usage: ./findMarkerBundles <marker size in cm> <max new marker error> <max track error> <cam image topic> <cam info topic> <output frame> <list of bundle XML files...>" << endl;
    std::cout << std::endl;
    return 0;
  }

  // Get params from command line
  float marker_size = atof(argv[1]);
  SizedMarkerDetector::max_new_marker_error = atof(argv[2]);
  SizedMarkerDetector::max_track_error = atof(argv[3]);
  std::string cam_image_topic = argv[4];
  std::string cam_info_topic = argv[5];
  output_frame = argv[6];
  int n_args_before_list = 7;
  int n_bundles = argc - n_args_before_list;

  std::vector<MultiMarkerBundle> multi_marker_bundles;
  multi_marker_bundles.reserve(n_bundles);
  std::vector<int> master_ids;
  master_ids.reserve(n_bundles);

  std::set<double> markers_size;

  // Load the marker bundle XML files
  for(int i=0; i < n_bundles; i++)
  {
    MultiMarker loadHelper;
    if(loadHelper.Load(argv[i + n_args_before_list], FILE_FORMAT_XML))
    {
      vector<int> id_vector = loadHelper.getIndices();
      multi_marker_bundles.emplace_back(id_vector);
      multi_marker_bundles.back().Load(argv[i + n_args_before_list], FILE_FORMAT_XML);
      master_ids.emplace_back(multi_marker_bundles.back().getMasterId());

      double bundle_marker_size = multi_marker_bundles.back().getMarkerSize();
      if(bundle_marker_size == 0)
        bundle_marker_size = marker_size;
      markers_size.insert(bundle_marker_size);
    }
    else
    {
      cout<<"Cannot load file "<< argv[i + n_args_before_list] << endl;
      return 0;
    }
  }

  std::cout << "Marker sizes are:" << std::endl;
  for(auto& marker_size : markers_size)
  {
    auto* tmp = new SizedMarkerDetector(marker_size);
    tmp->setBundles(multi_marker_bundles, master_ids);
    marker_detectors.emplace_back(tmp);
    std::cout << marker_size << std::endl;
  }

  // Set up camera, listeners, and broadcasters
  cam = new Camera(n, cam_info_topic);
  tf_listener = new tf::TransformListener(n);
  tf_broadcaster = new tf::TransformBroadcaster();
  arMarkerPub_ = n.advertise < ar_track_alvar_msgs::AlvarMarkers > ("ar_pose_marker", 0);
  arVisibleMarkerPub_ = n.advertise < ar_track_alvar_msgs::AlvarVisibleMarkers > ("ar_pose_visible_marker", 0);
  rvizMarkerPub_ = n.advertise < visualization_msgs::Marker > ("visualization_marker", 0);

  //Give tf a chance to catch up before the camera callback starts asking for transforms
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  //Subscribe to topics and set up callbacks
  ROS_INFO ("Subscribing to image topic");
  image_transport::ImageTransport it_(n);
  cam_sub_ = it_.subscribe (cam_image_topic, 2, &getCapCallback);

  ros::spin();

  return 0;
}
