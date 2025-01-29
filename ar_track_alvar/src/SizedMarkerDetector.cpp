#include "ar_track_alvar/SizedMarkerDetector.h"

namespace alvar {

  double SizedMarkerDetector::max_new_marker_error = 0.;
  double SizedMarkerDetector::max_track_error = 0.;

  SizedMarkerDetector::SizedMarkerDetector(double marker_size) : marker_size_(marker_size)
  {
    detector_.SetMarkerSize(marker_size_);
  }

  void SizedMarkerDetector::setBundles(const std::vector<MultiMarkerBundle>& bundles, const std::vector<int>& master_ids)
  {
    for(size_t  i = 0; i < bundles.size(); i++)
    {
      if(bundles.at(i).marker_size == marker_size_)
      {
        bundles_.emplace(i, bundles.at(i));
        poses_.emplace(i, Pose());
        poses_[i].Reset();
        bundle_seen_.emplace(i, false);
        master_ids_.emplace(i, master_ids[i]);
      }
    }
  }

  void SizedMarkerDetector::GetMultiMarkerPoses(cv::Mat& image, Camera *cam)
  {
    if (detector_.Detect(image, cam, true, false, SizedMarkerDetector::max_new_marker_error, SizedMarkerDetector::max_track_error, CVSEQ, true))
    {
      for(auto& bundle : bundles_)
        bundle.second.Update(detector_.markers, cam, poses_[bundle.first]);

      if(detector_.DetectAdditional(image, cam, false) > 0)
      {
        for(auto& bundle : bundles_)
          if (bundle.second.SetTrackMarkers(detector_, cam, poses_[bundle.first], image) > 0)
            bundle.second.Update(detector_.markers, cam, poses_[bundle.first]);
      }
    }
  }

  int SizedMarkerDetector::getMainId(int marker_id)
  {
    if(marker_id >= 0)
    {
      for(auto& bundle : bundles_)
        if(bundle.second.IsValidMarker(marker_id))
          return bundle.second.getMasterId();
    }

    return -1;
  }

  void SizedMarkerDetector::computeSeenMarkers()
  {
    for(auto& seen : bundle_seen_)
      seen.second = false;

    for (size_t i = 0; i < detector_.markers->size(); i++)
    {
      int id = detector_.markers->at(i).GetId();
      int main_id = getMainId(id);

      if(main_id >= 0)
      {
        //Mark the bundle that marker belongs to as "seen"
        for(auto& bundle : bundles_)
          for(auto bundel_id : bundle.second.getIndices())
            if(bundel_id == id)
            {
              bundle_seen_[bundle.first] = true;
              break;
            }
      }
    }
  }

  void SizedMarkerDetector::createMarkersMsgs(const std_msgs::Header& header, const std::string& output_frame, const tf::StampedTransform& cam_to_output)
  {
    resetMarkersMsgs();

    visualization_msgs::Marker rviz_marker;
    ar_track_alvar_msgs::AlvarMarker ar_pose_marker;
    ar_track_alvar_msgs::AlvarVisibleMarker ar_visible_marker;

    initMarkerMsgs(header, output_frame,
                   rviz_marker, ar_pose_marker, ar_visible_marker);

    for (size_t i = 0; i < detector_.markers->size(); i++)
    {
      int id = detector_.markers->at(i).GetId();
      int main_id = getMainId(id);

      if(main_id >= 0)
      {
        // Don't draw if it is a master tag...we do this later, a bit differently
        //Now we want to draw it even if it is the master tag to know where it has been seen
        // given that we can have a transform between the real position of the tag
        // and the returned pose

        tf::Transform pose = getMarkerTransform(detector_.markers->at(i).pose);

        makeRvizMsg(VISIBLE_MARKER, id, marker_size_, pose, rviz_marker);
        rviz_markers_.emplace_back(rviz_marker);

        makeArMarkerMsg(id, pose, cam_to_output, ar_pose_marker);
        ar_visible_marker.id = id;
        ar_visible_marker.main_id = main_id;
        ar_visible_marker.confidence = detector_.markers->at(i).GetError(Marker::TRACK_ERROR);
        ar_visible_marker.size = marker_size_;
        ar_visible_marker.pose = ar_pose_marker.pose;
        ar_visible_markers_.emplace_back (ar_visible_marker);
      }
    }

    //Draw the main markers, whether they are visible or not -- but only if at least 1 marker from their bundle is currently seen
    for(const auto& seen : bundle_seen_)
    {
      if(seen.second)
      {
        tf::Transform pose = getMarkerTransform(poses_[seen.first]);
        int master_id = master_ids_[seen.first];

        transforms_.emplace_back(makeStampedTransform(master_id, pose, header));

        makeRvizMsg(MAIN_MARKER, master_id, marker_size_, pose, rviz_marker);
        rviz_markers_.emplace_back(rviz_marker);

        makeArMarkerMsg(master_id, pose, cam_to_output, ar_pose_marker);
        ar_pose_markers_.emplace_back(ar_pose_marker);
      }
    }
  }

  void SizedMarkerDetector::makeArMarkerMsg(int id, const tf::Transform& marker_pose, const tf::StampedTransform& cam_to_output, ar_track_alvar_msgs::AlvarMarker& ar_pose_marker)
  {
    tf::Transform tag_pose_output = cam_to_output * marker_pose;

    tf::poseTFToMsg(tag_pose_output, ar_pose_marker.pose.pose);
    ar_pose_marker.id = id;
  }

  tf::StampedTransform SizedMarkerDetector::makeStampedTransform(int id, const tf::Transform& marker_pose, const std_msgs::Header& header)
  {
    std::string marker_frame = "ar_marker_" + std::to_string(id);
    tf::StampedTransform cam_to_marker(marker_pose, header.stamp, header.frame_id, marker_frame.c_str());
    return cam_to_marker;
  }

  void SizedMarkerDetector::makeRvizMsg(int type, int id, double m_size, const tf::Transform& marker_pose, visualization_msgs::Marker& rviz_marker)
  {
    //Create the rviz visualization message
    tf::poseTFToMsg(marker_pose, rviz_marker.pose);
    rviz_marker.id = id;

    rviz_marker.scale.x = 1.0 * m_size/100.0;
    rviz_marker.scale.y = 1.0 * m_size/100.0;
    rviz_marker.scale.z = 0.2 * m_size/100.0;

    rviz_marker.ns = (type==MAIN_MARKER) ? "main_shapes" : "basic_shapes";

    //Determine a color and opacity, based on marker type
    if(type==MAIN_MARKER)
    {
      rviz_marker.color.r = 1.0f;
      rviz_marker.color.g = 0.0f;
      rviz_marker.color.b = 0.0f;
      rviz_marker.color.a = 1.0;
    }
    else if(type==VISIBLE_MARKER)
    {
      rviz_marker.color.r = 0.0f;
      rviz_marker.color.g = 1.0f;
      rviz_marker.color.b = 0.0f;
      rviz_marker.color.a = 0.7;
    }
  }

  void SizedMarkerDetector::initMarkerMsgs(const std_msgs::Header& header, const std::string& output_frame,
                                          visualization_msgs::Marker& rviz_marker,
                                          ar_track_alvar_msgs::AlvarMarker& ar_pose_marker,
                                          ar_track_alvar_msgs::AlvarVisibleMarker& ar_visible_marker)
  {
    rviz_marker.header.frame_id = header.frame_id;
    rviz_marker.header.stamp = header.stamp;

    rviz_marker.type = visualization_msgs::Marker::CUBE;
    rviz_marker.action = visualization_msgs::Marker::ADD;
    rviz_marker.lifetime = ros::Duration (1.0);

    ar_pose_marker.header.frame_id = output_frame;
    ar_pose_marker.header.stamp = header.stamp;
    ar_pose_marker.pose.header.frame_id = output_frame;
    ar_pose_marker.pose.header.stamp = header.stamp;

    ar_visible_marker.header = ar_pose_marker.header;
    ar_visible_marker.pose.header = ar_pose_marker.header;
  }

  void SizedMarkerDetector::resetMarkersMsgs()
  {
    rviz_markers_.clear();
    ar_pose_markers_.clear();
    ar_visible_markers_.clear();
    transforms_.clear();
  }

  tf::Transform SizedMarkerDetector::getMarkerTransform(const Pose &p)
  {
    double px = p.translation[0]/100.0;
    double py = p.translation[1]/100.0;
    double pz = p.translation[2]/100.0;
    double qx = p.quaternion[1];
    double qy = p.quaternion[2];
    double qz = p.quaternion[3];
    double qw = p.quaternion[0];

    //Get the marker pose in the camera frame
    tf::Quaternion rotation (qx,qy,qz,qw);
    tf::Vector3 origin (px,py,pz);
    tf::Transform t(rotation, origin);  //transform from cam to marker

    //tf::Vector3 markerOrigin (0, 0, 0);
    //tf::Transform m (tf::Quaternion::getIdentity(), markerOrigin);
    //tf::Transform marker_pose = t * m;

    return t;
  }

} // namespace alvar