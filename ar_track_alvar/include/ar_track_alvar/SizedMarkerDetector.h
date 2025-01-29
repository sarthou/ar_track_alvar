#ifndef SIZEDMAKERDETECTOR_H
#define SIZEDMAKERDETECTOR_H

#include <unordered_map>
#include <vector>

#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarVisibleMarker.h>
#include <tf/transform_datatypes.h>

#define MAIN_MARKER 1
#define VISIBLE_MARKER 2
#define GHOST_MARKER 3

namespace alvar {

class SizedMarkerDetector
{
public:
  static double max_new_marker_error;
  static double max_track_error;

  SizedMarkerDetector(double marker_size);

  void setBundles(const std::vector<MultiMarkerBundle>& bundles, const std::vector<int>& master_ids);

  void GetMultiMarkerPoses(cv::Mat& image, Camera *cam);

  int getMainId(int marker_id);
  void computeSeenMarkers();

  void createMarkersMsgs(const std_msgs::Header& header, const std::string& output_frame, const tf::StampedTransform& cam_to_output);

  double getMarkerSize() { return marker_size_; }

  std::vector<visualization_msgs::Marker> rviz_markers_;
  std::vector<ar_track_alvar_msgs::AlvarMarker> ar_pose_markers_;
  std::vector<ar_track_alvar_msgs::AlvarVisibleMarker> ar_visible_markers_;
  std::vector<tf::StampedTransform> transforms_;

private:
  MarkerDetector<MarkerData> detector_;
  double marker_size_; 

  std::unordered_map<int, MultiMarkerBundle> bundles_;
  std::unordered_map<int, Pose> poses_;
  std::unordered_map<int, bool> bundle_seen_;
  std::unordered_map<int, int> master_ids_;

  void makeArMarkerMsg(int id, const tf::Transform& marker_pose, const tf::StampedTransform& cam_to_output, ar_track_alvar_msgs::AlvarMarker& ar_pose_marker);
  tf::StampedTransform makeStampedTransform(int id, const tf::Transform& marker_pose, const std_msgs::Header& header);
  void makeRvizMsg(int type, int id, double m_size, const tf::Transform& marker_pose, visualization_msgs::Marker& rviz_marker);

  void initMarkerMsgs(const std_msgs::Header& header, const std::string& output_frame,
                      visualization_msgs::Marker& rviz_marker,
                      ar_track_alvar_msgs::AlvarMarker& ar_pose_marker,
                      ar_track_alvar_msgs::AlvarVisibleMarker& ar_visible_marker);
  void resetMarkersMsgs();

  tf::Transform getMarkerTransform(const Pose &p);
};

} //namespace alvar

#endif //SIZEDMAKERDETECTOR_H