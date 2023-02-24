#ifndef WP_MARKER_MANAGER_H
#define WP_MARKER_MANAGER_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include "seed_rviz_plugin/WpMarkerManager.h"
#include "seed_rviz_plugin/MarkerState.h"

class WpMarkerManager
{
public:
  WpMarkerManager(ros::NodeHandle _nh);

  void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void alignMarker(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  visualization_msgs::Marker makeMarker(
    visualization_msgs::InteractiveMarker &msg, std::string _text, bool _mesh);

  void addMarker(std::string _name, std::string _desc, geometry_msgs::Pose _pose, bool _interact);
  void deleteMarker(std::string _name);
  void updateMarker(std::string _name, geometry_msgs::Pose _pose);
  void disableInteractive(std::string _name);

  bool serviceCallback(seed_rviz_plugin::WpMarkerManager::Request &_req,
    seed_rviz_plugin::WpMarkerManager::Response &_res);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer wp_manager_server_;
  ros::Publisher marker_state_pub_;
  seed_rviz_plugin::MarkerState marker_state_;

protected:
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

};

#endif // WP_MARKER_MANAGER_H
