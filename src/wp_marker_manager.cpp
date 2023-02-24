#include <stdio.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include "wp_marker_manager.h"

WpMarkerManager::WpMarkerManager(const ros::NodeHandle _nh) :
 nh_(_nh)
{
  server_.reset( new interactive_markers::InteractiveMarkerServer("wp_marker","",false) );
  wp_manager_server_ =
    nh_.advertiseService("wp_manager", &WpMarkerManager::serviceCallback, this);
  marker_state_pub_ = nh_.advertise<seed_rviz_plugin::MarkerState>( "marker_state", 1 );

}

void WpMarkerManager::processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  // when marker is moved, publish marker state for pointlist panel
  marker_state_.name = feedback->marker_name;
  marker_state_.x = feedback->pose.position.x;
  marker_state_.y = feedback->pose.position.y;
  marker_state_.theta = (tf::getYaw(feedback->pose.orientation) * 180.0 / M_PI);
  marker_state_pub_.publish(marker_state_);

}

void WpMarkerManager::alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  // always set z position to 0.1
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.z = 0.1;
  server_->setPose( feedback->marker_name, pose );
  server_->applyChanges();
}

visualization_msgs::Marker WpMarkerManager::makeMarker( visualization_msgs::InteractiveMarker &msg, std::string _desc, bool _mesh=true)
{
  visualization_msgs::Marker marker;

  if(_mesh){
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://seed_rviz_plugin/media/wp_flag.dae";
    marker.scale.x = msg.scale;
    marker.scale.y = msg.scale;
    marker.scale.z = msg.scale;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 0.9;
    marker.color.a = 0.5;
  }
  else{
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = _desc;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.pose.position. z = 0.5;
  }

  return marker;
}

void WpMarkerManager::addMarker(std::string _name, std::string _desc, geometry_msgs::Pose _pose, bool _interact=true)
{
  visualization_msgs::InteractiveMarker present_marker;
  visualization_msgs::InteractiveMarker int_marker;
  if(server_->get(_name,present_marker)){
    int_marker.pose = present_marker.pose;
    server_->erase(_name);
  }
  else{
    int_marker.pose = _pose;
  }
  int_marker.header.frame_id = "map";
  int_marker.name = _name;
  int_marker.description = _desc;
  int_marker.scale = 1;

  // add interactive marker (set rotate axis and can be moved 3d)
  visualization_msgs::InteractiveMarkerControl control;
  if(_interact){
    tf::Quaternion orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "move_plane";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    int_marker.controls.push_back(control);
  }

  // make a box which also moves in the plane
  control.name = "marker";
  control.markers.push_back( makeMarker(int_marker,_desc) );
  control.markers.push_back( makeMarker(int_marker,_desc,false) );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  server_->insert(int_marker);
  server_->setCallback(int_marker.name, boost::bind(&WpMarkerManager::processFeedback, this, _1));

  // set different callback for POSE_UPDATE feedback
  server_->setCallback(int_marker.name,boost::bind(&WpMarkerManager::alignMarker, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );

  server_->applyChanges();
}

void WpMarkerManager::deleteMarker(std::string _name)
{
  if(_name == ""){  //delete all markers
    server_->clear();
    server_->applyChanges();
  }
  else{
    server_->erase(_name);
    server_->applyChanges();
  }
}

void WpMarkerManager::updateMarker(std::string _name, geometry_msgs::Pose _pose)
{
  server_->setPose(_name,_pose);
  server_->applyChanges();

}

void WpMarkerManager::disableInteractive(std::string _name)
{
  visualization_msgs::InteractiveMarker int_marker;
  geometry_msgs::Pose pose;

  if(server_->get(_name,int_marker))
  {
    pose = int_marker.pose;
    server_->erase(_name);
    addMarker(int_marker.name,"",pose,false);
  }
}

bool WpMarkerManager::serviceCallback(seed_rviz_plugin::WpMarkerManager::Request &_req,
    seed_rviz_plugin::WpMarkerManager::Response &_res)
{
  geometry_msgs::Pose pose;
  pose.position.x = _req.x;
  pose.position.y = _req.y;
  pose.position.z = 0.1;

  tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,_req.theta *( M_PI / 180.0));

  pose.orientation.x = quat.getX();
  pose.orientation.y = quat.getY();
  pose.orientation.z = quat.getZ();
  pose.orientation.w = quat.getW();

  /*
  // add_int_marker : add interactive marker
  // add_marker : add only marker(not interactive)
  // delete : delete marker
  // update : update description and position
  */

  if(_req.command == "add_int_marker"){
    addMarker(_req.index,_req.description,pose);
  }
  else if(_req.command == "add_marker"){
    addMarker(_req.index,_req.description,pose,false);
  }
  else if(_req.command == "delete"){
    deleteMarker(_req.index);
  }
  else if(_req.command == "update"){
    updateMarker(_req.index,pose);
  }

  _res.result = "succeeded";

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"wp_marker_manager_node");
  ros::NodeHandle nh;

  WpMarkerManager wmm(nh);

  ros::spin();
}