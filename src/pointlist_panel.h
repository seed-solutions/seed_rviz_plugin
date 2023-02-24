/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef POINTLIST_PANEL_H
#define POINTLIST_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <dynamic_reconfigure/client.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "seed_rviz_plugin/WpMarkerManager.h"
#include "seed_rviz_plugin/MarkerState.h"
#include "task_programmer/WaypointsConfig.h"
#endif

class QLineEdit;
class QTableWidget;
class QCheckBox;
class QPushButton;

namespace seed_rviz_plugin
{

class PointListPanel: public rviz::Panel
{
Q_OBJECT
public:
  PointListPanel( QWidget* parent = 0 );

  virtual void onInitialize();
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:

protected Q_SLOTS:
  void loadPoints();
  void addPoints();
  void savePoints();
  void delPoints();
  void delOkClicked();
  void delCancelClicked();

  void updateCheckState(int _value);
  void updateValue(int _value);
  void headerClicked(int _column);

private:
  struct Waypoints
  {
    std::string name;
    geometry_msgs::Pose pose;
    double theta;
  };

  void setRowValue(int _row, std::vector<Waypoints>& _wp_list);
  void markerStateCallback(const seed_rviz_plugin::MarkerState& _data);
  void loadWayPoints();
  void setWaypointsList(int _index, std::string _description, double _x, double _y, double _theta);
  bool callService(std::string _command, std::string _index="", std::string _description="",
    double _x=0, double _y=0, double _theta=0);

protected:
  QTableWidget* point_table_;
  QDialog* del_dialog_;
  QLineEdit* map_dir_name_editor_;
  QPushButton* load_btn_;
  QPushButton* save_btn_;
  QPushButton* del_btn_;
  QPushButton* add_btn_;

  std::vector<QCheckBox*> del_list_;
  std::string pkg_path_;
  std::vector<Waypoints> wp_list_original_,wp_list_;

  ros::NodeHandle nh_;
  ros::ServiceClient wp_manager_client_;
  seed_rviz_plugin::WpMarkerManager wp_manager_srv_;
  ros::Subscriber marker_state_sub_,waypoint_editor_sub_;
  dynamic_reconfigure::Client<task_programmer::WaypointsConfig> client_;
  task_programmer::WaypointsConfig wp_config_;
  tf::TransformListener listener_;

  enum column{
    view = 0,
    edit = 1,
    name = 2,
    x = 3,
    y = 4,
    theta = 5,
  };

};

} // end namespace seed_rviz_plugin

#endif // POINTLIST_PANEL_H
