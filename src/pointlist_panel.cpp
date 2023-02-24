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
 *       unotice, this list of conditions and the following disclaimer in the
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

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

// add
#include <QTableWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSignalMapper>
#include <QHeaderView>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>
#include <iostream>

#include <geometry_msgs/Twist.h>

#include "pointlist_panel.h"
#include "my_header.h"


namespace seed_rviz_plugin
{
PointListPanel::PointListPanel( QWidget* parent )
  : rviz::Panel( parent )
  , client_("/waypoints_editor")
{
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "map directory:" ));
  map_dir_name_editor_ = new QLineEdit;
  map_dir_name_editor_->setEnabled(false);
  topic_layout->addWidget( map_dir_name_editor_ );

  point_table_ = new QTableWidget;
  point_table_->setColumnCount(6);
  point_table_->verticalHeader()->hide();
  QStringList headers;
  headers << " 表示 " << " 編集 " << "名称" << "x[m]" << "y[m]" << "θ[deg]";
  point_table_->setHorizontalHeaderLabels(headers);

  QHeaderView* header = point_table_->horizontalHeader();
  header->setSectionResizeMode(QHeaderView::Stretch);
  header->setSectionResizeMode(0,QHeaderView::ResizeToContents);
  header->setSectionResizeMode(1,QHeaderView::ResizeToContents);
  connect(header,SIGNAL(sectionClicked(const int&)),this,SLOT(headerClicked(const int&)));

  load_btn_ = new QPushButton("読込");
  save_btn_ = new QPushButton("保存");
  del_btn_ = new QPushButton("行削除");
  add_btn_ = new QPushButton("行追加");
  save_btn_->setEnabled(false);
  del_btn_->setEnabled(false);
  add_btn_->setEnabled(false);

  QHBoxLayout* btn_layout = new QHBoxLayout;
  btn_layout->addWidget(load_btn_);
  btn_layout->addWidget(save_btn_);
  btn_layout->addWidget(del_btn_);
  btn_layout->addWidget(add_btn_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  layout->addWidget( point_table_ );
  layout->addLayout( btn_layout );
  setLayout( layout );

  connect(load_btn_, SIGNAL(clicked()),this,SLOT(loadPoints()));
  connect(add_btn_, SIGNAL(clicked()),this,SLOT(addPoints()));
  connect(del_btn_, SIGNAL(clicked()),this,SLOT(delPoints()));
  connect(save_btn_, SIGNAL(clicked()),this,SLOT(savePoints()));

}

void PointListPanel::onInitialize()
{
  if(ros::ok())
  {
    pkg_path_ = ros::package::getPath("task_programmer");
    wp_manager_client_ = nh_.serviceClient<seed_rviz_plugin::WpMarkerManager>("wp_manager");
    marker_state_sub_ = nh_.subscribe("marker_state",1, &PointListPanel::markerStateCallback,this);
  }
  ros::spinOnce();
}

void PointListPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

void PointListPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void PointListPanel::loadPoints()
{
  loadWayPoints();

  point_table_->setRowCount(0);
  for(size_t i=0; i < wp_list_original_.size(); ++i){
    point_table_->insertRow(point_table_->rowCount() );
    setRowValue(i,wp_list_original_);
  }
  std::copy(wp_list_original_.begin(), wp_list_original_.end(), std::back_inserter(wp_list_) );

  headerClicked(column::view);

  save_btn_->setEnabled(true);
  del_btn_->setEnabled(true);
  add_btn_->setEnabled(true);
  add_btn_->setEnabled(true);
}

void PointListPanel::addPoints()
{
  point_table_->insertRow(point_table_->rowCount());
  int row_number = point_table_->rowCount()-1;
  Waypoints wp;

  std::string suffix ="";
  for(size_t i=0;i<wp_list_.size();++i){
    if(wp_list_.at(i).name.c_str() == "p" + std::to_string(row_number)){
      suffix = "_2";
      break;
    }
  }
  wp.name = "p" + std::to_string(row_number) + suffix;

  tf::StampedTransform transform_now;
  listener_.lookupTransform("/map", "/base_link",ros::Time(0), transform_now);
  wp.pose.position.x = transform_now.getOrigin().x();
  wp.pose.position.y = transform_now.getOrigin().y();
  wp.pose.position.z = transform_now.getOrigin().z();
  wp.pose.orientation.x = transform_now.getRotation().x();
  wp.pose.orientation.y = transform_now.getRotation().y();
  wp.pose.orientation.z = transform_now.getRotation().z();
  wp.pose.orientation.w = transform_now.getRotation().w();
  wp.theta = tf::getYaw(wp.pose.orientation) * 180.0 / M_PI;

  wp_list_.push_back(wp);
  setRowValue(row_number, wp_list_);

}

void PointListPanel::savePoints()
{
  std::string map_dir;
  if(client_.getCurrentConfiguration(wp_config_, ros::Duration(0.5))){
    map_dir = wp_config_.dir_name.c_str();
  }
  else{
    map_dir = "";
    QMessageBox::critical(this, tr("Error"), tr("保存先が見つかりません"));
    return ;
  }

  QString text = "保存しますか?";
  QMessageBox::StandardButton res = QMessageBox::question(
      this, tr(""), tr(text.toStdString().c_str()), QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  switch (res)
  {
  case QMessageBox::Yes:
      break;
  case QMessageBox::No:
      return;
      break;
  default:
      break;
  }

  std::string file_path = pkg_path_ + map_dir_name_editor_->text().toStdString() + "/waypoints.yaml";
  std::ofstream ofs_delete(file_path, std::ios_base::trunc);
  ofs_delete.flush();
  ofs_delete.close();

  std::ofstream ofs(file_path, std::ios_base::app);
  for(size_t i=0; i < wp_list_.size(); ++i){
    //save postion------------
    ofs << "- pose:"            << std::endl;
    ofs << "    name: "         << wp_list_.at(i).name << std::endl;
    ofs << "    position:"      << std::endl;
    ofs << "        x: "        << wp_list_.at(i).pose.position.x << std::endl;
    ofs << "        y: "        << wp_list_.at(i).pose.position.y << std::endl;
    ofs << "        z: "        << wp_list_.at(i).pose.position.z << std::endl;
    ofs << "    orientation:"   << std::endl;
    ofs << "        x: "        << wp_list_.at(i).pose.orientation.x << std::endl;
    ofs << "        y: "        << wp_list_.at(i).pose.orientation.y << std::endl;
    ofs << "        z: "        << wp_list_.at(i).pose.orientation.z << std::endl;
    ofs << "        w: "        << wp_list_.at(i).pose.orientation.w << std::endl;
    //------------------------------
  }
  ofs.flush();
  ofs.close();
  loadPoints();

}

void PointListPanel::delPoints()
{
  del_dialog_ = new QDialog();
  del_dialog_->setWindowTitle("Delete Points");

  QWidget* widget = new QWidget();
  QVBoxLayout* main_layout = new QVBoxLayout;
  QVBoxLayout* point_layout = new QVBoxLayout;
  QScrollArea* scroll = new QScrollArea;

  del_list_.clear();
  for (size_t i = 0; i < wp_list_.size(); ++i) {
    QCheckBox* point = new QCheckBox(QString::fromStdString(wp_list_.at(i).name), this);
    point_layout->addWidget(point);
    del_list_.push_back(point);
  }

  widget->setLayout(point_layout);
  scroll->setWidget(widget);

  QPushButton* ok_btn = new QPushButton("OK");
  QPushButton* cancel_btn = new QPushButton("Cancel");
  connect(cancel_btn, SIGNAL(clicked()),this,SLOT(delCancelClicked()));
  connect(ok_btn, SIGNAL(clicked()),this,SLOT(delOkClicked()));

  QHBoxLayout* btn_layout = new QHBoxLayout;
  btn_layout->addWidget(ok_btn);
  btn_layout->addWidget(cancel_btn);

  main_layout->addWidget(scroll);
  main_layout->addLayout(btn_layout);

  del_dialog_->setLayout(main_layout);
  del_dialog_->show();

}

void PointListPanel::delOkClicked()
{
  for(size_t i=0; i < del_list_.size(); ++i){
    if(del_list_.at(i)->isChecked()){
      for(auto it = wp_list_.begin(); it != wp_list_.end();){
        if(it->name == del_list_.at(i)->text().toStdString()){
          it = wp_list_.erase(it);
        }
        else{
          ++it;
        }
      }
    }
  }

  callService("delete");
  point_table_->setRowCount(0);
  for(size_t i=0; i < wp_list_.size(); ++i){
    point_table_->insertRow(point_table_->rowCount() );
    setRowValue(i,wp_list_);
  }
  del_dialog_->reject();
  headerClicked(column::view);

}

void PointListPanel::delCancelClicked()
{
  del_dialog_->reject();
}

void PointListPanel::updateCheckState(int _index)
{
  // ROS_INFO("update states");
  tf::StampedTransform transform_all;
  static tf::TransformBroadcaster br_all;
  tf::Quaternion q;

  QCheckBox* view_check_box = static_cast<QWidget*>(point_table_->cellWidget(_index,column::view))->findChild<QCheckBox *>();
  QCheckBox* edit_check_box = static_cast<QWidget*>(point_table_->cellWidget(_index,column::edit))->findChild<QCheckBox *>();
  QLineEdit* name_line = static_cast<QLineEdit*>(point_table_->cellWidget(_index,column::name));
  QDoubleSpinBox* x_spin = static_cast<QDoubleSpinBox*>(point_table_->cellWidget(_index,column::x));
  QDoubleSpinBox* y_spin = static_cast<QDoubleSpinBox*>(point_table_->cellWidget(_index,column::y));
  QDoubleSpinBox* th_spin = static_cast<QDoubleSpinBox*>(point_table_->cellWidget(_index,column::theta));

  //check same name already exists
  for(size_t i=0;i<wp_list_.size();++i){
    if(i != _index && wp_list_.at(i).name.c_str() == name_line->text()){
      name_line->setText(QString::fromStdString(wp_list_.at(i).name + "_2"));
      break;
    }
  }

  bool view_state = view_check_box->isChecked();
  bool edit_state = edit_check_box->isChecked();

  if(view_state && !edit_state){  // add only marker(not interactive)
    edit_check_box->setEnabled(true);
    callService("add_marker",std::to_string(_index),name_line->text().toStdString(),
      x_spin->value(),y_spin->value(),th_spin->value());
    name_line->setEnabled(false);
    x_spin->setEnabled(false);
    y_spin->setEnabled(false);
    th_spin->setEnabled(false);
  }
  else if(view_state && edit_state){ // add interactive marker
    callService("add_int_marker",std::to_string(_index),name_line->text().toStdString(),
      x_spin->value(),y_spin->value(),th_spin->value());
    name_line->setEnabled(true);
    x_spin->setEnabled(true);
    y_spin->setEnabled(true);
    th_spin->setEnabled(true);
  }
  else{
    view_check_box->setChecked(false);
    edit_check_box->setEnabled(false);
    callService("delete",std::to_string(_index),name_line->text().toStdString());
  }

  setWaypointsList(_index,name_line->text().toStdString(),
      x_spin->value(),y_spin->value(),th_spin->value());

  // change background color, if value is changed
  if(_index  > wp_list_original_.size() - 1){
    name_line->setStyleSheet("QLineEdit{background-color: magenta;}");
    x_spin->setStyleSheet("QDoubleSpinBox{background-color: magenta;}");
    y_spin->setStyleSheet("QDoubleSpinBox{background-color: magenta;}");
    th_spin->setStyleSheet("QDoubleSpinBox{background-color: magenta;}");
    return ;
  }
  if(name_line->text().toStdString() != wp_list_original_.at(_index).name.c_str() ){
    name_line->setStyleSheet("QLineEdit{background-color: magenta;}");
  }
  else{
    name_line->setStyleSheet("QLineEdit{}");
  }
  if(std::round(x_spin->value()*100) != std::round(wp_list_original_.at(_index).pose.position.x*100) ){
    x_spin->setStyleSheet("QDoubleSpinBox{background-color: magenta;}");
  }
  else{
    x_spin->setStyleSheet("QDoubleSpinBox{}");
  }
  if(std::round(y_spin->value()*100) != std::round(wp_list_original_.at(_index).pose.position.y*100) ){
    y_spin->setStyleSheet("QDoubleSpinBox{background-color: magenta;}");
  }
  else{
    y_spin->setStyleSheet("QDoubleSpinBox{}");
  }
  if(std::round(th_spin->value()*100) != std::round(wp_list_original_.at(_index).theta*100) ){
    th_spin->setStyleSheet("QDoubleSpinBox{background-color: magenta;}");
  }
  else{
    th_spin->setStyleSheet("QDoubleSpinBox{}");
  }
}

void PointListPanel::updateValue(int _index)
{
  QLineEdit* name_line = static_cast<QLineEdit*>(point_table_->cellWidget(_index,column::name));
  QDoubleSpinBox* x_spin = static_cast<QDoubleSpinBox*>(point_table_->cellWidget(_index,column::x));
  QDoubleSpinBox* y_spin = static_cast<QDoubleSpinBox*>(point_table_->cellWidget(_index,column::y));
  QDoubleSpinBox* th_spin = static_cast<QDoubleSpinBox*>(point_table_->cellWidget(_index,column::theta));

  setWaypointsList(_index,name_line->text().toStdString(),
      x_spin->value(),y_spin->value(),th_spin->value());
  callService("update",std::to_string(_index),name_line->text().toStdString(),
      x_spin->value(),y_spin->value(),th_spin->value());

  // change background color, if value is changed
  if(_index > wp_list_original_.size() - 1){
    return;
  }
  if(std::round(x_spin->value()*100) != std::round(wp_list_original_.at(_index).pose.position.x*100) ){
    x_spin->setStyleSheet("QDoubleSpinBox{background-color: magenta;}");
  }
  else{
    x_spin->setStyleSheet("QDoubleSpinBox{}");
  }
  if(std::round(y_spin->value()*100) != std::round(wp_list_original_.at(_index).pose.position.y*100) ){
    y_spin->setStyleSheet("QDoubleSpinBox{background-color: magenta;}");
  }
  else{
    y_spin->setStyleSheet("QDoubleSpinBox{}");
  }
  if(std::round(th_spin->value()*100) != std::round(wp_list_original_.at(_index).theta*100) ){
    th_spin->setStyleSheet("QDoubleSpinBox{background-color: magenta;}");
  }
  else{
    th_spin->setStyleSheet("QDoubleSpinBox{}");
  }

}

void PointListPanel::headerClicked(int _column)
{
  bool has_checked = false;
  switch (_column)
  {
    case column::view:
      for(int row = 0; row < point_table_->rowCount();++row){
        has_checked = static_cast<QWidget*>(point_table_->cellWidget(row,column::view))->findChild<QCheckBox *>()->isChecked();
        break;
      }
      if(has_checked){
        for(int row = 0; row < point_table_->rowCount();++row){
          static_cast<QWidget*>(point_table_->cellWidget(row,column::view))->findChild<QCheckBox *>()->setChecked(false);
          static_cast<QWidget*>(point_table_->cellWidget(row,column::edit))->findChild<QCheckBox *>()->setChecked(false);
          updateCheckState(row);
        }
      }
      else{
        for(int row = 0; row < point_table_->rowCount();++row){
          static_cast<QWidget*>(point_table_->cellWidget(row,column::view))->findChild<QCheckBox *>()->setChecked(true);
          updateCheckState(row);
        }
      }
      break;
    case column::edit:
      for(int row = 0; row < point_table_->rowCount();++row){
        has_checked = static_cast<QWidget*>(point_table_->cellWidget(row,column::edit))->findChild<QCheckBox *>()->isChecked();
        break;
      }
      if(has_checked){
        for(int row = 0; row < point_table_->rowCount();++row){
          static_cast<QWidget*>(point_table_->cellWidget(row,column::edit))->findChild<QCheckBox *>()->setChecked(false);
          updateCheckState(row);
        }
      }
      else{
        for(int row = 0; row < point_table_->rowCount();++row){
          if(static_cast<QWidget*>(point_table_->cellWidget(row,column::edit))->findChild<QCheckBox *>()->isEnabled()){
            static_cast<QWidget*>(point_table_->cellWidget(row,column::edit))->findChild<QCheckBox *>()->setChecked(true);
            updateCheckState(row);
          }
        }
      }
      break;
    default:
      break;
  }
}

void PointListPanel::setRowValue(int _row, std::vector<Waypoints>& _wp_list)
{
  QSignalMapper* signal_mapper_check = new QSignalMapper(this);
  QSignalMapper* signal_mapper_value = new QSignalMapper(this);
  QSignalMapper* signal_mapper_description = new QSignalMapper(this);

  // checkbox set
  QCheckBox* check_view = new QCheckBox();
  check_view->setStyleSheet("QCheckBox::indicator { width:30px; height: 30px;}");
  QWidget* check_view_w = new QWidget();
  QHBoxLayout* view_layout = new QHBoxLayout(check_view_w);
  view_layout->addWidget(check_view);
  view_layout->setAlignment(Qt::AlignCenter);
  view_layout->setContentsMargins(0,0,0,0);

  QCheckBox* check_edit = new QCheckBox();
  check_edit->setStyleSheet("QCheckBox::indicator { width:30px; height: 30px;}");
  QWidget* check_edit_w = new QWidget();
  QHBoxLayout* edit_layout = new QHBoxLayout(check_edit_w);
  edit_layout->addWidget(check_edit);
  edit_layout->setAlignment(Qt::AlignCenter);
  edit_layout->setContentsMargins(0,0,0,0);
  check_edit->setEnabled(false);

  // position set
  QLineEdit* name = new QLineEdit();
  QDoubleSpinBox* x_spinbox = new QDoubleSpinBox();
  QDoubleSpinBox* y_spinbox = new QDoubleSpinBox();
  QDoubleSpinBox* theta_spinbox = new QDoubleSpinBox();
  name->setStyleSheet("QLineEdit{}");
  x_spinbox->setStyleSheet("QDoubleSpinBox{}");
  y_spinbox->setStyleSheet("QDoubleSpinBox{}");
  theta_spinbox->setStyleSheet("QDoubleSpinBox{}");
  x_spinbox->setSizePolicy( QSizePolicy( QSizePolicy::Ignored, sizePolicy().verticalPolicy() ) );
  y_spinbox->setSizePolicy( QSizePolicy( QSizePolicy::Ignored, sizePolicy().verticalPolicy() ) );
  theta_spinbox->setSizePolicy( QSizePolicy( QSizePolicy::Ignored, sizePolicy().verticalPolicy() ) );

  name->setEnabled(false);
  x_spinbox->setEnabled(false);
  y_spinbox->setEnabled(false);
  theta_spinbox->setEnabled(false);

  x_spinbox->setMinimum(std::numeric_limits<int32_t>::min());
  x_spinbox->setMaximum(std::numeric_limits<int32_t>::max());
  y_spinbox->setMinimum(std::numeric_limits<int32_t>::min());
  y_spinbox->setMaximum(std::numeric_limits<int32_t>::max());
  theta_spinbox->setRange(-180,180);
  x_spinbox->setSingleStep(0.1);
  y_spinbox->setSingleStep(0.1);
  theta_spinbox->setSingleStep(10);

  point_table_->setCellWidget(_row,column::view,check_view_w);
  point_table_->setCellWidget(_row,column::edit,check_edit_w);
  point_table_->setCellWidget(_row,column::name,name);
  point_table_->setCellWidget(_row,column::x,x_spinbox);
  point_table_->setCellWidget(_row,column::y,y_spinbox);
  point_table_->setCellWidget(_row,column::theta,theta_spinbox);

  name->setText(QString::fromStdString(_wp_list.at(_row).name));
  x_spinbox->setValue(_wp_list.at(_row).pose.position.x);
  y_spinbox->setValue(_wp_list.at(_row).pose.position.y);
  theta_spinbox->setValue(_wp_list.at(_row).theta);

  connect(check_view, SIGNAL(clicked()), signal_mapper_check, SLOT(map()));
  signal_mapper_check->setMapping(check_view,(int)_row);
  connect(check_edit, SIGNAL(clicked()), signal_mapper_check, SLOT(map()));
  signal_mapper_check->setMapping(check_edit,(int)_row);

  connect(name, SIGNAL(editingFinished()), signal_mapper_description, SLOT(map()));
  signal_mapper_description->setMapping(name,(int)_row);

  connect(x_spinbox, SIGNAL(valueChanged(double)), signal_mapper_value, SLOT(map()));
  signal_mapper_value->setMapping(x_spinbox,(int)_row);
  connect(y_spinbox, SIGNAL(valueChanged(double)), signal_mapper_value, SLOT(map()));
  signal_mapper_value->setMapping(y_spinbox,(int)_row);
  connect(theta_spinbox, SIGNAL(valueChanged(double)), signal_mapper_value, SLOT(map()));
  signal_mapper_value->setMapping(theta_spinbox,(int)_row);

  connect(signal_mapper_check,SIGNAL(mapped(int)),this,SLOT(updateCheckState(int)));
  connect(signal_mapper_value,SIGNAL(mapped(int)),this,SLOT(updateValue(int)));
  connect(signal_mapper_description,SIGNAL(mapped(int)),this,SLOT(updateCheckState(int)));

}

void PointListPanel::markerStateCallback(const seed_rviz_plugin::MarkerState& _data)
{
  int row_number = atoi(_data.name.c_str());

  static_cast<QDoubleSpinBox*>(point_table_->cellWidget(row_number,column::x))->setValue(_data.x);
  static_cast<QDoubleSpinBox*>(point_table_->cellWidget(row_number,column::y))->setValue(_data.y);
  static_cast<QDoubleSpinBox*>(point_table_->cellWidget(row_number,column::theta))->setValue(_data.theta);

}

void PointListPanel::loadWayPoints()
{
  std::string map_dir;
  if(client_.getCurrentConfiguration(wp_config_, ros::Duration(0.5))){
    map_dir = wp_config_.dir_name.c_str();
  }
  else{
    map_dir = "";
    QMessageBox::critical(this, tr("Error"), tr("データが見つかりません"));
    return ;
  }

  map_dir_name_editor_->setText( map_dir.c_str() );

  callService("delete");

  std::string file_path = pkg_path_ + map_dir_name_editor_->text().toStdString() + "/waypoints.yaml";

  std::ifstream file(file_path);
  if(!file){
    std::ofstream ofs(file_path);
    ofs.flush();
    ofs.close();
  }

  YAML::Node config = YAML::LoadFile(file_path);
  const YAML::Node &wp_node_tmp = config;
  const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;

  wp_list_original_.clear();
  wp_list_.clear();
  if(wp_node != NULL){
    for(int i=0; i < wp_node->size(); i++){
      Waypoints wp;
      if((*wp_node)[i]["pose"]["name"]){
        wp.name = (*wp_node)[i]["pose"]["name"].as<std::string>();
      }
      else{
        wp.name = "p" + std::to_string(i);
      }
      wp.pose.position.x = (*wp_node)[i]["pose"]["position"]["x"].as<double>();
      wp.pose.position.y = (*wp_node)[i]["pose"]["position"]["y"].as<double>();
      wp.pose.position.z = (*wp_node)[i]["pose"]["position"]["z"].as<double>();

      wp.pose.orientation.x = (*wp_node)[i]["pose"]["orientation"]["x"].as<double>();
      wp.pose.orientation.y = (*wp_node)[i]["pose"]["orientation"]["y"].as<double>();
      wp.pose.orientation.z = (*wp_node)[i]["pose"]["orientation"]["z"].as<double>();
      wp.pose.orientation.w = (*wp_node)[i]["pose"]["orientation"]["w"].as<double>();
      wp.theta = tf::getYaw(wp.pose.orientation) * 180.0 / M_PI;
      wp_list_original_.push_back(wp);
    }
  }
}

void PointListPanel::setWaypointsList(int _index, std::string _description, double _x, double _y, double _theta)
{
  wp_list_.at(_index).name = _description;
  wp_list_.at(_index).pose.position.x = _x;
  wp_list_.at(_index).pose.position.y = _y;
  tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,_theta *( M_PI / 180.0));
  quaternionTFToMsg(quat,wp_list_.at(_index).pose.orientation);
}

bool PointListPanel::callService(std::string _command, std::string _index, std::string _description,
  double _x, double _y, double _theta)
{
  /*
  command : add_int_marker(interactive marker), add_marker(not interactive), delete, update
  */
  wp_manager_srv_.request.command = _command;
  wp_manager_srv_.request.index = _index;
  wp_manager_srv_.request.description = _description;
  wp_manager_srv_.request.x = _x;
  wp_manager_srv_.request.y = _y;
  wp_manager_srv_.request.theta = _theta;
  if(wp_manager_client_.call(wp_manager_srv_)){
    return true;
  }
  else{
    return false;
  }
}

} // end namespace seed_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(seed_rviz_plugin::PointListPanel,rviz::Panel )
