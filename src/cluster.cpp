/*
 * Copyright (c) 2020, Robobrain.
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

/* Author: Konstantinos Konstantinidis */

#include "cluster.hpp"

Cluster::Cluster(unsigned long int id, const pointList& new_points, const double& dt, const std::string& world_frame, const tf::Transform& ego_pose){

  this->id = id;
  this->r = rand() / double(RAND_MAX);
  this->g = rand() / double(RAND_MAX);
  this->b = rand() / double(RAND_MAX);
  a = 1.0;
  age = 1;
  frame_name = world_frame;

  new_cluster = new_points;

  ego_coordinates.first = ego_pose.getOrigin().getX();
  ego_coordinates.second= ego_pose.getOrigin().getY();


  calcMean(new_points);
  previous_mean_values = mean_values;

  LshapeTracker l_shape_tracker_ukf(mean_values.first, mean_values.second, dt);
  this->Lshape = l_shape_tracker_ukf;
  Lshape.BoxModel(cx, cy, cvx, cvy);
  
}

  double Cluster::distanceFromEgoRobot(){
    return( pow(pow(cx - ego_coordinates.first,2.0)+pow(cy - ego_coordinates.second,2.0),0.5));
  }

  double Cluster::speed(){
    return( pow(pow(cvx,2.0) + pow(cvy,2.0),0.5));
  }

void Cluster::update(const pointList& new_points, const double dt, const tf::Transform& ego_pose) {

  ego_coordinates.first = ego_pose.getOrigin().getX();
  ego_coordinates.second= ego_pose.getOrigin().getY();

  age++;
  previous_mean_values = mean_values;
  new_cluster = new_points;
  
  calcMean(new_points);

  Lshape.update(mean_values.first, mean_values.second, dt);
  Lshape.BoxModel(cx, cy, cvx, cvy);

}

geometry_msgs::Pose Cluster::getPose(){

  geometry_msgs::Pose msg;
  msg.position.x = cx;
  msg.position.y = cy;

  //quaternion.setRPY(0, 0, psi);
  //msg.orientation = tf2::toMsg(quaternion);
  return msg;

}

void Cluster::calcMean(const pointList& c){

  double sum_x = 0, sum_y = 0;

  for(unsigned int i = 0; i<c.size(); ++i){

    sum_x = sum_x + c[i].first;
    sum_y = sum_y + c[i].second;
  }

    this->mean_values.first = sum_x / c.size();
    this->mean_values.second= sum_y / c.size();
}

visualization_msgs::Marker Cluster::getArrowVisualisationMessage() {

  visualization_msgs::Marker arrow_marker;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  //arrow_marker.header.frame_id = frame_name;
  arrow_marker.header.frame_id = frame_name;
  arrow_marker.header.stamp = ros::Time::now();
  arrow_marker.ns = "velocities";
  arrow_marker.action = visualization_msgs::Marker::ADD;
  arrow_marker.color.a = 1.0;
  arrow_marker.color.g = g;
  arrow_marker.color.b = b;
  arrow_marker.color.r = r;
  arrow_marker.id = this->id;
  arrow_marker.scale.x = 0.05;    //Shaft diameter of the arrow
  arrow_marker.scale.y = 0.1;    //Head  diameter of the arrow

  geometry_msgs::Point p;
  p.x = cx; 
  p.y = cy; 
  p.z = 0;
  arrow_marker.points.push_back(p);

  p.x = cx + cvx *1; 
  p.y = cy + cvy *1; 
  p.z = 0;
  arrow_marker.points.push_back(p);
  return arrow_marker;
}
visualization_msgs::Marker Cluster::getClusterVisualisationMessage() {

  visualization_msgs::Marker cluster_vmsg;
  cluster_vmsg.header.frame_id  = frame_name;
  cluster_vmsg.header.stamp = ros::Time::now();
  cluster_vmsg.ns = "clusters";
  cluster_vmsg.action = visualization_msgs::Marker::ADD;
  cluster_vmsg.pose.orientation.w = 1.0;
  cluster_vmsg.type = visualization_msgs::Marker::POINTS;
  cluster_vmsg.scale.x = 0.02;
  cluster_vmsg.scale.y = 0.02;
  cluster_vmsg.id = this->id;

  cluster_vmsg.color.g = this->g;
  cluster_vmsg.color.b = this->b;
  cluster_vmsg.color.r = this->r;
  cluster_vmsg.color.a = 1.0;


  geometry_msgs::Point p;
 
  for(unsigned int j=0; j<new_cluster.size(); ++j){
    p.x = new_cluster[j].first;
    p.y = new_cluster[j].second;
    p.z = 0;
    cluster_vmsg.points.push_back(p);
  }

  return cluster_vmsg;
}
