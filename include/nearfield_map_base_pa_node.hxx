/******************************************************************************
*                                                                             *
* nearfield_map_base_pa_node.hxx                                              *
* ==============================                                              *
*                                                                             *
*******************************************************************************
*                                                                             *
* github repository                                                           *
*   https://github.com/peterweissig/ros_nearfield_map                         *
*                                                                             *
* Chair of Automation Technology, Technische Universität Chemnitz             *
*   https://www.tu-chemnitz.de/etit/proaut                                    *
*                                                                             *
*******************************************************************************
*                                                                             *
* New BSD License                                                             *
*                                                                             *
* Copyright (c) 2015-2016, Peter Weissig, Technische Universität Chemnitz     *
* All rights reserved.                                                        *
*                                                                             *
* Redistribution and use in source and binary forms, with or without          *
* modification, are permitted provided that the following conditions are met: *
*     * Redistributions of source code must retain the above copyright        *
*       notice, this list of conditions and the following disclaimer.         *
*     * Redistributions in binary form must reproduce the above copyright     *
*       notice, this list of conditions and the following disclaimer in the   *
*       documentation and/or other materials provided with the distribution.  *
*     * Neither the name of the Technische Universität Chemnitz nor the       *
*       names of its contributors may be used to endorse or promote products  *
*       derived from this software without specific prior written permission. *
*                                                                             *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" *
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   *
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  *
* ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY      *
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR          *
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER  *
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT          *
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY   *
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH *
* DAMAGE.                                                                     *
*                                                                             *
******************************************************************************/

// local headers
#include "nearfield_map_base_pa_node.h"

// ros headers
#include <parameter_pa_ros.h>

// standard headers
#include <string>

//**************************[cNearfieldMapBasePaNode]**************************
template <typename NEARFIELDMAP>
  cNearfieldMapBasePaNode<NEARFIELDMAP>::cNearfieldMapBasePaNode() :
  NEARFIELDMAP(0.1), tf_listener_(ros::Duration(20), true) {

    cParameterPaRos paramloader;

    // output throttle
    paramloader.load("~throttle_count"    , output_throttle_.skip_count_);
    paramloader.load("~throttle_time"     , output_throttle_.skip_time_ );

    // output filter (bounding box)
    paramloader.load("~filter_frame", nodeparams_.filter_frame_);
    paramloader.load("~filter_dx"   , nodeparams_.filter_dx_   );
    paramloader.load("~filter_dy"   , nodeparams_.filter_dy_   );
    paramloader.load("~filter_dz"   , nodeparams_.filter_dz_   );
    changeSettings();

    // global octomap parameter
    paramloader.load("~output_frame",
      NEARFIELDMAP::rosparams_base_.output_frame_);

    // octomap parameter
    double temp;

    temp = 0.1 ;
    paramloader.load("~map_resolution"    , temp);
    NEARFIELDMAP::setResolution(temp);

    temp = 0.5 ;
    paramloader.load("~map_prob_threshold", temp);
    NEARFIELDMAP::setOccupancyThres(temp);

    temp = 0.12;
    paramloader.load("~map_clamp_min"     , temp);
    NEARFIELDMAP::setClampingThresMin(temp);

    temp = 0.97;
    paramloader.load("~map_clamp_max"     , temp);
    NEARFIELDMAP::setClampingThresMax(temp);

    // sensor specific parameter
    paramloader.load("~camera_prob_hit"     ,
      params_addcloud_camera_.map_prob_hit_     );
    paramloader.load("~camera_prob_miss"    ,
      params_addcloud_camera_.map_prob_miss_    );
    paramloader.load("~laser_prob_hit"      ,
      params_addcloud_laser_scan_.map_prob_hit_ );
    paramloader.load("~laser_prob_miss"     ,
      params_addcloud_laser_scan_.map_prob_miss_);
    paramloader.load("~laser_full_prob_hit" ,
      params_addcloud_laser_full_.map_prob_hit_ );
    paramloader.load("~laser_full_prob_miss",
      params_addcloud_laser_full_.map_prob_miss_);

    // topics in
    paramloader.load_topic("~topic_in_camera"    ,
      nodeparams_.topic_in_camera_    );
    paramloader.load_topic("~topic_in_laser_scan",
      nodeparams_.topic_in_laser_scan_);
    paramloader.load_topic("~topic_in_laser_full",
      nodeparams_.topic_in_laser_full_);

    // topics out
    paramloader.load_topic("~topic_out_nearfield",
      nodeparams_.topic_out_nearfield_);

    paramloader.load_topic("~topic_out_octomap"        ,
      nodeparams_.topic_out_octomap_);
    paramloader.load_topic("~topic_out_octomap_full"   ,
      nodeparams_.topic_out_octomap_full_);
    paramloader.load_topic("~topic_out_cloud_free"     ,
      nodeparams_.topic_out_cloud_free_);
    paramloader.load_topic("~topic_out_cloud_occupied" ,
      nodeparams_.topic_out_cloud_occupied_);

    // services as topic
    paramloader.load_topic("~topic_in_clear" , nodeparams_.topic_in_clear_ );


    // puplisher for the octomap (binary data)
    pub_octomap_         = nh_.advertise<octomap_msgs::Octomap>(
      nodeparams_.topic_out_octomap_, 2, true);
    // puplisher for the octomap (full data)
    pub_octomap_full_    = nh_.advertise<octomap_msgs::Octomap>(
      nodeparams_.topic_out_octomap_full_, 2, true);

    // puplisher for free voxels as pointcloud
    pub_cloud_free_      = nh_.advertise<sensor_msgs::PointCloud2>(
      nodeparams_.topic_out_cloud_free_, 2, true);
    // puplisher for occupied voxels as pointcloud
    pub_cloud_occupied_  = nh_.advertise<sensor_msgs::PointCloud2>(
      nodeparams_.topic_out_cloud_occupied_, 2, true);
    // puplisher for requested nearfield map (pointcloud)
    pub_nearfield_       = nh_.advertise<sensor_msgs::PointCloud2>(
      nodeparams_.topic_out_nearfield_, 2, true);

    // subscriber for clearing the octomap
    sub_clear_ = nh_.subscribe<std_msgs::Empty>(
      nodeparams_.topic_in_clear_, 1,
      &cNearfieldMapBasePaNode::clearCallbackSub, this);

    std::string str_service("~");
    paramloader.resolve_ressourcename(str_service);
    // service for clearing the octomap
    srv_clear_   = nh_.advertiseService(str_service + "clear",
      &cNearfieldMapBasePaNode::clearCallbackSrv, this);
    // service for receiving the size of the octomap
    srv_getsize_ = nh_.advertiseService(str_service + "getsize",
      &cNearfieldMapBasePaNode::getSizeCallbackSrv, this);
    // service for saving the octomap
    srv_save_    = nh_.advertiseService(str_service + "save",
      &cNearfieldMapBasePaNode::saveCallbackSrv, this);
    // service for loading a octomap
    srv_load_    = nh_.advertiseService(str_service + "load",
      &cNearfieldMapBasePaNode::loadCallbackSrv, this);
    // service for requesting a nearfield map (pointcloud) via output topic
    srv_send_ = nh_.advertiseService(str_service + "send",
      &cNearfieldMapBasePaNode::sendCallbackSrv, this);
    // service for requesting a nearfield map (pointcloud) via this service
    srv_request_ = nh_.advertiseService(str_service + "request",
      &cNearfieldMapBasePaNode::requestCallbackSrv, this);
    // service for changing parameters of the nearfield map
    srv_change_settings_ = nh_.advertiseService(
      str_service + "change_settings",
      &cNearfieldMapBasePaNode::changeSettingsCallbackSrv, this);

    // count number of inserted pointclouds
    count_camera_     = 0;
    count_laser_scan_ = 0;
    count_laser_full_ = 0;
}

//**************************[~cNearfieldMapBasePaNode]*************************
template <typename NEARFIELDMAP>
  cNearfieldMapBasePaNode<NEARFIELDMAP>::~cNearfieldMapBasePaNode() {

}

//**************************[publish]******************************************
template <typename NEARFIELDMAP>
  void cNearfieldMapBasePaNode<NEARFIELDMAP>::publish() {

    if (! output_throttle_.checkNewInput(NEARFIELDMAP::getOutputTime())) {
        return;
    }

    if (pub_nearfield_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2Ptr msg;
        if (getNearfield(msg)) {
            pub_nearfield_.publish(msg);
        }
    }

    if (pub_octomap_.getNumSubscribers() > 0) {
        pub_octomap_.publish(NEARFIELDMAP::getOctomap());
    }
    if (pub_octomap_full_.getNumSubscribers() > 0) {
        pub_octomap_.publish(NEARFIELDMAP::getOctomapFull());
    }

    if (pub_cloud_occupied_.getNumSubscribers() > 0) {
        pub_cloud_occupied_.publish(NEARFIELDMAP::getOctomapPcd(
          NEARFIELDMAP::tree_depth, true));
    }
    if (pub_cloud_free_.getNumSubscribers() > 0) {
        pub_cloud_free_.publish(NEARFIELDMAP::getOctomapPcdFree(
          NEARFIELDMAP::tree_depth, true));
    }
}

//**************************[getNearfield]*************************************
template <typename NEARFIELDMAP>
  bool cNearfieldMapBasePaNode<NEARFIELDMAP>::getNearfield(
  sensor_msgs::PointCloud2Ptr &msg) {

    if (! output_filter_.updateTf(tf_listener_,
      NEARFIELDMAP::rosparams_base_.output_frame_,
      NEARFIELDMAP::getOutputTime())) {
        return false;
    }

    return output_filter_.filterCloud(
      NEARFIELDMAP::getOctomapPcd(NEARFIELDMAP::tree_depth, true), msg);
}

//**************************[checkTF]******************************************
template <typename NEARFIELDMAP>
  bool cNearfieldMapBasePaNode<NEARFIELDMAP>::checkTF(
  const std_msgs::Header &header, tf::StampedTransform &transform) {

    try {
        tf_listener_.waitForTransform(
          NEARFIELDMAP::rosparams_base_.output_frame_,
          header.frame_id, header.stamp, ros::Duration(0.2));
        tf_listener_.lookupTransform(
          NEARFIELDMAP::rosparams_base_.output_frame_,
          header.frame_id, header.stamp, transform);
    } catch (tf::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return false;
    }

    return true;
}

//**************************[clearCallbackSub]*********************************
template <typename NEARFIELDMAP>
  void cNearfieldMapBasePaNode<NEARFIELDMAP>::clearCallbackSub(
  const std_msgs::EmptyConstPtr &msg) {

    ROS_INFO("cNearfieldMapBasePaNode::clear()");

    count_camera_     = 0;
    count_laser_scan_ = 0;
    count_laser_full_ = 0;

    NEARFIELDMAP::clear();
}

//**************************[clearCallbackSrv]*********************************
template <typename NEARFIELDMAP>
  bool cNearfieldMapBasePaNode<NEARFIELDMAP>::clearCallbackSrv(
  std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

    clearCallbackSub(std_msgs::EmptyConstPtr(new std_msgs::Empty));

    return true;
}

//**************************[getSizeCallbackSrv]*******************************
template <typename NEARFIELDMAP>
  bool cNearfieldMapBasePaNode<NEARFIELDMAP>::getSizeCallbackSrv (
  nearfield_map::NearfieldMapGetSize::Request  &req,
  nearfield_map::NearfieldMapGetSize::Response &res) {

    ROS_INFO("cNearfieldMapBasePaNode::getsize()");

    res.size = NEARFIELDMAP::size();
    res.memoryusage = (int64_t) NEARFIELDMAP::memoryUsage();

    res.count_camera     = count_camera_    ;
    res.count_laser_scan = count_laser_scan_;
    res.count_laser_full = count_laser_full_;

    return true;
}

//**************************[saveCallbackSrv]**********************************
template <typename NEARFIELDMAP>
  bool cNearfieldMapBasePaNode<NEARFIELDMAP>::saveCallbackSrv(
  nearfield_map::NearfieldMapFileName::Request  &req,
  nearfield_map::NearfieldMapFileName::Response &res) {

    ROS_INFO_STREAM("cNearfieldMapBasePaNode::save(" << req.filename << ")");

    std::string filename;
    filename = req.filename;
    cParameterPaRos par;
    par.replace_findpack(filename);
    res.ok = NEARFIELDMAP::write(filename);

    return res.ok;
}

//**************************[loadCallbackSrv]**********************************
template <typename NEARFIELDMAP>
  bool cNearfieldMapBasePaNode<NEARFIELDMAP>::loadCallbackSrv(
  nearfield_map::NearfieldMapFileName::Request  &req,
  nearfield_map::NearfieldMapFileName::Response &res) {

    ROS_INFO_STREAM("cNearfieldMapBasePaNode::load(" << req.filename << ")");

    std::string filename;
    filename = req.filename;
    cParameterPaRos par;
    par.replace_findpack(filename);

    res.ok = NEARFIELDMAP::readFull(filename);

    publish();

    return res.ok;
}

//**************************[sendCallbackSrv]**********************************
template <typename NEARFIELDMAP>
  bool cNearfieldMapBasePaNode<NEARFIELDMAP>::sendCallbackSrv(
  std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

    output_throttle_.resetThrottle();
    publish();

    return true;
}

//**************************[requestCallbackSrv]*******************************
template <typename NEARFIELDMAP>
  bool cNearfieldMapBasePaNode<NEARFIELDMAP>::requestCallbackSrv(
  nearfield_map::NearfieldMapRequest::Request  &req,
  nearfield_map::NearfieldMapRequest::Response &res) {

    sensor_msgs::PointCloud2Ptr result;
    if (getNearfield(result)) {
        sensor_msgs::PointCloud2_<std::allocator<void> >::_data_type temp;
        temp.swap(result->data);
        res.cloud_unfiltered = *result;
        res.cloud_unfiltered.data.swap(temp);
    }

    return true;
}

//**************************[changeSettingsCallbackSrv]************************
template <typename NEARFIELDMAP>
  bool cNearfieldMapBasePaNode<NEARFIELDMAP>::changeSettingsCallbackSrv(
  nearfield_map::NearfieldMapChangeSettings::Request  &req,
  nearfield_map::NearfieldMapChangeSettings::Response &res) {

    ROS_INFO_STREAM("cNearfieldMapBasePaNode::changeSettings("
      << "throttle: " << req.throttle_count << ", " << req.throttle_time
      << "; filter: " << req.filter_dx << ", " << req.filter_dy
      << ", " << req.filter_dz << ", \"" << req.filter_frame << "\")");

    output_throttle_.skip_count_ = req.throttle_count;
    output_throttle_.skip_time_  = req.throttle_time;

    nodeparams_.filter_frame_ = req.filter_frame;
    nodeparams_.filter_dx_    = req.filter_dx   ;
    nodeparams_.filter_dy_    = req.filter_dy   ;
    nodeparams_.filter_dz_    = req.filter_dz   ;

    changeSettings();

    return true;
}

//**************************[changeSettings]***********************************
template <typename NEARFIELDMAP>
  void cNearfieldMapBasePaNode<NEARFIELDMAP>::changeSettings() {

    output_throttle_.block_all_input_ =
      (output_throttle_.skip_count_ < 0) ||
      (output_throttle_.skip_time_ < 0);

    std::stringstream ss;

    ss << "!block: "
      << nodeparams_.filter_dx_ << " "
      << nodeparams_.filter_dy_ << " "
      << nodeparams_.filter_dz_ << " "
      << nodeparams_.filter_frame_;

    output_filter_.filters_.clear();
    output_filter_.filters_.add(ss.str());
}
