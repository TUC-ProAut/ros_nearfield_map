/******************************************************************************
*                                                                             *
* nearfield_map_pa_node.cpp                                                   *
* =========================                                                   *
*                                                                             *
*******************************************************************************
*                                                                             *
* github repository                                                           *
*   https://github.com/TUC-ProAut/ros_nearfield_map                           *
*                                                                             *
* Chair of Automation Technology, Technische Universität Chemnitz             *
*   https://www.tu-chemnitz.de/etit/proaut                                    *
*                                                                             *
*******************************************************************************
*                                                                             *
* New BSD License                                                             *
*                                                                             *
* Copyright (c) 2015-2019, Peter Weissig, Technische Universität Chemnitz     *
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
#include "nearfield_map_pa_node.h"

// ros headers
#include <parameter_pa/parameter_pa_ros.h>

// standard headers
#include <string>

//**************************[main]*********************************************
int main(int argc, char **argv) {

    ros::init(argc, argv, "nearfield_map_pa_node");
    cNearfieldMapPaNode octomap;

    ros::spin();

    return 0;
}

//**************************[cNearfieldMapPaNode]******************************
cNearfieldMapPaNode::cNearfieldMapPaNode() {

    cParameterPaRos paramloader;

    // Subscriber for depth images from camera
    if (nodeparams_.topic_in_camera_ != "") {
        sub_camera_ = nh_.subscribe<sensor_msgs::PointCloud2>(
          nodeparams_.topic_in_camera_, 1,
          &cNearfieldMapPaNode::addPcdCameraCallbackSub, this);
    }
    // Subscriber for single laserscans
    if (nodeparams_.topic_in_laser_scan_ != "") {
        sub_laser_scan_ = nh_.subscribe<sensor_msgs::PointCloud2>(
          nodeparams_.topic_in_laser_scan_, 10,
          &cNearfieldMapPaNode::addPcdLaserScanCallbackSub, this);
    }
    // Subscriber for full laserscans
    if (nodeparams_.topic_in_laser_full_ != "") {
        sub_laser_full_ = nh_.subscribe<sensor_msgs::PointCloud2>(
          nodeparams_.topic_in_laser_full_, 1,
          &cNearfieldMapPaNode::addPcdLaserFullCallbackSub, this);
    }
}

//**************************[~cNearfieldMapPaNode]*****************************
cNearfieldMapPaNode::~cNearfieldMapPaNode() {

}

//**************************[addPcdCameraCallbackSub]**************************
void cNearfieldMapPaNode::addPcdCameraCallbackSub(
  const sensor_msgs::PointCloud2ConstPtr &msg) {

    if (!updateTime(msg->header.stamp)) {
        tf_listener_.clear();
        return;
    }

    tf::StampedTransform transform;
    if (! checkTF(msg->header, transform)) { return;}

    if (addCloud(msg, params_addcloud_camera_, transform)) {
        count_camera_++;
        publish();
    }
}

//**************************[addPcdLaserScanCallbackSub]***********************
void cNearfieldMapPaNode::addPcdLaserScanCallbackSub(
  const sensor_msgs::PointCloud2ConstPtr &msg) {

    if (!updateTime(msg->header.stamp)) {
        tf_listener_.clear();
        return;
    }

    tf::StampedTransform transform;
    if (! checkTF(msg->header, transform)) { return;}

    if (addCloud(msg, params_addcloud_laser_scan_, transform)) {
        count_laser_scan_++;
        publish();
    }
}

//**************************[addPcdLaserFullCallbackSub]***********************
void cNearfieldMapPaNode::addPcdLaserFullCallbackSub(
  const sensor_msgs::PointCloud2ConstPtr &msg) {

    if (!updateTime(msg->header.stamp)) {
        tf_listener_.clear();
        return;
    }

    tf::StampedTransform transform;
    if (! checkTF(msg->header, transform)) { return;}

    if (addCloud(msg, params_addcloud_laser_full_, transform)) {
        count_laser_full_++;
        publish();
    }
}
