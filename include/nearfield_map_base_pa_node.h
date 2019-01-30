/******************************************************************************
*                                                                             *
* nearfield_map_base_pa_node.h                                                *
* ============================                                                *
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

#ifndef __NEARFIELD_MAP_BASE_PA_NODE_H
#define __NEARFIELD_MAP_BASE_PA_NODE_H

// local headers
#include "nearfield_map_pa_node_parameter.h"

#include "nearfield_map/NearfieldMapFileName.h"
#include "nearfield_map/NearfieldMapGetSize.h"
#include "nearfield_map/NearfieldMapChangeSettings.h"
#include "nearfield_map/NearfieldMapRequest.h"

// ros headers
#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
//#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
//#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>

#include <octomap_msgs/Octomap.h>

#include <octomap_pa/addcloud_parameter.h>
#include <parameter_pa/parameter_pa_ros.h>
#include <pcdfilter_pa/pcdfilter_pa_ros.h>
#include <pcdfilter_pa/pcdfilter_pa_ros_throttle.h>


// standard headers
#include <string>
#include <vector>

//**************************[cNearfieldMapBasePaNode]**************************
template <typename NEARFIELDMAP>
  class cNearfieldMapBasePaNode : public NEARFIELDMAP {
  public:

    typedef NEARFIELDMAP                          MapTypeBase;
    typedef cNearfieldMapBasePaNode<NEARFIELDMAP> MapTypeFull;

    //! default constructor
    cNearfieldMapBasePaNode();

    //! default destructor
    ~cNearfieldMapBasePaNode();

    //! function for publishing all topics (nearfield map and debugging)
    void publish(void);

    //! function for retrieving requested nearfield map as pointcloud
    bool getNearfield(sensor_msgs::PointCloud2Ptr &msg);

    //! checking if transform from header to octomap frame (output_frame_)
    //! is possible and returns fransform
    bool checkTF(const std_msgs::Header &header,
      tf::StampedTransform &transform);

    void changeSettings(void);

  protected:

    //! parameters
    cNearfieldMapPaNodeParameter nodeparams_;

    //! parameter for insertion of depth images from camera
    cAddCloudParameter params_addcloud_camera_;
    //! parameter for insertion of single laserscans
    cAddCloudParameter params_addcloud_laser_scan_;
    //! parameter for insertion of full laserscans
    cAddCloudParameter params_addcloud_laser_full_;


    //! number of inserted depth images from camera (pointclouds)
    int count_camera_;
    //! number of inserted single laserscans
    int count_laser_scan_;
    //! number of inserted full laserscans (pointclouds)
    int count_laser_full_;

    //! node handler for topic subscription and advertising
    ros::NodeHandle nh_;

    //! transformation of different frames
    tf::TransformListener tf_listener_;

    // output throttle for auto-update
    cPcdFilterPaRosThrottle output_throttle_;

    // output filter
    cPcdFilterPaRos output_filter_;

    //! subscriber for depth images from camera
    ros::Subscriber sub_camera_;
    //! subscriber for a laserscan
    ros::Subscriber sub_laser_scan_;
    //! subscriber for single laserscans
    ros::Subscriber sub_laser_full_;

    //! puplisher for the octomap (binary data)
    ros::Publisher pub_octomap_;
    //! puplisher for the octomap (full data)
    ros::Publisher pub_octomap_full_;
    //! puplisher for free voxels as pointcloud
    ros::Publisher pub_cloud_free_;
    //! puplisher for occupied voxels as pointcloud
    ros::Publisher pub_cloud_occupied_;
    //! puplisher for requested nearfield map (pointcloud)
    ros::Publisher pub_nearfield_;


    //! subscriber for clearing the octomap
    ros::Subscriber sub_clear_;
    void clearCallbackSub(const std_msgs::EmptyConstPtr &msg);

    //! service for clearing the octomap
    ros::ServiceServer srv_clear_;
    bool clearCallbackSrv(std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res);

    //! service for receiving the size of the octomap
    ros::ServiceServer srv_getsize_;
    bool getSizeCallbackSrv(
        nearfield_map::NearfieldMapGetSize::Request  &req,
        nearfield_map::NearfieldMapGetSize::Response &res);

    //! service for saving the octomap
    ros::ServiceServer srv_save_;
    bool saveCallbackSrv(
        nearfield_map::NearfieldMapFileName::Request  &req,
        nearfield_map::NearfieldMapFileName::Response &res);
    //! service for loading a octomap
    ros::ServiceServer srv_load_;
    bool loadCallbackSrv(
        nearfield_map::NearfieldMapFileName::Request  &req,
        nearfield_map::NearfieldMapFileName::Response &res);

    //! service for requesting a nearfield map (pointcloud) via output topic
    ros::ServiceServer srv_send_;
    bool sendCallbackSrv(std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res);
    //! service for requesting a nearfield map (pointcloud) via this service
    ros::ServiceServer srv_request_;
    bool requestCallbackSrv(
        nearfield_map::NearfieldMapRequest::Request  &req,
        nearfield_map::NearfieldMapRequest::Response &res);

    //! service for changing parameters of the nearfield map
    ros::ServiceServer srv_change_settings_;
    bool changeSettingsCallbackSrv(
        nearfield_map::NearfieldMapChangeSettings::Request  &req,
        nearfield_map::NearfieldMapChangeSettings::Response &res);

};

#include "nearfield_map_base_pa_node.hxx"

#endif // __NEARFIELD_MAP_BASE_PA_NODE_H
