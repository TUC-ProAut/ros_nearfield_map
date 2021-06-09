/******************************************************************************
*                                                                             *
* nearfield_map_degrading_native_node.h                                       *
* =====================================                                       *
*                                                                             *
*******************************************************************************
*                                                                             *
* Repository:                                                                 *
*   https://github.com/TUC-ProAut/ros_nearfield_map                           *
*                                                                             *
* Chair of Automation Technology, Technische Universität Chemnitz             *
*   https://www.tu-chemnitz.de/etit/proaut                                    *
*                                                                             *
* Author:                                                                     *
*   Peter Weissig                                                             *
*                                                                             *
*******************************************************************************
*                                                                             *
* New BSD License                                                             *
*                                                                             *
* Copyright (c) 2015-2021 TU Chemnitz                                         *
* All rights reserved.                                                        *
*                                                                             *
* Redistribution and use in source and binary forms, with or without          *
* modification, are permitted provided that the following conditions are met: *
*    * Redistributions of source code must retain the above copyright notice, *
*      this list of conditions and the following disclaimer.                  *
*    * Redistributions in binary form must reproduce the above copyright      *
*      notice, this list of conditions and the following disclaimer in the    *
*      documentation and/or other materials provided with the distribution.   *
*    * Neither the name of the copyright holder nor the names of its          *
*      contributors may be used to endorse or promote products derived from   *
*      this software without specific prior written permission.               *
*                                                                             *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS         *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR  *
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR           *
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,       *
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,         *
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,    *
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     *
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF      *
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                  *
*                                                                             *
******************************************************************************/

#ifndef __NEARFIELD_MAP_DEGRADING_NATIVE_NODE_H
#define __NEARFIELD_MAP_DEGRADING_NATIVE_NODE_H

// local headers
#include "nearfield_map_base_pa_node.h"

#include "nearfield_map/NearfieldMapGetSize.h"

// ros headers
#include <ros/ros.h>

#include <octomap_pa/octree_stamped_native_ros.h>


//**************************[cNearfieldMapDegradingNativeNode]*****************
class cNearfieldMapDegradingNativeNode : public
  cNearfieldMapBasePaNode<cOctreeStampedNativeRos> {
  public:
    //! default constructor
    cNearfieldMapDegradingNativeNode();

    //! default destructor
    ~cNearfieldMapDegradingNativeNode();

  protected:

    //! callback function for receiving a depth images from camera
    void addPcdCameraCallbackSub(
        const sensor_msgs::PointCloud2ConstPtr &msg);
    //! callback function for receiving a laserscan
    void addPcdLaserScanCallbackSub(
        const sensor_msgs::PointCloud2ConstPtr &msg);
    //! callback function for single laserscans
    void addPcdLaserFullCallbackSub(
        const sensor_msgs::PointCloud2ConstPtr &msg);

    //! service for receiving the size of the octomap
    bool getSizeCallbackSrv(
        nearfield_map::NearfieldMapGetSize::Request  &req,
        nearfield_map::NearfieldMapGetSize::Response &res);
};

#endif // __NEARFIELD_MAP_DEGRADING_NATIVE_NODE_H
