/******************************************************************************
*                                                                             *
* nearfield_map_pa_node_parameter.cpp                                         *
* ===================================                                         *
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
* Copyright (c) 2015-2017, Peter Weissig, Technische Universität Chemnitz     *
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
#include "nearfield_map_pa_node_parameter.h"

#include <string>

//**************************[cNearfieldMapPaNodeParameter]*********************
cNearfieldMapPaNodeParameter::cNearfieldMapPaNodeParameter() {
    // topics
    topic_in_camera_     = "~/in_camera"    ;
    topic_in_laser_scan_ = "~/in_laser_scan";
    topic_in_laser_full_ = "~/in_laser_full";

    topic_out_nearfield_ = "~/out_nearfield";

    topic_out_octomap_      = "~/out_octomap"     ;
    topic_out_octomap_full_ = "~/out_octomap_full";

    topic_out_cloud_free_      = "~/out_cloud_free"     ;
    topic_out_cloud_occupied_  = "~/out_cloud_occupied" ;

    // services
    topic_in_clear_          = "~/in_clear"       ;

    // bounding box for output
    filter_frame_ = "base_footprint";
    filter_dx_ = 5;
    filter_dy_ = 5;
    filter_dz_ = 3;

}
