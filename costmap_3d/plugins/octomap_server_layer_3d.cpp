/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Badger Technologies LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: C. Andy Martin
 *********************************************************************/
#include <costmap_3d/octomap_server_layer_3d.h>

namespace costmap_3d
{

OctomapServerLayer3D::OctomapServerLayer3D() : super()
{
}

OctomapServerLayer3D::~OctomapServerLayer3D()
{
}

void OctomapServerLayer3D::updateBounds(const geometry_msgs::Pose robot_pose,
                                        const geometry_msgs::Point& rolled_min,
                                        const geometry_msgs::Point& rolled_max,
                                        Costmap3D* bounds_map)
{
  if (costmap_ && changed_cells_)
  {
    Costmap3DPtr tracked_changes = octomap_server_->getTrackingBounds(name_, costmap_);
    touch(*tracked_changes);
  }
  super::updateBounds(robot_pose, rolled_min, rolled_max, bounds_map);
}

void OctomapServerLayer3D::initialize(LayeredCostmap3D* parent, std::string name, tf::TransformListener *tf)
{
  super::initialize(parent, name, tf);
  // Note that the master costmap's resolution is not set yet.
  // Our setResolution method will be called later once it is set.
}

void OctomapServerLayer3D::deactivate()
{
  super::deactivate();
  octomap_server_.reset();
}

void OctomapServerLayer3D::activate()
{
  octomap_server_.reset(new octomap_server::OctomapServer(ros::NodeHandle("~/" + name_ +
                                                                          "/octomap_server")));
  super::activate();
}

void OctomapServerLayer3D::reset()
{
  super::reset();

  // reset octomap server
  if (octomap_server_)
  {
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    // Direct-call the server. We could call it via ros service, but we have
    // direct access to the object, and a direct call is more efficient.
    octomap_server_->resetSrv(req, res);
  }
}

void OctomapServerLayer3D::resetAABBUnlocked(Costmap3DIndex min, Costmap3DIndex max)
{
  super::resetAABB(min, max);

  if (octomap_server_)
  {
    octomap_server::OctomapServer::BBXSrv::Request req;
    octomap_server::OctomapServer::BBXSrv::Response res;
    req.min = fromOctomapPoint(costmap_->keyToCoord(min));
    req.max = fromOctomapPoint(costmap_->keyToCoord(max));
    // Direct-call the server. We could call it via ros service, but we have
    // direct access to the object, and a direct call is more efficient.
    octomap_server_->eraseBBXSrv(req, res);
  }
};

void OctomapServerLayer3D::matchSize(const geometry_msgs::Point& min, const geometry_msgs::Point& max, double resolution)
{
  super::matchSize(min, max, resolution);
  // XXX set octomap server limits here.
}

}  // namespace costmap_3d
