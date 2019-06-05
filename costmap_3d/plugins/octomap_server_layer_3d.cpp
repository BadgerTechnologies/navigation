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
#include <std_srvs/Empty.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

namespace costmap_3d
{

OctomapServerLayer3D::OctomapServerLayer3D() : super()
{
}

OctomapServerLayer3D::~OctomapServerLayer3D()
{
}

void OctomapServerLayer3D::initialize(LayeredCostmap3D* parent, std::string name, tf::TransformListener *tf)
{
  super::initialize(parent, name, tf);
  // Note that the master costmap's resolution is not set yet.
  // Our setResolution method will be called later once it is set.

  // Get names of topics and servers from parameter server
  pnh_ = ros::NodeHandle("~/" + name);

  std::string octomap_server_ns = pnh_.param("octomap_server_namespace", std::string(""));
  std::string octomap_server_name = pnh_.param("octomap_server_node_name", std::string("octomap_server"));
  std::string topic_prefix = octomap_server_ns;
  if (octomap_server_ns.size() > 0 && octomap_server_ns.rbegin()[0] != '/')
  {
    topic_prefix = octomap_server_ns + "/";
  }
  std::string srv_prefix = topic_prefix + octomap_server_name + "/";

  reset_srv_name_ = srv_prefix + "reset";
  erase_bbx_srv_name_ = srv_prefix + "erase_bbx";
  map_topic_ = pnh_.param("map_topic", topic_prefix + "octomap_binary");
  map_update_topic_ = pnh_.param("map_update_topic", map_topic_ + "_updates");

  ROS_INFO_STREAM("OctomapServerLayer3D " << name << ": Initialized with " <<
                  "map_topic: " << map_topic_ <<
                  "map_update_topic: " << map_update_topic_ <<
                  "reset_srv_name: " << reset_srv_name_ <<
                  "erase_bbx_srv_name: " << erase_bbx_srv_name_);
}

void OctomapServerLayer3D::deactivate()
{
  super::deactivate();
  // Unsubscribe from everything.
  map_sub_.shutdown();
  map_update_sub_.shutdown();
  reset_srv_.shutdown();
  erase_bbx_srv_.shutdown();
}

void OctomapServerLayer3D::activate()
{
  map_sub_ = pnh_.subscribe<octomap_msgs::Octomap>(map_topic_, 10, std::bind(&OctomapServerLayer3D::mapCallback,
                                                                             this, std::placeholders::_1));
  map_update_sub_ = pnh_.subscribe<octomap_msgs::OctomapUpdate>(map_update_topic_, 10,
                                                                std::bind(&OctomapServerLayer3D::mapUpdateCallback,
                                                                          this, std::placeholders::_1));
  reset_srv_ = pnh_.serviceClient<std_srvs::Empty>(reset_srv_name_);
  erase_bbx_srv_ = pnh_.serviceClient<octomap_msgs::BoundingBoxQuery>(erase_bbx_srv_name_);

  // Wait a while for the servers to exist, but not too long.
  // We will handle them not existing below.
  if (!reset_srv_.waitForExistence(ros::Duration(10.0)))
  {
    ROS_WARN_STREAM("OctomapServerLayer3D " << name_ << ": Failed to wait for service " << reset_srv_name_);
  }
  if (!erase_bbx_srv_.waitForExistence(ros::Duration(10.0)))
  {
    ROS_WARN_STREAM("OctomapServerLayer3D " << name_ << ": Failed to wait for service " << erase_bbx_srv_name_);
  }

  super::activate();
}

void OctomapServerLayer3D::reset()
{
  super::reset();

  // reset octomap server
  if (reset_srv_.exists())
  {
    std_srvs::Empty srv;
    reset_srv_.call(srv);
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ <<
                             ": Unable to call octomap server's reset service");
  }
}

void OctomapServerLayer3D::resetAABBUnlocked(Costmap3DIndex min, Costmap3DIndex max)
{
  super::resetAABBUnlocked(min, max);

  if (erase_bbx_srv_.exists())
  {
    octomap_msgs::BoundingBoxQuery srv;
    srv.request.min = fromOctomapPoint(costmap_->keyToCoord(min));
    srv.request.max = fromOctomapPoint(costmap_->keyToCoord(max));
    erase_bbx_srv_.call(srv);
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ <<
                             ": Unable to call octomap server's erase_bbx service");
  }
};

void OctomapServerLayer3D::matchSize(const geometry_msgs::Point& min, const geometry_msgs::Point& max, double resolution)
{
  if (resolution != costmap_->getResolution())
  {
    // Resolution changed, need to request full message.
    // Do this by re-subscribing to the map topic.
    map_sub_.shutdown();
    map_sub_ = pnh_.subscribe<octomap_msgs::Octomap>(map_topic_, 10, std::bind(&OctomapServerLayer3D::mapCallback,
                                                                               this, std::placeholders::_1));
  }
  super::matchSize(min, max, resolution);

  // If octomap server ever exposes its limits via dynamic reconfigure, we
  // could set them here.
}

void OctomapServerLayer3D::mapCallback(const octomap_msgs::OctomapConstPtr& map_msg)
{
  mapUpdateInternal(map_msg.get(), nullptr);
}

void OctomapServerLayer3D::mapUpdateCallback(const octomap_msgs::OctomapUpdateConstPtr& map_update_msg)
{
  mapUpdateInternal(&map_update_msg->octomap_update, &map_update_msg->octomap_bounds);
}

void OctomapServerLayer3D::mapUpdateInternal(const octomap_msgs::Octomap* map_msg,
                                             const octomap_msgs::Octomap* bounds_msg)
{
  // Verify binary, resolution and frame match.
  if (!map_msg->binary)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received non-binary map, ignoring");
  }
  else if (bounds_msg != nullptr && !bounds_msg->binary)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received non-binary bounds map, ignoring");
  }
  else if(costmap_ && std::abs(map_msg->resolution - costmap_->getResolution()) > 1e-6)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received map with resolution " <<
                             map_msg->resolution << " but costmap resolution is " <<
                             costmap_->getResolution() << ", ignoring");
  }
  else if(costmap_ && bounds_msg != nullptr && std::abs(bounds_msg->resolution - costmap_->getResolution()) > 1e-6)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received bounds map with resolution " <<
                             map_msg->resolution << " but costmap resolution is " <<
                             costmap_->getResolution() << ", ignoring");
  }
  else if(map_msg->header.frame_id != layered_costmap_3d_->getGlobalFrameID())
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received map in frame " <<
                             map_msg->header.frame_id << " but global frame is " <<
                             layered_costmap_3d_->getGlobalFrameID() << ", ignoring");
  }
  else if(bounds_msg != nullptr && bounds_msg->header.frame_id != layered_costmap_3d_->getGlobalFrameID())
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received bounds map in frame " <<
                             bounds_msg->header.frame_id << " but global frame is " <<
                             layered_costmap_3d_->getGlobalFrameID() << ", ignoring");
  }
  else if(costmap_)
  {
    std::shared_ptr<octomap::AbstractOcTree> abstract_map(octomap_msgs::binaryMsgToMap(*map_msg));
    octomap::OcTree* map = dynamic_cast<octomap::OcTree*>(abstract_map.get());
    if (map == nullptr)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": unable to dynamic cast received tree");
    }
    else
    {
      std::shared_ptr<octomap::AbstractOcTree> abstract_bounds_map;
      if (bounds_msg != nullptr)
      {
        abstract_bounds_map.reset(octomap_msgs::binaryMsgToMap(*bounds_msg));
      }
      else
      {
        // If no bounds are given, use the whole universe
        Costmap3D* universe_bounds_map = new Costmap3D(costmap_->getResolution());
        universe_bounds_map->setNodeValueAtDepth(Costmap3DIndex(), 0, LETHAL);
        abstract_bounds_map.reset(universe_bounds_map);
      }
      octomap::OcTree* bounds_map = dynamic_cast<octomap::OcTree*>(abstract_bounds_map.get());
      if (bounds_map == nullptr)
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": unable to dynamic cast received bounds");
      }
      else
      {
        // Use zero log odds as the threshold for any incoming binary map
        updateCells(*map, *bounds_map, 0.0);
      }
    }
  }
}

}  // namespace costmap_3d
