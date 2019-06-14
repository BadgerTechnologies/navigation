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
#include <costmap_3d/layered_costmap_3d.h>
#include <costmap_3d/costmap_3d_ros.h>
#include <tf/transform_datatypes.h>

namespace costmap_3d
{

Costmap3DROS::Costmap3DROS(std::string name, tf::TransformListener& tf) :
    super(name, tf),
    initialized_(false),
    plugin_loader_("costmap_3d", "costmap_3d::Layer3D")
{
  std::lock_guard<std::mutex> initialize_lock(initialized_mutex_);
  ros::NodeHandle private_nh("~/" + name + "/");

  layered_costmap_3d_.reset(new LayeredCostmap3D(layered_costmap_));

  if (private_nh.hasParam("costmap_3d/plugins"))
  {
    XmlRpc::XmlRpcValue my_list;
    private_nh.getParam("costmap_3d/plugins", my_list);
    for (int32_t i = 0; i < my_list.size(); ++i)
    {
      std::string pname = static_cast<std::string>(my_list[i]["name"]);
      std::string type = static_cast<std::string>(my_list[i]["type"]);
      ROS_INFO("Using 3D plugin \"%s\"", pname.c_str());

      boost::shared_ptr<Layer3D> plugin = plugin_loader_.createInstance(type);
      plugin->initialize(layered_costmap_3d_.get(), name + "/costmap_3d/" + pname, &tf_);
      layered_costmap_3d_->addPlugin(plugin);
    }
  }

  publisher_.reset(new Costmap3DPublisher(private_nh, layered_costmap_3d_.get(), "costmap_3d"));

  dsrv_.reset(new dynamic_reconfigure::Server<Costmap3DConfig>(ros::NodeHandle("~/" + name + "/costmap_3d")));
  dynamic_reconfigure::Server<Costmap3DConfig>::CallbackType cb = std::bind(&Costmap3DROS::reconfigureCB,
                                                                            this,
                                                                            std::placeholders::_1,
                                                                            std::placeholders::_2);
  dsrv_->setCallback(cb);
  initialized_ = true;
}

Costmap3DROS::~Costmap3DROS()
{
  // Publisher must be freed before the 3D costmap
  publisher_.reset();
}

void Costmap3DROS::reconfigureCB(Costmap3DConfig &config, uint32_t level)
{
  geometry_msgs::Point min, max;

  // Get the x/y values from the 2D costmap
  if (layered_costmap_3d_->isRolling())
  {
    // If we are rolling, set the origin to zero, as we will shift the limits
    // based on the base position ourselves in updateMap, since the octomap is
    // always originated at (0, 0, 0).
    min.x = 0.0;
    min.y = 0.0;
  }
  else
  {
    // If we are not rolling, use the 2D origin as the minimum.
    min.x = layered_costmap_->getCostmap()->getOriginX();
    min.y = layered_costmap_->getCostmap()->getOriginY();
  }
  max.x = min.x + layered_costmap_->getCostmap()->getSizeInMetersX();
  max.y = min.y + layered_costmap_->getCostmap()->getSizeInMetersY();
  min.z = config.map_z_min;
  max.z = config.map_z_max;

  layered_costmap_3d_->setBounds(min, max);
  layered_costmap_3d_->setResolution();

  footprint_3d_padding_ = config.footprint_3d_padding;
}

void Costmap3DROS::updateMap()
{
  std::lock_guard<std::mutex> initialize_lock(initialized_mutex_);

  if (initialized_ && !isPaused())
  {
    // First update the 3D map.
    // We update 3D first, in case any 3D layers need to affect 2D layers.
    // Get global pose of robot
    tf::Stamped < tf::Pose > pose;
    if (getRobotPose (pose))
    {
      geometry_msgs::Pose pose_msg;
      tf::poseTFToMsg(pose, pose_msg);
      layered_costmap_3d_->updateMap(pose_msg);
    }

    // Now update the 2D map
    super::updateMap();
  }
}

void Costmap3DROS::start()
{
  // check if we're stopped or just paused
  if (isStopped())
  {
    layered_costmap_3d_->activate();
  }

  super::start();
}

void Costmap3DROS::stop()
{
  super::stop();
  layered_costmap_3d_->deactivate();
}

void Costmap3DROS::resetLayers()
{
  layered_costmap_3d_->reset();
  super::resetLayers();
}

void Costmap3DROS::lock()
{
  super::lock();
  layered_costmap_3d_->lock();
}

void Costmap3DROS::unlock()
{
  layered_costmap_3d_->unlock();
  super::unlock();
}

std::vector<std::string> Costmap3DROS::getLayerNames()
{
  std::lock_guard<LayeredCostmap3D> lock(*layered_costmap_3d_);
  std::vector<std::string> plugin_names;

  for (auto plugin : layered_costmap_3d_->getPlugins())
  {
    plugin_names.push_back(plugin->getName());
  }

  return plugin_names;
}

void Costmap3DROS::clearAABB(geometry_msgs::Point min, geometry_msgs::Point max)
{
  // No need to reset the master map, as the next update will pull in the changes to each layer.
  clearAABB(min, max, getLayerNames());
}

void Costmap3DROS::clearAABB(geometry_msgs::Point min, geometry_msgs::Point max, const std::vector<std::string>& layers)
{
  std::lock_guard<LayeredCostmap3D> lock(*layered_costmap_3d_);

  for (auto plugin : layered_costmap_3d_->getPlugins())
  {
    // Only clear layers that are in the layers list argument.
    // This is not super efficient if the number of layers is large.
    // If the number of layers ever became large, we should store an index to
    // them by name.
    if (std::find(layers.begin(), layers.end(), plugin->getName()) != layers.end())
    {
      plugin->resetAABB(min, max);
    }
  }
}

double Costmap3DROS::footprintCost(geometry_msgs::Pose pose, double padding)
{
  // TODO: implement
  return INFINITY;
}

bool Costmap3DROS::footprintCollision(geometry_msgs::Pose pose, double padding)
{
  // TODO: implement
  return true;
}

double Costmap3DROS::footprintDistance(geometry_msgs::Pose pose, double padding)
{
  // TODO: implement
  return 0.0;
}

double Costmap3DROS::footprintSignedDistance(geometry_msgs::Pose pose, double padding)
{
  // TODO: implement
  return 0.0;
}

}  // namespace costmap_3d
