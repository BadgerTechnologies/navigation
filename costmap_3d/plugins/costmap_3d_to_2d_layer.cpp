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
#include "costmap_3d/GenericPluginConfig.h"
#include <costmap_3d/costmap_3d_to_2d_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_3d::Costmap3DTo2DLayer, costmap_2d::Layer)

namespace costmap_3d
{

Costmap3DTo2DLayer::Costmap3DTo2DLayer()
    : dsrv_(NULL), copy_full_map_(true)
{
}

Costmap3DTo2DLayer::~Costmap3DTo2DLayer()
{
  if (dsrv_)
    delete dsrv_;
}

void Costmap3DTo2DLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);

  reset();

  if (dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_3d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_3d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &Costmap3DTo2DLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void Costmap3DTo2DLayer::reconfigureCB(costmap_3d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    // XXX use max
    enabled_ = config.enabled;
    switch (config.combination_method)
    {
      case GenericPlugin_Maximum:
        use_maximum_ = true;
        break;
      case GenericPlugin_Overwrite:
        use_maximum_ = false;
        break;
      default:
      case GenericPlugin_Nothing:
        config.enabled = false;
        enabled_ = false;
        break;
    }
  }
}

void Costmap3DTo2DLayer::matchSize()
{
  copy_full_map_ = true;
  super::matchSize();
}

void Costmap3DTo2DLayer::activate()
{
}

void Costmap3DTo2DLayer::deactivate()
{
  reset();
}

void Costmap3DTo2DLayer::reset()
{
  current_ = false;
  copy_full_map_ = true;
}

void Costmap3DTo2DLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  useExtraBounds(min_x, min_y, max_x, max_y);
}

void Costmap3DTo2DLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!use_maximum_)
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  else
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void Costmap3DTo2DLayer::updateFrom3D(const Costmap3D& map, const Costmap3D& delta, const Costmap3D& bounds_map)
{
  current_ = true;
  // XXX implement me
}

}  // namespace costmap_2d
