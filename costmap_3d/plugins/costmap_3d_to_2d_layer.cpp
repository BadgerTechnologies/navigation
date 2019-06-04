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
#include "costmap_2d/cost_values.h"
#include "costmap_3d/GenericPluginConfig.h"
#include "costmap_3d/costmap_3d.h"
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
  super::resetMaps();
  tracking_map_.clear();
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
  // Note: this function is only ever called during the costmap update
  // process, so we do not need to worry about synchrnoization w/ the layered
  // costmap.
  current_ = true;

  Costmap3D::iterator it, end;
  if (!copy_full_map_)
  {
    // erase all entries that are in bound_map
    it = map.begin_leafs();
    end = map.end_leafs();
    for(; it != end; ++it)
    {
      // TODO ME
    }
  }
  if (copy_full_map_)
  {
    it = map.begin_leafs();
    end = map.end_leafs();
    copy_full_map_ = false;
  }
  else
  {
    it = delta.begin_leafs();
    end = delta.end_leafs();
  }

  // copy the whole 3D map to the 2D tracking map
  for(; it != end; ++it)
  {
    auto k = it.getKey();
    auto half_size = it.getSize() / 2.0;
    auto x = it.getX();
    auto y = it.getY();
    // need to update all appropriate entries based on size/depth
    tracking_map_[std::make_pair(k[0], k[1])].update(k[2], it->getValue());
    addExtraBounds(x - half_size, y - half_size, x + half_size, y + half_size);
  }
}

uint8_t Costmap3DTo2DLayer::OccupancyTrackingValue::toCostmap2D(Cost value) const
{
  if (value >= LETHAL) return costmap_2d::LETHAL_OBSTACLE;
  if (value == FREE) return costmap_2d::FREE_SPACE;
  if (value < FREE) return costmap_2d::NO_INFORMATION;

  // return a linear interpolation of 3D Cost values to 2D cost values
  return static_cast<uint8_t>((costmap_2d::LETHAL_OBSTACLE - costmap_2d::FREE_SPACE) *
                              (value - FREE) / (LETHAL - FREE)) + costmap_2d::FREE_SPACE;
}

void Costmap3DTo2DLayer::OccupancyTrackingValue::update(int z, Cost cost)
{
  uint8_t new_cost = toCostmap2D(cost);
  auto it = finite_costs_.find(z);

  if (it != finite_costs_.end() && it->second == new_cost)
  {
    // nothing to do, the update is the same thing.
  }
  else if (new_cost == costmap_2d::NO_INFORMATION)
  {
    // no information is special. erase this entry if it exists.
    if (it != finite_costs_.end())
    {
      erase(it);
    }
  }
  else
  {
    if (it != finite_costs_.end())
    {
      auto counter_it = finite_costs_counters_.find(it->second);
      assert(counter_it != finite_costs_counters_.end());
      assert(counter_it->second > 0);
      --(counter_it->second);
      if (counter_it->second == 0)
      {
        finite_costs_counters_.erase(counter_it);
      }
      it->second = new_cost;
    }
    else
    {
      finite_costs_[z] = new_cost;
    }
    ++(finite_costs_counters_[new_cost]);
    // largest cost counter in the map will be the end
    cost_ = finite_costs_counters_.rbegin()->first;
  }
}

void Costmap3DTo2DLayer::OccupancyTrackingValue::erase(int z)
{
  auto it = finite_costs_.find(z);
  if (it != finite_costs_.end())
  {
    erase(it);
  }
}

void Costmap3DTo2DLayer::OccupancyTrackingValue::erase(FiniteCostsType::iterator it)
{
  auto counter_it = finite_costs_counters_.find(it->second);
  assert(counter_it != finite_costs_counters_.end());
  assert(counter_it->second > 0);
  --(counter_it->second);
  if (counter_it->second == 0)
  {
    finite_costs_counters_.erase(counter_it);
    if (it->second == cost_)
    {
      if (finite_costs_counters_.size() > 0)
      {
        // largest cost counter in the map will be the end
        cost_ = finite_costs_counters_.rbegin()->first;
      }
      else
      {
        cost_ = costmap_2d::NO_INFORMATION;
      }
    }
  }
  finite_costs_.erase(it);
}

}  // namespace costmap_2d
