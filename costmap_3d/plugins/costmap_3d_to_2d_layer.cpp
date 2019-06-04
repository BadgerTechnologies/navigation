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

  default_value_ = costmap_2d::NO_INFORMATION;

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
  current_ = false;
  copy_full_map_ = true;
}

void Costmap3DTo2DLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  if (layered_costmap_->isRolling())
  {
    costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
    if (getOriginX() != master->getOriginX() || getOriginY() != master->getOriginY())
    {
      updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
      // We could attempt to see if we only shifted in X or Y and minimize the
      // impact on bounds. However, prefer the simplicity of just updating
      // everything on a shift.
      addExtraBounds(getOriginX(), getOriginY(), getSizeInMetersX(), getSizeInMetersY());
    }
  }
  if (enabled_)
  {
    {
      // First erase the bounding-box of the 2D map.
      int min_map_x, min_map_y;
      int max_map_x, max_map_y;
      worldToMapEnforceBounds(extra_min_x_, extra_min_y_, min_map_x, min_map_y);
      worldToMapEnforceBounds(extra_max_x_, extra_max_y_, max_map_x, max_map_y);
      for (int row = min_map_y; row <= max_map_y; ++row)
      {
        const int row_len = (max_map_x - min_map_x) + 1;
        const int row_step = size_x_;
        memset(costmap_ + min_map_x + row * row_step, default_value_, row_len);
      }
    }

    {
      // Update the regions that have changed
      std::lock_guard<LayeredCostmap3D> lock_layered_costmap_3d(*layered_costmap_3d_);
      // Create a bound-box iterator over the 3D costmap.
      const Costmap3D* master_3d = layered_costmap_3d_->getCostmap3D();
      Costmap3DIndex min_index, max_index;
      master_3d->coordToKeyClamped(extra_min_x_, extra_min_y_, -std::numeric_limits<double>::max(),
                                   min_index);
      master_3d->coordToKeyClamped(extra_max_x_, extra_max_y_, std::numeric_limits<double>::max(),
                                   max_index);
      auto it = master_3d->begin_leafs_bbx(min_index, max_index);
      auto end = master_3d->end_leafs_bbx();

      while (it != end)
      {
        double x = it.getX();
        double y = it.getY();
        unsigned int map_row, map_col;
        if (worldToMap(x, y, map_row, map_col))
        {
          const unsigned int map_index = getIndex(map_row, map_col);
          const unsigned char cost = toCostmap2D(it->getValue());
          if (costmap_[map_index] == costmap_2d::NO_INFORMATION || cost > costmap_[map_index])
          {
            costmap_[map_index] = cost;
          }
        }
        ++it;
      }
    }
    useExtraBounds(min_x, min_y, max_x, max_y);
  }
}

void Costmap3DTo2DLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!use_maximum_)
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  else
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void Costmap3DTo2DLayer::updateFrom3D(LayeredCostmap3D* layered_costmap_3d, const Costmap3D& delta, const Costmap3D& bounds_map)
{
  // cache a pointer to the 3d layered costmap to use later
  layered_costmap_3d_ = layered_costmap_3d;

  // Note: this function is only ever called during the costmap update
  // process, so we do not need to worry about synchronization w/ the layered
  // costmap.
  current_ = true;

  // Get our extra bounds added for this update.
  Costmap3D::iterator it, end;
  if (copy_full_map_)
  {
    it = layered_costmap_3d_->getCostmap3D()->begin_leafs();
    end = layered_costmap_3d_->getCostmap3D()->end_leafs();
    copy_full_map_ = false;
  }
  else
  {
    it = bounds_map.begin_leafs();
    end = bounds_map.end_leafs();
  }

  while (it != end)
  {
    double half_size = it.getSize() / 2.0;
    double x = it.getX();
    double y = it.getY();
    addExtraBounds(x - half_size, y - half_size, x + half_size, y + half_size);
    ++it;
  }
}

unsigned char Costmap3DTo2DLayer::toCostmap2D(Cost value) const
{
  if (value >= LETHAL) return costmap_2d::LETHAL_OBSTACLE;
  if (value == FREE) return costmap_2d::FREE_SPACE;
  if (value < FREE) return costmap_2d::NO_INFORMATION;

  // return a linear interpolation of 3D Cost values to 2D cost values
  return static_cast<uint8_t>((costmap_2d::LETHAL_OBSTACLE - costmap_2d::FREE_SPACE) *
                              (value - FREE) / (LETHAL - FREE)) + costmap_2d::FREE_SPACE;
}

}  // namespace costmap_2d
