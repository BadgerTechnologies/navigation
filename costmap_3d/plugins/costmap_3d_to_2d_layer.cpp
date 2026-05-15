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
#include <costmap_3d/costmap_3d_to_2d_layer.h>
#include <costmap_2d/cost_values.h>
#include <costmap_3d/GenericPluginConfig.h>
#include <costmap_3d/costmap_3d.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_3d::Costmap3DTo2DLayer, costmap_2d::Layer)

namespace costmap_3d
{

Costmap3DTo2DLayer::Costmap3DTo2DLayer()
    : copy_full_map_(true)
{
}

Costmap3DTo2DLayer::~Costmap3DTo2DLayer()
{
}

void Costmap3DTo2DLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);

  default_value_ = costmap_2d::NO_INFORMATION;

  reset();

  dsrv_.reset(new dynamic_reconfigure::Server<costmap_3d::GenericPluginConfig>(nh));
  dsrv_->setCallback(std::bind(&Costmap3DTo2DLayer::reconfigureCB, this,
                               std::placeholders::_1, std::placeholders::_2));
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
  addExtraBounds(getOriginX(), getOriginY(), getOriginX() + getSizeInMetersX(), getOriginY() + getSizeInMetersY());
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
  layered_costmap_3d_ = NULL;
  copy_full_map_ = true;
  addExtraBounds(getOriginX(), getOriginY(), getOriginX() + getSizeInMetersX(), getOriginY() + getSizeInMetersY());
}

bool Costmap3DTo2DLayer::fillWorldBoxFrom3D(
    double world_min_x, double world_min_y,
    double world_max_x, double world_max_y)
{
  // The updateMap method locks the master costmap, locking both the 2D
  // and 3D costmaps (since they use the same lock). Therefore, the 3D
  // costmap can not change, and trying to lock here would cause a deadlock.
  Costmap3DConstPtr master_3d = layered_costmap_3d_->getCostmap3D();

  // Octomap cell coordinates are from center, where costmap_2d's are from
  // bottom-left. We must move the origin to the center to get the correct
  // results.
  Costmap3DIndex map_origin_index;
  if (!master_3d->coordToKeyChecked(origin_x_ + resolution_/2.0, origin_y_ + resolution_/2.0, 0.0, map_origin_index))
  {
    // We don't bother handling when the origin of the 2D map is off the 3D
    // map space. Partial updating around the boundaries is tricky.
    ROS_WARN_STREAM_THROTTLE(
        5.0,
        "Costmap3DTo2DLayer: 2D map origin is not in 3D map index space! "
        "Marking layer as stale for safety.");
    return false;
  }

  int min_map_x, min_map_y, max_map_x, max_map_y;
  worldToMapEnforceBounds(world_min_x, world_min_y, min_map_x, min_map_y);
  worldToMapEnforceBounds(world_max_x, world_max_y, max_map_x, max_map_y);

  // Clear the region before filling.
  for (int row = min_map_y; row <= max_map_y; ++row)
  {
    const int row_len = (max_map_x - min_map_x) + 1;
    memset(costmap_ + min_map_x + row * size_x_, default_value_, row_len);
  }

  // Fill from the 3D costmap via BBX traversal.
  Costmap3DIndex min_index, max_index;
  master_3d->coordToKeyClamped(world_min_x, world_min_y, -std::numeric_limits<double>::max(), min_index);
  master_3d->coordToKeyClamped(world_max_x, world_max_y, std::numeric_limits<double>::max(), max_index);
  const octomap::key_type map_ox = map_origin_index[0];
  const octomap::key_type map_oy = map_origin_index[1];
  min_index[0] = std::max(min_index[0], map_ox);
  min_index[1] = std::max(min_index[1], map_oy);
  max_index[0] = std::min(max_index[0], map_ox + size_x_ - 1);
  max_index[1] = std::min(max_index[1], map_oy + size_y_ - 1);

  assert(resolution_ > 0.0);
  assert((master_3d->getResolution() - resolution_) < 1e-6);

  const unsigned int tree_depth = master_3d->getTreeDepth();
  auto it = master_3d->begin_leafs_bbx(min_index, max_index);
  const auto end = master_3d->end_leafs_bbx();
  while (it != end)
  {
    octomap::key_type min_kx, min_ky, max_kx, max_ky;
    clipLeafToMap(it, map_ox, map_oy, tree_depth, min_kx, min_ky, max_kx, max_ky);
    const unsigned char cost = toCostmap2D(it->getValue());
    for (octomap::key_type y = min_ky; y <= max_ky; ++y)
      for (octomap::key_type x = min_kx; x <= max_kx; ++x)
      {
        const unsigned int map_index = getIndex(x - map_ox, y - map_oy);
        if (costmap_[map_index] == costmap_2d::NO_INFORMATION || cost > costmap_[map_index])
          costmap_[map_index] = cost;
      }
    ++it;
  }
  return true;
}

void Costmap3DTo2DLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  if (layered_costmap_->isRolling())
  {
    costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
    if (getOriginX() != master->getOriginX() || getOriginY() != master->getOriginY())
    {
      double old_origin_x = getOriginX();
      double old_origin_y = getOriginY();
      updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
      double new_origin_x = getOriginX();
      double new_origin_y = getOriginY();
      double new_max_x = new_origin_x + getSizeInMetersX();
      double new_max_y = new_origin_y + getSizeInMetersY();
      double old_max_x = old_origin_x + getSizeInMetersX();
      double old_max_y = old_origin_y + getSizeInMetersY();

      if (enabled_ && layered_costmap_3d_)
      {
        // Fill only the newly exposed strips from the current 3D costmap state.
        // updateOrigin already reset newly exposed cells to default_value_; this
        // re-populates them from the 3D costmap and marks them for propagation.
        if (new_origin_x != old_origin_x)
        {
          double strip_min_x = (new_origin_x > old_origin_x) ? old_max_x : new_origin_x;
          double strip_max_x = (new_origin_x > old_origin_x) ? new_max_x : old_origin_x;
          if (!fillWorldBoxFrom3D(strip_min_x, new_origin_y, strip_max_x, new_max_y))
            current_ = false;
          addExtraBounds(strip_min_x, new_origin_y, strip_max_x, new_max_y);
        }
        if (new_origin_y != old_origin_y)
        {
          double strip_min_y = (new_origin_y > old_origin_y) ? old_max_y : new_origin_y;
          double strip_max_y = (new_origin_y > old_origin_y) ? new_max_y : old_origin_y;
          // Use the stable X range (intersection of old and new map) to avoid
          // re-filling cells already covered by the X strip above.
          double y_strip_min_x = std::max(new_origin_x, old_origin_x);
          double y_strip_max_x = std::min(new_max_x, old_max_x);
          if (!fillWorldBoxFrom3D(y_strip_min_x, strip_min_y, y_strip_max_x, strip_max_y))
            current_ = false;
          addExtraBounds(y_strip_min_x, strip_min_y, y_strip_max_x, strip_max_y);
        }
      }
    }
  }

  // The 2D cells were already updated directly in updateFrom3D (or fillWorldBoxFrom3D
  // above for origin-shift strips). Just propagate the dirty region to the master.
  if (enabled_ && layered_costmap_3d_)
  {
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

void Costmap3DTo2DLayer::updateFrom3D(
    LayeredCostmap3D* layered_costmap_3d,
    const Costmap3D& bounds_map)
{
  // cache a pointer to the 3d layered costmap to use later
  layered_costmap_3d_ = layered_costmap_3d;

  // Note: this function is only ever called during the costmap update
  // process, so we do not need to worry about synchronization w/ the layered
  // costmap.

  if (copy_full_map_)
  {
    // Populate the entire 2D layer from the current 3D costmap state.
    current_ = fillWorldBoxFrom3D(getOriginX(), getOriginY(),
                                  getOriginX() + getSizeInMetersX(), getOriginY() + getSizeInMetersY());
    if (!current_) return;
    addExtraBounds(getOriginX(), getOriginY(),
                   getOriginX() + getSizeInMetersX(), getOriginY() + getSizeInMetersY());
    copy_full_map_ = false;
    return;
  }

  Costmap3DConstPtr master_3d = layered_costmap_3d_->getCostmap3D();
  Costmap3DIndex map_origin_index;
  if (!master_3d->coordToKeyChecked(origin_x_ + resolution_/2.0, origin_y_ + resolution_/2.0, 0.0, map_origin_index))
  {
    ROS_WARN_STREAM_THROTTLE(
        5.0,
        "Costmap3DTo2DLayer: 2D map origin is not in 3D map index space! "
        "Marking layer as stale for safety.");
    current_ = false;
    return;
  }
  current_ = true;

  const octomap::key_type map_ox = map_origin_index[0];
  const octomap::key_type map_oy = map_origin_index[1];
  const unsigned int tree_depth = master_3d->getTreeDepth();

  // Pass 1: clear every 2D cell touched by bounds_map (collapsed over z),
  // and accumulate the tight x,y world bounds of the cleared region.
  double tight_min_wx = std::numeric_limits<double>::max();
  double tight_min_wy = std::numeric_limits<double>::max();
  double tight_max_wx = -std::numeric_limits<double>::max();
  double tight_max_wy = -std::numeric_limits<double>::max();
  for (auto it = bounds_map.begin_leafs(), end = bounds_map.end_leafs(); it != end; ++it)
  {
    octomap::key_type min_kx, min_ky, max_kx, max_ky;
    clipLeafToMap(it, map_ox, map_oy, tree_depth, min_kx, min_ky, max_kx, max_ky);
    for (octomap::key_type ky = min_ky; ky <= max_ky; ++ky)
      for (octomap::key_type kx = min_kx; kx <= max_kx; ++kx)
        costmap_[getIndex(kx - map_ox, ky - map_oy)] = default_value_;
    double half_size = it.getSize() / 2.0;
    double wx = it.getX(), wy = it.getY();
    addExtraBounds(wx - half_size, wy - half_size, wx + half_size, wy + half_size);
    tight_min_wx = std::min(tight_min_wx, wx - half_size);
    tight_min_wy = std::min(tight_min_wy, wy - half_size);
    tight_max_wx = std::max(tight_max_wx, wx + half_size);
    tight_max_wy = std::max(tight_max_wy, wy + half_size);
  }

  if (tight_min_wx > tight_max_wx)
    return;

  // Pass 2: re-derive 2D costs from master_3d over the tight x,y bounds,
  // spanning all z levels. Traversing the full master (not just the delta)
  // ensures 3D cells absent from bounds_map — e.g. a static floor-level wall
  // cell when only a higher-z cell at the same x,y changed this cycle —
  // still contribute to the 2D projection.
  Costmap3DIndex min_index, max_index;
  master_3d->coordToKeyClamped(tight_min_wx, tight_min_wy, -std::numeric_limits<double>::max(), min_index);
  master_3d->coordToKeyClamped(tight_max_wx, tight_max_wy, std::numeric_limits<double>::max(), max_index);
  min_index[0] = std::max(min_index[0], map_ox);
  min_index[1] = std::max(min_index[1], map_oy);
  max_index[0] = std::min(max_index[0], map_ox + (octomap::key_type)(size_x_ - 1));
  max_index[1] = std::min(max_index[1], map_oy + (octomap::key_type)(size_y_ - 1));
  for (auto it = master_3d->begin_leafs_bbx(min_index, max_index),
       end = master_3d->end_leafs_bbx(); it != end; ++it)
  {
    octomap::key_type min_kx, min_ky, max_kx, max_ky;
    clipLeafToMap(it, map_ox, map_oy, tree_depth, min_kx, min_ky, max_kx, max_ky);
    const unsigned char cost = toCostmap2D(it->getValue());
    for (octomap::key_type ky = min_ky; ky <= max_ky; ++ky)
      for (octomap::key_type kx = min_kx; kx <= max_kx; ++kx)
      {
        unsigned int idx = getIndex(kx - map_ox, ky - map_oy);
        if (costmap_[idx] == costmap_2d::NO_INFORMATION || cost > costmap_[idx])
          costmap_[idx] = cost;
      }
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

}  // namespace costmap_3d
