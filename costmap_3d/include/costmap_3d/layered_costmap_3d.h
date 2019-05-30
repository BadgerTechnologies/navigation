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
#ifndef COSTMAP_3D_LAYERED_COSTMAP_3D_H_
#define COSTMAP_3D_LAYERED_COSTMAP_3D_H_

#include <vector>
#include <string>
#include <limits>
#include <mutex>
#include <map>
#include <functional>
#include <costmap_3d/layer_3d.h>
#include <costmap_3d/costmap_3d.h>
#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/Point.h>

namespace costmap_3d
{

class Layer3D;

/**
 * @class LayeredCostmap3D
 * @brief Instantiates different 3D layer plugins and aggregates them into one 3D costmap
 */
class LayeredCostmap3D
{
public:
  /**
   * @brief Constructor for a 3D layered costmap
   */
  LayeredCostmap3D(costmap_2d::LayeredCostmap* layered_costmap_2d);

  /**
   * @brief Destructor
   */
  ~LayeredCostmap3D();

  /**
   * @brief Update the underlying costmap with new data.
   */
  void updateMap(geometry_msgs::Pose robot_pose);

  /**
   * @brief Activate each layer (if deactivated).
   */
  void activate();

  /**
   * @brief Activate each layer (if deactivated).
   */
  void deactivate();

  /**
   * @brief Reset the entire costmap, deleting all state.
   */
  void reset();

  bool isCurrent() const;

  // The caller must be holding the lock to keep the costmap from updating on
  // them. It is highly recommended to use RAII lock acquisition, such as
  // std::lock_guard.
  const Costmap3D* getCostmap3D() const;

  /** Returns if this costmap's x/y bounds are relative to the base
   */
  bool isRolling() const;

  const std::vector<boost::shared_ptr<Layer3D>>& getPlugins();

  void addPlugin(boost::shared_ptr<Layer3D> plugin);

  // Lock the master costmap (used by planners to keep the costmap consistent
  // while either planning, or copying the costmap to use for planning).
  void lock();
  void unlock();

  double getResolution() const;

  /**
   * @brief Set the resolution.
   *
   * If no resolution is given, sets the resolution to match the 2D costmap.
   */
  void setResolution(double resolution=0.0);

  void getBounds(geometry_msgs::Point* min, geometry_msgs::Point* max);
  void setBounds(const geometry_msgs::Point& min, const geometry_msgs::Point& max);

  // Pass the complete costmap, the delta map from the last update, and the
  // bounds map on every completion.
  using UpdateCompleteCallback = std::function<void(const Costmap3D& map, const Costmap3D& delta_map, const Costmap3D& bounds_map)>;

  /**
   * @brief Register a callback to be called when a 3D costmap update is complete.
   */
  void registerUpdateCompleteCallback(const std::string callback_id, UpdateCompleteCallback cb);

  /**
   * @brief Unregister a callback to be called when a 3D costmap update is complete.
   */
  void unregisterUpdateCompleteCallback(const std::string callback_id);

  /**
   * @brief Get a pointer to the corresponding 2D layered costmap.
   */
  costmap_2d::LayeredCostmap* getLayeredCostmap2D() {return layered_costmap_2d_;}

private:
  // Lock must be held while calling this internal function
  void sizeChange();

  class LockLayers
  {
  public:
    LockLayers(LayeredCostmap3D* layered_costmap_3d);

    // lock all layers preventing them from updating any state needed to be
    // consistent between updateBounds and updateCosts
    void lock();

    // unlock all layers.
    void unlock();
  private:
    LayeredCostmap3D* layered_costmap_3d_;
  };

  LockLayers lock_layers_;

  // Master 3D cost map.
  // Starts off NULL until our resolution is setup.
  Costmap3DPtr costmap_;
  std::mutex costmap_mutex_;

  // Pointer to our sibling 2D costmap.
  // We can use this to find our global frame.
  // Also, we can use this pointer to keep the 2D costmap in sync with the 3D layers.
  costmap_2d::LayeredCostmap* layered_costmap_2d_;

  std::vector<boost::shared_ptr<Layer3D>> plugins_;
  std::map<std::string, UpdateCompleteCallback> update_complete_callbacks_;

  // The 3D footprint is described in the SRDF.

  // Uncomment when ready to actually query the sucker
//  planning_scene::PlanningScenePtr planning_scene_;

  bool size_changed_;
  double resolution_;
  geometry_msgs::Point min_point_, max_point_;
};

}  // namespace costmap_3d

#endif  // COSTMAP_3D_LAYERED_COSTMAP_3D_H_
