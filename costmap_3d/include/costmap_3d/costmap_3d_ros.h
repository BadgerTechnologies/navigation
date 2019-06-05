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
#ifndef COSTMAP_3D_COSTMAP_3D_ROS_H_
#define COSTMAP_3D_COSTMAP_3D_ROS_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_3d/layered_costmap_3d.h>
#include <costmap_3d/costmap_3d_publisher.h>
#include <costmap_3d/Costmap3DConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.h>

namespace costmap_3d
{

/** @brief A ROS wrapper for a 3D Costmap. */
class Costmap3DROS : public costmap_2d::Costmap2DROS
{
  using super = costmap_2d::Costmap2DROS;
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap3DROS(std::string name, tf::TransformListener& tf);
  virtual ~Costmap3DROS();

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  virtual void start();

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  virtual void stop();

  /** @brief Update the costmap from all layers, publishing as necessary. */
  virtual void updateMap();

  /**
   * @brief Reset all layers
   */
  virtual void resetLayers();

  /** @brief Returns true if all layers have current sensor data. */
  virtual bool isCurrent()
  {
    return super::isCurrent() && layered_costmap_3d_->isCurrent();
  }

  /* Unlike Costmap2D, provide no interface to the 3D layered costmap internals.
   * Instead provide specific interfaces to clear 3D layers and get 3D collision
   * information from the map. Note that altering the included 2D master
   * costmap has no effect on the 3D data. The 3D costmap may be mirrored to
   * the 2D costmap using the costmap3DTo2D plugin. */

  /** @brief Lock the master costmap to prevent updates.
   *
   * This is important for planners or other users who require a consistent
   * view over a period of time.
   *
   * This class implements the BasicLockable requirements so you can use a
   * costmap_3d_ros as a template argument to std::lock_guard, for instance.
   */
  virtual void lock();

  /** @brief Unlock the master costmap to prevent updates.
   *
   * This is important for planners or other users who require a consistent
   * view over a period of time.
   *
   * This class implements the BasicLockable requirements so you can use a
   * costmap_3d_ros as a template argument to std::lock_guard, for instance.
   */
  virtual void unlock();

  /** @brief Get the names of the layers in the costmap. */
  virtual std::vector<std::string> getLayerNames();

  /** @brief Clear the costmap within the given axis aligned bounding box of the given position across all layers. */
  virtual void clearAABB(geometry_msgs::Point min, geometry_msgs::Point max);

  /** @brief Clear the costmap within the given axis aligned bounding box of the given position for the given layers. */
  virtual void clearAABB(geometry_msgs::Point min, geometry_msgs::Point max, const std::vector<std::string>& layers);

  /** @brief Get the cost to put the robot base at the given pose.
   *
   * It is assumed the pose is in the frame of the costmap, and the current
   * state of the costmap is queried at the given pose.
   * Currently the padding is applied in all directions.
   * A negative return value indicates a collision. */
  virtual double footprintCost(geometry_msgs::Pose pose, double padding = NAN);

  /** @brief Return whether the given pose is in collision. */
  virtual bool footprintCollision(geometry_msgs::Pose pose, double padding = NAN);

  /** @brief Return minimum distance to nearest costmap object.
   * This returns the minimum unsigned distance. So a collision will return 0,
   * no matter how deeply in collision. */
  virtual double footprintDistance(geometry_msgs::Pose pose, double padding = NAN);

  /** @brief Return minimum signed distance to nearest costmap object.
   * This returns the minimum signed distance. So, the deeper a pose goes into
   * obstacles, the more negative the return value becomes. */
  virtual double footprintSignedDistance(geometry_msgs::Pose pose, double padding = NAN);

protected:
  std::shared_ptr<LayeredCostmap3D> layered_costmap_3d_;

private:
  void reconfigureCB(costmap_3d::Costmap3DConfig &config, uint32_t level);
  pluginlib::ClassLoader<Layer3D> plugin_loader_;
  std::shared_ptr<Costmap3DPublisher> publisher_;
  std::shared_ptr<dynamic_reconfigure::Server<costmap_3d::Costmap3DConfig>> dsrv_;

  float footprint_3d_padding_;
};
// class Costmap3DROS
}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_3D_ROS_H
