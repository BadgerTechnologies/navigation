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

#include <string>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <limits>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/utility.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_3d/layered_costmap_3d.h>
#include <costmap_3d/costmap_3d_publisher.h>
#include <costmap_3d/GetPlanCost3DAction.h>
#include <costmap_3d/GetPlanCost3DService.h>
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
   * return value represents the cost of the pose, normalized to 0.0 is free
   * and 1.0 is lethal.
   * The caller must be holding the lock. */
  virtual double footprintCost(geometry_msgs::Pose pose, double padding = NAN);

  /** @brief Return whether the given pose is in collision.
   *
   * The caller must be holding the lock.*/
  virtual bool footprintCollision(geometry_msgs::Pose pose, double padding = NAN);

  /** @brief Return minimum distance to nearest costmap object.
   * This returns the minimum unsigned distance. So a collision will return <=0.0.
   * Negative values are not exact minimum distances. If exact minimum is
   * required use footprintSignedDistance.
   * The caller must be holding the lock. */
  virtual double footprintDistance(geometry_msgs::Pose pose, double padding = NAN);

  /** @brief Return minimum signed distance to nearest costmap object.
   * This returns the minimum signed distance. So, the deeper a pose goes into
   * obstacles, the more negative the return value becomes.
   * The caller must be holding the lock. */
  virtual double footprintSignedDistance(geometry_msgs::Pose pose, double padding = NAN);

protected:
  std::shared_ptr<LayeredCostmap3D> layered_costmap_3d_;

private:
  // Because the parent class starts a thread calling updateMap right away,
  // keep track if we are fully initialized to prevent doing anything with a
  // partially constructed object.
  std::mutex initialized_mutex_;
  bool initialized_;
  void reconfigureCB(costmap_3d::Costmap3DConfig &config, uint32_t level);
  pluginlib::ClassLoader<Layer3D> plugin_loader_;
  std::shared_ptr<Costmap3DPublisher> publisher_;
  std::shared_ptr<dynamic_reconfigure::Server<costmap_3d::Costmap3DConfig>> dsrv_;
  std::shared_ptr<actionlib::SimpleActionServer<GetPlanCost3DAction>> get_plan_cost_action_srv_;
  ros::ServiceServer get_plan_cost_srv_;

  void getPlanCost3DActionCallback(const actionlib::SimpleActionServer<GetPlanCost3DAction>::GoalConstPtr& goal);
  bool getPlanCost3DServiceCallback(GetPlanCost3DService::Request& request, GetPlanCost3DService::Response& response);

  void updateMeshResource();
  std::string getFileNameFromPackageURL(const std::string& url);
  std::string footprint_mesh_resource_;
  double footprint_3d_padding_;

  using FCLFloat = float;
  using FCLRobotModel = fcl::BVHModel<fcl::OBBRSS<FCLFloat>>;
  using FCLRobotModelPtr = std::shared_ptr<FCLRobotModel>;

  FCLRobotModelPtr robot_model_;

  using FCLCollisionObject = fcl::CollisionObject<FCLFloat>;
  using FCLCollisionObjectPtr = std::shared_ptr<FCLCollisionObject>;
  FCLCollisionObjectPtr getRobotCollisionObject(const geometry_msgs::Pose& pose);
  FCLCollisionObjectPtr getWorldCollisionObject();

  class DistanceCacheKey
  {
  public:
    DistanceCacheKey(const std::string& mesh_id_,
                     const std::string& frame_id_,
                     const geometry_msgs::Pose& pose)
        : mesh_id(mesh_id_),
          frame_id(frame_id_),
          binned_pose(binPose(pose))
    {
    }

    std::string mesh_id;
    std::string frame_id;
    geometry_msgs::Pose binned_pose;
    // Bin a pose.
    geometry_msgs::Pose binPose(const geometry_msgs::Pose& pose)
    {
      geometry_msgs::Pose rv;
      rv.position.x = std::round(pose.position.x * 4) / 4;
      rv.position.y = std::round(pose.position.y * 4) / 4;
      rv.position.z = std::round(pose.position.z * 4) / 4;
      rv.orientation.x = std::round(pose.orientation.x * 32) / 32;
      rv.orientation.y = std::round(pose.orientation.y * 32) / 32;
      rv.orientation.z = std::round(pose.orientation.z * 32) / 32;
      rv.orientation.w = std::round(pose.orientation.w * 32) / 32;
      return rv;
    }
  };
  struct DistanceCacheKeyHash
  {
    size_t operator()(const DistanceCacheKey& key) const
    {
      size_t rv;
      rv = std::hash<std::string>{}(key.mesh_id);
      // circular shift left 8 bits
      rv = rv << 8 | (rv >> (std::numeric_limits<size_t>::digits - 8));
      rv ^= std::hash<std::string>{}(key.frame_id);
      // circular shift left 8 bits
      rv = rv << 8 | (rv >> (std::numeric_limits<size_t>::digits - 8));
      rv ^= std::hash<double>{}(key.binned_pose.position.x +
                                key.binned_pose.position.y * 256 +
                                key.binned_pose.position.z * 256 * 256);
      // circular shift left 8 bits
      rv = rv << 8 | (rv >> (std::numeric_limits<size_t>::digits - 8));
      rv ^= std::hash<double>{}(key.binned_pose.orientation.w +
                                key.binned_pose.orientation.z * 8 +
                                key.binned_pose.orientation.y * 8 * 8 +
                                key.binned_pose.orientation.x * 8 * 8 * 8);
      return rv;
    }
  };
  struct DistanceCacheKeyEqual
  {
    bool operator()(const DistanceCacheKey& lhs, const DistanceCacheKey& rhs) const
    {
      return lhs.frame_id == rhs.frame_id && lhs.mesh_id == rhs.mesh_id &&
          lhs.binned_pose.position.x == rhs.binned_pose.position.x &&
          lhs.binned_pose.position.y == rhs.binned_pose.position.y &&
          lhs.binned_pose.position.z == rhs.binned_pose.position.z &&
          lhs.binned_pose.orientation.w == rhs.binned_pose.orientation.w &&
          lhs.binned_pose.orientation.x == rhs.binned_pose.orientation.x &&
          lhs.binned_pose.orientation.y == rhs.binned_pose.orientation.y &&
          lhs.binned_pose.orientation.z == rhs.binned_pose.orientation.z;
    }
  };
  class DistanceCacheEntry
  {
  public:
    DistanceCacheEntry() {}
    DistanceCacheEntry(const DistanceCacheEntry& rhs)
        : octomap_box(rhs.octomap_box),
          octomap_box_tf(rhs.octomap_box_tf),
          mesh_triangle(rhs.mesh_triangle),
          mesh_triangle_tf(rhs.mesh_triangle_tf)
    {
    }
    const DistanceCacheEntry& operator=(const DistanceCacheEntry& rhs)
    {
      octomap_box = rhs.octomap_box;
      octomap_box_tf = rhs.octomap_box_tf;
      mesh_triangle = rhs.mesh_triangle;
      mesh_triangle_tf = rhs.mesh_triangle_tf;
      return *this;
    }
    DistanceCacheEntry(const fcl::DistanceResult<FCLFloat>& result)
    {
      octomap_box = std::dynamic_pointer_cast<fcl::Box<FCLFloat>>(result.primitive1);
      octomap_box_tf = result.tf1;
      mesh_triangle = std::dynamic_pointer_cast<fcl::TriangleP<FCLFloat>>(result.primitive2);
      mesh_triangle_tf = result.tf2;
    }
    FCLFloat distanceToNewPose(geometry_msgs::Pose pose)
    {
      // Turn pose into tf
      fcl::Transform3<FCLFloat> new_tf(
          fcl::Translation3<FCLFloat>(pose.position.x, pose.position.y, pose.position.z) *
          fcl::Quaternion<FCLFloat>(pose.orientation.w,
                                    pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z));

      FCLFloat dist;
      fcl::detail::GJKSolver_libccd<FCLFloat> solver;
      fcl::Vector3<FCLFloat> p1, p2;
      solver.shapeTriangleDistance(*octomap_box, octomap_box_tf,
                                   mesh_triangle->a, mesh_triangle->b, mesh_triangle->c, new_tf,
                                   &dist, &p1, &p2);
      return dist;
    }
    std::shared_ptr<fcl::Box<FCLFloat>> octomap_box;
    fcl::Transform3<FCLFloat> octomap_box_tf;
    std::shared_ptr<fcl::TriangleP<FCLFloat>> mesh_triangle;
    fcl::Transform3<FCLFloat> mesh_triangle_tf;
  };
  std::unordered_map<DistanceCacheKey, DistanceCacheEntry, DistanceCacheKeyHash, DistanceCacheKeyEqual> distance_cache_;

};
// class Costmap3DROS
}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_3D_ROS_H
