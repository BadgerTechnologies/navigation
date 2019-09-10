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
#ifndef COSTMAP_3D_COSTMAP_3D_QUERY_H_
#define COSTMAP_3D_COSTMAP_3D_QUERY_H_

#include <string>
#include <memory>
#include <unordered_map>
#include <limits>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/utility.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <costmap_3d/layered_costmap_3d.h>

namespace costmap_3d
{

/** @brief Query a 3D Costmap. */
class Costmap3DQuery
{
public:
  /**
   * @brief  Constructor for the query object
   */
  Costmap3DQuery();
  virtual ~Costmap3DQuery();

  /** @brief Update the layered costmap to query from.
   * Note: must be called anytime the Costmap3D is reallocated, such as when
   * the resolution changes. */
  virtual void setCostmap(const std::shared_ptr<LayeredCostmap3D>& layered_costmap_3d);

  /** @brief Update the default mesh to use for queries. */
  void updateMeshResource(const std::string& mesh_resource, double padding = 0.0);

  /** @brief Get the cost to put the robot base at the given pose.
   *
   * It is assumed the pose is in the frame of the costmap, and the current
   * state of the costmap is queried at the given pose.
   * Currently the padding is applied in all directions.
   * return value represents the cost of the pose, normalized to 0.0 is free
   * and 1.0 is lethal.
   * The caller must be holding the lock on the associated costmap. */
  virtual double footprintCost(geometry_msgs::Pose pose, double lethal_threshold = LETHAL);

  /** @brief Return whether the given pose is in collision.
   *
   * The caller must be holding the lock on the associated costmap. */
  virtual bool footprintCollision(geometry_msgs::Pose pose, double lethal_threshold = LETHAL);

  /** @brief Return minimum distance to nearest costmap object.
   * This returns the minimum unsigned distance. So a collision will return <=0.0.
   * Negative values are not exact minimum distances. If exact minimum is
   * required use footprintSignedDistance.
   * The caller must be holding the lock on the associated costmap. */
  virtual double footprintDistance(geometry_msgs::Pose pose, double padding = NAN);

  /** @brief Return minimum signed distance to nearest costmap object.
   * This returns the minimum signed distance. So, the deeper a pose goes into
   * obstacles, the more negative the return value becomes.
   * The caller must be holding the lock on the associated costmap. */
  virtual double footprintSignedDistance(geometry_msgs::Pose pose, double padding = NAN);

protected:
  std::shared_ptr<LayeredCostmap3D> layered_costmap_3d_;

  virtual void updateCompleteCallback(LayeredCostmap3D* layered_costmap_3d,
                                      const Costmap3D& delta,
                                      const Costmap3D& bounds_map);

private:
  std::string getFileNameFromPackageURL(const std::string& url);
  std::string footprint_mesh_resource_;

  using FCLFloat = float;
  using FCLRobotModel = fcl::BVHModel<fcl::OBBRSS<FCLFloat>>;
  using FCLRobotModelPtr = std::shared_ptr<FCLRobotModel>;

  FCLRobotModelPtr robot_model_;

  using FCLCollisionObject = fcl::CollisionObject<FCLFloat>;
  using FCLCollisionObjectPtr = std::shared_ptr<FCLCollisionObject>;
  FCLCollisionObjectPtr robot_obj_;
  FCLCollisionObjectPtr world_obj_;
  inline const FCLCollisionObjectPtr& getRobotCollisionObject(const geometry_msgs::Pose& pose)
  {
    robot_obj_->setTransform(
        fcl::Transform3<FCLFloat>(
            fcl::Translation3<FCLFloat>(pose.position.x, pose.position.y, pose.position.z) *
            fcl::Quaternion<FCLFloat>(pose.orientation.w,
                                      pose.orientation.x,
                                      pose.orientation.y,
                                      pose.orientation.z)));

    return robot_obj_;
  }
  inline const FCLCollisionObjectPtr& getWorldCollisionObject() { return world_obj_; }

  class DistanceCacheKey
  {
  public:
    DistanceCacheKey(const geometry_msgs::Pose& pose)
        : binned_pose(pose)
    {
    }

    geometry_msgs::Pose binned_pose;
    // Bin a pose.
    inline geometry_msgs::Pose binPose(const geometry_msgs::Pose& pose)
    {
      geometry_msgs::Pose rv;
      // 1/4m
      rv.position.x = std::round(pose.position.x * 4) / 4;
      rv.position.y = std::round(pose.position.y * 4) / 4;
      rv.position.z = std::round(pose.position.z * 4) / 4;
      // bin orientation by RPY angles.
      // another way of binning would be binning by axis/angle
      double yaw, pitch, roll;
      tf2::getEulerYPR<geometry_msgs::Quaternion>(pose.orientation, yaw, pitch, roll);
      // A 1/4 of a radian is about 14 degrees
      yaw = std::round(yaw * 4) / 4;
      pitch = std::round(pitch * 4) / 4;
      roll = std::round(roll * 4) / 4;
      tf2::Quaternion binned_q;
      binned_q.setRPY(roll, pitch, yaw);
      rv.orientation = tf2::toMsg(binned_q);
      return rv;
    }
    inline geometry_msgs::Pose binPoseMicro(const geometry_msgs::Pose& pose)
    {
      geometry_msgs::Pose rv;
      // 1/4m
      rv.position.x = std::round(pose.position.x * 1024) / 1024;
      rv.position.y = std::round(pose.position.y * 1024) / 1024;
      rv.position.z = std::round(pose.position.z * 1024) / 1024;
      // bin orientation by RPY angles.
      // another way of binning would be binning by axis/angle
      double yaw, pitch, roll;
      tf2::getEulerYPR<geometry_msgs::Quaternion>(pose.orientation, yaw, pitch, roll);
      // A 1/4 of a radian is about 14 degrees
      yaw = std::round(yaw * 1024) / 1024;
      pitch = std::round(pitch * 1024) / 1024;
      roll = std::round(roll * 1024) / 1024;
      tf2::Quaternion binned_q;
      binned_q.setRPY(roll, pitch, yaw);
      rv.orientation = tf2::toMsg(binned_q);
      return rv;
    }
    inline void binPose()
    {
      binned_pose = binPose(binned_pose);
    }
    inline void binPoseMicro()
    {
      binned_pose = binPoseMicro(binned_pose);
    }
  };
  struct DistanceCacheKeyHash
  {
    size_t operator()(const DistanceCacheKey& key) const
    {
      size_t rv;
      rv = std::hash<double>{}(key.binned_pose.orientation.w +
                               key.binned_pose.orientation.z * 8 +
                               key.binned_pose.orientation.y * 8 * 8 +
                               key.binned_pose.orientation.x * 8 * 8 * 8 +
                               key.binned_pose.position.x * 1024 +
                               key.binned_pose.position.y * 1024 * 1024 +
                               key.binned_pose.position.z * 1024 * 1024 * 1024);
      return rv;
    }
  };
  struct DistanceCacheKeyEqual
  {
    bool operator()(const DistanceCacheKey& lhs, const DistanceCacheKey& rhs) const
    {
      return lhs.binned_pose.position.x == rhs.binned_pose.position.x &&
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
          mesh_triangle(rhs.mesh_triangle)
//          mesh_triangle_tf(rhs.mesh_triangle_tf)
    {
    }
    const DistanceCacheEntry& operator=(const DistanceCacheEntry& rhs)
    {
      octomap_box = rhs.octomap_box;
      octomap_box_tf = rhs.octomap_box_tf;
      mesh_triangle = rhs.mesh_triangle;
//      mesh_triangle_tf = rhs.mesh_triangle_tf;
      return *this;
    }
    DistanceCacheEntry(const fcl::DistanceResult<FCLFloat>& result)
    {
      assert(result.primitive1);
      assert(result.primitive2);
      octomap_box = std::dynamic_pointer_cast<fcl::Box<FCLFloat>>(result.primitive1);
      octomap_box_tf = result.tf1;
      mesh_triangle = std::dynamic_pointer_cast<fcl::TriangleP<FCLFloat>>(result.primitive2);
//      mesh_triangle_tf = result.tf2;
      assert(octomap_box);
      assert(mesh_triangle);
    }
    void setupResult(fcl::DistanceResult<FCLFloat>* result)
    {
      result->primitive1 = octomap_box;
      result->primitive2 = mesh_triangle;
      result->tf1 = octomap_box_tf;
//      result->mesh_triangle_tf = result.tf2;
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
//    fcl::Transform3<FCLFloat> mesh_triangle_tf;
  };
  using DistanceCache = std::unordered_map<DistanceCacheKey, DistanceCacheEntry, DistanceCacheKeyHash, DistanceCacheKeyEqual>;
  // Map a pair of strings for (mesh_id, frame_id) to the correct
  // DistanceCache instance.
  using DistanceCacheMap = std::map<std::pair<std::string, std::string>, DistanceCache>;
  DistanceCacheMap distance_caches_;
  DistanceCacheMap micro_distance_caches_;
  std::pair<std::string, std::string> last_mesh_frame_;
  DistanceCache* last_distance_cache_;
  DistanceCache* last_micro_distance_cache_;

};
// class Costmap3DROS
}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_3D_ROS_H
