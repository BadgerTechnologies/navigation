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
   * @brief  Construct a query object associated with a layered costmap 3D.
   * The query will always be performed on the current layered costmap 3D,
   * and the corresponding costmap 3D must be locked during queries.
   */
  Costmap3DQuery(
      const std::shared_ptr<LayeredCostmap3D>& layered_costmap_3d,
      const std::string& mesh_resource,
      double padding = 0.0,
      unsigned int pose_bins_per_meter = 4,
      unsigned int pose_bins_per_radian = 4,
      unsigned int pose_micro_bins_per_meter = 1024,
      unsigned int pose_micro_bins_per_radian = 1024);

  /**
   * @brief  Construct a query object on a particular costmap 3D.
   * This constructor creates an internal copy of the passed costmap
   * and all queries will be performed on that copy.
   * This is useful for doing a buffered query.
   * Note that the costmap in question should not update during the
   * constructor. If it is the underlying costmap in the LayeredCostmap3D,
   * be sure to lock the LayeredCostmap3D during construction.
   */
  Costmap3DQuery(
      const Costmap3DConstPtr& costmap_3d,
      const std::string& mesh_resource,
      double padding = 0.0,
      unsigned int pose_bins_per_meter = 4,
      unsigned int pose_bins_per_radian = 4,
      unsigned int pose_micro_bins_per_meter = 1024,
      unsigned int pose_micro_bins_per_radian = 1024);

  /**
   * Copy constructor.
   * This is useful to have multiple threads querying the same map.
   * Query objects are not multi-thread safe, so to query the same
   * map from multiple threads, create copies of the query objects. The
   * underlying map is not copied, as std::shared_ptr is used.
   */
  Costmap3DQuery(const Costmap3DQuery& rhs);

  virtual ~Costmap3DQuery();

  /** @brief Get the cost to put the robot base at the given pose.
   *
   * It is assumed the pose is in the frame of the costmap.
   * return value represents the cost of the pose
   * negative is collision, zero is free.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap. */
  virtual double footprintCost(geometry_msgs::Pose pose);

  /** @brief Return whether the given pose is in collision.
   *
   * It is assumed the pose is in the frame of the costmap.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap.
   */
  virtual bool footprintCollision(geometry_msgs::Pose pose);

  /** @brief Return minimum distance to nearest costmap object.
   *
   * It is assumed the pose is in the frame of the costmap.
   * This returns the minimum unsigned distance. So a collision will return <0.0.
   * Negative values are not exact minimum distances. If exact minimum is
   * required use footprintSignedDistance.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap.
   */
  virtual double footprintDistance(geometry_msgs::Pose pose);

  /** @brief Return minimum signed distance to nearest costmap object.
   *
   * It is assumed the pose is in the frame of the costmap.
   * This returns the minimum signed distance. So, the deeper a pose goes into
   * obstacles, the more negative the return value becomes.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap.
   */
  virtual double footprintSignedDistance(geometry_msgs::Pose pose);

protected:
  std::shared_ptr<LayeredCostmap3D> layered_costmap_3d_;

  /** @brief Ensure query map matches currently active costmap.
   * Note: must be called on every query to ensure that the correct Costmap3D is being queried.
   * The LayeredCostmap3D can reallocate the Costmap3D, such as when
   * the resolution changes.
   * Note: assumes the costmap is locked. The costmap should always be locked
   * during query calls when this is called.
   */
  virtual void checkCostmap();

  /** @brief Update the mesh to use for queries. */
  virtual void updateMeshResource(const std::string& mesh_resource, double padding = 0.0);

private:
  std::string getFileNameFromPackageURL(const std::string& url);

  using FCLFloat = float;
  using FCLRobotModel = fcl::BVHModel<fcl::OBBRSS<FCLFloat>>;
  using FCLRobotModelPtr = std::shared_ptr<FCLRobotModel>;

  FCLRobotModelPtr robot_model_;

  using FCLCollisionObject = fcl::CollisionObject<FCLFloat>;
  using FCLCollisionObjectPtr = std::shared_ptr<FCLCollisionObject>;
  FCLCollisionObjectPtr robot_obj_;

  std::shared_ptr<const octomap::OcTree> octree_ptr_;
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
    DistanceCacheKey(const geometry_msgs::Pose& pose, int bins_per_meter, int bins_per_radian)
    {
      binned_pose_ = binPose(pose, bins_per_meter, bins_per_radian);
    }

    size_t hash_value() const
    {
      size_t rv = 0;
      hash_combine(rv, binned_pose_.orientation.x);
      hash_combine(rv, binned_pose_.orientation.y);
      hash_combine(rv, binned_pose_.orientation.z);
      hash_combine(rv, binned_pose_.orientation.w);
      hash_combine(rv, binned_pose_.position.x);
      hash_combine(rv, binned_pose_.position.y);
      hash_combine(rv, binned_pose_.position.z);
      return rv;
    }

    bool operator==(const DistanceCacheKey& rhs) const
    {
      return binned_pose_.orientation.x == rhs.binned_pose_.orientation.x &&
             binned_pose_.orientation.y == rhs.binned_pose_.orientation.y &&
             binned_pose_.orientation.z == rhs.binned_pose_.orientation.z &&
             binned_pose_.orientation.w == rhs.binned_pose_.orientation.w &&
             binned_pose_.position.x == rhs.binned_pose_.position.x &&
             binned_pose_.position.y == rhs.binned_pose_.position.y &&
             binned_pose_.position.z == rhs.binned_pose_.position.z;
    }

  protected:
    geometry_msgs::Pose binned_pose_;

    //! Borrow boost's hash_combine directly to avoid having to pull in boost
    template <class T>
    inline void hash_combine(std::size_t& seed, const T& v) const
    {
      std::hash<T> hasher;
      seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    }

    // Bin a pose.
    inline geometry_msgs::Pose binPose(const geometry_msgs::Pose& pose,
                                       int bins_per_meter,
                                       int bins_per_radian)
    {
      geometry_msgs::Pose rv;
      rv.position.x = std::round(pose.position.x * bins_per_meter) / bins_per_meter;
      rv.position.y = std::round(pose.position.y * bins_per_meter) / bins_per_meter;
      rv.position.z = std::round(pose.position.z * bins_per_meter) / bins_per_meter;
      // bin orientation by RPY angles.
      // another way of binning would be binning by axis/angle
      double yaw, pitch, roll;
      tf2::getEulerYPR<geometry_msgs::Quaternion>(pose.orientation, yaw, pitch, roll);
      yaw = std::round(yaw * bins_per_radian) / bins_per_radian;
      pitch = std::round(pitch * bins_per_radian) / bins_per_radian;
      roll = std::round(roll * bins_per_radian) / bins_per_radian;
      tf2::Quaternion binned_q;
      binned_q.setRPY(roll, pitch, yaw);
      rv.orientation = tf2::toMsg(binned_q);
      return rv;
    }
  };

  struct DistanceCacheKeyHash
  {
    size_t operator()(const DistanceCacheKey& key) const
    {
      return key.hash_value();
    }
  };

  struct DistanceCacheKeyEqual
  {
    bool operator()(const DistanceCacheKey& lhs, const DistanceCacheKey& rhs) const
    {
      return lhs == rhs;
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
    {
    }
    const DistanceCacheEntry& operator=(const DistanceCacheEntry& rhs)
    {
      octomap_box = rhs.octomap_box;
      octomap_box_tf = rhs.octomap_box_tf;
      mesh_triangle = rhs.mesh_triangle;
      return *this;
    }
    DistanceCacheEntry(const fcl::DistanceResult<FCLFloat>& result)
    {
      assert(result.primitive1);
      assert(result.primitive2);
      octomap_box = std::dynamic_pointer_cast<fcl::Box<FCLFloat>>(result.primitive1);
      octomap_box_tf = result.tf1;
      mesh_triangle = std::dynamic_pointer_cast<fcl::TriangleP<FCLFloat>>(result.primitive2);
      assert(octomap_box);
      assert(mesh_triangle);
    }
    void setupResult(fcl::DistanceResult<FCLFloat>* result)
    {
      result->primitive1 = octomap_box;
      result->primitive2 = mesh_triangle;
      result->tf1 = octomap_box_tf;
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
  };
  using DistanceCache = std::unordered_map<DistanceCacheKey, DistanceCacheEntry, DistanceCacheKeyHash, DistanceCacheKeyEqual>;
  /**
   * The distance cache allows us to find a very good distance guess quickly.
   * The cache memorizes to a hash table for a pose rounded to the number of
   * bins the mesh triangle and octomap box pair for the last distance query
   * in that pose bin. The distance is recalculated with the new pose, and
   * used as the starting guess to radically prune the search tree for
   * distance queries. This results in no loss of correctness and a huge
   * speed-up when distance queries are over the same space.
   */
  DistanceCache distance_cache_;
  /**
   * The micro-distance cache allows us to return a guess as to the distance
   * based on hitting the micro-cache. The number of bins in the micro-cache
   * must be large to prevent gross error. If the micro-cache is hit on a
   * distance query, instead of making a query, the cached triangle is
   * transformed to the new pose, and the distance between that triangle and
   * the previous octomap box is returned immediately. This results in a
   * massive speed-up when querying very small changes in pose (which may
   * be done to calculate derivatives, for instance). Do note that this
   * cache is a potential source of minor error, so in cases where that is
   * important use a very high number of bins (which will reduce its
   * efficiency for the sake of accuracy).
   */
  DistanceCache micro_distance_cache_;
  unsigned int last_layered_costmap_update_number_;
  //! Distance cache bins per meter for binning the pose's position
  unsigned int pose_bins_per_meter_;
  //! Distance cache bins per radian for binning the pose's orientation
  unsigned int pose_bins_per_radian_;
  //! Micro-distance cache bins per meter for binning the pose's position
  unsigned int pose_micro_bins_per_meter_;
  //! Micro-distance cache bins per radian for binning the pose's position
  unsigned int pose_micro_bins_per_radian_;
};

using Costmap3DQueryPtr = std::shared_ptr<Costmap3DQuery>;
using Costmap3DQueryConstPtr = std::shared_ptr<const Costmap3DQuery>;

}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_3D_QUERY_H_
