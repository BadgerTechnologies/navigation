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

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/utility.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <costmap_3d/layered_costmap_3d.h>
#include <costmap_3d/crop_hull.h>
#include <costmap_3d_msgs/GetPlanCost3DService.h>
#include <costmap_3d/bin_pose.h>
#include <costmap_3d/fcl_helper.h>
#include <costmap_3d/interior_collision_lut.h>
#include <costmap_3d/octree_solver.h>

namespace costmap_3d
{

/** @brief Query a 3D Costmap. */
class Costmap3DQuery
{
public:
  using QueryRegionScale = Eigen::Vector3d;

  /**
   * @brief  Construct a query object associated with a layered costmap 3D.
   * The query will always be performed on the current layered costmap 3D,
   * and the corresponding costmap 3D must be locked during queries.
   */
  Costmap3DQuery(
      const LayeredCostmap3D* layered_costmap_3d,
      const std::string& mesh_resource,
      double padding = 0.0,
      Costmap3DQuery::QueryRegionScale = Costmap3DQuery::QueryRegionScale::Zero(),
      unsigned int pose_bins_per_meter = 4,
      unsigned int pose_bins_per_rotation = 16,
      unsigned int pose_milli_bins_per_meter = 20,
      unsigned int pose_milli_bins_per_rotation = 80,
      unsigned int pose_micro_bins_per_meter = 1024,
      unsigned int pose_micro_bins_per_rotation = 4096);

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
      Costmap3DQuery::QueryRegionScale = Costmap3DQuery::QueryRegionScale::Zero(),
      unsigned int pose_bins_per_meter = 4,
      unsigned int pose_bins_per_rotation = 32,
      unsigned int pose_milli_bins_per_meter = 20,
      unsigned int pose_milli_bins_per_rotation = 160,
      unsigned int pose_micro_bins_per_meter = 1024,
      unsigned int pose_micro_bins_per_rotation = 8192);

  virtual ~Costmap3DQuery();

  /// Which region of the map to query at the query pose.
  using QueryRegion = uint8_t;
  static constexpr QueryRegion ALL = costmap_3d_msgs::GetPlanCost3DService::Request::COST_QUERY_REGION_ALL;
  static constexpr QueryRegion LEFT = costmap_3d_msgs::GetPlanCost3DService::Request::COST_QUERY_REGION_LEFT;
  static constexpr QueryRegion RIGHT = costmap_3d_msgs::GetPlanCost3DService::Request::COST_QUERY_REGION_RIGHT;
  static constexpr QueryRegion RECTANGULAR_PRISM =
    costmap_3d_msgs::GetPlanCost3DService::Request::COST_QUERY_REGION_RECTANGULAR_PRISM;
  static constexpr QueryRegion MAX = RECTANGULAR_PRISM+1;

  /// What kind of obstacles to consider for the query.
  using QueryObstacles = uint8_t;
  static constexpr QueryObstacles LETHAL_ONLY =
    costmap_3d_msgs::GetPlanCost3DService::Request::COST_QUERY_OBSTACLES_LETHAL_ONLY;
  static constexpr QueryObstacles NONLETHAL_ONLY =
    costmap_3d_msgs::GetPlanCost3DService::Request::COST_QUERY_OBSTACLES_NONLETHAL_ONLY;
  static constexpr QueryObstacles OBSTACLES_MAX = NONLETHAL_ONLY+1;

  /** @brief Get the cost to put the robot base at the given pose.
   *
   * The region of the map considered is limited by the query_region.
   *
   * It is assumed the pose is in the frame of the costmap.
   * return value represents the cost of the pose
   * negative is collision, zero is free.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap. */
  virtual double footprintCost(const geometry_msgs::Pose& pose, QueryRegion query_region = ALL);

  /** @brief Return whether the given pose is in collision.
   *
   * The region of the map considered is limited by the query_region.
   *
   * It is assumed the pose is in the frame of the costmap.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap.
   */
  virtual bool footprintCollision(const geometry_msgs::Pose& pose, QueryRegion query_region = ALL);

  /** @brief Return minimum distance to nearest costmap object.
   *
   * The region of the map considered is limited by the query_region.
   *
   * It is assumed the pose is in the frame of the costmap.
   * This returns the minimum distance, or negative for penetration.
   * Negative values are not based on penetration depth and may always be a
   * constant value like -1.0. If penetrations must be modeled use
   * footprintSignedDistance. A case where penetrations should be modeled would
   * be to approximate the motion necessary to avoid the penetrating obstacle.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap.
   */
  virtual double footprintDistance(const geometry_msgs::Pose& pose,
                                   QueryRegion query_region = ALL,
                                   double relative_error = 0.05);

  /** @brief Return minimum signed distance to nearest costmap object.
   *
   * The region of the map considered is limited by the query_region.
   *
   * It is assumed the pose is in the frame of the costmap.
   * This returns the signed distance. For non-penetration cases, the return
   * value is the same as footprintDistance. For penetrating collisions, an
   * approximation of one of the negative penetration depths will be returned.
   * This approximation has the property that for most cases the derivative
   * obtained by a small perturbation in pose will point away from the
   * penetration. However, if the pose is moved enough away from the
   * penetration, some other penetrating point may be chosen in the case of
   * multiple penetrations. This should normally result in a net negative
   * direction to the derivative, but the more penetrations the worse the
   * approximation becomes. Finding the absolute deepest penetration depth is
   * not worth the extra computational cost, and for navigation purposes is
   * rarely necessary.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap.
   */
  virtual double footprintSignedDistance(const geometry_msgs::Pose& pose,
                                         QueryRegion query_region = ALL,
                                         double relative_error = 0.05,
                                         bool exact_signed_distance = false);

  using FCLFloat = double;
  struct DistanceOptions
  {
    //! Which region of the costmap to query
    QueryRegion query_region = ALL;
    //! What kind of obstacles are considered for the query
    QueryObstacles query_obstacles = LETHAL_ONLY;
    //! Whether to find the signed distance on collision
    bool signed_distance = false;
    /** Whether to find the exact signed distance, or relaxed signed distance
     * (where derivatives are consistent, collisions are correct, but the exact
     * penetration depth is not returned). Only effective if signed_distance is true
     */
    bool exact_signed_distance = false;
    //! Acceptable relative error limit (improves runtime)
    double relative_error = 0.05;
    /**
     * If set to true, a milli or micro cache hit that is above the cache
     * threshold parameters set in the query will be directly used to calculate
     * the distance. This can introduce an error up to the size of the
     * resolution of the corresponding cache, and is set by default to speed up
     * queries. Turn this off to get exact distance results when there are no
     * collisions. Note: normal cache hits are never directly used.
     */
    bool directly_use_cache_when_above_threshold = true;
    /**
     * If set to true, a milli or micro cache hit that is at or below the cache
     * threshold parameters set in the query will still be directly used to
     * calculate the distance as long as the new pose is not in collision.
     * Finding the collision distance is much faster than the true overall
     * signed distance. If no collision is found, then the cache entry is used
     * directly as if it had been above the threshold.  This can introduce a
     * large relative error as some other costmap cell may be closer (within
     * the resolution of the corresponding cache) and is therefore off by
     * default.  Turn this on to get even faster queries at the expense of
     * relative error when near a costmap voxel.
     */
    bool directly_use_cache_when_below_threshold = false;
    /** Limit search distance.
     * This is useful for very small limits (near zero) or very large limits.
     * For in-between values it can defeat the usefulness of all the caches
     * and actually result in worse performance.
     */
    FCLFloat distance_limit = std::numeric_limits<FCLFloat>::max();
  };

  class DistanceResult
  {
  public:
    DistanceResult() { clear(); }
    void clear()
    {
      mesh_triangle_id = -1;
    }
    explicit operator bool() const
    {
      return mesh_triangle_id >= 0;
    }
    fcl::Box<FCLFloat> octomap_box;
    fcl::Vector3<FCLFloat> octomap_box_center;
    int mesh_triangle_id;
  };

  /** @brief Alternate footprintDistance interface taking a DistanceOptions */
  virtual double footprintDistance(
      const geometry_msgs::Pose& pose, const DistanceOptions& opts, DistanceResult* result = nullptr);

  /** @brief Recalculate footprintDistance with a previous result at a new pose. */
  virtual double footprintDistance(const geometry_msgs::Pose& pose, const DistanceResult& result);

  /** @brief get a const reference to the padded robot mesh points being used */
  const pcl::PointCloud<pcl::PointXYZ>& getRobotMeshPoints() const {return *robot_mesh_points_;}

  /** @brief get a const reference to the mesh polygons being used */
  const std::vector<pcl::Vertices>& getRobotMeshPolygons() const {return robot_mesh_.polygons;}

  /** @brief Set scale for query region.
   *
   * Some query regions have a scaling vector, such as rectangular prism.
   * It is more efficient to set the scaling per-query object in most cases.
   * This method must hold the internal lock, so this interface is not useful
   * for parallel queries. In such cases, use the DistanceOptions to set the
   * scale per queried pose.
   */
  void setQueryRegionDefaultScale(QueryRegionScale default_scale);

  /** @brief Set the cache bin sizes.
   *
   * Note: this does not invalidate any caches but does grab the instance lock.
   */
  void setCacheBinSize(
      unsigned int pose_bins_per_meter = 4,
      unsigned int pose_bins_per_rotation = 32,
      unsigned int pose_milli_bins_per_meter = 20,
      unsigned int pose_milli_bins_per_rotation = 160,
      unsigned int pose_micro_bins_per_meter = 1024,
      unsigned int pose_micro_bins_per_rotation = 8192);

  /** @brief Calculate the distance cache thresholds.
   *
   * Calculate the thresholds for the distance cache given details about the
   * usage of the query object (two_d by default) and a threshold factor (which
   * defaults to 1.05). The mode and the factor are memorized and the
   * thresholds are re-calculated if the bin sizes are changed.
   */
  void setCacheThresholdParameters(
      bool threshold_two_d_mode = false,
      double threshold_factor = 1.05);

  /** @brief set the layered costmap update number.
   *
   * This is useful for buffered queries to store which costmap update they represent.
   */
  void setLayeredCostmapUpdateNumber(unsigned int n) { last_layered_costmap_update_number_ = n; }

  /** @brief get the layered costmap update number.
   *
   * This is useful for buffered queries to store which costmap update they represent.
   */
  unsigned int getLayeredCostmapUpdateNumber() const { return last_layered_costmap_update_number_; }

  /** @brief change the costmap associated with this query.
   *
   * This is useful for a buffered query to save the robot mesh LUT, which
   * would be re-created if a new query is allocated.
   *
   * Note: for a buffered query it is necessary to ensure that this method is
   * only called when there are no outstanding queries. Buffered queries are
   * optimized to not hold the instance_mutex_ during the query, as the only
   * time a buffered query can change is when either updateCostmap or
   * updateMeshResource is called. This is not true for an associated query, as
   * the costmap may change between queries with no update method being called.
   */
  void updateCostmap(const Costmap3DConstPtr& costmap_3d);

  using FCLRobotModel = fcl::BVHModel<fcl::OBBRSS<FCLFloat>>;
  using FCLRobotModelPtr = std::shared_ptr<FCLRobotModel>;
  using FCLRobotModelConstPtr = std::shared_ptr<const FCLRobotModel>;

  /** @brief get the FCL robot model being used.
   *
   * The main use-case of this method is for testing purposes to verify
   * results by doing an FCL broadphase distance check.
   */
  FCLRobotModelConstPtr getFCLRobotModel() const { return robot_model_; }

  // returns path to package file, or empty on error
  static std::string getFileNameFromPackageURL(const std::string& url);

protected:
  const LayeredCostmap3D* layered_costmap_3d_;

  // Use std::shared_timed_mutex to only require C++14 support.
  // std::shared_mutex was not added until C++17. This class does not use the
  // timed aspect, so if all users are on C++17 this can be converted to
  // C++17's std::shared_mutex
  using shared_mutex = std::shared_timed_mutex;
  using shared_lock = std::shared_lock<shared_mutex>;
  using unique_lock = std::unique_lock<shared_mutex>;

  /** @brief Ensure query map matches currently active costmap.
   * Note: must be called on every query to ensure that the correct Costmap3D is being queried.
   * The LayeredCostmap3D can reallocate the Costmap3D, such as when
   * the resolution changes.
   * Note: assumes the costmap is locked. The costmap should always be locked
   * during query calls when this is called.
   * If the associated layered_costmap_3d_ is empty the new_octree is used instead.
   */
  virtual void checkCostmap(std::shared_ptr<const octomap::OcTree> new_octree = nullptr);

  /** @brief See if checkCostmap may be needed.
   * Note: must be called holding at least a shared lock on the instance_mutex_
   * Returns true if checkCostmap should be called. The whole point of this
   * method is to keep from having to obtain an upgrade lock unless the costmap
   * may actually have changed, which can be determined while holding a shared
   * lock.
   */
  virtual bool needCheckCostmap(std::shared_ptr<const octomap::OcTree> new_octree = nullptr);

  /** @brief Update the mesh to use for queries.
   *
   * Note: for a buffered query it is necessary to ensure that this method is
   * only called when there are no outstanding queries. Buffered queries are
   * optimized to not hold the instance_mutex_ during the query, as the only
   * time a buffered query can change is when either updateCostmap or
   * updateMeshResource is called. This is not true for an associated query, as
   * the costmap may change between queries with no update method being called.
   */
  virtual void updateMeshResource(const std::string& mesh_resource, double padding = 0.0);

  /** @brief core of distance calculations */
  virtual double calculateDistance(const geometry_msgs::Pose& pose,
                                   const DistanceOptions& opts,
                                   DistanceResult* return_result = nullptr);

private:
  // Common initialization between all constructors
  void init();
  // Protect instance state except for distance caches. This way buffered
  // queries can skip using the instance_mutex_ as they are only ever updated
  // when there are no queries outstanding. The only state then that must be
  // synchronized are the distance caches.
  shared_mutex instance_mutex_;

  // Save the PCL model of the mesh to use with crop hull
  pcl::PolygonMesh robot_mesh_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr robot_mesh_points_;

  // Use the costmap_3d version of CropHull that is thread safe
  CropHull<pcl::PointXYZ> crop_hull_;

  using FCLSolver = fcl::detail::GJKSolver_libccd<FCLFloat>;

  FCLRobotModelPtr robot_model_;
  // The halfspaces are indexed the same as the robot model, and shared with
  // the interior collision LUT, so make them a shared pointer
  std::shared_ptr<std::vector<fcl::Halfspace<FCLFloat>>> robot_model_halfspaces_;

  using FCLCollisionObject = fcl::CollisionObject<FCLFloat>;
  using FCLCollisionObjectPtr = std::shared_ptr<FCLCollisionObject>;

  std::shared_ptr<const octomap::OcTree> octree_ptr_;
  // Store a copy of the octree with only nonlethal nodes present to make
  // nonlethal queries more efficient. Since the vanilla octree only stores the
  // maximum occupancy of children nodes inside inner nodes, there is no
  // efficient way to query nonlethal. Fix this by making a copy of the octree
  // when the first nonlethal query is issued to use for nonlethal queries.
  std::shared_ptr<const octomap::OcTree> nonlethal_octree_ptr_;

  QueryRegionScale query_region_scale_;

  void padPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr points, float padding)
  {
    for (unsigned int index = 0; index < points->size(); index++)
    {
      float x = points->points[index].x;
      float y = points->points[index].y;
      float dist = sqrt(x * x + y * y);
      if (dist == 0.0)
      {
        // Do not pad the origin.
        // The code below will divide by zero, go to the next point
        continue;
      }
      float nx = x / dist;
      float ny = y / dist;
      points->points[index].x = nx * (dist + padding);
      points->points[index].y = ny * (dist + padding);
    }  // for point
  }

  // Add fcl triangles to the mesh vector for all triangles in a PCL polygon
  void addPCLPolygonToFCLTriangles(
      const pcl::Vertices& polygon,
      std::vector<fcl::Triangle>* fcl_triangles);

  // Add a padded PCL mesh into the given robot model
  void addPCLPolygonMeshToRobotModel(
      const pcl::PolygonMesh& pcl_mesh,
      double padding,
      FCLRobotModel* robot_model);

  InteriorCollisionLUT<FCLFloat> interior_collision_lut_;
  double last_octomap_resolution_;
  void checkInteriorCollisionLUT();
  FCLFloat boxHalfspaceSignedDistance(
      const fcl::Box<FCLFloat>& box,
      const fcl::Vector3<FCLFloat>& box_center,
      int mesh_triangle_id,
      const fcl::Transform3<FCLFloat>& mesh_tf) const;

  class DistanceCacheKey
  {
  public:
    // Distance cache keys aren't valid until they are initialized.
    // DistanceCacheKeys are used on the fast-path of the queries, so dynamic
    // memory needs to be avoided. The user of the cache then will allocate
    // a DistanceCacheKey on their stack and then have the DistanceCache
    // initialize it separately.
    DistanceCacheKey() = default;

    void initialize(
        const geometry_msgs::Pose& pose,
        QueryRegion query_region,
        QueryObstacles query_obstacles,
        int bins_per_meter = 0,
        int bins_per_rotation = 0)
    {
      // If bins_per_meter or bins_per_rotation are zero, the cache is disabled.
      // The key is allowed to be calculated anyway as an exact key (and this
      // may simplify the caller in cases where it wants to setup a cache key
      // even if the caches are disabled, then check later).
      if (bins_per_meter > 0 && bins_per_rotation > 0)
      {
        binned_pose_ = binPose(pose, bins_per_meter, bins_per_rotation);
      }
      else
      {
        binned_pose_ = pose;
      }
      query_region_ = query_region;
      query_obstacles_ = query_obstacles;
      hash_ = hash_value();
    }

    // Directly store the hash value in a public location for speed.
    size_t hash_;

    bool operator==(const DistanceCacheKey& rhs) const
    {
      return binned_pose_.orientation.x == rhs.binned_pose_.orientation.x &&
             binned_pose_.orientation.y == rhs.binned_pose_.orientation.y &&
             binned_pose_.orientation.z == rhs.binned_pose_.orientation.z &&
             binned_pose_.orientation.w == rhs.binned_pose_.orientation.w &&
             binned_pose_.position.x == rhs.binned_pose_.position.x &&
             binned_pose_.position.y == rhs.binned_pose_.position.y &&
             binned_pose_.position.z == rhs.binned_pose_.position.z &&
             query_region_ == rhs.query_region_ &&
             query_obstacles_ == rhs.query_obstacles_;
    }
    QueryObstacles getQueryObstacles() const { return query_obstacles_; }

  protected:
    geometry_msgs::Pose binned_pose_;
    QueryRegion query_region_;
    QueryObstacles query_obstacles_;

    size_t hash_value() const
    {
      // Compute the hash off the raw bits by treating the doubles as if they
      // were unsigned 64-bit integers, multiplying them by medium sized
      // consecutive primes and summing them up. This operation is SIMD
      // friendly and much faster than std::hash, and works well for the types
      // of floating point coordinates encountered in Costmap queries.
      union {double d; uint64_t uint;} u[8] = {
          binned_pose_.orientation.x,
          binned_pose_.orientation.y,
          binned_pose_.orientation.z,
          binned_pose_.orientation.w,
          binned_pose_.position.x,
          binned_pose_.position.y,
          binned_pose_.position.z,
          {.uint = (static_cast<uint64_t>(query_region_) << 8 | static_cast<uint64_t>(query_obstacles_))},
      };
      uint64_t primes[8] = {
          30011,
          30013,
          30029,
          30047,
          30059,
          30071,
          30089,
          30091,
      };
      // Make the hash SIMD friendly by using primes and addition instead of
      // hash_combine which must be done sequentially.
      uint64_t rv = 0;
      for (unsigned i=0; i < 8; ++i)
      {
        rv += u[i].uint * primes[i];
      }
      return static_cast<size_t>(rv);
    }
  };

  struct DistanceCacheKeyHash
  {
    size_t operator()(const DistanceCacheKey& key) const
    {
      return key.hash_;
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
    DistanceCacheEntry() { clear(); }
    DistanceCacheEntry(const DistanceCacheEntry& rhs)
        : distance(rhs.distance),
          octomap_box(rhs.octomap_box),
          octomap_box_center(rhs.octomap_box_center),
          mesh_triangle_id(rhs.mesh_triangle_id)
    {
    }
    const DistanceCacheEntry& operator=(const DistanceCacheEntry& rhs)
    {
      distance = rhs.distance;
      octomap_box = rhs.octomap_box;
      octomap_box_center = rhs.octomap_box_center;
      mesh_triangle_id = rhs.mesh_triangle_id;
      return *this;
    }
    explicit DistanceCacheEntry(const OcTreeMeshSolver<FCLSolver>::DistanceResult& result)
        : distance(result.min_distance),
          octomap_box(result.octomap_box),
          octomap_box_center(result.octomap_box_center),
          mesh_triangle_id(result.mesh_triangle_id)
    {
    }
    explicit DistanceCacheEntry(const DistanceResult& result)
        : octomap_box(result.octomap_box),
          octomap_box_center(result.octomap_box_center),
          mesh_triangle_id(result.mesh_triangle_id)
    {
    }
    void setupResult(OcTreeMeshSolver<FCLSolver>::DistanceResult* result)
    {
      result->octomap_box = octomap_box;
      result->octomap_box_center = octomap_box_center;
      result->mesh_triangle_id = mesh_triangle_id;
    }
    void setupResult(DistanceResult* result)
    {
      if (result)
      {
        result->octomap_box = octomap_box;
        result->octomap_box_center = octomap_box_center;
        result->mesh_triangle_id = mesh_triangle_id;
      }
    }
    void clear()
    {
      mesh_triangle_id = -1;
    }
    explicit operator bool() const
    {
      return mesh_triangle_id >= 0;
    }
    bool getCostmapIndexAndDepth(
        const octomap::OcTreeSpace& octree_space,
        Costmap3DIndex* index,
        unsigned int* depth) const
    {
      if (*this)
      {
        // Use the size and location of the box to derive the correct octree key
        // (cell index) and depth.
        //
        // The depth can be found by looking at the ratio of the largest node
        // size to the size of our box. The base-2 logarithm of that value
        // will be the depth we are at, rounded to the nearest integer.
        //
        // Note: if this function needs to be optimized in the future, it could
        // use __builtin_clz. The initial use case of this function only calls
        // it for every main distance cache entry, which is small. Therefore
        // a standard C++ implementation was chosen.
        unsigned int d = static_cast<unsigned int>(std::round(std::log2(
                    octree_space.getNodeSize(0) / octomap_box.side[0])));
        const auto& box_center = octomap_box_center;
        // Be sure to adjust the key based on the depth.
        *index = octree_space.coordToKey(box_center[0], box_center[1], box_center[2], d);
        *depth = d;
        return true;
      }
      return false;
    }
    FCLFloat distance;
    fcl::Box<FCLFloat> octomap_box;
    fcl::Vector3<FCLFloat> octomap_box_center;
    int mesh_triangle_id;
  };
  // Internal class to decompose a region of interest into a set of halfspaces.
  // Because this internal class is used on the fast-paths, avoid dynamic memory.
  class RegionsOfInterestAtPose
  {
  public:
    // Construct a region of interest at a pose. Internally store the set of halfspaces.
    RegionsOfInterestAtPose(
        QueryRegion query_region, const QueryRegionScale& query_region_scale, const geometry_msgs::Pose& pose)
    {
      if (query_region == LEFT || query_region == RIGHT)
      {
        constexpr unsigned int NUM_ROIS = 1;
        static_assert(NUM_ROIS <= ROIS_CAPACITY, "ROIS_CAPACITY is too small, increase to match NUM_ROIS");
        rois_size_ = NUM_ROIS;
        fcl::Vector3<FCLFloat> normal;
        // Note, for FCL halfspaces, the inside space is that below the plane, so
        // the normal needs to point away from the query region
        if (query_region == LEFT)
        {
          normal << 0.0, -1.0, 0.0;
        }
        else if (query_region == RIGHT)
        {
          normal << 0.0, 1.0, 0.0;
        }
        rois_[0] = fcl::transform(fcl::Halfspace<FCLFloat>(normal, 0.0), poseToFCLTransform<FCLFloat>(pose));
      }
      else if (query_region == RECTANGULAR_PRISM)
      {
        constexpr unsigned int NUM_ROIS = 5;
        static_assert(NUM_ROIS <= ROIS_CAPACITY, "ROIS_CAPACITY is too small, increase to match NUM_ROIS");
        rois_size_ = NUM_ROIS;
        fcl::Vector3<FCLFloat> normals[NUM_ROIS] = {
            fcl::Vector3<FCLFloat>(-1.0, 0.0, 0.0),
            fcl::Vector3<FCLFloat>(0.0, 1.0, 0.0),
            fcl::Vector3<FCLFloat>(0.0, -1.0, 0.0),
            fcl::Vector3<FCLFloat>(0.0, 0.0, 1.0),
            fcl::Vector3<FCLFloat>(0.0, 0.0, -1.0),
        };
        FCLFloat distances[NUM_ROIS] = {
            0.0,
            query_region_scale(1) / 2.0,
            query_region_scale(1) / 2.0,
            query_region_scale(2) / 2.0,
            query_region_scale(2) / 2.0,
        };
        const auto fcl_xform = poseToFCLTransform<FCLFloat>(pose);
        for (unsigned int i = 0; i < NUM_ROIS; ++i)
        {
          rois_[i] = fcl::transform(fcl::Halfspace<FCLFloat>(normals[i], distances[i]), fcl_xform);
        }
      }
      assert(rois_size_ <= ROIS_CAPACITY);
    }
    // Test if the given distance cache entry is inside the region.
    // Regions are always inclusive. A voxel on the region boundary is
    // considered "inside".
    bool distanceCacheEntryInside(const DistanceCacheEntry& cache_entry) const
    {
      const auto& box = cache_entry.octomap_box;
      const auto& box_center = cache_entry.octomap_box_center;

      for (unsigned int i = 0; i < rois_size_; ++i)
      {
        if (costmap_3d::boxHalfspaceSignedDistance<FCLFloat>(box, box_center, rois_[i]) > 0)
        {
          return false;
        }
      }
      // In or on each halfspace
      return true;
    }
    // Apply this region to the DistanceRequest.
    void setupRequest(OcTreeMeshSolver<FCLSolver>::DistanceRequest* request) const
    {
      request->roi_ptr = rois_;
      request->roi_size = rois_size_;
    }
  private:
    unsigned int rois_size_ = 0;
    // To avoid dynamic memory allocation use a fixed-size array.
    // This is an added point of maintenance, but is necessary for optimum
    // query performance. If a new region is added that requires more than
    // the current number of regions, ROIS_CAPACITY must also be changed
    // to match.
    static constexpr unsigned int ROIS_CAPACITY = 5;
    fcl::Halfspace<FCLFloat> rois_[ROIS_CAPACITY];
  };
  // used by distance calculation to find interior collisions
  double handleDistanceInteriorCollisions(
      const DistanceCacheEntry& cache_entry,
      const geometry_msgs::Pose& pose);
  using DistanceCacheMap =
    std::unordered_map<DistanceCacheKey, DistanceCacheEntry, DistanceCacheKeyHash, DistanceCacheKeyEqual>;
  template <bool exact_cache = false>
  class DistanceCacheImpl
  {
  public:
    using DistanceFunction = std::function<double(const DistanceCacheEntry&, const geometry_msgs::Pose&)>;

    void setDistanceFunction(DistanceFunction distance_function)
    {
      distance_function_ = distance_function;
    }

    void setRobotMeshPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& robot_mesh_points)
    {
      if (exact_cache)
      {
        // exact caches don't need the robot mesh points
        return;
      }
      // Called sparingly, not on normal planning path, so just grab write lock
      unique_lock cache_write_lock(mutex_);
      robot_mesh_points_ = robot_mesh_points;
      calculateDistanceThresholds();
    }

    void setBinSize(unsigned int bins_per_meter, unsigned int bins_per_rotation)
    {
      if (exact_cache)
      {
        // exact caches don't need bins
        return;
      }
      bool changed = false;
      {
        shared_lock cache_read_lock(mutex_);
        if (bins_per_meter != bins_per_meter_ || bins_per_rotation != bins_per_rotation_)
        {
          changed = true;
        }
      }
      if (changed)
      {
        unique_lock cache_write_lock(mutex_);
        if (bins_per_meter != bins_per_meter_ || bins_per_rotation != bins_per_rotation_)
        {
          cache_.clear();
          bins_per_meter_ = bins_per_meter;
          bins_per_rotation_ = bins_per_rotation;
          ROS_INFO_STREAM(debug_prefix_ << "distance cache bins per meter: " << bins_per_meter_);
          ROS_INFO_STREAM(debug_prefix_ << "distance cache bins per rotation: " << bins_per_rotation);
          calculateDistanceThresholds();
        }
      }
    }

    void setThresholdParameters(bool threshold_two_d_mode, double threshold_factor)
    {
      if (exact_cache)
      {
        // exact caches don't need thresholds, just skip as printing them is pointless
        return;
      }
      bool changed = false;
      {
        shared_lock cache_read_lock(mutex_);
        if (threshold_two_d_mode_ != threshold_two_d_mode || threshold_factor_ != threshold_factor)
        {
          changed = true;
        }
      }
      if (changed)
      {
        unique_lock cache_write_lock(mutex_);
        if (threshold_two_d_mode_ != threshold_two_d_mode || threshold_factor_ != threshold_factor)
        {
          threshold_two_d_mode_ = threshold_two_d_mode;
          threshold_factor_ = threshold_factor;
          calculateDistanceThresholds();
        }
      }
    }

    void calculateDistanceThresholds()
    {
      if (exact_cache)
      {
        // exact caches don't need thresholds, just skip as printing them is pointless
        return;
      }
      if (!robot_mesh_points_)
      {
        ROS_INFO_STREAM(debug_prefix_ << "distance cache: skipping threshold calculation: no robot mesh points");
        return;
      }
      // The distance thresholds must be greater than the maximum translation
      // error plus the maximum rotation error, otherwise the query may not
      // return a collision when one is actually present. In a robot with full
      // three-dimensional movement this would equate to the length of the
      // diagonal of a bin plus the chord length corresponding to the size of a
      // rotational bin given the maximum radius of the robot mesh. For
      // two-dimensional movement of a robot on a plane aligned with the
      // costmap, the threshold is lower at only the diagonal of the bin
      // projected into the plane as a square and the chord length given the
      // radius of the robot mesh projected into the plane (the robot footprint
      // radius).
      Eigen::Vector4f origin(0.0, 0.0, 0.0, 0.0), max_pt(0.0, 0.0, 0.0, 0.0);
      if (threshold_two_d_mode_)
      {
        // Create a 2D version of the mesh cloud by truncating the Z value.
        pcl::PointCloud<pcl::PointXYZ> mesh_points_2d(*robot_mesh_points_);
        for (auto& point : mesh_points_2d)
        {
          point.z = 0.0;
        }
        pcl::getMaxDistance(mesh_points_2d, origin, max_pt);
      }
      else
      {
        pcl::getMaxDistance(*robot_mesh_points_, origin, max_pt);
      }
      const Eigen::Vector3f max_pt3 = max_pt.head<3>();
      double mesh_radius = max_pt3.norm();
      ROS_INFO_STREAM(debug_prefix_ << "distance cache mesh radius: " << mesh_radius << "m");
      if (bins_per_meter_ > 0)
      {
        const double diagonal_factor = threshold_two_d_mode_ ? std::sqrt(2.0) : std::sqrt(3.0);
        // Because binPoseAngularDistanceLimit measures the maximum rotational size
        // of a bin, it may return up to 2 * M_PI, but the maximum chord length is
        // at an angle of M_PI, so do not use an angular error higher than M_PI.
        const double max_angular_error = std::min(M_PI, binPoseAngularDistanceLimit(
            bins_per_rotation_, threshold_two_d_mode_));
        // Add the diagonal length of positional error plus the chord length of the
        // maximum angular error.
        threshold_ = diagonal_factor / bins_per_meter_ +
          2.0 * mesh_radius * std::sin(0.5 * max_angular_error);
        threshold_ *= threshold_factor_;
        ROS_INFO_STREAM(debug_prefix_ << "distance cache threshold: " << threshold_ << "m");
      }
      else
      {
        threshold_ = std::numeric_limits<double>::infinity();
      }
    }

    inline void initializeKey(
        const geometry_msgs::Pose& pose,
        const QueryRegion query_region,
        const QueryObstacles query_obstacles,
        DistanceCacheKey* key_ptr)
    {
      // Do not grab the mutex at all, if the caller alters the number of bins
      // per pose this key will just bin wrong, which does mean the cache will
      // miss, but because a cache is only to improve performance there is no
      // error introduced by this. Grabbing the lock is more expensive as it
      // holds off other writers to the cache while binning the pose.
      key_ptr->initialize(pose, query_region, query_obstacles, bins_per_meter_, bins_per_rotation_);
    }

    // First return value is true when cache is hit.
    // Second return value is true for a fast hit.
    std::pair<bool, bool> checkCache(
        const DistanceCacheKey& key,
        const geometry_msgs::Pose& pose,
        DistanceCacheEntry* cache_entry_ptr,
        double* distance_ptr,
        const RegionsOfInterestAtPose* rois_ptr = nullptr,
        bool track_statistics = false,
        const std::chrono::high_resolution_clock::time_point* start_time_ptr = nullptr,
        std::atomic<unsigned int>* fast_hits_since_clear_ptr = nullptr,
        std::atomic<uint64_t>* fast_hits_since_clear_us_ptr = nullptr,
        bool exact_signed_distance = false,
        bool directly_use_cache_when_above_threshold = false)
    {
      // Be sure the entry is cleared in case the caller is reusing the same
      // entry, otherwise there will be a false hit.
      cache_entry_ptr->clear();
      if (exact_cache || (bins_per_meter_ > 0 && bins_per_rotation_ > 0))
      {
        shared_lock cache_read_lock(mutex_);
        auto found_entry = cache_.find(key);
        if (found_entry != cache_.end())
        {
          *cache_entry_ptr = found_entry->second;
        }
      }
      if (*cache_entry_ptr && (!rois_ptr || rois_ptr->distanceCacheEntryInside(*cache_entry_ptr)))
      {
        // Cache hit, find the distance between the mesh triangle at the new
        // pose and the octomap box, and use this as our initial guess in the
        // result. This greatly prunes the search tree, yielding a big increase
        // in runtime performance.
        assert(distance_function_);
        double distance = distance_function_(*cache_entry_ptr, pose);
        *distance_ptr = distance;
        // Fast path.
        // Take the fast path on a hit in an exact cache, or
        // when not in exact signed distance and there is a collision or
        // when directly using the cache above the threshold and the cache
        // entry is still valid and above the threshold
        if (exact_cache ||
            (!exact_signed_distance && distance <= 0.0) ||
            (directly_use_cache_when_above_threshold &&
            std::isfinite(cache_entry_ptr->distance) &&
            cache_entry_ptr->distance > threshold_))
        {
          if (track_statistics)
          {
            fast_hits_since_clear_ptr->fetch_add(1, std::memory_order_relaxed);
            fast_hits_since_clear_us_ptr->fetch_add(
                std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::high_resolution_clock::now() - *start_time_ptr).count(),
                std::memory_order_relaxed);
          }
          return std::make_pair(true, true);
        }
        return std::make_pair(true, false);
      }
      return std::make_pair(false, false);
    }

    void updateCache(const DistanceCacheKey& key, const DistanceCacheEntry& entry)
    {
      if (exact_cache || (bins_per_meter_ > 0 && bins_per_rotation_ > 0))
      {
        // While it may seem expensive to copy the cache entries into the
        // caches, it prevents cache aliasing and avoids dynamic memory.
        unique_lock cache_write_lock(mutex_);
        cache_[key] = entry;
      }
    }

    // Delete any distance cache entries that have had their corresponding
    // octomap cells removed. It is fine to keep entries in the presence of
    // additions, as the entry only defines an upper bound. Because the size of
    // the tree is limited, the size of the cache has a natural limit. If this
    // limit is ever too large, a separate cache size may need to be set. Also
    // invalidate the fast path for any cache entries that are left. This
    // method is only useful for caches that are primarily used to limit the
    // tree, such as the main distance cache and the milli distance cache.
    // Other caches are unlikely to benefit from this and should simply be
    // cleared on an update.
    void handleCostmapUpdate(
        const octomap::OcTree* octree_ptr,
        const octomap::OcTree* nonlethal_octree_ptr)
    {
      unique_lock cache_write_lock(mutex_);
      size_t start_size = cache_.size();
      auto it = cache_.begin();
      while (it != cache_.end())
      {
        bool erase = true;
        Costmap3DIndex index;
        unsigned int depth;
        const octomap::OcTree* octree_to_query;
        if (it->first.getQueryObstacles() == NONLETHAL_ONLY)
        {
          octree_to_query = nonlethal_octree_ptr;
        }
        else
        {
          octree_to_query = octree_ptr;
        }
        if (it->second.getCostmapIndexAndDepth(*octree_to_query, &index, &depth))
        {
          unsigned int found_depth;
          auto* node = octree_to_query->search(index, depth, &found_depth);
          if (node &&
              !octree_to_query->nodeHasChildren(node) &&
              depth == found_depth &&
              octree_to_query->isNodeOccupied(node))
          {
            // The node exists and has no children (so its a leaf and not an
            // inner node), is at the correct depth and is considered occupied
            // for the proper octree. Keep this entry.
            erase = false;
          }
        }

        if (erase)
        {
          it = cache_.erase(it);
        }
        else
        {
          // While the entry is still good for setting an upper distance bound
          // on the octree solver, it is no good for the fast-path. This is
          // because the new costmap may have a closer box than previously.
          // Therefore invalidate this entry for the fast-path by setting the
          // recorded distance to infinity. The fast-path is only taken for
          // finite recorded distance.
          it->second.distance = std::numeric_limits<FCLFloat>::infinity();
          ++it;
        }
      }
      ROS_DEBUG_STREAM_NAMED("query_distance_cache",
          debug_prefix_ << "distance cache removed " << start_size - cache_.size() << " entries");
    }

    void clear()
    {
      unique_lock cache_write_lock(mutex_);
      cache_.clear();
    }

    void printDistanceCacheDebug()
    {
      ROS_DEBUG_STREAM_NAMED(
          "query_distance_cache", debug_prefix_ << "distance cache: " <<
          " size: " << cache_.size() <<
          " bucket count: " << cache_.bucket_count() <<
          " badness: " << cache_map_badness());
    }

    void setDebugPrefix(const std::string& debug_prefix)
    {
      debug_prefix_ = debug_prefix;
    }

  private:
    double cache_map_badness()
    {
      double ratio = cache_.size();
      ratio /= cache_.bucket_count();

      double cost = 0.0;
      for (auto const& entry : cache_)
        cost += cache_.bucket_size(cache_.bucket(entry.first));
      cost /= cache_.size();

      return std::max(0.0, cost / (1 + ratio) - 1);
    }

    shared_mutex mutex_;
    std::string debug_prefix_;
    // Zero means disabled
    unsigned int bins_per_meter_ = 0;
    unsigned int bins_per_rotation_ = 0;
    /**
     * Whether the caller wants 2D mode for distance cache thresholds.
     * When in 2D mode, the cache thresholds can be tighter and are calculated
     * differently than when not in 2D mode. The default is to run in 3D mode.
     * The milli and micro caches may be much more effective in 2D mode for a 2D
     * robot, as only the radius of the robot matters if query poses will be for
     * 2D navigation.
     */
    bool threshold_two_d_mode_ = false;
    /**
     * Safety factor for distance cache thresholds.
     * The minimum value that prevents false-negative collisions is increased by
     * this factor.
     */
    double threshold_factor_ = 1.05;
    /**
     * The distance cache allows a very fast path when the nearest obstacle is
     * more than a threshold of distance away. This results in a massive speed
     * up for cases where the nearest obstacle is further than the threshold_.
     * This value is automatically calculated based on the robot mesh, the
     * desired mode (2D or 3D) and the threshold_factor_.
     */
    double threshold_ = std::numeric_limits<double>::infinity();
    DistanceCacheMap cache_;
    DistanceFunction distance_function_;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr robot_mesh_points_;
  };

  using DistanceCache = DistanceCacheImpl<false>;
  using ExactDistanceCache = DistanceCacheImpl<true>;

  // Version of DistanceCache which does not bin on pose, but just remembers
  // the previous cache entry for a particular query mode. Includes a
  // DistanceCache member and wraps it in place of inheritance for speed.
  class LastDistanceCache
  {
  public:
    void setDistanceFunction(ExactDistanceCache::DistanceFunction distance_function)
    {
      cache_.setDistanceFunction(distance_function);
    }

    // Returns true on a hit.
    // Fast hits make no sense for a last distance cache, so return a bool.
    bool checkCache(
        const DistanceCacheKey& cache_key,
        const geometry_msgs::Pose& pose,
        DistanceCacheEntry* cache_entry_ptr,
        double* distance_ptr,
        const RegionsOfInterestAtPose* rois_ptr = nullptr)
    {
      // Reuse ExactDistanceCache, but provide a key with an all-zero pose so
      // the previous call results are re-used for a given query_region and
      // query_obstacles for any pose.
      return cache_.checkCache(
          cache_key,
          pose,
          cache_entry_ptr,
          distance_ptr,
          rois_ptr).first;
    }

    void updateCache(const DistanceCacheKey& key, const DistanceCacheEntry& entry)
    {
      cache_.updateCache(key, entry);
    }

    inline void initializeKey(
        const QueryRegion query_region,
        const QueryObstacles query_obstacles,
        DistanceCacheKey* key_ptr)
    {
      key_ptr->initialize(geometry_msgs::Pose(), query_region, query_obstacles);
    }

    void clear()
    {
      cache_.clear();
    }

  private:
    ExactDistanceCache cache_;
  };

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
  DistanceCache milli_distance_cache_;
  DistanceCache micro_distance_cache_;
  // Use an exact cache to immediately return the distance for an exact
  // duplicate query. The costmap is often queried for the same pose, for
  // instance by the path planner and by the path validator.
  ExactDistanceCache exact_distance_cache_;
  // If all other caches are missed, using the previous result provides a decent
  // bound and is better than no bound at all.
  LastDistanceCache last_distance_cache_;
  unsigned int last_layered_costmap_update_number_;
  // Statistics gathered between clearing cycles
  void printStatistics();
  void clearStatistics();
  // Only track statistics if the named logger is enabled.
  bool track_statistics_ = false;
  // Use std::atomic so we can increment with only the read lock held.
  // The write lock will be held when resetting these so they all reset
  // atomically
  std::atomic<unsigned int> queries_since_clear_;
  std::atomic<unsigned int> empties_since_clear_;  // an empty is a query on an empty costmap
  std::atomic<unsigned int> hits_since_clear_;
  std::atomic<unsigned int> fast_milli_hits_since_clear_;
  std::atomic<unsigned int> slow_milli_hits_since_clear_;
  std::atomic<unsigned int> fast_micro_hits_since_clear_;
  std::atomic<unsigned int> slow_micro_hits_since_clear_;
  std::atomic<unsigned int> exact_hits_since_clear_;
  std::atomic<uint64_t> misses_since_clear_us_;
  std::atomic<uint64_t> hits_since_clear_us_;
  std::atomic<uint64_t> fast_milli_hits_since_clear_us_;
  std::atomic<uint64_t> slow_milli_hits_since_clear_us_;
  std::atomic<uint64_t> fast_micro_hits_since_clear_us_;
  std::atomic<uint64_t> slow_micro_hits_since_clear_us_;
  std::atomic<uint64_t> exact_hits_since_clear_us_;
  std::atomic<size_t> hit_fcl_bv_distance_calculations_;
  std::atomic<size_t> hit_fcl_primitive_distance_calculations_;
  std::atomic<size_t> miss_fcl_bv_distance_calculations_;
  std::atomic<size_t> miss_fcl_primitive_distance_calculations_;
};

using Costmap3DQueryPtr = std::shared_ptr<Costmap3DQuery>;
using Costmap3DQueryConstPtr = std::shared_ptr<const Costmap3DQuery>;

}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_3D_QUERY_H_
