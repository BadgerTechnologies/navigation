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
#include <costmap_3d/costmap_3d_query.h>

#include <chrono>
#include <sstream>
#include <tuple>

#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance_result.h>
#include <fcl/geometry/shape/sphere.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <costmap_3d/octree_solver.h>

// In some environments, pcl::io isn't available because it depends on VTK,
// with its enormous dependencies, so allow for an alternative implementation:
#if(USE_STL_LOADER)
  #include <stl_loader/stl_loader.h>
#else
  #include <pcl/io/vtk_lib_io.h>
#endif

namespace costmap_3d
{

Costmap3DQuery::Costmap3DQuery(
    const LayeredCostmap3D* layered_costmap_3d,
    const std::string& mesh_resource,
    double padding,
    Costmap3DQuery::QueryRegionScale query_region_scale,
    unsigned int pose_bins_per_meter,
    unsigned int pose_bins_per_rotation,
    unsigned int pose_milli_bins_per_meter,
    unsigned int pose_milli_bins_per_rotation,
    unsigned int pose_micro_bins_per_meter,
    unsigned int pose_micro_bins_per_rotation)
  : layered_costmap_3d_(layered_costmap_3d),
    query_region_scale_(query_region_scale)
{
  init();
  updateMeshResource(mesh_resource, padding);
  setCacheBinSize(
      pose_bins_per_meter,
      pose_bins_per_rotation,
      pose_milli_bins_per_meter,
      pose_milli_bins_per_rotation,
      pose_micro_bins_per_meter,
      pose_micro_bins_per_rotation);
}

Costmap3DQuery::Costmap3DQuery(const Costmap3DConstPtr& costmap_3d,
    const std::string& mesh_resource,
    double padding,
    Costmap3DQuery::QueryRegionScale query_region_scale,
    unsigned int pose_bins_per_meter,
    unsigned int pose_bins_per_rotation,
    unsigned int pose_milli_bins_per_meter,
    unsigned int pose_milli_bins_per_rotation,
    unsigned int pose_micro_bins_per_meter,
    unsigned int pose_micro_bins_per_rotation)
  : layered_costmap_3d_(nullptr),
    query_region_scale_(query_region_scale)
{
  init();
  // Make a local copy of the costmap in question
  // It would be awesome if the Costmap3D had a way to snapshot
  // or copy-on-write. As it stands, for many scenarios involving
  // space-limited local costmaps, copying a 3D costmap will only take a
  // couple milliseconds and is better than leaving the costmap locked for
  // an entire planning cycle.
  octree_ptr_.reset(new Costmap3D(*costmap_3d));
  updateMeshResource(mesh_resource, padding);
  setCacheBinSize(
      pose_bins_per_meter,
      pose_bins_per_rotation,
      pose_milli_bins_per_meter,
      pose_milli_bins_per_rotation,
      pose_micro_bins_per_meter,
      pose_micro_bins_per_rotation);
  // For a buffered query, go ahead and setup the interior collision LUT now,
  // as the resolution can not change.
  checkInteriorCollisionLUT();
}

Costmap3DQuery::~Costmap3DQuery()
{
}

void Costmap3DQuery::init()
{
  clearStatistics();
  robot_model_halfspaces_.reset(new std::vector<fcl::Halfspace<FCLFloat>>);
  last_octomap_resolution_ = 0.0;
  auto distance_function = std::bind(
      &Costmap3DQuery::handleDistanceInteriorCollisions,
      this,
      std::placeholders::_1,
      std::placeholders::_2);
  distance_cache_.setDistanceFunction(distance_function);
  milli_distance_cache_.setDistanceFunction(distance_function);
  micro_distance_cache_.setDistanceFunction(distance_function);
  exact_distance_cache_.setDistanceFunction(distance_function);
  last_distance_cache_.setDistanceFunction(distance_function);
  milli_distance_cache_.setDebugPrefix("milli-");
  micro_distance_cache_.setDebugPrefix("micro-");
  exact_distance_cache_.setDebugPrefix("exact ");
}

void Costmap3DQuery::setCacheBinSize(
      unsigned int pose_bins_per_meter,
      unsigned int pose_bins_per_rotation,
      unsigned int pose_milli_bins_per_meter,
      unsigned int pose_milli_bins_per_rotation,
      unsigned int pose_micro_bins_per_meter,
      unsigned int pose_micro_bins_per_rotation)
{
  distance_cache_.setBinSize(pose_bins_per_meter, pose_bins_per_rotation);
  milli_distance_cache_.setBinSize(pose_milli_bins_per_meter, pose_milli_bins_per_rotation);
  micro_distance_cache_.setBinSize(pose_micro_bins_per_meter, pose_micro_bins_per_rotation);
}

void Costmap3DQuery::setCacheThresholdParameters(
    bool threshold_two_d_mode,
    double threshold_factor)
{
  distance_cache_.setThresholdParameters(threshold_two_d_mode, threshold_factor);
  milli_distance_cache_.setThresholdParameters(threshold_two_d_mode, threshold_factor);
  micro_distance_cache_.setThresholdParameters(threshold_two_d_mode, threshold_factor);
}

void Costmap3DQuery::updateCostmap(const Costmap3DConstPtr& costmap_3d)
{
  // Copy the given costmap
  std::shared_ptr<const octomap::OcTree> new_octree(new Costmap3D(*costmap_3d));
  unique_lock instance_lock(instance_mutex_);
  checkCostmap(new_octree);
}

// Caller must hold at least a shared lock
bool Costmap3DQuery::needCheckCostmap(std::shared_ptr<const octomap::OcTree> new_octree)
{
  bool need_update = false;
  {
    if (layered_costmap_3d_)
    {
      if (layered_costmap_3d_->getCostmap3D() != octree_ptr_ ||
         (last_octomap_resolution_ != layered_costmap_3d_->getCostmap3D()->getResolution()) ||
         (last_layered_costmap_update_number_ != layered_costmap_3d_->getNumberOfUpdates()))
      {
        need_update = true;
      }
    }
    else if (new_octree)
    {
      if (new_octree != octree_ptr_)
      {
        need_update = true;
      }
    }
  }
  return need_update;
}

// Caller must already hold the instance mutex
void Costmap3DQuery::checkCostmap(std::shared_ptr<const octomap::OcTree> new_octree)
{
  // Check if we need to update w/ just the upgrade lock held. This way only
  // one thread has to pay to upgrade to an exclusive lock (which forces no new
  // readers and waits for all existing readers to finish).
  bool need_update = needCheckCostmap(new_octree);
  if (need_update)
  {
    if (layered_costmap_3d_)
    {
      if (layered_costmap_3d_->getCostmap3D() != octree_ptr_)
      {
        // The octomap has been reallocated, change where we are pointing.
        octree_ptr_ = layered_costmap_3d_->getCostmap3D();
      }
    }
    else if (new_octree)
    {
      if (new_octree != octree_ptr_)
      {
        // There is a new octomap allocated, change where we are pointing.
        octree_ptr_ = new_octree;
      }
    }
    nonlethal_octree_ptr_ = std::static_pointer_cast<const Costmap3D>(octree_ptr_)->nonlethalOnly();
    checkInteriorCollisionLUT();

    // The costmap has been updated since the last query, reset our caches
    distance_cache_.printDistanceCacheDebug();
    distance_cache_.handleCostmapUpdate(octree_ptr_.get(), nonlethal_octree_ptr_.get());

    milli_distance_cache_.printDistanceCacheDebug();
    milli_distance_cache_.handleCostmapUpdate(octree_ptr_.get(), nonlethal_octree_ptr_.get());

    // Drop the micro cache as even if we removed invalid entries we can not
    // use the fast path which is the micro cache's strong point.
    micro_distance_cache_.printDistanceCacheDebug();
    micro_distance_cache_.clear();

    // We must drop the exact cache after the costmap has changed
    exact_distance_cache_.printDistanceCacheDebug();
    exact_distance_cache_.clear();

    // Invalidate the last cache. We could keep entries that still point to
    // valid boxes, but there is little to gain, as any misses on the next
    // cycle will quickly have a last cache entry to speed them up.
    last_distance_cache_.clear();

    printStatistics();
    clearStatistics();
    if (layered_costmap_3d_)
    {
      last_layered_costmap_update_number_ = layered_costmap_3d_->getNumberOfUpdates();
    }
  }
}

void Costmap3DQuery::clearStatistics()
{
  queries_since_clear_ = 0;
  empties_since_clear_ = 0;
  hits_since_clear_ = 0;
  fast_milli_hits_since_clear_ = 0;
  slow_milli_hits_since_clear_ = 0;
  fast_micro_hits_since_clear_ = 0;
  slow_micro_hits_since_clear_ = 0;
  exact_hits_since_clear_ = 0;
  misses_since_clear_us_ = 0;
  hits_since_clear_us_ = 0;
  fast_milli_hits_since_clear_us_ = 0;
  slow_milli_hits_since_clear_us_ = 0;
  fast_micro_hits_since_clear_us_ = 0;
  slow_micro_hits_since_clear_us_ = 0;
  exact_hits_since_clear_us_ = 0;
  hit_fcl_bv_distance_calculations_ = 0;
  hit_fcl_primitive_distance_calculations_ = 0;
  miss_fcl_bv_distance_calculations_ = 0;
  miss_fcl_primitive_distance_calculations_ = 0;
}

void Costmap3DQuery::printStatistics()
{
  unsigned int cache_misses = queries_since_clear_ -
      empties_since_clear_ -
      hits_since_clear_ -
      fast_milli_hits_since_clear_ -
      slow_milli_hits_since_clear_ -
      fast_micro_hits_since_clear_ -
      slow_micro_hits_since_clear_ -
      exact_hits_since_clear_;
  double hit_ratio = static_cast<double>(hits_since_clear_) / queries_since_clear_;
  double empty_ratio = static_cast<double>(empties_since_clear_) / queries_since_clear_;
  double fast_milli_hit_ratio = static_cast<double>(fast_milli_hits_since_clear_) / queries_since_clear_;
  double slow_milli_hit_ratio = static_cast<double>(slow_milli_hits_since_clear_) / queries_since_clear_;
  double fast_micro_hit_ratio = static_cast<double>(fast_micro_hits_since_clear_) / queries_since_clear_;
  double slow_micro_hit_ratio = static_cast<double>(slow_micro_hits_since_clear_) / queries_since_clear_;
  double exact_hit_ratio = static_cast<double>(exact_hits_since_clear_) / queries_since_clear_;
  uint64_t total_us = misses_since_clear_us_ +
      hits_since_clear_us_ +
      fast_milli_hits_since_clear_us_ +
      slow_milli_hits_since_clear_us_ +
      fast_micro_hits_since_clear_us_ +
      slow_micro_hits_since_clear_us_ +
      exact_hits_since_clear_us_;
  std::ostringstream ss;
  ss << "Costmap3DQuery statistics:"
      "\n\tqueries this cycle: " << queries_since_clear_ <<
      "\n\tcache misses: " << cache_misses <<
      "\n\tcache hits: " << hits_since_clear_ <<
      "\n\tcache empties: " << empties_since_clear_ <<
      "\n\tempty query ratio: " << empty_ratio <<
      "\n\tcache hit ratio: " << hit_ratio <<
      "\n\tslow milli cache hits: " << slow_milli_hits_since_clear_ <<
      "\n\tslow milli cache hit ratio: " << slow_milli_hit_ratio <<
      "\n\tfast milli cache hits: " << fast_milli_hits_since_clear_ <<
      "\n\tfast milli cache hit ratio: " << fast_milli_hit_ratio <<
      "\n\tslow micro cache hits: " << slow_micro_hits_since_clear_ <<
      "\n\tslow micro cache hit ratio: " << slow_micro_hit_ratio <<
      "\n\tfast micro cache hits: " << fast_micro_hits_since_clear_ <<
      "\n\tfast micro cache hit ratio: " << fast_micro_hit_ratio <<
      "\n\texact cache hits: " << exact_hits_since_clear_ <<
      "\n\texact cache hit ratio: " << exact_hit_ratio <<
      "\n\ttotal usecs: " << total_us <<
      "\n\tmiss usecs/query: " << static_cast<double>(misses_since_clear_us_) / cache_misses <<
      "\n\thit usecs/query: " << static_cast<double>(hits_since_clear_us_) / hits_since_clear_ <<
      "\n\tslow milli hit usecs/query: " <<
      static_cast<double>(slow_milli_hits_since_clear_us_) / slow_milli_hits_since_clear_ <<
      "\n\tfast milli hit usecs/query: " <<
      static_cast<double>(fast_milli_hits_since_clear_us_) / fast_milli_hits_since_clear_ <<
      "\n\tslow micro hit usecs/query: " <<
      static_cast<double>(slow_micro_hits_since_clear_us_) / slow_micro_hits_since_clear_ <<
      "\n\tfast micro hit usecs/query: " <<
      static_cast<double>(fast_micro_hits_since_clear_us_) / fast_micro_hits_since_clear_ <<
      "\n\texact hit usecs/query: " <<
      static_cast<double>(exact_hits_since_clear_us_) / exact_hits_since_clear_ <<
      "\n\tmiss FCL BV distance calculations: " << miss_fcl_bv_distance_calculations_ <<
      "\n\tmiss FCL primitive distance calculations: " << miss_fcl_primitive_distance_calculations_ <<
      "\n\thit FCL BV distance calculations: " << hit_fcl_bv_distance_calculations_ <<
      "\n\thit FCL primitive distance calculations: " << hit_fcl_primitive_distance_calculations_;
  // Leverage the fact that the print arguments are only run if the log is
  // enabled to make it simple to track if statistics tracking should be
  // enabled. There is no need to pay for the atomic operations for statistics
  // tracking if it is disabled.
  bool should_track_statistics = false;
  ROS_DEBUG_STREAM_NAMED(
      "query_statistics",
      // Weird construction here to get the side effect of setting
      // should_track_statistics when the named debug is enabled. Also skips
      // printing the first cycle as it will have invalid (zero) statistics
      (((should_track_statistics = true) && track_statistics_) ? ss.str() :
      "Costmap3DQuery: begin tracking statistics"));
  track_statistics_ = should_track_statistics;
}

void Costmap3DQuery::addPCLPolygonToFCLTriangles(
    const pcl::Vertices& polygon,
    std::vector<fcl::Triangle>* fcl_triangles)
{
  // Assume the polygons are convex. Break them into triangles.
  const std::size_t zero_index = polygon.vertices[0];
  for (int i=1; i < polygon.vertices.size() - 1; ++i)
  {
    fcl_triangles->push_back(fcl::Triangle(zero_index, polygon.vertices[i], polygon.vertices[i+1]));
  }
}

void Costmap3DQuery::addPCLPolygonMeshToRobotModel(
    const pcl::PolygonMesh& pcl_mesh,
    double padding,
    FCLRobotModel* robot_model)
{
  robot_mesh_points_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_mesh.cloud, *robot_mesh_points_);

  padPoints(robot_mesh_points_, padding);

  std::vector<fcl::Vector3<FCLFloat>> fcl_points;
  std::vector<fcl::Triangle> fcl_triangles;

  for (auto pcl_point : *robot_mesh_points_)
  {
    fcl_points.push_back(convertPCLPointToFCL<FCLFloat>(pcl_point));
  }

  for (auto polygon : pcl_mesh.polygons)
  {
    addPCLPolygonToFCLTriangles(polygon, &fcl_triangles);
  }

  robot_model->addSubModel(fcl_points, fcl_triangles);
}

void Costmap3DQuery::updateMeshResource(const std::string& mesh_resource, double padding)
{
  unique_lock write_lock(instance_mutex_);
  if (mesh_resource == mesh_resource_ && padding == padding_)
  {
    return;
  }
  std::string filename = getFileNameFromPackageURL(mesh_resource);
  if (filename.size() == 0)
  {
    return;
  }
  int pcl_rv = pcl::io::loadPolygonFileSTL(filename, robot_mesh_);
  if (pcl_rv < 0)
  {
    ROS_ERROR_STREAM("Costmap3DQuery: unable to load STL mesh file " << filename
                     << " query object will always return collision!");
    return;
  }
  robot_model_.reset(new FCLRobotModel());
  robot_model_->beginModel();
  addPCLPolygonMeshToRobotModel(robot_mesh_, padding, robot_model_.get());
  robot_model_->endModel();
  robot_model_->computeLocalAABB();

  crop_hull_.setHullCloud(robot_mesh_points_);
  crop_hull_.setHullIndices(robot_mesh_.polygons);
  crop_hull_.setCropOutside(true);

  // Calculate halfspaces for each mesh triangle
  robot_model_halfspaces_->clear();
  robot_model_halfspaces_->resize(robot_model_->num_tris);
  for (unsigned int i = 0; i < robot_model_->num_tris; ++i)
  {
    fcl::Triangle tri = robot_model_->tri_indices[i];
    (*robot_model_halfspaces_)[i] = convertTriangleToHalfspace<FCLFloat>(
        fcl::TriangleP<FCLFloat>(
            robot_model_->vertices[tri[0]],
            robot_model_->vertices[tri[1]],
            robot_model_->vertices[tri[2]]));
  }
  distance_cache_.setRobotMeshPoints(robot_mesh_points_);
  milli_distance_cache_.setRobotMeshPoints(robot_mesh_points_);
  micro_distance_cache_.setRobotMeshPoints(robot_mesh_points_);
}

std::string Costmap3DQuery::getFileNameFromPackageURL(const std::string& url)
{
  /* Unfortunately the resource retriever does not have a way to get a path from a URL
   * (it only returns the contents of a URL in memory), and equally unfortunate, PCL does not
   * have a way to parse an STL from memory. Therefore we have to duplicate some of the
   * (slightly-modified) resource retriever code here. */
  // Reference: https://github.com/ros/resource_retriever/blob/kinetic-devel/src/retriever.cpp
  std::string mod_url = url;
  if (url.find("package://") == 0)
  {
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find("/");
    if (pos == std::string::npos)
    {
      ROS_ERROR_STREAM("Costmap3DQuery: Could not parse package:// format URL "
                       << url
                       << " query object will always return collision!");
      return "";
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = ros::package::getPath(package.c_str());

    if (package_path.empty())
    {
      ROS_ERROR_STREAM("Costmap3DQuery: Package [" << package << "] from URL "
                       << url << " does not exist, "
                       << " query object will always return collision!");
      return "";
    }

    mod_url = package_path + mod_url;
  }
  else if (url.find("file://") == 0)
  {
    mod_url.erase(0, strlen("file://"));
  }

  return mod_url;
}

double Costmap3DQuery::footprintCost(const geometry_msgs::Pose& pose, Costmap3DQuery::QueryRegion query_region)
{
  // This could be implemented as signed distance, or as some other metric.
  // For now, simply translate a collision as cost.
  return footprintCollision(pose, query_region) ? -1.0 : 0.0;
}

bool Costmap3DQuery::footprintCollision(const geometry_msgs::Pose& pose, Costmap3DQuery::QueryRegion query_region)
{
  // It is more correct and even more efficient to query the distance to find
  // collisions than it is to use FCL to directly find octomap collisions.
  // This is because our distance query correctly handles interior collisions,
  // which requires finding the nearest octomap box, which an FCL collision
  // will not do.
  DistanceOptions opts;
  opts.query_region = query_region;
  // For collision only queries, we only need to check when bounding volumes overlap.
  // Make cache-misses in such cases very fast by limiting the bound distance.
  // Bound to just above zero, as zero would be considered a collision.
  // Remember that min for double is the smallest positive (normalized) double.
  opts.distance_limit = std::numeric_limits<double>::min();
  // We want exact answers for collision only checks.
  opts.relative_error = 0.0;
  return calculateDistance(pose, opts) <= 0.0;
}

// Discern if the given octomap box is an interior collision and adjust
// distance or signed distance appropriately.
// This is done by using PCL's point-in-a-mesh call which works on concave
// meshes that represent closed polyhedra. FCL handles concave meshes as
// surfaces, not volumes.
double Costmap3DQuery::handleDistanceInteriorCollisions(
      const DistanceCacheEntry& cache_entry,
      const geometry_msgs::Pose& pose)
{
  FCLFloat distance;

  // Turn pose into tf
  const fcl::Transform3<FCLFloat> pose_tf(costmap_3d::poseToFCLTransform<FCLFloat>(pose));

  // Start with the interior collision check as it is very fast.
  // Find out if the center of the box is inside the given footprint volume mesh.
  distance = interior_collision_lut_.distance(
      cache_entry.octomap_box,
      cache_entry.octomap_box_center,
      pose_tf,
      pose_tf.inverse());

  if (distance < 0.0)
  {
    return distance;
  }

  // We need to calculate the distance between the mesh triangle at the new
  // pose and the box.
  //
  // As of the time this code was written, the normal FCL API does not
  // allow box/triangle distance or signed distance queries.
  // Yet FCL internally does such checks all the time, so use the
  // internal mechanism for now.
  fcl::detail::GJKSolver_libccd<FCLFloat> solver;
  fcl::Transform3<FCLFloat> box_tf = fcl::Transform3<FCLFloat>::Identity();
  box_tf.translation() = cache_entry.octomap_box_center;
  const fcl::Triangle& tri_id = robot_model_->tri_indices[cache_entry.mesh_triangle_id];
  solver.shapeTriangleDistance(
      cache_entry.octomap_box,
      box_tf,
      robot_model_->vertices[tri_id[0]],
      robot_model_->vertices[tri_id[1]],
      robot_model_->vertices[tri_id[2]],
      pose_tf,
      &distance);

  // Box/triangle intersect, use penetration depth with box/halfspace model.
  if (distance < 0.0)
  {
    distance = boxHalfspaceSignedDistance(
        cache_entry.octomap_box,
        cache_entry.octomap_box_center,
        cache_entry.mesh_triangle_id,
        pose_tf);
  }

  return distance;
}

Costmap3DQuery::FCLFloat Costmap3DQuery::boxHalfspaceSignedDistance(
    const fcl::Box<FCLFloat>& box,
    const fcl::Vector3<FCLFloat>& box_center,
    int mesh_triangle_id,
    const fcl::Transform3<FCLFloat>& mesh_tf) const
{
  fcl::Halfspace<FCLFloat> halfspace(fcl::transform(
      (*robot_model_halfspaces_)[mesh_triangle_id], mesh_tf));
  return costmap_3d::boxHalfspaceSignedDistance<FCLFloat>(box, box_center, halfspace);
}

double Costmap3DQuery::footprintDistance(const geometry_msgs::Pose& pose,
                                         const Costmap3DQuery::DistanceResult& result)
{
  if (!result)
    return std::numeric_limits<FCLFloat>::max();

  // Reuse the result and return the new distance. In cases where the caller
  // knows this is the correct behavior (such as when making minor
  // perturbations to estimate derivatives), this is faster than having to
  // calculate the hash and find the cache entry the normal way.
  return handleDistanceInteriorCollisions(DistanceCacheEntry(result), pose);
}

double Costmap3DQuery::calculateDistance(const geometry_msgs::Pose& pose,
                                         const Costmap3DQuery::DistanceOptions& opts,
                                         Costmap3DQuery::DistanceResult* return_result)
{
  // Make convenience aliases of some of the options
  const QueryRegion query_region = opts.query_region;
  const QueryObstacles query_obstacles = opts.query_obstacles;
  // Let compiler optimize track_statistics by adding it to the stack.
  const bool track_statistics = track_statistics_;

  std::chrono::high_resolution_clock::time_point start_time;
  if (track_statistics)
  {
    start_time = std::chrono::high_resolution_clock::now();
    queries_since_clear_.fetch_add(1, std::memory_order_relaxed);
  }

  // Use the passed in distance limit up to the "min" float (close to zero)
  // We do not want to use a zero or negative distance limit from the options
  // because that will prevent us from correctly searching the octomap/mesh
  FCLFloat pose_distance = std::max(opts.distance_limit, std::numeric_limits<FCLFloat>::min());

  if (!robot_model_)
  {
    // We failed to create a robot model.
    // The failure would have been logged, so simply return collision.
    if (return_result)
      return_result->clear();
    return -1.0;
  }

  if (layered_costmap_3d_ != nullptr)
  {
    // We only need to hold the instance mutex during checkCostmap. The costmap
    // itself stays locked so the tree we are pointing to won't change. Only
    // the internal state of the query object needs to be protected. Buffered
    // queries can skip over this lock completely as the internal state only
    // ever changes on calls to updateCostmap or updateMeshResource which must
    // happen with no outstanding queries.
    bool need_check_costmap = false;
    {
      // Improve concurrency by only holding a shared lock to check if we may
      // need an exclusive lock.
      shared_lock read_instance_lock(instance_mutex_);
      need_check_costmap = needCheckCostmap();
    }
    if (need_check_costmap)
    {
      unique_lock write_instance_lock(instance_mutex_);
      // Even after getting here we may not have to do anything in
      // checkCostmap, as multiple shared locks could race on seeing if we may
      // need to update. The checkCostmap() call handles this by re-checking
      // needCheckCostmap().
      checkCostmap();
    }
  }

  std::shared_ptr<const octomap::OcTree> octree_to_query;
  if (query_obstacles == NONLETHAL_ONLY)
  {
    octree_to_query = nonlethal_octree_ptr_;
  }
  else
  {
    octree_to_query = octree_ptr_;
  }
  assert(octree_to_query);

  // Handle and count empty queries specially.
  if (octree_to_query->size() == 0 || !octree_to_query->isNodeOccupied(octree_to_query->getRoot()))
  {
    if (track_statistics)
    {
      empties_since_clear_.fetch_add(1, std::memory_order_relaxed);
    }
    if (return_result)
      return_result->clear();
    return std::numeric_limits<double>::max();
  }

  double distance;
  DistanceCacheKey exact_cache_key;
  DistanceCacheEntry cache_entry;
  exact_distance_cache_.initializeKey(pose, query_region, query_obstacles, &exact_cache_key);
  if (exact_distance_cache_.checkCache(
        exact_cache_key,
        pose,
        &cache_entry,
        &distance,
        nullptr,
        track_statistics,
        &start_time,
        &exact_hits_since_clear_,
        &exact_hits_since_clear_us_).first)
  {
    cache_entry.setupResult(return_result);
    return distance;
  }

  typename OcTreeMeshSolver<FCLSolver>::DistanceResult result;

  // Setup the regions of interest corresponding to the query region.
  // This has to be done prior to checking the distance caches, so the cache entry
  // can be checked to ensure it is still inside the region.
  // Store the region on the stack to avoid dynamic memory allocation.
  RegionsOfInterestAtPose rois(query_region, query_region_scale_, pose);

  DistanceCacheKey milli_cache_key;
  milli_distance_cache_.initializeKey(pose, query_region, query_obstacles, &milli_cache_key);
  bool milli_hit = false;
  bool fast_hit = false;
  std::tie(milli_hit, fast_hit) = milli_distance_cache_.checkCache(
      milli_cache_key,
      pose,
      &cache_entry,
      &distance,
      &rois,
      track_statistics,
      &start_time,
      &fast_milli_hits_since_clear_,
      &fast_milli_hits_since_clear_us_,
      opts.exact_signed_distance,
      opts.directly_use_cache_when_above_threshold);
  if (fast_hit)
  {
    cache_entry.setupResult(return_result);
    return distance;
  }
  else if (milli_hit)
  {
    // Too close to directly use the cache, but use the
    // calculated distance as the pose_distance upper bound.
    if (distance < pose_distance)
    {
      pose_distance = distance;
      cache_entry.setupResult(&result);
    }
  }

  DistanceCacheKey micro_cache_key;
  micro_distance_cache_.initializeKey(pose, query_region, query_obstacles, &micro_cache_key);
  bool micro_hit = false;
  std::tie(micro_hit, fast_hit) = micro_distance_cache_.checkCache(
      micro_cache_key,
      pose,
      &cache_entry,
      &distance,
      &rois,
      track_statistics,
      &start_time,
      &fast_micro_hits_since_clear_,
      &fast_micro_hits_since_clear_us_,
      opts.exact_signed_distance,
      opts.directly_use_cache_when_above_threshold);
  if (fast_hit)
  {
    cache_entry.setupResult(return_result);
    return distance;
  }
  else if (micro_hit)
  {
    // Too close to directly use the cache, but use the
    // calculated distance as the pose_distance upper bound.
    if (distance < pose_distance)
    {
      pose_distance = distance;
      cache_entry.setupResult(&result);
    }
  }

  // The keys are needed to update below.
  // Initialize them even if skipping the cache checks.
  DistanceCacheKey cache_key;
  distance_cache_.initializeKey(pose, query_region, query_obstacles, &cache_key);
  DistanceCacheKey last_cache_key;
  last_distance_cache_.initializeKey(query_region, query_obstacles, &cache_key);
  bool cache_hit = false;
  // Assume a milli hit or micro hit is much better than the normal distance
  // cache at setting an upper bound, skip the normal cache lookup in such
  // cases.
  if (!milli_hit && !micro_hit)
  {
    if (distance_cache_.checkCache(
          cache_key,
          pose,
          &cache_entry,
          &distance,
          &rois).first)
    {
      if (distance < pose_distance)
      {
        cache_hit = true;
        pose_distance = distance;
        cache_entry.setupResult(&result);
      }
    }
    // Missed the distance cache, but use the last entry if available.
    else if (last_distance_cache_.checkCache(
          cache_key,
          pose,
          &cache_entry,
          &distance,
          &rois))
    {
      if (distance < pose_distance)
      {
        pose_distance = distance;
        cache_entry.setupResult(&result);
      }
    }
  }

  if (opts.directly_use_cache_when_below_threshold && (micro_hit || milli_hit))
  {
    // When directly using the cache when below the threshold, just look for collisions.
    result.min_distance = std::numeric_limits<FCLFloat>::min();
  }
  else
  {
    // limit search to nearest cache entry
    result.min_distance = pose_distance;
  }

  if (result.min_distance > 0.0 || opts.exact_signed_distance)
  {
    // Because FCL's OcTree/Mesh distance treats the Mesh as hollow, we must
    // use our own distance code which treats the Mesh as a closed mesh
    // defining a filled volume. Otherwise, there are cases where an octomap
    // box is inside the mesh, but the distance is positive.
    // The solver and octree_solver need to be on the stack as they are not
    // thread-safe.
    FCLSolver solver;
    // Use our interior collision LUT to model the robot as a volume.
    // Use box-halfspace distance to model box/mesh penetrations for signed distance.
    OcTreeMeshSolver<FCLSolver> octree_solver(
        &solver,
        std::bind(&InteriorCollisionLUT<FCLFloat>::distance,
                  &interior_collision_lut_,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5),
        std::bind(&Costmap3DQuery::boxHalfspaceSignedDistance,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4));

    typename OcTreeMeshSolver<FCLSolver>::DistanceRequest request;
    request.enable_signed_distance = true;
    request.enable_exact_signed_distance = opts.exact_signed_distance;
    request.rel_err = opts.relative_error;
    rois.setupRequest(&request);

    octree_solver.distance(
        octree_to_query.get(),
        robot_model_.get(),
        fcl::Transform3<FCLFloat>::Identity(),
        poseToFCLTransform<FCLFloat>(pose),
        request,
        &result);
  }

  // When directly using the micro or milli cache even below the threshold,
  // use the cache distance as long as there is no collision
  if (opts.directly_use_cache_when_below_threshold && (micro_hit || milli_hit) && result.min_distance > 0.0)
  {
    distance = pose_distance;
  }
  else
  {
    distance = result.min_distance;
  }

  // Note that it is possible for the result to be empty. The octomap might
  // only contain non-lethal leaves and we may have missed every cache.
  // Or the query region may be set to something other than ALL and there are
  // no map entries in the queried region.
  // If we get no result primitives, do not add null pointers to the cache!
  if (result.mesh_triangle_id >= 0)
  {
    DistanceCacheEntry new_entry(result);
    new_entry.distance = distance;

    // Update distance caches.
    distance_cache_.updateCache(cache_key, new_entry);
    milli_distance_cache_.updateCache(milli_cache_key, new_entry);
    micro_distance_cache_.updateCache(micro_cache_key, new_entry);
    exact_distance_cache_.updateCache(exact_cache_key, new_entry);
    last_distance_cache_.updateCache(last_cache_key, new_entry);

    new_entry.setupResult(return_result);
  }
  else
  {
    if (return_result)
      return_result->clear();
  }

  if (track_statistics)
  {
    if (result.mesh_triangle_id < 0)
    {
      // Count a query that found nothing as an empty (as it will run very fast
      // and skew other results, as it would otherwise be counted as a miss.)
      empties_since_clear_.fetch_add(1, std::memory_order_relaxed);
    }
    else if (micro_hit)
    {
      slow_micro_hits_since_clear_.fetch_add(1, std::memory_order_relaxed);
      slow_micro_hits_since_clear_us_.fetch_add(
          std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start_time).count(),
          std::memory_order_relaxed);
    }
    else if (milli_hit)
    {
      slow_milli_hits_since_clear_.fetch_add(1, std::memory_order_relaxed);
      slow_milli_hits_since_clear_us_.fetch_add(
          std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start_time).count(),
          std::memory_order_relaxed);
    }
    else if (cache_hit)
    {
      hits_since_clear_.fetch_add(1, std::memory_order_relaxed);
      hits_since_clear_us_.fetch_add(
          std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start_time).count(),
          std::memory_order_relaxed);
    }
    else
    {
      // Currently last cache hits are lumped into misses
      misses_since_clear_us_.fetch_add(
          std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start_time).count(),
          std::memory_order_relaxed);
    }
    if (micro_hit || milli_hit || cache_hit)
    {
      hit_fcl_bv_distance_calculations_.fetch_add(result.bv_distance_calculations);
      hit_fcl_primitive_distance_calculations_.fetch_add(result.primitive_distance_calculations);
    }
    else
    {
      miss_fcl_bv_distance_calculations_.fetch_add(result.bv_distance_calculations);
      miss_fcl_primitive_distance_calculations_.fetch_add(result.primitive_distance_calculations);
    }
  }

  return distance;
}

double Costmap3DQuery::footprintDistance(const geometry_msgs::Pose& pose,
                                         Costmap3DQuery::QueryRegion query_region,
                                         double relative_error)
{
  DistanceOptions opts;
  opts.query_region = query_region;
  opts.relative_error = relative_error;
  return calculateDistance(pose, opts);
}

double Costmap3DQuery::footprintSignedDistance(const geometry_msgs::Pose& pose,
                                               Costmap3DQuery::QueryRegion query_region,
                                               double relative_error,
                                               bool exact_signed_distance)
{
  DistanceOptions opts;
  opts.query_region = query_region;
  opts.relative_error = relative_error;
  opts.signed_distance = true;
  opts.exact_signed_distance = exact_signed_distance;
  return calculateDistance(pose, opts);
}

double Costmap3DQuery::footprintDistance(const geometry_msgs::Pose& pose,
                                         const Costmap3DQuery::DistanceOptions& opts,
                                         Costmap3DQuery::DistanceResult* result)
{
  return calculateDistance(pose, opts, result);
}

void Costmap3DQuery::checkInteriorCollisionLUT()
{
  if (last_octomap_resolution_ != octree_ptr_->getResolution())
  {
    // Resolution changed, need to setup our interior collision LUT
    last_octomap_resolution_ = octree_ptr_->getResolution();
    const double box_size = last_octomap_resolution_;
    // Instead of adding the orientation to the LUT, just be more than double the
    // spatial resolution. This gives acceptable results without using much
    // memory.
    const double lut_res = box_size / 2.5;
    interior_collision_lut_.setup(box_size, lut_res, *robot_model_, crop_hull_, robot_model_halfspaces_);
  }
}

}  // namespace costmap_3d
