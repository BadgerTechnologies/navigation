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
#include <sstream>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance_result.h>
#include <fcl/geometry/shape/sphere.h>
#include <pcl/io/vtk_lib_io.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>

namespace costmap_3d
{

Costmap3DQuery::Costmap3DQuery(const std::shared_ptr<LayeredCostmap3D>& layered_costmap_3d,
    const std::string& mesh_resource,
    double padding)
    : layered_costmap_3d_(layered_costmap_3d)
{
  std::lock_guard<std::mutex> lock(query_mutex_);
  checkCostmap();
  updateMeshResource(mesh_resource, padding);
  std::stringstream ss;
  ss << "costmap_3d_query_" << mesh_resource << "_" << padding;
  costmap_update_complete_callback_id_ = ss.str();
  layered_costmap_3d_->registerUpdateCompleteCallback(
      costmap_update_complete_callback_id_,
      std::bind(&Costmap3DQuery::updateCompleteCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));
}

Costmap3DQuery::~Costmap3DQuery()
{
}

void Costmap3DQuery::checkCostmap()
{
  if (layered_costmap_3d_->getCostmap3D() != octree_ptr_)
  {
    // The octomap has been reallocated, change where we are pointing.
    octree_ptr_ = layered_costmap_3d_->getCostmap3D();
    fcl_octree_ptr_.reset(new fcl::OcTree<FCLFloat>(octree_ptr_));
    world_obj_ = FCLCollisionObjectPtr(new fcl::CollisionObject<FCLFloat>(fcl_octree_ptr_));
  }
}

void Costmap3DQuery::updateCompleteCallback(LayeredCostmap3D* layered_costmap_3d,
                                            const Costmap3D& delta,
                                            const Costmap3D& bounds_map)
{
  std::lock_guard<std::mutex> lock(query_mutex_);
  // For simplicity, on every update, clear out the collision cache.
  // This is not strictly necessary. The mesh is stored in the cache and does
  // not change. The only thing that is changing is the costmap. We could go
  // through the delta map and only remove entries that have had the
  // corresponding octomap cell go away. This may be impelemnted as a future
  // improvement.
  distance_cache_.clear();
  micro_distance_cache_.clear();
  // Ensure we are pointing to the latest costmap.
  checkCostmap();
}

void Costmap3DQuery::updateMeshResource(const std::string& mesh_resource, double padding)
{
  footprint_mesh_resource_ = mesh_resource;
  std::string filename = getFileNameFromPackageURL(footprint_mesh_resource_);
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileSTL(filename, mesh);
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *mesh_points);

  // Convert the PCL PolygonMesh to the FCL BVHModel applying padding
  std::vector<fcl::Vector3<FCLFloat>> fcl_points;
  std::vector<fcl::Triangle> fcl_triangles;
  for (auto pcl_point : *mesh_points)
  {
    // Apply padding very naively. To finely control padding, just use an alternate mesh
    FCLFloat x = pcl_point.x > 0 ? pcl_point.x + padding : pcl_point.x - padding;
    FCLFloat y = pcl_point.y > 0 ? pcl_point.y + padding : pcl_point.y - padding;
    FCLFloat z = pcl_point.z > 0 ? pcl_point.z + padding : pcl_point.z - padding;

    fcl_points.push_back(fcl::Vector3<FCLFloat>(x, y, z));
  }
  for (auto polygon : mesh.polygons)
  {
    // Assume the polygons are convex. Break them into triangles.
    const std::size_t zero_index = polygon.vertices[0];
    for (int i=1; i < polygon.vertices.size() - 1; ++i)
    {
      fcl_triangles.push_back(fcl::Triangle(zero_index, polygon.vertices[i], polygon.vertices[i+1]));
    }
  }
  robot_model_.reset(new FCLRobotModel());
  robot_model_->beginModel();
  robot_model_->addSubModel(fcl_points, fcl_triangles);
  robot_model_->endModel();
  robot_obj_ = FCLCollisionObjectPtr(new FCLCollisionObject(robot_model_));
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
      throw std::ios_base::failure("Could not parse package:// format URL " + url);
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = ros::package::getPath(package.c_str());

    if (package_path.empty())
    {
      throw std::ios_base::failure("Package [" + package + "] from URL " + url + " does not exist");
    }

    mod_url = package_path + mod_url;
  }
  else if (url.find("file://") == 0)
  {
    mod_url.erase(0, strlen("file://"));
  }

  return mod_url;
}

double Costmap3DQuery::footprintCost(geometry_msgs::Pose pose)
{
  // TODO: implement as cost query. For now, just translate a collision to cost
  return footprintCollision(pose) ? -1.0 : 0.0;
}

bool Costmap3DQuery::footprintCollision(geometry_msgs::Pose pose)
{
  std::lock_guard<std::mutex> lock(query_mutex_);
  assert(world_obj_);
  assert(robot_obj_);

  FCLCollisionObjectPtr robot(getRobotCollisionObject(pose));
  FCLCollisionObjectPtr world(getWorldCollisionObject());

  fcl::CollisionRequest<FCLFloat> request;
  fcl::CollisionResult<FCLFloat> result;

  fcl::collide(world.get(), robot.get(), request, result);

  return result.isCollision();
}

double Costmap3DQuery::footprintDistance(geometry_msgs::Pose pose)
{
  std::lock_guard<std::mutex> lock(query_mutex_);
  assert(world_obj_);
  assert(robot_obj_);

  FCLCollisionObjectPtr robot(getRobotCollisionObject(pose));
  FCLCollisionObjectPtr world(getWorldCollisionObject());

  FCLFloat pose_distance = std::numeric_limits<FCLFloat>::max();
//  FCLFloat pose_distance = 1.0;

  fcl::DistanceRequest<FCLFloat> request;
  fcl::DistanceResult<FCLFloat> result;

  DistanceCacheKey micro_cache_key(pose);
  // first bin for micro. if micro, just use the result directly.
  micro_cache_key.binPoseMicro();
  auto micro_cache_entry = micro_distance_cache_.find(micro_cache_key);
  if (micro_cache_entry != micro_distance_cache_.end())
  {
    return micro_cache_entry->second.distanceToNewPose(pose);
  }

  DistanceCacheKey cache_key(pose);
  cache_key.binPose();
  auto cache_entry = distance_cache_.find(cache_key);
  if (cache_entry != distance_cache_.end())
  {
    pose_distance = cache_entry->second.distanceToNewPose(pose);
    cache_entry->second.setupResult(&result);
  }
  else if(distance_cache_.size() > 0)
  {
    auto begin = distance_cache_.begin();
    pose_distance = begin->second.distanceToNewPose(pose);
    begin->second.setupResult(&result);
  }

  request.rel_err = 0.001;
  request.abs_err = 0.0;
  request.distance_tolerance = .001;
  request.enable_nearest_points = true;

  result.min_distance = pose_distance;

  double distance = fcl::distance(world.get(), robot.get(), request, result);

  // Update distance caches
  const DistanceCacheEntry& new_entry = DistanceCacheEntry(result);
  distance_cache_[cache_key] = new_entry;
  micro_distance_cache_[micro_cache_key] = new_entry;

  return distance;
}

double Costmap3DQuery::footprintSignedDistance(geometry_msgs::Pose pose)
{
  std::lock_guard<std::mutex> lock(query_mutex_);
  // TODO: Figure out how to apply distance cache to signed distance.
  // TODO: Think about how to make this work w/ meshes representing a solid and octomaps
  assert(world_obj_);
  assert(robot_obj_);
  FCLCollisionObjectPtr robot(getRobotCollisionObject(pose));
  FCLCollisionObjectPtr world(getWorldCollisionObject());

  fcl::DistanceRequest<FCLFloat> request;
  fcl::DistanceResult<FCLFloat> result;

  request.enable_signed_distance = true;

  return fcl::distance(world.get(), robot.get(), request, result);
}

}  // namespace costmap_3d
