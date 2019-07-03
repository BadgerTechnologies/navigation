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
#include <costmap_3d/costmap_3d_ros.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/geometry/shape/sphere.h>
#include <pcl/io/vtk_lib_io.h>
#include <costmap_3d/layered_costmap_3d.h>
#include <tf/transform_datatypes.h>

namespace costmap_3d
{

Costmap3DROS::Costmap3DROS(std::string name, tf::TransformListener& tf) :
    super(name, tf),
    initialized_(false),
    plugin_loader_("costmap_3d", "costmap_3d::Layer3D")
{
  std::lock_guard<std::mutex> initialize_lock(initialized_mutex_);
  ros::NodeHandle private_nh("~/" + name + "/");

  if (!private_nh.getParam("costmap_3d/footprint_mesh_resource", footprint_mesh_resource_))
  {
    ROS_ERROR("Unable to find footprint_mesh_resource parameter");
  }

  layered_costmap_3d_.reset(new LayeredCostmap3D(layered_costmap_));

  if (private_nh.hasParam("costmap_3d/plugins"))
  {
    XmlRpc::XmlRpcValue my_list;
    private_nh.getParam("costmap_3d/plugins", my_list);
    for (int32_t i = 0; i < my_list.size(); ++i)
    {
      std::string pname = static_cast<std::string>(my_list[i]["name"]);
      std::string type = static_cast<std::string>(my_list[i]["type"]);
      ROS_INFO("Using 3D plugin \"%s\"", pname.c_str());

      boost::shared_ptr<Layer3D> plugin = plugin_loader_.createInstance(type);
      plugin->initialize(layered_costmap_3d_.get(), name + "/costmap_3d/" + pname, &tf_);
      layered_costmap_3d_->addPlugin(plugin);
    }
  }

  publisher_.reset(new Costmap3DPublisher(private_nh, layered_costmap_3d_.get(), "costmap_3d"));

  dsrv_.reset(new dynamic_reconfigure::Server<Costmap3DConfig>(ros::NodeHandle("~/" + name + "/costmap_3d")));
  dynamic_reconfigure::Server<Costmap3DConfig>::CallbackType cb = std::bind(&Costmap3DROS::reconfigureCB,
                                                                            this,
                                                                            std::placeholders::_1,
                                                                            std::placeholders::_2);
  dsrv_->setCallback(cb);

  updateMeshResource();
  get_plan_cost_action_srv_.reset(new actionlib::SimpleActionServer<GetPlanCost3DAction>(
          private_nh,
          "get_plan_cost_3d",
          std::bind(&Costmap3DROS::getPlanCost3DActionCallback, this, std::placeholders::_1),
          false));
  get_plan_cost_action_srv_->start();
  get_plan_cost_srv_ = private_nh.advertiseService("get_plan_cost_3d",
            &Costmap3DROS::getPlanCost3DServiceCallback, this);

  initialized_ = true;
}

Costmap3DROS::~Costmap3DROS()
{
  // Publisher must be freed before the 3D costmap
  publisher_.reset();
}

void Costmap3DROS::reconfigureCB(Costmap3DConfig &config, uint32_t level)
{
  geometry_msgs::Point min, max;

  // Get the x/y values from the 2D costmap
  if (layered_costmap_3d_->isRolling())
  {
    // If we are rolling, set the origin to zero, as we will shift the limits
    // based on the base position ourselves in updateMap, since the octomap is
    // always originated at (0, 0, 0).
    min.x = 0.0;
    min.y = 0.0;
  }
  else
  {
    // If we are not rolling, use the 2D origin as the minimum.
    min.x = layered_costmap_->getCostmap()->getOriginX();
    min.y = layered_costmap_->getCostmap()->getOriginY();
  }
  max.x = min.x + layered_costmap_->getCostmap()->getSizeInMetersX();
  max.y = min.y + layered_costmap_->getCostmap()->getSizeInMetersY();
  min.z = config.map_z_min;
  max.z = config.map_z_max;

  layered_costmap_3d_->setBounds(min, max);
  layered_costmap_3d_->setResolution();

  footprint_3d_padding_ = config.footprint_3d_padding;
}

void Costmap3DROS::updateMap()
{
  std::lock_guard<std::mutex> initialize_lock(initialized_mutex_);

  if (initialized_ && !isPaused())
  {
    // First update the 3D map.
    // We update 3D first, in case any 3D layers need to affect 2D layers.
    // Get global pose of robot
    tf::Stamped < tf::Pose > pose;
    if (getRobotPose (pose))
    {
      geometry_msgs::Pose pose_msg;
      tf::poseTFToMsg(pose, pose_msg);
      layered_costmap_3d_->updateMap(pose_msg);
    }

    // Now update the 2D map
    super::updateMap();
  }
}

void Costmap3DROS::start()
{
  // check if we're stopped or just paused
  if (isStopped())
  {
    layered_costmap_3d_->activate();
  }

  super::start();
}

void Costmap3DROS::stop()
{
  super::stop();
  layered_costmap_3d_->deactivate();
}

void Costmap3DROS::resetLayers()
{
  layered_costmap_3d_->reset();
  super::resetLayers();
}

void Costmap3DROS::lock()
{
  super::lock();
  layered_costmap_3d_->lock();
}

void Costmap3DROS::unlock()
{
  layered_costmap_3d_->unlock();
  super::unlock();
}

std::vector<std::string> Costmap3DROS::getLayerNames()
{
  std::lock_guard<LayeredCostmap3D> lock(*layered_costmap_3d_);
  std::vector<std::string> plugin_names;

  for (auto plugin : layered_costmap_3d_->getPlugins())
  {
    plugin_names.push_back(plugin->getName());
  }

  return plugin_names;
}

void Costmap3DROS::clearAABB(geometry_msgs::Point min, geometry_msgs::Point max)
{
  // No need to reset the master map, as the next update will pull in the changes to each layer.
  clearAABB(min, max, getLayerNames());
}

void Costmap3DROS::clearAABB(geometry_msgs::Point min, geometry_msgs::Point max, const std::vector<std::string>& layers)
{
  std::lock_guard<LayeredCostmap3D> lock(*layered_costmap_3d_);

  for (auto plugin : layered_costmap_3d_->getPlugins())
  {
    // Only clear layers that are in the layers list argument.
    // This is not super efficient if the number of layers is large.
    // If the number of layers ever became large, we should store an index to
    // them by name.
    if (std::find(layers.begin(), layers.end(), plugin->getName()) != layers.end())
    {
      plugin->resetAABB(min, max);
    }
  }
}

double Costmap3DROS::footprintCost(geometry_msgs::Pose pose, double padding)
{
  // TODO: implement
  ROS_WARN_THROTTLE(1.0, "costmap_3d: footprintCost is not yet implemented");
  // XXX normalize return to [0.0, 1.0]?
  return INFINITY;
}

Costmap3DROS::FCLCollisionObjectPtr Costmap3DROS::getRobotCollisionObject(const geometry_msgs::Pose& pose)
{
  FCLCollisionObjectPtr obj(new FCLCollisionObject(robot_model_));

  obj->setTransform(
      fcl::Transform3<FCLFloat>(
          fcl::Translation3<FCLFloat>(pose.position.x, pose.position.y, pose.position.z) *
          fcl::Quaternion<FCLFloat>(pose.orientation.w,
                                    pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z)));

  return obj;
}

Costmap3DROS::FCLCollisionObjectPtr Costmap3DROS::getWorldCollisionObject()
{
  std::shared_ptr<const octomap::OcTree> octree_ptr(layered_costmap_3d_->getCostmap3D());
  std::shared_ptr<fcl::OcTree<FCLFloat>> fcl_octree_ptr(new fcl::OcTree<FCLFloat>(octree_ptr));
  return FCLCollisionObjectPtr(new fcl::CollisionObject<FCLFloat>(fcl_octree_ptr));
}

bool Costmap3DROS::footprintCollision(geometry_msgs::Pose pose, double padding)
{
  FCLCollisionObjectPtr robot(getRobotCollisionObject(pose));
  FCLCollisionObjectPtr world(getWorldCollisionObject());

  fcl::CollisionRequest<FCLFloat> request;
  fcl::CollisionResult<FCLFloat> result;

  fcl::collide(world.get(), robot.get(), request, result);

  return result.isCollision();
}

double Costmap3DROS::footprintDistance(geometry_msgs::Pose pose, double padding)
{
#if 1
  if (footprintCollision(pose, padding))
  {
    return -1.0;
  }
#endif
  FCLCollisionObjectPtr robot(getRobotCollisionObject(pose));
  FCLCollisionObjectPtr world(getWorldCollisionObject());

  FCLFloat pose_distance = std::numeric_limits<FCLFloat>::max();
//  FCLFloat pose_distance = 1.0;

#if 0
  {
    fcl::DistanceRequest<FCLFloat> request;
    fcl::DistanceResult<FCLFloat> result;

    request.abs_err = .001;
    request.distance_tolerance = .001;

    std::shared_ptr<fcl::Sphere<FCLFloat>> dot(new fcl::Sphere<FCLFloat>(0.0));

    FCLCollisionObjectPtr obj(new FCLCollisionObject(dot));
    obj->setTransform(
        fcl::Transform3<FCLFloat>(
            fcl::Translation3<FCLFloat>(pose.position.x, pose.position.y, pose.position.z) *
            fcl::Quaternion<FCLFloat>(pose.orientation.w,
                                      pose.orientation.x,
                                      pose.orientation.y,
                                      pose.orientation.z)));
    pose_distance = fcl::distance(world.get(), obj.get(), request, result);
  }
#endif

  fcl::DistanceRequest<FCLFloat> request;
  fcl::DistanceResult<FCLFloat> result;

  request.abs_err = 0.001;
  request.distance_tolerance = .001;

  result.min_distance = pose_distance;

  double distance = fcl::distance(world.get(), robot.get(), request, result);

  return distance;
}

double Costmap3DROS::footprintSignedDistance(geometry_msgs::Pose pose, double padding)
{
  FCLCollisionObjectPtr robot(getRobotCollisionObject(pose));
  FCLCollisionObjectPtr world(getWorldCollisionObject());

  fcl::DistanceRequest<FCLFloat> request;
  fcl::DistanceResult<FCLFloat> result;

  request.enable_signed_distance = true;

  return fcl::distance(world.get(), robot.get(), request, result);
}

void Costmap3DROS::updateMeshResource()
{
  std::string filename = getFileNameFromPackageURL(footprint_mesh_resource_);
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileSTL(filename, mesh);
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *mesh_points);

  // Convert the PCL PolygonMesh to the FCL BVHModel
  std::vector<fcl::Vector3<FCLFloat>> fcl_points;
  std::vector<fcl::Triangle> fcl_triangles;
  for (auto pcl_point : *mesh_points)
  {
    fcl_points.push_back(fcl::Vector3<FCLFloat>(pcl_point.x, pcl_point.y, pcl_point.z));
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
}

std::string Costmap3DROS::getFileNameFromPackageURL(const std::string& url)
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

void Costmap3DROS::getPlanCost3DActionCallback(
    const actionlib::SimpleActionServer<GetPlanCost3DAction>::GoalConstPtr& goal)
{
}

bool Costmap3DROS::getPlanCost3DServiceCallback(
    GetPlanCost3DService::Request& request,
    GetPlanCost3DService::Response& response)
{
  // Be sure the costmap is locked while querying.
  // Do not bother locking the 2D costmap, as only the 3D is queried.
  std::lock_guard<LayeredCostmap3D> lock(*layered_costmap_3d_);

  ROS_INFO("Starting getPlanCost3DServiceCallback");
  // For now, query each pose in a row. We could setup a continuous collision
  // FCL or a broad-phase check which are faster in the lazy case.

  // TODO: handle lethal_threshold.
  // TODO: handle footprint_mesh_resource
  // TODO: handle clipping AABB

  bool collision_only = request.collision_only;
  bool use_distance_for_cost = request.use_distance_for_cost;
  bool exact_signed_distance = request.exact_signed_distance;

  if (collision_only)
  {
    use_distance_for_cost = false;
  }
  if (!use_distance_for_cost)
  {
    exact_signed_distance = false;
  }

  if (use_distance_for_cost)
  {
    response.plan_cost = std::numeric_limits<double>::max();
  }
  else
  {
    response.plan_cost = 0.0;
  }

  for (int i = 0; i < request.poses.size(); ++i)
  {
    bool collision = false;
    // NOTE: Costmap3D does not support time (its not a 4D costmap!) so ignore the stamp.
    const auto pose = request.poses[i].pose;
    // Warn if the frame doesn't match. We currently don't transform the poses.
    if (request.poses[i].header.frame_id != layered_costmap_3d_->getGlobalFrameID())
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Costmap3DROS::getPlanCost3DServiceCallback expects poses in frame " <<
                               layered_costmap_3d_->getGlobalFrameID() << " but pose was in frame " <<
                               request.poses[i].header.frame_id);
    }
    double pose_cost;
    if (collision_only)
    {
      pose_cost = footprintCollision(pose, request.padding) ? 1.0 : 0.0;
    }
    else
    {
      if (use_distance_for_cost)
      {
        if (exact_signed_distance)
        {
          pose_cost = footprintSignedDistance(pose, request.padding);
        }
        else
        {
          pose_cost = footprintDistance(pose, request.padding);
        }
      }
      else
      {
        pose_cost = footprintCost(pose, request.padding);
      }
    }
    if (use_distance_for_cost)
    {
      // non-positive distance is a collision
      if (pose_cost <= 0.0)
      {
        collision = true;
      }
      response.plan_cost = std::min(response.plan_cost, pose_cost);
    }
    else
    {
      // lethal (or higher) cost is a collision
      if (pose_cost >= 1.0)
      {
        collision = true;
      }
      // Make the plan cost more and more lethal for each non-zero cost.
      // Ensure that just one lethal pose will keep the cost lethal.
      response.plan_cost = 1.0 - ((1.0 - response.plan_cost) * (1.0 - pose_cost));
    }
    response.pose_costs.push_back(pose_cost);
    if (collision)
    {
      response.in_collision = true;
      response.lethal_indices.push_back(i);
    }

    if (request.lazy && response.in_collision)
    {
      break;
    }
  }

  ROS_INFO_STREAM("Finished getting " << request.poses.size() << " poses.");
  return true;
}

}  // namespace costmap_3d
