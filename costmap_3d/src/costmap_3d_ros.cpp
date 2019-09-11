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
// Uncomment to compile in auto profiling support for optimizing queries
//#define COSTMAP_3D_ROS_AUTO_PROFILE_QUERY 1
#if (COSTMAP_3D_ROS_AUTO_PROFILE_QUERY) > 0
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#endif
#include <cmath>
#include <costmap_3d/costmap_3d_ros.h>
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

const Costmap3DROS::QueryMap* Costmap3DROS::getQuery(const std::string& footprint_mesh_resource, double padding)
{
  if (footprint_mesh_resource == "")
  {
    footprint_mesh_resource = footprint_mesh_resource_;
  }
  if (!isfinite(padding))
  {
    padding = footprint_3d_padding_;
  }
  auto query_it = query_map_.find(std::make_pair(footprint_mesh_resource, padding));
  if (query_it == query_map_.end())
  {
    // Query object does not exist, create it and add it to the map
    std::shared_ptr<Costmap3DQuery> query;
    query.reset(new Costmap3DQuery());
    query->setCostmap(layered_costmap_3d_);
    query->updateMeshResource(footprint_mesh_resource, padding);
    query_it = query_map_.insert(std::make_pair(std::make_pair(footprint_mesh_resource, padding), query)).first;
  }
  return &(*query_it);
}

double Costmap3DROS::footprintCost(geometry_msgs::Pose pose,
                                   const std::string& footprint_mesh_resource,
                                   double padding)
{
  return getQuery(footprint_mesh_resource, padding)->footprintCost(pose);
}

bool Costmap3DROS::footprintCollision(geometry_msgs::Pose pose,
                                      const std::string& footprint_mesh_resource,
                                      double padding)
{
  return getQuery(footprint_mesh_resource, padding)->footprintCollision(pose);
}

double Costmap3DROS::footprintDistance(geometry_msgs::Pose pose,
                                       const std::string& footprint_mesh_resource,
                                       double padding)
{
  return getQuery(footprint_mesh_resource, padding)->footprintDistance(pose);
}

double Costmap3DROS::footprintSignedDistance(geometry_msgs::Pose pose,
                                             const std::string& footprint_mesh_resource,
                                             double padding)
{
  return getQuery(footprint_mesh_resource, padding)->footprintSignedDistance(pose);
}

void Costmap3DROS::getPlanCost3DActionCallback(
    const actionlib::SimpleActionServer<GetPlanCost3DAction>::GoalConstPtr& goal)
{
  GetPlanCost3DResult result;
  processPlanCost3D(*goal, result);
  get_plan_cost_action_srv_->setSucceeded(result);
}

bool Costmap3DROS::getPlanCost3DServiceCallback(
    GetPlanCost3DService::Request& request,
    GetPlanCost3DService::Response& response)
{
  processPlanCost3D(request, response);
  return true;
}

template <typename RequestType, typename ResponseType>
void Costmap3DROS::processPlanCost3D(RequestType& request, ResponseType& response)
{
  // Be sure the costmap is locked while querying.
  // Do not bother locking the 2D costmap, as only the 3D is queried.
  std::lock_guard<LayeredCostmap3D> lock(*layered_costmap_3d_);

#if (COSTMAP_3D_ROS_AUTO_PROFILE_QUERY) > 0
  ROS_INFO("Starting getPlanCost3DServiceCallback");
  ros::Time start_time = ros::Time::now();
  kill(getpid(), 12);
#endif

  // TODO: handle footprint_mesh_resource
  // TODO: handle padding

  if (request.cost_query_mode == GetPlanCost3DService::Request::COST_QUERY_MODE_DISTANCE ||
      request.cost_query_mode == GetPlanCost3DService::Request::COST_QUERY_MODE_EXACT_SIGNED_DISTANCE)
  {
    response.plan_cost = std::numeric_limits<double>::max();
  }
  else
  {
    response.plan_cost = 0.0;
  }

  response.pose_costs.reserve(request.poses.size());
  response.lethal_indices.reserve(request.poses.size());
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
    if (request.cost_query_mode == GetPlanCost3DService::Request::COST_QUERY_MODE_COLLISON_ONLY)
    {
      pose_cost = footprintCollision(pose, request.footprint_mesh_resource, request.padding) ? -1.0 : 0.0;
    }
    else if (request.cost_query_mode == GetPlanCost3DService::Request::COST_QUERY_MODE_DISTANCE)
    {
      pose_cost = footprintDistance(pose, request.footprint_mesh_resource, request.padding);
    }
    else if (request.cost_query_mode == GetPlanCost3DService::Request::COST_QUERY_MODE_EXACT_SIGNED_DISTANCE)
    {
      pose_cost = footprintSignedDistance(pose, request.footprint_mesh_resource, request.padding);
    }
    else
    {
      pose_cost = footprintCost(pose, request.footprint_mesh_resource, request.padding);
    }
    // negative is a collision
    if (pose_cost < 0.0)
    {
      collision = true;
    }
    if (request.cost_query_mode == GetPlanCost3DService::Request::COST_QUERY_MODE_DISTANCE ||
        request.cost_query_mode == GetPlanCost3DService::Request::COST_QUERY_MODE_EXACT_SIGNED_DISTANCE)
    {
      // in distance mode, the plan_cost is the minimum distance across all poses
      response.plan_cost = std::min(response.plan_cost, pose_cost);
    }
    else
    {
      // in collision or cost mode, plan cost will either be negative if there is a collision
      // or the aggregate non-lethal cost.
      if (collision)
      {
        if (response.plan_cost >= 0.0)
        {
          // this pose is in collision, but the plan hasn't seen a collision yet.
          // reset the plan_cost to this pose's cost
          response.plan_cost = pose_cost;
        }
        else
        {
          // otherwise, add the lethal pose_cost to the plan_cost (making it more negative)
          response.plan_cost += pose_cost;
        }
      }
      else
      {
        if (response.plan_cost >= 0.0)
        {
          // not in collision, plan not in collision, add the cost
          response.plan_cost += pose_cost;
        }
        // don't add the non-lethal cost to a lethal plan
      }
    }
    response.pose_costs.push_back(pose_cost);
    if (collision)
    {
      response.lethal_indices.push_back(i);
    }

    if (request.lazy && response.in_collision)
    {
      break;
    }
  }

#if (COSTMAP_3D_ROS_AUTO_PROFILE_QUERY) > 0
  kill(getpid(), 12);
  ROS_INFO_STREAM("Finished getting " << request.poses.size() << " poses in " <<
                  (ros::Time::now() - start_time).toSec() << " seconds.");
#endif
}

}  // namespace costmap_3d
