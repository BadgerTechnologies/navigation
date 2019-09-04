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
#include <costmap_3d/point_cloud_layer_3d.h>
#include <pluginlib/class_list_macros.h>
#include <tf_conversions/tf_eigen.h>

PLUGINLIB_EXPORT_CLASS(costmap_3d::PointCloudLayer3D, costmap_3d::Layer3D)

namespace costmap_3d
{

PointCloudLayer3D::PointCloudLayer3D() : super()
{
}

PointCloudLayer3D::~PointCloudLayer3D()
{
}

void PointCloudLayer3D::initialize(LayeredCostmap3D* parent, std::string name, tf::TransformListener *tf)
{
  super::initialize(parent, name, tf);

  // Get names of topics and servers from parameter server
  pnh_ = ros::NodeHandle("~/" + name);

  cloud_topic_ = pnh_.param("cloud_topic", std::string("cloud"));

  ROS_INFO_STREAM("PointCloudLayer3D " << name << ": initializing");
  ROS_INFO_STREAM("  cloud_topic: " << cloud_topic_);

  dsrv_.reset(new dynamic_reconfigure::Server<costmap_3d::GenericPluginConfig>(pnh_));
  dsrv_->setCallback(std::bind(&PointCloudLayer3D::reconfigureCallback, this,
                               std::placeholders::_1, std::placeholders::_2));

  activate();
}

void PointCloudLayer3D::reconfigureCallback(costmap_3d::GenericPluginConfig &config, uint32_t level)
{
  std::lock_guard<Layer3D> lock(*this);
  enabled_ = config.enabled;
  combination_method_ = config.combination_method;
}

void PointCloudLayer3D::deactivate()
{
  super::deactivate();
  unsubscribe();
}

void PointCloudLayer3D::activate()
{
  super::activate();
  subscribe();
}

void PointCloudLayer3D::subscribe()
{
  std::lock_guard<Layer3D> lock(*this);
  cloud_sub_ = pnh_.subscribe<PointCloud>(cloud_topic_, 1, std::bind(&PointCloudLayer3D::pointCloudCallback,
                                                                     this, std::placeholders::_1));
}

void PointCloudLayer3D::unsubscribe()
{
  std::lock_guard<Layer3D> lock(*this);
  cloud_sub_.shutdown();
}

void PointCloudLayer3D::pointCloudCallback(const PointCloud::ConstPtr& cloud_msg)
{
  // Find the global frame transform.
  ros::Time stamp(pcl_conversions::fromPCL(cloud_msg->header.stamp));
  tf::StampedTransform global_frame_transform;
  try
  {
    tf_->lookupTransform(layered_costmap_3d_->getGlobalFrameID(),
                         cloud_msg->header.frame_id,
                         stamp,
                         global_frame_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "unable to transform: " << ex.what());
    return;
  }
  Eigen::Affine3d global_frame_transform_eigen;
  tf::transformTFToEigen(global_frame_transform, global_frame_transform_eigen);
  PointCloud::Ptr cloud_ptr(new PointCloud());
  pcl::transformPointCloud(*cloud_msg, *cloud_ptr, global_frame_transform_eigen);

  std::lock_guard<Layer3D> lock(*this);
  if (costmap_)
  {
    Costmap3D new_cells(costmap_->getResolution());
    Costmap3D unchanged_cells(costmap_->getResolution());
    for (auto pt : cloud_msg->points)
    {
      geometry_msgs::Point point;
      point.x = pt.x;
      point.y = pt.y;
      point.z = pt.z;
      Costmap3DIndex key;
      if (costmap_->coordToKeyChecked(toOctomapPoint(point), key))
      {
        Costmap3D::NodeType* node = costmap_->search(key);
        if (!node)
        {
          new_cells.setNodeValue(key, LETHAL);
        }
        else
        {
          unchanged_cells.setNodeValue(key, LETHAL);
        }
      }
    }
    Costmap3D erase_cells(*costmap_);
    // Remove the unchanged cells from the erase cells
    erase_cells.setTreeValues(NULL, &unchanged_cells, false, true);
    eraseCells(erase_cells);
    markAndClearCells(new_cells);
  }
}

}  // namespace costmap_3d
