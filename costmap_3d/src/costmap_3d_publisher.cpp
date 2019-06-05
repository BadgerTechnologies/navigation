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
#include <costmap_3d/costmap_3d_publisher.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapUpdate.h>
#include <octomap_msgs/conversions.h>

namespace costmap_3d
{

Costmap3DPublisher::Costmap3DPublisher(const ros::NodeHandle& nh,
                                       LayeredCostmap3D* layered_costmap_3d,
                                       std::string topic_name)
    : nh_(nh), layered_costmap_3d_(layered_costmap_3d), send_full_map_(true)
{
  costmap_pub_ = nh_.advertise<octomap_msgs::Octomap>(topic_name, 1,
                                                      std::bind(&Costmap3DPublisher::connectCallback, this,
                                                                std::placeholders::_1));
  costmap_update_pub_ = nh_.advertise<octomap_msgs::OctomapUpdate>(topic_name + "_updates", 1);

  update_complete_id = topic_name + "_publisher";
  layered_costmap_3d_->registerUpdateCompleteCallback(update_complete_id,
                                                      std::bind(&Costmap3DPublisher::updateCompleteCallback, this,
                                                                std::placeholders::_1,
                                                                std::placeholders::_2,
                                                                std::placeholders::_3));
}

Costmap3DPublisher::~Costmap3DPublisher()
{
  layered_costmap_3d_->unregisterUpdateCompleteCallback(update_complete_id);
}

void Costmap3DPublisher::connectCallback(const ros::SingleSubscriberPublisher& pub)
{
  octomap_msgs::Octomap msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = layered_costmap_3d_->getGlobalFrameID();
  {
    // Be sure to lock the costmap while using it
    std::lock_guard<LayeredCostmap3D> costmap_lock(*layered_costmap_3d_);
    octomap_msgs::fullMapToMsg(*layered_costmap_3d_->getCostmap3D(), msg);
  }
  pub.publish(msg);
}

void Costmap3DPublisher::updateCompleteCallback(LayeredCostmap3D* layered_costmap_3d,
                                                const Costmap3D& delta_map,
                                                const Costmap3D& bounds_map)
{
  // The layered costmap already holds the lock when calling the update
  // complete, no need to lock
  ros::Time stamp = ros::Time::now();
  std::string frame = layered_costmap_3d_->getGlobalFrameID();
  octomap_msgs::OctomapUpdate msg;
  msg.header.frame_id = frame;
  msg.header.stamp = stamp;
  msg.octomap_update.header.frame_id = frame;
  msg.octomap_update.header.stamp = stamp;
  msg.octomap_bounds.header.frame_id = frame;
  msg.octomap_bounds.header.stamp = stamp;
  // always send costs along with map
  octomap_msgs::fullMapToMsg(delta_map, msg.octomap_update);
  // bounds map is only ever binary
  octomap_msgs::binaryMapToMsg(bounds_map, msg.octomap_bounds);
  costmap_update_pub_.publish(msg);
}

}  // end namespace costmap_2d
