#!/usr/bin/env python
"""
Test the performance of 3D costmap by sending a veyr large get plan cost service requests.
"""

import sys
import math
import random
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import costmap_3d.srv
import geometry_msgs.msg


if __name__ == "__main__":

    rospy.init_node("test_costmap3d_get_plan_cost_performance", anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    try:
        xform = tfBuffer.lookup_transform('odom', 'base_footprint',
                rospy.Time(0.0), rospy.Duration(10.0))

    except tf2_ros.TransformException as e:
        rospy.logerr("Cannot determine base footprint transform")
        sys.exit(1)

    get_cost_srv = rospy.ServiceProxy("/test_costmap_3d/costmap/get_plan_cost_3d",
            costmap_3d.srv.GetPlanCost3DService)
    req = costmap_3d.srv.GetPlanCost3DServiceRequest()
    req.lazy = False
    req.header.frame_id = "odom"
    req.collision_only = False
    req.use_distance_for_cost = True
    req.exact_signed_distance = False
    req.lethal_threshold = 1.0
    req.footprint_mesh_resource = ""
    req.footprint_clipping_aabb_min.x = float('nan')
    req.footprint_clipping_aabb_min.y = float('nan')
    req.footprint_clipping_aabb_min.z = float('nan')
    req.footprint_clipping_aabb_max.x = float('nan')
    req.footprint_clipping_aabb_max.y = float('nan')
    req.footprint_clipping_aabb_max.z = float('nan')
    req.padding = float('nan')
    for i in range(0,10):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "base_footprint"
        pose.pose.position.x = random.uniform(-4.0, 4.0)
        pose.pose.position.y = random.uniform(-4.0, 4.0)
        pose.pose.position.z = 0.0
        theta = random.uniform(-math.pi, math.pi)
        for i in range(0,20):
            # Simulate optimization step
            pose.pose.position.x += random.uniform(-.05, .05)
            pose.pose.position.y += random.uniform(-.05, .05)
            theta += random.uniform(-math.pi/36, math.pi/36)

            q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            req.poses.append(tf2_geometry_msgs.do_transform_pose(pose, xform))
            # simulate calls for jacobian
            pose.pose.position.x += .001
            req.poses.append(tf2_geometry_msgs.do_transform_pose(pose, xform))
            pose.pose.position.x -= .001
            pose.pose.position.y += .001
            req.poses.append(tf2_geometry_msgs.do_transform_pose(pose, xform))
            pose.pose.position.y -= .001
            theta += .0062
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            req.poses.append(tf2_geometry_msgs.do_transform_pose(pose, xform))

#    rospy.loginfo("Request: " + str(req))
    res = get_cost_srv(req)
    rospy.loginfo("Result: " + str(res))
    rospy.loginfo("Number of lethals: " + str(len(res.lethal_indices)))
