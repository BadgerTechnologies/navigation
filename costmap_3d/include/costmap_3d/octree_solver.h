/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  Copyright (c) 2019-2020, Badger Technologies, LLC
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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
 */

#ifndef COSTMAP_3D_COSTMAP_MESH_DISTANCE_H
#define COSTMAP_3D_COSTMAP_MESH_DISTANCE_H

#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>
#include <functional>

#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <octomap/octomap.h>

namespace costmap_3d
{

/** Find distance between an FCL OcTree and a mesh representing a closed volume.
 *
 * FCL does not currently have any API support for treating a mesh as a closed
 * volume. For Costmap purposes, the mesh is always treated as a closed (but
 * not necessarily convex) volume. Because of this, FCL's octree distance
 * function will not find interior collisions and is therefore inadequate for
 * costmap queries.
 */
template <typename NarrowPhaseSolver>
class OcTreeMeshSolver
{
  using S = typename NarrowPhaseSolver::S;
  // Only OBBRSS Meshes are supported
  using BV = typename fcl::OBBRSS<S>;
public:
  // Function returning the negative penetration distance of the box into the
  // mesh. If the box is not entirely within the mesh, this function may simply
  // return any non-negative number. This function must always return the
  // negative penetration distance when the entire box is inside the mesh. It
  // is OK for it to miss cases when the box and the mesh touch, as those are
  // handled directly by the solver traversal.  The solver needs to know when
  // an interior collision happens (and how deep it is), but knowing that is
  // outside of its scope. It uses this function to find out if there is an
  // interior collision and the penetration depth.
  //
  // The box_to_mesh_tf must put the box into the *mesh* frame.
  using InteriorCollisionFunction = std::function<S(const fcl::Box<S>& box,
                                                    const fcl::Vector3<S>& box_center,
                                                    const fcl::Transform3<S>& mesh_tf,
                                                    const fcl::Transform3<S>& inverse_mesh_tf,
                                                    int* mesh_triangle_id_ptr)>;
  using SignedDistanceFunction = std::function<S(const fcl::Box<S>& box,
                                                 const fcl::Vector3<S>& box_center,
                                                 int mesh_triangle_id,
                                                 const fcl::Transform3<S>& mesh_tf)>;

  // If interior_collision_function is given, that function is used to detect
  // interior collisions, and to directly estimate the distance for an interior
  // collision. This makes searching all interior collisions fast enough to
  // find the deepest penetrating collision for calculating signed distance,
  // and also correctly models the mesh as a closed volume.
  // If no interior_collision_function is given, the nearest triangle to the
  // mesh will be used.
  // If signed_distance_function is given and doing signed distance queries,
  // the returned distance for a collision between the mesh and the octomap
  // will use this function to modify the distance. This can be used to model
  // the signed distance for the boundary as a halfspace-box penetration, for
  // instance.
  // When doing signed distance queries, all octomap boxes that touch the mesh,
  // or are in the volume (when interior_collision_function is given) are
  // queried to find the deepest penetration to return as the signed distance.
  // When doing non-signed distance, once it is known the mesh and map collide,
  // -1.0 is immediately returned.
  OcTreeMeshSolver(
      const NarrowPhaseSolver* solver,
      InteriorCollisionFunction interior_collision_function = InteriorCollisionFunction(),
      SignedDistanceFunction signed_distance_function = SignedDistanceFunction())
      : solver_(solver),
        interior_collision_function_(interior_collision_function),
        signed_distance_function_(signed_distance_function)
  {
  }

  struct DistanceRequest
  {
    // Compute accurate penetration depth of first collision found
    bool enable_signed_distance = false;

    // Compute accurate penetration depth of deepest collision (only works
    // properly if enable_signed_distance is also set).
    bool enable_exact_signed_distance = false;

    // relative error, between 0 and 1
    S rel_err = 0.0;

    // Only consider octomap nodes inside all of the set of halfspaces.
    // If roi_ptr is nullptr or roi_size is zero, this does nothing (the region
    // is assumed to be the universe).
    const fcl::Halfspace<S>* roi_ptr = nullptr;
    size_t roi_size = 0;
  };

  struct DistanceResult
  {
    // The distance query will be pruned by the initial min_distance speeding up
    // the query. This is very useful if an upper bound is already known on the
    // distance. Otherwise this distance will be set if the distance query found
    // an answer.
    S min_distance = std::numeric_limits<S>::infinity();

    // If set to non-null, write the nearest points corresponding to the found
    // distance. This must point to an array of at least two fcl::Vector3<S>. If
    // there is a collision, this may return a collision point or some point
    // inside the colliding volume which are not necessarily the furthest or
    // closest points.
    fcl::Vector3<S>* nearest_points = nullptr;

    // Box geometry of the octomap box which resulted in min_distance.
    fcl::Box<S> octomap_box;
    fcl::Vector3<S> octomap_box_center;

    // The mesh triangle id of the triangle which resulted in min_distance. If
    // min_distance is not found, this will always be -1 which may be reliably
    // used to determine if a distance was found or not.
    int mesh_triangle_id = -1;

    // The number of octomap box to mesh triangle calculations performed during
    // this distance computation.
    size_t primitive_distance_calculations = 0;

    // The number of octomap bounding volume to mesh bounding volume calculations
    // performed during this distance computation.
    size_t bv_distance_calculations = 0;
  };

  /** Distance between a octomap::OcTree and fcl BVHModel (mesh)
   *
   * Note: this method is NOT thread safe and modifies the solver state for
   * efficiency reasons. To run multiple parallel queries, simply instantiate
   * multiple solvers.
   */
  void distance(const octomap::OcTree* tree1,
                const fcl::BVHModel<BV>* tree2,
                const fcl::Transform3<S>& tf1,
                const fcl::Transform3<S>& tf2,
                const DistanceRequest& request,
                DistanceResult* result);

private:
  const NarrowPhaseSolver* solver_;
  InteriorCollisionFunction interior_collision_function_;
  SignedDistanceFunction signed_distance_function_;
  // Store many of the query options here to prevent having to put them on the
  // stack during recursion.
  const octomap::OcTree* octree_;
  const fcl::Halfspace<S>* roi_ptr_;
  size_t roi_size_;
  const fcl::BVHModel<BV>* mesh_;
  DistanceResult* dresult_;
  fcl::Transform3<S> mesh_tf_;
  fcl::Transform3<S> mesh_tf_inverse_;
  const typename fcl::OcTree<S>::OcTreeNode* leaf_;
  const fcl::AABB<S>* leaf_bv_;
  fcl::Vector3<S> leaf_bv_center_;
  S leaf_bv_radius_;
  // For an average query, around a thousand OBB comparisons happen, and each
  // one of them needs the inverse TF to put the octomap BV into the OBB
  // frame. It is much cheaper on average to pre-compute all the inverse TFs
  std::vector<fcl::Transform3<S>> world_to_obb_internal_tfs_;
  double rel_err_factor_ = 1.0;
  bool interior_collision_;
  bool signed_distance_ = false;
  bool exact_signed_distance_ = false;

  template <bool check_roi>
  bool OcTreeMeshDistanceRecurse(const typename fcl::OcTree<S>::OcTreeNode* root1,
                                 fcl::AABB<S>* bv1,
                                 S bv1_radius);

  // Recurse the mesh on a specific OcTree cell
  bool MeshDistanceRecurse(int root2);

  // Start mesh distance checks. This function checks the interior collision
  // LUT prior to recursion.
  bool MeshDistanceStart(const typename fcl::OcTree<S>::OcTreeNode* root1,
                         const fcl::AABB<S>& bv1,
                         S bv1_radius);
};

template <typename NarrowPhaseSolver>
void OcTreeMeshSolver<NarrowPhaseSolver>::distance(
    const octomap::OcTree* tree1,
    const fcl::BVHModel<BV>* tree2,
    const fcl::Transform3<S>& tf1,
    const fcl::Transform3<S>& tf2,
    const OcTreeMeshSolver<NarrowPhaseSolver>::DistanceRequest& request,
    OcTreeMeshSolver<NarrowPhaseSolver>::DistanceResult* result)
{
  octree_ = tree1;
  mesh_ = tree2;
  dresult_ = result;
  mesh_tf_ = tf1.inverse() * tf2;
  mesh_tf_inverse_ = mesh_tf_.inverse();
  rel_err_factor_ = std::max(std::min(1.0 - request.rel_err, 1.0), 0.0);
  interior_collision_ = false;
  roi_size_ = request.roi_size;

  world_to_obb_internal_tfs_.clear();
  world_to_obb_internal_tfs_.reserve(tree2->getNumBVs());
  for (unsigned int bv_index = 0; bv_index < tree2->getNumBVs(); ++bv_index)
  {
    const fcl::OBB<S>& obb = tree2->getBV(bv_index).bv.obb;
    fcl::Transform3<S> obb_internal_tf;
    obb_internal_tf.linear() = obb.axis;
    obb_internal_tf.translation() = obb.To;
    world_to_obb_internal_tfs_.push_back((mesh_tf_ * obb_internal_tf).inverse());
  }

  // Skip if the entire tree is not occupied, as the occupied check is carried
  // out prior to descending the tree. This is done to skip having to check
  // distances to unoccupied children nodes. Doing this check here prior to the
  // first recursion is a small optimization to avoid having to double-check in
  // each call.
  if (!octree_->isNodeOccupied(tree1->getRoot()))
    return;

  // Construct the root BV for the tree.
  S delta = octree_->getNodeSize(1);
  fcl::AABB<S> root_bv(fcl::Vector3<S>(-delta, -delta, -delta), fcl::Vector3<S>(delta, delta, delta));

  if (roi_size_ == 0)
  {
    roi_ptr_ = nullptr;
    OcTreeMeshDistanceRecurse<false>(
        tree1->getRoot(),
        &root_bv,
        root_bv.radius());
  }
  else
  {
    roi_ptr_ = request.roi_ptr;
    OcTreeMeshDistanceRecurse<true>(
        tree1->getRoot(),
        &root_bv,
        root_bv.radius());
  }
}

template <typename S>
inline double sphereOBBSignedDistance(
    S radius,
    const fcl::Vector3<S>& sphere_center,
    const fcl::OBB<S>& obb,
    const fcl::Transform3<S>& world_to_obb_internal_tf)
{
  // Find the sphere center in the obb's frame.
  const fcl::Vector3<S> sphere_center_in_obb = (world_to_obb_internal_tf * sphere_center);
  // Simplify the problem by leveraging the symmetry of the problem. The
  // distance is the same for any point as its projection into the first octant
  // (removing the sign).
  const fcl::Vector3<S> abs_sphere_center = sphere_center_in_obb.cwiseAbs();

  // Translate the problem so the corner of the (now axis-aligned) obb is at
  // the origin.
  const fcl::Vector3<S> sphere_center_shifted = abs_sphere_center - obb.extent;

  bool x_negative = std::signbit(sphere_center_shifted(0));
  bool y_negative = std::signbit(sphere_center_shifted(1));
  bool z_negative = std::signbit(sphere_center_shifted(2));
  bool x_bigger_y = sphere_center_shifted(0) > sphere_center_shifted(1);
  bool x_bigger_z = sphere_center_shifted(0) > sphere_center_shifted(2);
  bool y_bigger_z = sphere_center_shifted(1) > sphere_center_shifted(2);
  // This routine is run thousands of times per query and must run as fast as
  // possible. Avoid the short-circuiting behavior of logical and and use
  // bitwise and instead. This is allowed because C++ requires booleans to be
  // represented by 1 for true and 0 for false. In practice using bitwise and
  // here saves about 10 instruction cycles on average. While this may sound
  // negligible, this entire routine (including using bitwise and) runs in
  // about 36 instruction cycles on the machine it was tested on.
  bool x_biggest = x_bigger_y & x_bigger_z;
  bool y_biggest = !x_bigger_y & y_bigger_z;
  bool z_biggest = !x_bigger_z & !y_bigger_z;
  bool inside = x_negative & y_negative & z_negative;
  // Since the box's corner is now on the origin, its sides are all on
  // coordinate planes, so either clamp to the appropriate box side or use the
  // shifted sphere center as the box point coordinates.
  bool clamp_x = x_negative & !(inside & x_biggest);
  bool clamp_y = y_negative & !(inside & y_biggest);
  bool clamp_z = z_negative & !(inside & z_biggest);
  // Use a LUT to select the clamping to avoid branching here, branching here
  // is expensive as it is hard to predict if clamping is necessary or not.
  S x_lut[2] = {sphere_center_shifted(0), 0.0};
  S y_lut[2] = {sphere_center_shifted(1), 0.0};
  S z_lut[2] = {sphere_center_shifted(2), 0.0};
  fcl::Vector3<S> clamped_point(
      x_lut[clamp_x],
      y_lut[clamp_y],
      z_lut[clamp_z]);
  S norm = clamped_point.norm();
  // Using a LUT for signed norm is slower in practice as being *inside* the
  // box is infrequent, so the branch predictor *usually* gets it right. Let
  // the compiler generate a branch here if it wants to by using a simple
  // ternary to set signed_norm.
  S signed_norm = inside ? -norm : norm;
  return signed_norm - radius;
}

template <typename S>
inline S distanceOctomapOBB(
    S radius,
    const fcl::Vector3<S>& bv1_center,
    const fcl::OBB<S>& obb,
    const fcl::Transform3<S>& world_to_obb_internal_tf)
{
  return sphereOBBSignedDistance(
    radius,
    bv1_center,
    obb,
    world_to_obb_internal_tf);
}

template <typename S>
inline double sphereRSSSignedDistance(
    S radius,
    const fcl::Transform3<S>& sphere_tf,
    const fcl::RSS<S>& rss,
    const fcl::Transform3<S>& rss_tf)
{
  // Find the sphere center C in the RSS's (final) frame.
  fcl::Transform3<S> rss_internal_tf;
  rss_internal_tf.linear() = rss.axis;
  rss_internal_tf.translation() = rss.To;
  const fcl::Transform3<S> X_RS = (rss_tf * rss_internal_tf).inverse() * sphere_tf;
  const fcl::Vector3<S> p_RC = X_RS.translation();

  // Find N, the nearest point *inside* the RSS rectangle to the sphere center C (measured
  // and expressed in frame R)
  fcl::Vector3<S> p_RN(
      std::min(rss.l[0], std::max(0.0, p_RC(0))),
      std::min(rss.l[1], std::max(0.0, p_RC(1))),
      0.0);

  return (p_RC - p_RN).norm() - rss.r - radius;
}

template <typename S>
inline S distanceOctomapRSS(
    S radius,
    const fcl::Vector3<S>& bv1_center,
    const fcl::RSS<S>& bv2,
    const fcl::Transform3<S>& tf2)
{
  fcl::Transform3<S> sphere_tf(fcl::Transform3<S>::Identity());
  sphere_tf.translation() = bv1_center;

  return sphereRSSSignedDistance(
    radius,
    sphere_tf,
    bv2,
    tf2);
}

template <typename S>
inline S distanceOctomapOBBRSS(
    S radius,
    const fcl::Vector3<S>& bv1_center,
    const fcl::OBBRSS<S>& bv2,
    const fcl::Transform3<S>& world_to_bv2_internal_tf)
{
  // For most meshes, the OBB will be a better fit than the RSS and will result
  // in fewer collisions. This is because most meshes will be angular and boxy.
  // Also it seems FCL's fitting is tighter for OBB than RSS. Both are now
  // cheap to calculate accurate signed distance with the above distance
  // functions. The signed distance is important as it is better for steering
  // the searching of the octree in the correct direction (as it points to the
  // *deepest* collision).
  return distanceOctomapOBB(
      radius,
      bv1_center,
      bv2.obb,
      world_to_bv2_internal_tf);
}

// Version of FCL's computeChildBV that is branchless and inline. The octree
// descent uses computeChildBV on a fast-path, so it needs to run as fast as
// possible.
template <typename S>
static inline void computeChildMinMax(
    const fcl::AABB<S>& root_bv,
    unsigned int i,
    fcl::Vector3<S>* child_min,
    fcl::Vector3<S>* child_max)
{
  const fcl::Vector3<S> root_bv_min = root_bv.min_;
  const fcl::Vector3<S> root_bv_max = root_bv.max_;
  const fcl::Vector3<S> root_bv_center = (root_bv_min + root_bv_max) * 0.5;
  (*child_min)[0] = (i&1) ? root_bv_center[0] : root_bv_min[0];
  (*child_max)[0] = (i&1) ? root_bv_max[0] : root_bv_center[0];
  (*child_min)[1] = (i&2) ? root_bv_center[1] : root_bv_min[1];
  (*child_max)[1] = (i&2) ? root_bv_max[1] : root_bv_center[1];
  (*child_min)[2] = (i&4) ? root_bv_center[2] : root_bv_min[2];
  (*child_max)[2] = (i&4) ? root_bv_max[2] : root_bv_center[2];
}

// return -1 if bv1 out of roi, 1 if in, and 0 if on.
template <typename S>
inline int checkROI(const fcl::AABB<S>& bv1, const fcl::Halfspace<S>* roi, size_t roi_size)
{
  fcl::Vector3<S> bv1_center = bv1.center();
  fcl::Vector3<S> bv1_diag = bv1.max_ - bv1.min_;
  bool all_in = true;

  for (unsigned int roi_index=0; roi_index < roi_size; ++roi_index)
  {
    // This is performance critical code.
    // So do not call boxHalfSpaceSignedDistance, but repeat the work here, as
    // we do not want to spend the time to create a Box from an AABB.
    // Also, we know that the AABB is axis-aligned with the world frame and
    // skip rotating the halfspace normal into the boxes frame.
    const fcl::Halfspace<S>& region(roi[roi_index]);
    fcl::Vector3<S> normal = region.n;
    fcl::Vector3<S> n_dot_d(normal[0] * bv1_diag[0], normal[1] * bv1_diag[1], normal[2] * bv1_diag[2]);
    fcl::Vector3<S> n_dot_d_abs = n_dot_d.cwiseAbs();
    S bv1_extent = 0.5 * (n_dot_d_abs[0] + n_dot_d_abs[1] + n_dot_d_abs[2]);
    S center_dist = region.signedDistance(bv1_center);
    // If the distance from the center of the AABB in bv1 to the halfspace is
    // bigger than the maximum extent of the AABB, the AABB is outside the
    // boundary of the halfspace. If the negative of the center distance is
    // bigger than the extent, the whole AABB is inside the halfspace.
    // Otherwise, its on the boundary.
    bool out = (center_dist > bv1_extent);
    bool in = (-center_dist > bv1_extent);
    if (out)
    {
      return -1;
    }
    all_in = (all_in && in);
  }
  if (all_in)
  {
    return 1;
  }
  return 0;
}

template <typename NarrowPhaseSolver>
bool OcTreeMeshSolver<NarrowPhaseSolver>::MeshDistanceRecurse(int root2)
{
  const octomap::OcTree* tree1 = octree_;
  const typename fcl::OcTree<S>::OcTreeNode* root1 = leaf_;
  const fcl::AABB<S>& bv1 = *leaf_bv_;
  const fcl::Vector3<S>& bv1_center = leaf_bv_center_;
  S bv1_radius = leaf_bv_radius_;
  const fcl::BVHModel<BV>* tree2 = mesh_;
  const fcl::Transform3<S>& tf2 = mesh_tf_;
  if (tree2->getBV(root2).isLeaf())
  {
    fcl::Box<S> box(bv1.max_ - bv1.min_);
    fcl::Transform3<S> box_tf = fcl::Transform3<S>::Identity();
    fcl::Vector3<S> box_center = (bv1.max_ + bv1.min_) * 0.5;
    box_tf.translation() = box_center;

    int primitive_id = tree2->getBV(root2).primitiveId();
    const fcl::Triangle& tri_id = tree2->tri_indices[primitive_id];
    const fcl::Vector3<S>& p1 = tree2->vertices[tri_id[0]];
    const fcl::Vector3<S>& p2 = tree2->vertices[tri_id[1]];
    const fcl::Vector3<S>& p3 = tree2->vertices[tri_id[2]];

    S dist;
    fcl::Vector3<S> closest_p1, closest_p2;
    bool get_closest_points = (dresult_->nearest_points != nullptr);
    solver_->shapeTriangleDistance(box, box_tf, p1, p2, p3, tf2, &dist,
        get_closest_points ? &closest_p1 : nullptr,
        get_closest_points ? &closest_p2 : nullptr);
    dresult_->primitive_distance_calculations++;

    if (dist < 0.0 && signed_distance_ && signed_distance_function_)
    {
      dist = signed_distance_function_(
          box,
          box_center,
          primitive_id,
          tf2);
    }

    if (dist < dresult_->min_distance)
    {
      dresult_->min_distance = dist;
      dresult_->octomap_box = box;
      dresult_->octomap_box_center = box_center;
      dresult_->mesh_triangle_id = primitive_id;
      if (get_closest_points)
      {
        dresult_->nearest_points[0] = closest_p1;
        dresult_->nearest_points[1] = closest_p2;
      }
    }

    return exact_signed_distance_ ? false : dist <= 0;
  }
  else
  {
    int children[2] = {
      tree2->getBV(root2).leftChild(),
      tree2->getBV(root2).rightChild()};
    const BV* bv2[2] = {
      &tree2->getBV(children[0]).bv,
      &tree2->getBV(children[1]).bv};
    S d[2] = {
      distanceOctomapOBBRSS(bv1_radius, bv1_center, *bv2[0], world_to_obb_internal_tfs_[children[0]]),
      distanceOctomapOBBRSS(bv1_radius, bv1_center, *bv2[1], world_to_obb_internal_tfs_[children[1]])};
    dresult_->bv_distance_calculations+=2;
    // Go left first if it is closer, otherwise go right first
    bool go_left = (d[0] < d[1]);
    unsigned int start = go_left ? 0 : 1;
    int step = go_left ? 1 : -1;
    // The negative step will take us to max unsigned which will terminate the
    // loop too.
    for (unsigned int child_index=start; child_index < 2; child_index+=step)
    {
      if (d[child_index] < rel_err_factor_ * dresult_->min_distance)
      {
        if (MeshDistanceRecurse(children[child_index]))
          return true;
      }
      else
      {
        // No need to continue the loop, the other entry is too far away.
        break;
      }
    }
  }

  return false;
}

template <typename NarrowPhaseSolver>
bool OcTreeMeshSolver<NarrowPhaseSolver>::MeshDistanceStart(
    const typename fcl::OcTree<S>::OcTreeNode* root1,
    const fcl::AABB<S>& bv1,
    S bv1_radius)
{
  const octomap::OcTree* tree1 = octree_;
  const fcl::BVHModel<BV>* tree2 = mesh_;
  // There is no way (that I can think of) to correctly direct the search to
  // avoid having to test each box that touches the bounding volume of the
  // whole mesh. So, to make it take a reasonable amount of time to check each
  // box that is tangential to the bounding volume of the mesh (because its
  // min_distance lower bound would be zero), create a LUT to use that takes
  // as input the centroid of the octomap box, rounded to a grid, and looks up
  // the mesh triangle that is closest to a box modeling that grid cell. This
  // will give close enough results to work well for costmap queries in a
  // reasonable amount of time. After looking up the mesh (instead of having
  // to search for it through the BVH), the mesh triangle is modeled as a
  // halfspace (all halfspace models can also be created at construction time)
  // and the box to halfspace distance can be calculated (this is a fast
  // calculation compared to a GJK run). This distance will then be used as
  // the penetration distance.
  //
  // The bound distance will never be negative, clip all negatives to zero, as
  // the overlap of bounding volumes does not give an accurate penetration
  // depth. This forces the search to try all boxes within the mesh.
  //
  // The LUT and halfspaces will be pre-calculated and passed to the solver.
  // If the solver does not have them, it will proceed normally. This way,
  // the solver can be used to create the LUT by finding the distance
  // between an octomap with only one cell occupied to identify the closest
  // mesh triangle to that spot. It is outside the scope of this class how
  // to determine if the octomap box is inside the mesh, but a method like
  // PCL's crop hull could be used on the centroid of the mesh. This LUT
  // mechanism does introduce some inaccuracies when the rounded centroid
  // appears in the mesh, but is actually outside the mesh. For this reason,
  // the LUT resolution should be more than double the octomap resolution.
  // This can easily be achieved by using the transform argument to the
  // distance function. Create an octomap with a single cell at the octomap
  // index corresponding to the cell near the origin. Then provide different
  // arguments for the transform to find the nearest mesh polygon for each
  // entry.
  //
  // The LUT could also be made more accurate by taking not only position but
  // orientation as an input. This can be abstracted at the LUT level by
  // passing the transform to put the box into the mesh frame (this
  // transform can be pre-calculated once for the whole distance operation).
  // Then the LUT can decide how to find the appropriate entry.
  if (interior_collision_function_)
  {
    // There is an interior collision function and we are at the root of the BVH.
    fcl::Box<S> box(bv1.max_ - bv1.min_);
    fcl::Vector3<S> box_center = (bv1.max_ + bv1.min_) * 0.5;
    int primitive_id;
    S collision_distance = interior_collision_function_(
        box,
        box_center,
        mesh_tf_,
        mesh_tf_inverse_,
        &primitive_id);
    if (collision_distance < 0.0)
    {
      // A known interior collision.
      interior_collision_ = true;
      if (collision_distance < dresult_->min_distance)
      {
        dresult_->min_distance = collision_distance;
        dresult_->octomap_box = box;
        dresult_->octomap_box_center = box_center;
        dresult_->mesh_triangle_id = primitive_id;
        if (dresult_->nearest_points)
        {
          // The "nearest" point could be calculated by the interior collision
          // function but is currently unimplemented. For now just return the
          // center point of the box (which is one known collision point).
          dresult_->nearest_points[0] = box_center;
          dresult_->nearest_points[1] = box_center;
        }
      }
      // For exact signed distance, keep going and checking other colliding cells
      return !exact_signed_distance_;
    }
    // If the current result is interior to the mesh, there is no need to
    // descend into the mesh, as any result will be less-penetrating.
    // This check is only helpful when exact signed distance is enabled, as
    // otherwise we would have already stopped once finding the first
    // collision.
    else if (interior_collision_)
    {
      return false;
    }
  }
  // Record the child octree node and octree bv information in the instance of
  // the class to avoid passing it around on the stack during mesh descent.
  leaf_ = root1;
  leaf_bv_ = &bv1;
  leaf_bv_center_ = bv1.center();
  leaf_bv_radius_ = bv1_radius;
  return MeshDistanceRecurse(0);
}

template <typename S>
static inline void SwapIfGreater(S* a, S* b)
{
  // Instead of using an if followed by std::swap, use ternary operations and a
  // temporary to get branchless swapping on architectures which support it
  // (such as cmov on x86).
  const bool swap = *a > *b;
  const S t = swap ? *a : *b;
  *a = swap ? *b : *a;
  *b = t;
}

template <typename S>
static inline uint64_t PackDistanceIndexUpTo8(const S distance, const unsigned index)
{
  // Pack a positive-offset version of the distance with the index for quick
  // compare and swap. Without packing, the compiler will use slower branches
  // for the swap-if-greater in the sort routine. With packing, it will be able
  // to use conditional moves, which are much cheaper as they do not cause
  // branches. Because Costmap3D distances can never be more than 2 to the
  // octomap depth times the octomap resolution, simply add a number larger
  // than the largest possible distance (1e6) to the distances to bias them
  // positive. This way the numbers can be directly bitwise compared without
  // explicit casting (and rounding) back to int. There is also plenty of
  // precision in a double floating point number to give up the bottom 3 bits
  // for the index. The worst possible precision of the packed distance then is
  // 1 / (2^(52-3) / 1e6) which is ~2nm, more than enough for a geometry
  // calculation, especially considering that GJK will internally use 1e-6 for
  // its tolerance.
  union {double d; uint64_t uint;} u = {distance + 1e6};
  return u.uint & ~0x07 | index;
}

static inline unsigned UnpackDistanceIndexUpTo8(const uint64_t di)
{
  return di & 0x07;
}

template <typename I, typename S>
static inline void ArgSortUpTo8(unsigned n, I* indices, const S* distances)
{
  // Sorting is dominated by the cost of comparing and branching, so using the
  // fewest possible comparisons yields optimal results. Therefore use the
  // well-known optimal sorting networks for 2-8 items taken from /The Art of
  // Programming/ by Knuth. Organize them so that (if possible) the
  // compiler/architecture can run comparisons and swaps in parallel (even if
  // the compiler is unable to do this via instruction-level parallelism, the
  // architecture itself may depending on its pipeline depth, for instance
  // while swapping one entry it could go ahead and execute the comparison for
  // the next when there is no data dependency on the swap).
  switch (n)
  {
    case 8:
      {
        // The compiler does a much better job ordering operations when using
        // individual local variables than with one local array of fixed size.
        // Presumably this is because it is easier to track data dependencies
        // on individual variables than an array. This does make the compiler
        // not vectorize the distance/index packing, but in practice the
        // speed-up in more efficient comparison ordering and interleaving is a
        // significant net gain and worth this limitation.
        uint64_t di0 = PackDistanceIndexUpTo8(distances[0], 0);
        uint64_t di1 = PackDistanceIndexUpTo8(distances[1], 1);
        uint64_t di2 = PackDistanceIndexUpTo8(distances[2], 2);
        uint64_t di3 = PackDistanceIndexUpTo8(distances[3], 3);
        uint64_t di4 = PackDistanceIndexUpTo8(distances[4], 4);
        uint64_t di5 = PackDistanceIndexUpTo8(distances[5], 5);
        uint64_t di6 = PackDistanceIndexUpTo8(distances[6], 6);
        uint64_t di7 = PackDistanceIndexUpTo8(distances[7], 7);
        // Depth 1
        SwapIfGreater(&di0, &di1);
        SwapIfGreater(&di2, &di3);
        SwapIfGreater(&di4, &di5);
        SwapIfGreater(&di6, &di7);
        // Depth 2
        SwapIfGreater(&di0, &di2);
        SwapIfGreater(&di1, &di3);
        SwapIfGreater(&di4, &di6);
        SwapIfGreater(&di5, &di7);
        // Depth 3
        SwapIfGreater(&di0, &di4);
        SwapIfGreater(&di1, &di5);
        SwapIfGreater(&di2, &di6);
        SwapIfGreater(&di3, &di7);
        // Depth 4
        SwapIfGreater(&di2, &di4);
        SwapIfGreater(&di3, &di5);
        // Depth 5
        SwapIfGreater(&di1, &di4);
        SwapIfGreater(&di3, &di6);
        // Depth 6
        SwapIfGreater(&di1, &di2);
        SwapIfGreater(&di3, &di4);
        SwapIfGreater(&di5, &di6);
        indices[0] = UnpackDistanceIndexUpTo8(di0);
        indices[1] = UnpackDistanceIndexUpTo8(di1);
        indices[2] = UnpackDistanceIndexUpTo8(di2);
        indices[3] = UnpackDistanceIndexUpTo8(di3);
        indices[4] = UnpackDistanceIndexUpTo8(di4);
        indices[5] = UnpackDistanceIndexUpTo8(di5);
        indices[6] = UnpackDistanceIndexUpTo8(di6);
        indices[7] = UnpackDistanceIndexUpTo8(di7);
      }
      break;
    case 7:
      {
        uint64_t di0 = PackDistanceIndexUpTo8(distances[0], 0);
        uint64_t di1 = PackDistanceIndexUpTo8(distances[1], 1);
        uint64_t di2 = PackDistanceIndexUpTo8(distances[2], 2);
        uint64_t di3 = PackDistanceIndexUpTo8(distances[3], 3);
        uint64_t di4 = PackDistanceIndexUpTo8(distances[4], 4);
        uint64_t di5 = PackDistanceIndexUpTo8(distances[5], 5);
        uint64_t di6 = PackDistanceIndexUpTo8(distances[6], 6);
        // Depth 1
        SwapIfGreater(&di1, &di2);
        SwapIfGreater(&di3, &di4);
        SwapIfGreater(&di5, &di6);
        // Depth 2
        SwapIfGreater(&di0, &di2);
        SwapIfGreater(&di3, &di5);
        SwapIfGreater(&di4, &di6);
        // Depth 3
        SwapIfGreater(&di0, &di4);
        SwapIfGreater(&di1, &di5);
        SwapIfGreater(&di2, &di6);
        // Depth 4
        SwapIfGreater(&di0, &di3);
        SwapIfGreater(&di2, &di5);
        // Depth 5
        SwapIfGreater(&di1, &di3);
        SwapIfGreater(&di2, &di4);
        // Depth 6
        SwapIfGreater(&di0, &di1);
        SwapIfGreater(&di2, &di3);
        SwapIfGreater(&di4, &di5);
        indices[0] = UnpackDistanceIndexUpTo8(di0);
        indices[1] = UnpackDistanceIndexUpTo8(di1);
        indices[2] = UnpackDistanceIndexUpTo8(di2);
        indices[3] = UnpackDistanceIndexUpTo8(di3);
        indices[4] = UnpackDistanceIndexUpTo8(di4);
        indices[5] = UnpackDistanceIndexUpTo8(di5);
        indices[6] = UnpackDistanceIndexUpTo8(di6);
      }
      break;
    case 6:
      {
        uint64_t di0 = PackDistanceIndexUpTo8(distances[0], 0);
        uint64_t di1 = PackDistanceIndexUpTo8(distances[1], 1);
        uint64_t di2 = PackDistanceIndexUpTo8(distances[2], 2);
        uint64_t di3 = PackDistanceIndexUpTo8(distances[3], 3);
        uint64_t di4 = PackDistanceIndexUpTo8(distances[4], 4);
        uint64_t di5 = PackDistanceIndexUpTo8(distances[5], 5);
        // Depth 1
        SwapIfGreater(&di0, &di1);
        SwapIfGreater(&di2, &di3);
        SwapIfGreater(&di4, &di5);
        // Depth 2
        SwapIfGreater(&di0, &di2);
        SwapIfGreater(&di3, &di5);
        SwapIfGreater(&di1, &di4);
        // Depth 3
        SwapIfGreater(&di0, &di1);
        SwapIfGreater(&di2, &di3);
        SwapIfGreater(&di4, &di5);
        // Depth 4
        SwapIfGreater(&di1, &di2);
        SwapIfGreater(&di3, &di4);
        // Depth 5
        SwapIfGreater(&di2, &di3);
        indices[0] = UnpackDistanceIndexUpTo8(di0);
        indices[1] = UnpackDistanceIndexUpTo8(di1);
        indices[2] = UnpackDistanceIndexUpTo8(di2);
        indices[3] = UnpackDistanceIndexUpTo8(di3);
        indices[4] = UnpackDistanceIndexUpTo8(di4);
        indices[5] = UnpackDistanceIndexUpTo8(di5);
      }
      break;
    case 5:
      {
        uint64_t di0 = PackDistanceIndexUpTo8(distances[0], 0);
        uint64_t di1 = PackDistanceIndexUpTo8(distances[1], 1);
        uint64_t di2 = PackDistanceIndexUpTo8(distances[2], 2);
        uint64_t di3 = PackDistanceIndexUpTo8(distances[3], 3);
        uint64_t di4 = PackDistanceIndexUpTo8(distances[4], 4);
        // Depth 1
        SwapIfGreater(&di1, &di2);
        SwapIfGreater(&di3, &di4);
        // Depth 2
        SwapIfGreater(&di0, &di2);
        SwapIfGreater(&di1, &di3);
        // Depth 3
        SwapIfGreater(&di0, &di3);
        SwapIfGreater(&di2, &di4);
        // Depth 4
        SwapIfGreater(&di0, &di1);
        SwapIfGreater(&di2, &di3);
        // Depth 5
        SwapIfGreater(&di1, &di2);
        indices[0] = UnpackDistanceIndexUpTo8(di0);
        indices[1] = UnpackDistanceIndexUpTo8(di1);
        indices[2] = UnpackDistanceIndexUpTo8(di2);
        indices[3] = UnpackDistanceIndexUpTo8(di3);
        indices[4] = UnpackDistanceIndexUpTo8(di4);
      }
      break;
    case 4:
      {
        uint64_t di0 = PackDistanceIndexUpTo8(distances[0], 0);
        uint64_t di1 = PackDistanceIndexUpTo8(distances[1], 1);
        uint64_t di2 = PackDistanceIndexUpTo8(distances[2], 2);
        uint64_t di3 = PackDistanceIndexUpTo8(distances[3], 3);
        // Depth 1
        SwapIfGreater(&di0, &di1);
        SwapIfGreater(&di2, &di3);
        // Depth 2
        SwapIfGreater(&di0, &di2);
        SwapIfGreater(&di1, &di3);
        // Depth 3
        SwapIfGreater(&di1, &di2);
        indices[0] = UnpackDistanceIndexUpTo8(di0);
        indices[1] = UnpackDistanceIndexUpTo8(di1);
        indices[2] = UnpackDistanceIndexUpTo8(di2);
        indices[3] = UnpackDistanceIndexUpTo8(di3);
      }
      break;
    case 3:
      {
        uint64_t di0 = PackDistanceIndexUpTo8(distances[0], 0);
        uint64_t di1 = PackDistanceIndexUpTo8(distances[1], 1);
        uint64_t di2 = PackDistanceIndexUpTo8(distances[2], 2);
        // Depth 1
        SwapIfGreater(&di0, &di1);
        // Depth 2
        SwapIfGreater(&di1, &di2);
        // Depth 3
        SwapIfGreater(&di0, &di1);
        indices[0] = UnpackDistanceIndexUpTo8(di0);
        indices[1] = UnpackDistanceIndexUpTo8(di1);
        indices[2] = UnpackDistanceIndexUpTo8(di2);
      }
      break;
    case 2:
      {
        const bool greater = distances[0] > distances[1];
        // C++ says bool is always 0 or 1, so for the simple 2 length case,
        // just directly use the boolean value. If entry 0 is the biggest,
        // greater is true (1) and the first index should be 1 (as entry 1 is
        // the smallest) and the other index is always the opposite.
        indices[0] = greater;
        indices[1] = !greater;
      }
      break;
    case 1:
      indices[0] = 0;
      break;
    case 0:
    default:
      break;
  }
}

template <typename NarrowPhaseSolver>
template <bool check_roi>
bool OcTreeMeshSolver<NarrowPhaseSolver>::OcTreeMeshDistanceRecurse(
    const typename fcl::OcTree<S>::OcTreeNode* root1,
    fcl::AABB<S>* bv1,
    S bv1_radius)
{
  const octomap::OcTree* tree1 = octree_;
  unsigned int nchildren;
  const typename fcl::OcTree<S>::OcTreeNode* children[8];
  fcl::Vector3<S> child_mins[8], child_maxs[8];

  // First calculate information about the children of this part of the octree.
  for (;;)
  {
    nchildren = 0;
    if (tree1->nodeHasChildren(root1))
    {
      for (unsigned int child_index = 0; child_index < 8; ++child_index)
      {
        if (tree1->nodeChildExists(root1, child_index))
        {
          const typename fcl::OcTree<S>::OcTreeNode* child = tree1->getNodeChild(root1, child_index);
          if (tree1->isNodeOccupied(child))
          {
            children[nchildren] = child;
            computeChildMinMax(*bv1, child_index, &child_mins[nchildren], &child_maxs[nchildren]);
            nchildren++;
          }
        }
      }
    }
    // If there are zero, two or more children, jump out of this loop and go
    // on, otherwise there is only one child present. When only one child is
    // present, skip all the below calculations descend into the single child.
    // The ROI and BV distance calculations below will all be more accurate
    // with the next single child (which represents a smaller volume) than the
    // current one so can simply be skipped. This also saves a recursive call
    // (which is only a small cost, but something).
    if (nchildren != 1)
      break;
    root1 = children[0];
    bv1_radius *= 0.5;
    bv1->min_ = child_mins[0];
    bv1->max_ = child_maxs[0];
  }

  // Check region of interest.
  bool entirely_inside_roi = false;
  if (check_roi)
  {
    int rv = checkROI<S>(*bv1, roi_ptr_, roi_size_);
    if (rv == -1)
    {
      // this octomap region is entirely out of the region of interest
      return false;
    }
    if (rv == 1)
    {
      // this octomap region is entirely inside the region of interest.
      // There is no need to check any sub-region. Null out the roi for
      // subsequent recursive calls.
      entirely_inside_roi = true;
    }
  }

  if (nchildren > 0)
  {
    S distances[8];
    unsigned sorted_indices[8];
    S radius = bv1_radius * 0.5;
    const BV& bv2 = mesh_->getBV(0).bv;
    for (unsigned int child_index = 0; child_index < nchildren; ++child_index)
    {
      const fcl::Vector3<S> child_bv_center = (child_mins[child_index] + child_maxs[child_index]) * 0.5;
      distances[child_index] = distanceOctomapOBBRSS(radius, child_bv_center, bv2, world_to_obb_internal_tfs_[0]);
    }
    dresult_->bv_distance_calculations += nchildren;
    // Visit the octree from closest to furthest and quit early when we have
    // crossed the result min distance. Do keep going though in exact signed
    // distance mode when this part of the octree overlaps the root bounding
    // volume of the mesh, as we must check every collision to find the deepest
    // in exact signed distance mode.
    ArgSortUpTo8(nchildren, sorted_indices, distances);
    for (unsigned child_index = 0; child_index < nchildren; ++child_index)
    {
      const unsigned sorted_index = sorted_indices[child_index];
      S min_distance = distances[sorted_index];
      if (min_distance < rel_err_factor_ * dresult_->min_distance || (exact_signed_distance_ && min_distance <= 0))
      {
        // Possible a better result is below, descend
        // No way to directly construct an AABB with a given min/max. Just use
        // the single-point constructor and directly set the max.
        fcl::AABB<S> child_bv(child_mins[sorted_index]);
        child_bv.max_ = child_maxs[sorted_index];
        if (check_roi && !entirely_inside_roi)
        {
          if (OcTreeMeshDistanceRecurse<true>(children[sorted_index], &child_bv, radius))
            return true;
        }
        else
        {
          if (OcTreeMeshDistanceRecurse<false>(children[sorted_index], &child_bv, radius))
            return true;
        }
      }
      else
      {
        // No need to continue the loop, all further entries are too far away.
        break;
      }
    }
  }
  else
  {
    // Because we descend the octree first, there is no need to check the
    // ROI when descending the mesh.
    return MeshDistanceStart(root1, *bv1, bv1_radius);
  }

  // This happens if the search under this region of the octree failed to find
  // the known closest distance within the error factors. The search will
  // continue...
  return false;
}

}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_MESH_DISTANCE_H
