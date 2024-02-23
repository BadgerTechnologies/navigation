/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Badger Technologies LLC
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
 *
 * Author: C. Andy Martin
 *********************************************************************/

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <functional>
#include <random>
#include <vector>

#include <fcl/config.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <costmap_3d/costmap_3d_query.h>
#include <costmap_3d/octree_solver.h>

static const std::string PACKAGE_URL("package://costmap_3d/");

template <typename S>
void generateRandomTransforms(S extents[6], fcl::aligned_vector<fcl::Transform3<S>>& transforms, std::size_t n);

// Version of FCL's sphereBoxDistance that correctly calculates signed distance
// when the sphere penetrates the box.
template <typename S>
double sphereBoxSignedDistance(
    const fcl::Sphere<S>& sphere,
    const fcl::Transform3<S>& X_FS,
    const fcl::Box<S>& box,
    const fcl::Transform3<S>& X_FB)
{
  // Find the sphere center C in the box's frame.
  const fcl::Transform3<S> X_BS = X_FB.inverse() * X_FS;
  const fcl::Vector3<S> p_BC = X_BS.translation();
  const S r = sphere.radius;
  const fcl::Vector3<S> box_half_sides = box.side / 2;

  // Find N, the nearest point *inside* the box to the sphere center C (measured
  // and expressed in frame B)
  fcl::Vector3<S> p_BN;
  bool N_is_not_C = fcl::detail::nearestPointInBox(box.side, p_BC, &p_BN);

  if (N_is_not_C)
  {
    // If N is not C, we know the sphere center is *outside* the box (but we
    // don't know yet if the they are completely separated).

    // Compute the position vector from the nearest point N to the sphere center
    // C in the frame B.
    fcl::Vector3<S> p_NC_B = p_BC - p_BN;
    return p_NC_B.norm() - r;
  }

  // Sphere center inside box. Find the shallowest of the possible penetration
  // depths (the shallowest penetration is the maximum of a negative number)
  // and subtract the sphere radius.
  return std::max<S>({
      p_BN(0) - box_half_sides(0),
      -box_half_sides(0) - p_BN(0),
      p_BN(1) - box_half_sides(1),
      -box_half_sides(1) - p_BN(1),
      p_BN(2) - box_half_sides(2),
      -box_half_sides(2) - p_BN(2)}) - r;
}

// Simple non-optimized version of sphere-OBB signed distance used to check the
// optimized version in the code.
template <typename S>
inline S sphereOBBSignedDistance(
    S radius,
    const fcl::Vector3<S>& sphere_center,
    const fcl::OBB<S>& obb,
    const fcl::Transform3<S>& obb_tf)
{
  fcl::Box<S> box;
  fcl::Transform3<S> box_tf;
  fcl::constructBox(obb, box, box_tf);
  fcl::Transform3<S> sphere_tf(fcl::Transform3<S>::Identity());
  sphere_tf.translation() = sphere_center;

  return sphereBoxSignedDistance(
    fcl::Sphere<S>(radius),
    sphere_tf,
    box,
    obb_tf * box_tf);
}

// Eigen version, only for time comparison, as it does not calculate signed
// distance but only exteriorDistance.
template <typename S>
inline S sphereOBBSignedDistanceEigen(
    S radius,
    const fcl::Vector3<S>& sphere_center,
    const fcl::OBB<S>& obb,
    const fcl::Transform3<S>& obb_tf)
{
  // Find the sphere center in the obb's frame.
  const fcl::Vector3<S> sphere_center_in_obb = (obb_tf * sphere_center);
  // Calculate distance using Eigen's AlignedBox::exteriorDistance
  Eigen::AlignedBox<S, 3> eigen_aabb(-obb.extent, obb.extent);
  return eigen_aabb.exteriorDistance(sphere_center_in_obb) - radius;
}


TEST(test_octree_solver, test_distance_octomap_rss)
{
  fcl::OBBRSS<double> obbrss;
  fcl::AABB<double> aabb;
  fcl::Vector3<double> aabb_center(0.0, 0.0, 0.0);
  aabb.min_ = fcl::Vector3<double>(-1.0, -1.0, -1.0);
  aabb.max_ = fcl::Vector3<double>(1.0, 1.0, 1.0);
  double radius = aabb.radius();
  fcl::Transform3<double> obbrss_tf(fcl::Transform3<double>::Identity());

  obbrss.obb.To = fcl::Vector3<double>(0.0, 0.0, 0.0);
  obbrss.obb.axis = fcl::Matrix3<double>::Identity();
  obbrss.obb.extent(0) = 1.0;
  obbrss.obb.extent(1) = 2.0;
  obbrss.obb.extent(2) = 3.0;
  double d_obb, d_obb2;
  obbrss_tf.translation() = fcl::Vector3<double>(0.0, 0.0, 0.0);
  d_obb = costmap_3d::distanceOctomapOBB(aabb.radius(), aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, -1.0 - std::sqrt(3.0), 1e-6);
  d_obb2 = sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, d_obb2, 1e-9);
  obbrss_tf.translation() = fcl::Vector3<double>(0.5, 0.0, 0.0);
  d_obb = costmap_3d::distanceOctomapOBB(aabb.radius(), aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, -0.5 - std::sqrt(3.0), 1e-6);
  d_obb2 = sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, d_obb2, 1e-9);
  obbrss_tf.translation() = fcl::Vector3<double>(1.0, 0.0, 0.0);
  d_obb = costmap_3d::distanceOctomapOBB(aabb.radius(), aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, -std::sqrt(3.0), 1e-6);
  d_obb2 = sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, d_obb2, 1e-9);
  obbrss_tf.translation() = fcl::Vector3<double>(3.0, 0.0, 0.0);
  d_obb = costmap_3d::distanceOctomapOBB(aabb.radius(), aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, 2.0 - std::sqrt(3.0), 1e-6);
  d_obb2 = sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, d_obb2, 1e-9);
  obbrss_tf.translation() = fcl::Vector3<double>(0.0, -1.75, 0.25);
  d_obb = costmap_3d::distanceOctomapOBB(aabb.radius(), aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, -.25 - std::sqrt(3.0), 1e-6);
  d_obb2 = sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, d_obb2, 1e-9);
  obbrss_tf.translation() = fcl::Vector3<double>(0.0, -1.75, 2.95);
  d_obb = costmap_3d::distanceOctomapOBB(aabb.radius(), aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, -.05 - std::sqrt(3.0), 1e-6);
  d_obb2 = sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, d_obb2, 1e-9);
  obbrss_tf.translation() = fcl::Vector3<double>(3.0, -1.75, 0.25);
  d_obb = costmap_3d::distanceOctomapOBB(aabb.radius(), aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, 2.0 - std::sqrt(3.0), 1e-6);
  d_obb2 = sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, d_obb2, 1e-9);
  obbrss_tf.translation() = fcl::Vector3<double>(3.0, -2.50, 0.25);
  d_obb = costmap_3d::distanceOctomapOBB(aabb.radius(), aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, std::sqrt(2.0*2.0 + 0.5*0.5) - std::sqrt(3.0), 1e-6);
  d_obb2 = sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, d_obb2, 1e-9);
  obbrss_tf.translation() = fcl::Vector3<double>(3.0, -2.50, 3.25);
  d_obb = costmap_3d::distanceOctomapOBB(aabb.radius(), aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, std::sqrt(2.0*2.0 + 0.5*0.5 + 0.25*0.25) - std::sqrt(3.0), 1e-6);
  d_obb2 = sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
  EXPECT_NEAR(d_obb, d_obb2, 1e-9);
  constexpr size_t n = 100000;
  std::chrono::high_resolution_clock::time_point start_time;
  fcl::aligned_vector<fcl::Transform3<double>> transforms;
  double extents[6] = {-10.0, -10.0, -10.0, 10.0, 10.0, 10.0};
  generateRandomTransforms(extents, transforms, n);
  for (unsigned i=0; i<n; ++i)
  {
    const fcl::Transform3<double>& obbrss_tf = transforms[i];
    const fcl::OBB<double>& obb = obbrss.obb;
    fcl::Transform3<double> obb_internal_tf;
    obb_internal_tf.linear() = obb.axis;
    obb_internal_tf.translation() = obb.To;
    const fcl::Transform3<double> inverse_tf = (obbrss_tf * obb_internal_tf).inverse();
    d_obb = costmap_3d::distanceOctomapOBB(radius, aabb_center, obbrss.obb, inverse_tf);
    d_obb2 = sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
    EXPECT_NEAR(d_obb, d_obb2, 1e-9);
  }
  double total_distance = 0;
  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned i=0;i<n;++i)
  {
    const fcl::Transform3<double>& obbrss_tf = transforms[i];
    total_distance += costmap_3d::distanceOctomapOBB(radius, aabb_center, obbrss.obb, obbrss_tf);
  }
  std::cout << "Branch-free implementation: " <<
    std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count() <<
    " total distance: " << total_distance << std::endl;
  total_distance = 0;
  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned i=0;i<n;++i)
  {
    const fcl::Transform3<double>& obbrss_tf = transforms[i];
    total_distance += sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
  }
  std::cout << "FCL-based implementation: " <<
    std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count() <<
    " total distance: " << total_distance << std::endl;
  // Time Eigen's implementation as a reference as well.
  total_distance = 0;
  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned i=0;i<n;++i)
  {
    const fcl::Transform3<double>& obbrss_tf = transforms[i];
    total_distance += sphereOBBSignedDistanceEigen(radius, aabb_center, obbrss.obb, obbrss_tf);
  }
  std::cout << "Eigen-based implementation: " <<
    std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count() <<
    " total distance: " << total_distance << std::endl;
}

void octree_solver_test(std::size_t n, bool negative_x_roi = false, bool non_negative_x_roi = false, bool skip_check = false);

TEST(test_octree_solver, test_against_fcl)
{
  octree_solver_test(15, false, false);
  octree_solver_test(15, true, false);
  octree_solver_test(15, false, true);
  std::chrono::high_resolution_clock::time_point start_time;
}

template <typename S>
S rand_interval(S rmin, S rmax)
{
  S t = rand() / ((S)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

template <typename S>
void eulerToMatrix(S a, S b, S c, fcl::Matrix3<S>& R)
{
  auto c1 = std::cos(a);
  auto c2 = std::cos(b);
  auto c3 = std::cos(c);
  auto s1 = std::sin(a);
  auto s2 = std::sin(b);
  auto s3 = std::sin(c);

  R << c1 * c2, - c2 * s1, s2,
      c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3, - c2 * s3,
      s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3;
}

template <typename S>
void generateRandomTransforms(S extents[6], fcl::aligned_vector<fcl::Transform3<S>>& transforms, std::size_t n)
{
  transforms.resize(n);
  for(std::size_t i = 0; i < n; ++i)
  {
    auto x = rand_interval(extents[0], extents[3]);
    auto y = rand_interval(extents[1], extents[4]);
    auto z = rand_interval(extents[2], extents[5]);

    const auto pi = fcl::constants<S>::pi();
    auto a = rand_interval((S)0, 2 * pi);
    auto b = rand_interval((S)0, 2 * pi);
    auto c = rand_interval((S)0, 2 * pi);

    {
      fcl::Matrix3<S> R;
      eulerToMatrix(a, b, c, R);
      fcl::Vector3<S> T(x, y, z);
      transforms[i].setIdentity();
      transforms[i].linear() = R;
      transforms[i].translation() = T;
    }
  }
}

template <typename S>
void generateBoxesFromOctomap(
    const fcl::OcTree<S>& tree,
    std::vector<std::shared_ptr<fcl::CollisionObject<S>>>* boxes,
    bool positive_x_only = false,
    bool negative_x_only = false)
{
  std::vector<std::array<S, 6>> tree_boxes = tree.toBoxes();

  for(std::size_t i = 0; i < tree_boxes.size(); ++i)
  {
    S x = tree_boxes[i][0];
    S y = tree_boxes[i][1];
    S z = tree_boxes[i][2];
    S size = tree_boxes[i][3];
    S cost = tree_boxes[i][4];
    S threshold = tree_boxes[i][5];

    if (positive_x_only && x + size / 2.0 < 0.0)
      continue;
    if (negative_x_only && x - size / 2.0 > 0.0)
      continue;
    std::shared_ptr<fcl::CollisionGeometry<S>> box(new fcl::Box<S>(size, size, size));
    box->cost_density = cost;
    box->threshold_occupied = threshold;
    std::shared_ptr<fcl::CollisionObject<S>> obj(
        new fcl::CollisionObject<S>(
            box,
            fcl::Transform3<S>(fcl::Translation3<S>(fcl::Vector3<S>(x, y, z)))));
    boxes->push_back(obj);
  }
}

template <typename S>
struct DistanceData
{
  fcl::DistanceRequest<S> request;
  fcl::DistanceResult<S> result;
  bool done = false;
};

template <typename S>
bool defaultDistanceFunction(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, void* cdata_, S& dist)
{
  auto* cdata = static_cast<DistanceData<S>*>(cdata_);
  const fcl::DistanceRequest<S>& request = cdata->request;
  fcl::DistanceResult<S>& result = cdata->result;

  if(cdata->done) { dist = result.min_distance; return true; }

  fcl::distance(o1, o2, request, result);

  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}

void octree_solver_test(std::size_t n, bool negative_x_roi, bool non_negative_x_roi, bool skip_check)
{
  using S = costmap_3d::Costmap3DQuery::FCLFloat;
  costmap_3d::Costmap3DPtr octree(new costmap_3d::Costmap3D(
          costmap_3d::Costmap3DQuery::getFileNameFromPackageURL(PACKAGE_URL + "test/aisles.bt")));
  // Ensure occupancy threshold is setup properly.
  octree->setOccupancyThres(0.5);
  std::shared_ptr<fcl::OcTree<S>> tree_ptr(new fcl::OcTree<S>(octree));

  // Use Costmap3DQuery to get BVH for test mesh
  costmap_3d::Costmap3DQuery query(octree, PACKAGE_URL + "test/test_robot.stl");
  costmap_3d::Costmap3DQuery::FCLRobotModelConstPtr m1 = query.getFCLRobotModel();
  std::shared_ptr<const fcl::CollisionGeometry<S>> m1_ptr(m1);

  std::vector<fcl::Halfspace<S>> roi;
  if (negative_x_roi)
  {
    fcl::Vector3<S> normal(1.0, 0.0, 0.0);
    fcl::Halfspace<S> negative_x(normal, 0);
    roi.push_back(negative_x);
  }
  if (non_negative_x_roi)
  {
    fcl::Vector3<S> normal(-1.0, 0.0, 0.0);
    fcl::Halfspace<S> non_negative_x(normal, 0);
    roi.push_back(non_negative_x);
  }

  fcl::aligned_vector<fcl::Transform3<S>> transforms;
  S extents[] = {-10, -10, -2, 10, 10, 2};

  // Ensure transforms are the same even if other tests use rand()
  srand(1);
  generateRandomTransforms(extents, transforms, n);
  // Be sure to test identity
  transforms[0] = fcl::Transform3<S>::Identity();

  std::chrono::high_resolution_clock::duration total_time(0);
  std::chrono::high_resolution_clock::time_point start_time;
  for(std::size_t i = 0; i < n; ++i)
  {
    fcl::Transform3<S> tf1(transforms[0]);
    fcl::Transform3<S> tf2(transforms[i]);
    fcl::detail::GJKSolver_libccd<S> solver;
    costmap_3d::OcTreeMeshSolver<fcl::detail::GJKSolver_libccd<S>> octree_solver(&solver);
    costmap_3d::OcTreeMeshSolver<fcl::detail::GJKSolver_libccd<S>>::DistanceRequest request;
    costmap_3d::OcTreeMeshSolver<fcl::detail::GJKSolver_libccd<S>>::DistanceResult result;
    request.rel_err = 0.0;
    request.enable_signed_distance = true;
    request.roi_ptr = roi.data();
    request.roi_size = roi.size();
    start_time = std::chrono::high_resolution_clock::now();
    octree_solver.distance(
        octree.get(),
        m1.get(),
        tf1,
        tf2,
        request,
        &result);
    S dist1 = result.min_distance;
    std::cout << " octree iteration " << i << ": " << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start_time).count() << "ns" << std::endl;
    total_time += std::chrono::high_resolution_clock::now() - start_time;

    // Check the result against FCL's broadphase distance
    std::vector<std::shared_ptr<fcl::CollisionObject<S>>> boxes;
    generateBoxesFromOctomap<S>(*tree_ptr, &boxes, non_negative_x_roi, negative_x_roi);
    for(std::size_t j = 0; j < boxes.size(); ++j)
      boxes[j]->setTransform(tf1 * boxes[j]->getTransform());

    fcl::DynamicAABBTreeCollisionManager<S> manager;
    for (auto box : boxes)
    {
      manager.registerObject(box.get());
    }
    manager.setup();

    DistanceData<S> cdata2;
    fcl::CollisionObject<S> obj1(std::const_pointer_cast<fcl::CollisionGeometry<S>>(m1_ptr), tf2);
    cdata2.request.abs_err = 0.0;
    cdata2.request.rel_err = 0.0;
    cdata2.request.enable_nearest_points = true;
    cdata2.request.enable_signed_distance = false;
    cdata2.result.min_distance = std::numeric_limits<S>::max();
    manager.distance(&obj1, &cdata2, fcl::DefaultDistanceFunction);
    S dist2 = cdata2.result.min_distance;

    if (dist1 > 1e-6 && dist2 > 1e-6)
    {
      EXPECT_NEAR(dist1, dist2, 1e-6);
    }
    else
    {
      // Signed distance is allowed to not be equivalent between FCL broadphase
      // and the costmap 3D octree solver
      EXPECT_TRUE(dist1 < 1e-6 && dist2 < 1e-6);
    }
  }
  std::cout << "Average time per octree solve: " << std::chrono::duration_cast<std::chrono::nanoseconds>(total_time).count() / n << "ns" << std::endl;
}

// Test the custom ArgSortUpTo8 function used by the octree_solver to very
// quickly sort up to 8 distances, returning the sorted indices.
TEST(test_octree_solver, test_arg_sort_up_to_8)
{
  constexpr unsigned incremental[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  std::chrono::high_resolution_clock::time_point start_time;
  for (unsigned n=0; n <= 8; ++n)
  {
    std::chrono::high_resolution_clock::duration arg_sort_time(0), std_sort_time(0);
    unsigned indices[8];
    unsigned iterations = 0;
    // To get accurate timing information for small n, always do at least 1000
    // iterations.
    while (iterations < 1000)
    {
      // Test every permutation, which is doable for up to 8 (8! is 40320).
      // Note that this is really overkill to prove the sorting networks are
      // correct, as the check below with the powers of 2 is all that is
      // necessary to prove the correctness of a sorting network. But
      // exercising every permutation is a good overall performance check
      // and is doable for the size lists that are being sorted.
      double permutation[8] = {0, 1, 2, 3, 4, 5, 6, 7};
      do
      {
        start_time = std::chrono::high_resolution_clock::now();
        costmap_3d::ArgSortUpTo8(n, indices, permutation);
        arg_sort_time += std::chrono::high_resolution_clock::now() - start_time;
        for (unsigned i=0; i < n; ++i)
        {
          EXPECT_EQ(i, permutation[indices[i]]);
        }
        std::copy(incremental, incremental + n, indices);
        start_time = std::chrono::high_resolution_clock::now();
        std::sort(indices, indices + n, [permutation](int a, int b){return permutation[a] < permutation[b];});
        std_sort_time += std::chrono::high_resolution_clock::now() - start_time;
        for (unsigned i=0; i < n; ++i)
        {
          EXPECT_EQ(i, permutation[indices[i]]);
        }
        iterations++;
      }
      while (std::next_permutation(permutation, permutation + n));
      if (n > 1)
      {
        // Now test sorting 0/1 sequences of every combination to ensure
        // sorting works properly with equal entries. Use a bitset to count the
        // number of set bits in a fairly standard way (popcount isn't added
        // until C++20).
        std::bitset<8> bits;
        for (unsigned p=0; p < (1<<n); ++p)
        {
          bits.reset();
          for (unsigned bit=0; bit<n; ++bit)
          {
            if (((1<<bit) & p) == 0)
            {
              permutation[bit] = 0;
            }
            else
            {
              permutation[bit] = 1;
              bits.set(bit);
            }
          }
          size_t set_count = bits.count();
          start_time = std::chrono::high_resolution_clock::now();
          costmap_3d::ArgSortUpTo8(n, indices, permutation);
          arg_sort_time += std::chrono::high_resolution_clock::now() - start_time;
          for (unsigned i = 0; i < n - set_count; ++i)
          {
            EXPECT_EQ(0, permutation[indices[i]]);
          }
          for (unsigned i = n - set_count; i < n; ++i)
          {
            EXPECT_EQ(1, permutation[indices[i]]);
          }
          std::copy(incremental, incremental + n, indices);
          start_time = std::chrono::high_resolution_clock::now();
          std::sort(indices, indices + n, [permutation](int a, int b){return permutation[a] < permutation[b];});
          std_sort_time += std::chrono::high_resolution_clock::now() - start_time;
          for (unsigned i = 0; i < n - set_count; ++i)
          {
            EXPECT_EQ(0, permutation[indices[i]]);
          }
          for (unsigned i = n - set_count; i < n; ++i)
          {
            EXPECT_EQ(1, permutation[indices[i]]);
          }
          iterations++;
        }
      }
    }
    std::cout << "Average time to std::sort " << n << ": " <<
        std::chrono::duration_cast<std::chrono::nanoseconds>(std_sort_time).count() / iterations <<
        "ns (iterations: " << iterations << ")" << std::endl;
    std::cout << "Average time to ArgSortUpTo8 " << n << ": " <<
        std::chrono::duration_cast<std::chrono::nanoseconds>(arg_sort_time).count() / iterations <<
        "ns (iterations: " << iterations << ")" << std::endl;
  }
  constexpr size_t n = 100000;
  std::array<std::array<double, 8>, n> darrs;
  std::array<unsigned, n> ns;
  std::mt19937 gen(1);
  std::uniform_int_distribution<> n_distr(2, 8);
  std::uniform_real_distribution<> d_distr(-1e6, 1e6);

  for (unsigned a=0; a<n; ++a)
  {
    ns[a] = n_distr(gen);
    for (unsigned i=0; i<ns[a]; ++i)
    {
      darrs[a][i] = d_distr(gen);
    }
  }
  // Check to make sure that ArgSortUpTo8 actually sorts these random numbers
  // within the acceptable tolerance (as the internal packing step does
  // introduce some absolute error at the nano-meter scale)
  for (unsigned a=0; a<n; ++a)
  {
    unsigned indices[8];
    costmap_3d::ArgSortUpTo8(ns[a], indices, darrs[a].data());
    for (unsigned i=0; i<ns[a]-1; ++i)
    {
      EXPECT_LT(darrs[a][indices[i]], darrs[a][indices[i+1]] + 1e-7);
    }
  }
  // Now time the same sorts, and compare to std::sort on the same data.
  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned a=0; a<n; ++a)
  {
    unsigned indices[8];
    costmap_3d::ArgSortUpTo8(ns[a], indices, darrs[a].data());
  }
  std::chrono::high_resolution_clock::duration arg_sort_time;
  arg_sort_time = std::chrono::high_resolution_clock::now() - start_time;
  std::cout << "Total time to ArgSortUpTo8 " << n << " random arrays of size 2-8: " <<
      std::chrono::duration_cast<std::chrono::nanoseconds>(arg_sort_time).count() <<
      "ns" << std::endl;
  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned a=0; a<n; ++a)
  {
    unsigned indices[8];
    const double* distances = darrs[a].data();
    std::sort(indices, indices + ns[a], [distances](int a, int b){return distances[a] < distances[b];});
  }
  std::chrono::high_resolution_clock::duration std_sort_time;
  std_sort_time = std::chrono::high_resolution_clock::now() - start_time;
  std::cout << "Total time to std::sort the same " << n << " random arrays of size 2-8: " <<
      std::chrono::duration_cast<std::chrono::nanoseconds>(std_sort_time).count() <<
      "ns" << std::endl;
}


// Bin a pose.
inline geometry_msgs::Pose binPose(const geometry_msgs::Pose& pose,
                                   int bins_per_meter,
                                   int bins_per_rotation)
{
  geometry_msgs::Pose rv;
  // std::round is slow on AVX2, but floor is fast, and where the
  // quantization happens is not important
  rv.position.x = std::floor(pose.position.x * bins_per_meter) / bins_per_meter;
  rv.position.y = std::floor(pose.position.y * bins_per_meter) / bins_per_meter;
  rv.position.z = std::floor(pose.position.z * bins_per_meter) / bins_per_meter;

  // Speed up the orientation binning by rounding the Cayley transform of
  // the quaternion versor instead of binning by Euler angles. It is more
  // expensive to get the Euler angles as it requires several atan2
  // operations. Getting the Cayley transform (and its inverse) requires
  // negating the versor if the scalar is negative, then a few simple
  // division/multiplication operations. The rounding error is fairly
  // uniform across SO(3). For more information, see:
  //
  // https://marc-b-reynolds.github.io/quaternions/2017/05/02/QuatQuantPart1.html
  //
  // The old technique exactly matches what the above calls 'ZYX', and what
  // is implemented below is the 'Basic Cayley'. If we ever find that the
  // Cayley transform is introducing too much angular error, another
  // reasonable compromise between runtime and accuracy is what the above
  // calls the 'Basic Harmonic Mean' method.
  double w = pose.orientation.w;
  Eigen::Vector3d v(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
  if (w < 0.0)
  {
    w = -w;
    v = -v;
  }
  if (w == 0.0)
  {
    // When w is zero, the log transform will not work. Also, there are still
    // two rotations that result in the same final state (the given axis and
    // its negative). To be sure the same bin is chosen, flip the axis if it
    // has a negative x component, or zero x and negative y, or zero x, zero y
    // and negative z.
    if (v(0) < 0 || (v(0) == 0 && (v(1) < 0 || v(1) == 0 && v(2) < 0)))
    {
      v = -v;
    }
  }
  double s = 1.0 / (1.0 + w);
  // To get roughly to radians scale the bins by 4, as the units of the
  // transform are almost exactly 1/4 of a radian near the origin
  v *= s * (bins_per_rotation);
  v = v.array().floor() / (bins_per_rotation);
  double s_inv = 2.0 / (1.0 + v.dot(v));
  v *= s_inv;
  rv.orientation.w = s_inv - 1.0;
  rv.orientation.x = v[0];
  rv.orientation.y = v[1];
  rv.orientation.z = v[2];
  return rv;
}

inline geometry_msgs::Pose binPose2(const geometry_msgs::Pose& pose,
                                   int bins_per_meter,
                                   int bins_per_rotation)
{
  geometry_msgs::Pose rv;
  // std::round is slow on AVX2, but floor is fast, and where the
  // quantization happens is not important
  rv.position.x = std::floor(pose.position.x * bins_per_meter) / bins_per_meter;
  rv.position.y = std::floor(pose.position.y * bins_per_meter) / bins_per_meter;
  rv.position.z = std::floor(pose.position.z * bins_per_meter) / bins_per_meter;

  if (bins_per_rotation <= 1)
  {
    // Handle case where bins_per_rotation is 1 (or nonsense). Set no rotation
    // no matter what the input orientation is.
    rv.orientation.w = 1;
    rv.orientation.x = 0;
    rv.orientation.y = 0;
    rv.orientation.z = 0;
  }

  // Fairly binning the orientation is a bit trickier. Just binning the raw
  // component values will create very uneven bins angularly. Decompose the
  // quaternion rotation into the rotation angle (alpha) and the inclination
  // angles of the rotational axis to the coordinate axes. The scale the angles
  // into fractions of a full rotation and bin those rotation values. This
  // yields bins of equiangular space.
  double w = pose.orientation.w;
  Eigen::Vector3d v(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
  // Quaternion representation or rotations have two values for the same
  // rotation. To ensure the same rotation is represented by the binned pose in
  // each of the two representations, use the non-negative one.
  if (w < 0.0)
  {
    w = -w;
    v = -v;
  }
  else if (w == 0.0)
  {
    // When w is zero there are still two possible representations of the same
    // rotation. They should be binned to the same value. 
    // To be sure the same bin is chosen, flip the axis if it
    // has a negative x component, or zero x and negative y, or zero x, zero y
    // and negative z.
    if (v(0) < 0 || (v(0) == 0 && (v(1) < 0 || v(1) == 0 && v(2) < 0)))
    {
      v = -v;
    }
  }

  double alpha = 2 * acos(w);
  const double half_sine = sin(alpha / 2);
  if (half_sine != 0)
  {
    v /= half_sine;
    // Division may quantize just a bit such that the value is beyond the
    // domain of acos. Clamp the values to the domain of acos so it does not
    // return nan.
    v(0) = std::min(std::max(v(0), -1.0), 1.0);
    v(1) = std::min(std::max(v(1), -1.0), 1.0);
    v(2) = std::min(std::max(v(2), -1.0), 1.0);
  }
  else
  {
    // If the half_sine is zero, w was nearly one and therefore the vector
    // components are all nearly zero. Do not divide by the sin of alpha, as
    // that would be division by zero. Instead ensure the correct values in
    // such cases.
    w = 1.0;
    v = Eigen::Vector3d(0, 0, 0);
  }
  // Convert v to the angles of the rotation axis to the coordinate axes.
  v = v.array().acos();
  // Scale radians to rotations.
  w = alpha / (2 * M_PI);
  v /= 2 * M_PI;
  // Bin rotations.
  w = std::floor(w * bins_per_rotation + .5) / bins_per_rotation;
  // If the angle is zero after binning, ensure the axis is always the same.
  if (w == 0)
  {
    // Use the x-axis as the rotation axis for zero rotation so all zero
    // rotation bins are the same.
    v = Eigen::Vector3d(0.0, .25, .25);
  }
  v = v * bins_per_rotation + Eigen::Vector3d(.5, .5, .5);
  v = v.array().floor() / bins_per_rotation;
  // If the rotation angle after binning is a half rotation, and the rotation
  // axis is near one of the negative coordinate axes the rotational axis angle
  // may also have been binned to a half rotation. A rotation axis angle of 0.5
  // will put the rotational axis on the corresponding negative coordinate
  // axis. A half rotation is the same either way whether the axis is negative
  // or positive. In such cases always choose the positive coordinate axis (no
  // axis rotation for the positive rotation axis) instead of a half rotation
  // so both possible representations result in the same bin. To handle when
  // bins_per_rotation can not represent a quarter rotation exactly, subtract
  // the other axes from a half rotation to get the correct opposite axis.
  double half_rotation = std::floor(0.5 * bins_per_rotation + .5) / bins_per_rotation;
  if (w == half_rotation)
  {
    if (v(0) >= half_rotation)
    {
      v(0) = 0.0;
      v(1) = half_rotation - v(1);
      v(2) = half_rotation - v(2);
    }
    if (v(1) >= 0.5)
    {
      v(0) = half_rotation - v(0);
      v(1) = 0.0;
      v(2) = half_rotation - v(2);
    }
    if (v(2) >= 0.5)
    {
      v(0) = half_rotation - v(0);
      v(1) = half_rotation - v(1);
      v(2) = 0.0;
    }
  }
  // Binned poses are never used directly for anything but comparision to other
  // binned poses. Simply leave the binned orientation in binned rotations.
  rv.orientation.w = w;
  rv.orientation.x = v(0);
  rv.orientation.y = v(1);
  rv.orientation.z = v(2);
  return rv;
}

template <bool cube_stretch=false>
inline geometry_msgs::Pose binPoseLog(const geometry_msgs::Pose& pose,
                                   int bins_per_meter,
                                   int bins_per_rotation)
{
  geometry_msgs::Pose rv;
  // std::round is slow on AVX2, but floor is fast, and where the
  // quantization happens is not important
  rv.position.x = std::floor(pose.position.x * bins_per_meter) / bins_per_meter;
  rv.position.y = std::floor(pose.position.y * bins_per_meter) / bins_per_meter;
  rv.position.z = std::floor(pose.position.z * bins_per_meter) / bins_per_meter;

  double w = pose.orientation.w;
  Eigen::Vector3d v(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
  if (w < 0.0)
  {
    w = -w;
    v = -v;
  }
  double s;
  // Must handle singularities of the log transform to avoid division by zero
  if (w != 0)
  {
    double s;
    // Because sin x = +/- sqrt(1 = cos^2 x), a is sin x
    double a = sqrt(1.0 - w * w);
    // When a is zero, w was nearly one, rotation is nearly zero, so make scale
    // 0 (v should be nearly zero anyway).
    if (a == 0.0)
    {
      s = 0.0;
    }
    else
    {
      // Scale the resulting log sphere up to be a unit sphere (where the surface
      // represents a rotation of 180 degrees about the axis).
      s = (2.0 / M_PI) * atan(a / w) / a;
    }
    v *= s;
  }
  else
  {
    // When w is zero, the log transform will not work. Also, there are still
    // two rotations that result in the same final state (the given axis and
    // its negative). To be sure the same bin is chosen, flip the axis if it
    // has a negative x component, or zero x and negative y, or zero x, zero y
    // and negative z.
    if (v(0) < 0 || (v(0) == 0 && (v(1) < 0 || v(1) == 0 && v(2) < 0)))
    {
      v = -v;
    }
  }
  // Before binning, stretch the ball of the log transform into a cube. This
  // keeps the bins all roughly the same size and prevents there from being any
  // tiny bins on a boundary.
  double stretch_factor;
  double pre_bin_length_v = v.norm();
  double abs_max_coord = v.cwiseAbs().maxCoeff();
  if (abs_max_coord != 0)
  {
    stretch_factor = pre_bin_length_v / abs_max_coord;
  }
  else
  {
    stretch_factor = 1.0;
  }
  if (cube_stretch) v *= stretch_factor;

  // To get bins size to rotations, scale down by 2 (since values of one
  // represent 180 degrees or a half rotation). Scale up by stretch factor to
  // stretch the ball into a cube.
  int bin_size = (bins_per_rotation / 2);
  v *= bin_size;
//  v += Eigen::Vector3d(.5, .5, .5);
  v = v.array().floor() / bin_size;
  // De-stretch the bin. The stretch factor must be re-calculated from the
  // binned position.
  abs_max_coord = v.cwiseAbs().maxCoeff();
  double post_bin_length_v = v.norm();
  double destretch_factor;
  if (abs_max_coord != 0)
  {
    destretch_factor = abs_max_coord / post_bin_length_v;
  }
  else
  {
    destretch_factor = 1.0;
  }
  if (cube_stretch) v *= destretch_factor;

  double length_v = v.norm();
  // If w was zero, we wean to leave v alone and use the value set above.
  if (w != 0)
  {
    if (length_v != 0)
    {
      double half_angle = (M_PI / 2.0) * length_v;
      // ? doesn't seem to be true
      // The floor can cause the length of v to become at or over 1.0, which
      // will cause the sin below to wrap. Switch the half_angle to the
      // supplementary angle.
      if (length_v >= 1.0)
      {
        half_angle = M_PI - half_angle;
      }
      w = cos(half_angle);
      v *= sin(half_angle) / length_v;
    }
    else
    {
      // The axis shrunk to nothing after binning, be sure to set w
      // appropriately to 1.0 to indicate no rotation.
      w = 1.0;
    }
  }

  // The math can leave some numbers very nearly zero. To get them to bin
  // properly (due to the 2:1 mapping and needing to test for zero/negative).
  constexpr double tolerance = 1e-9;
  w = std::abs(w) > tolerance ? w : 0.0;
  for (unsigned int i=0; i<3; ++i)
  {
    v(i) = std::abs(v(i)) > tolerance ? v(i) : 0.0;
  }

  if (w < 0.0 || w == 0.0 && (
        v(0) < 0.0 || v(0) == 0.0 && (
          v(1) < 0.0 || v(1) == 0.0 && (
            v(2) < 0.0))))
  {
    w = -w;
    v = -v;
  }

  // Remove any negative zeros. This is to allow for ultra-fast hashing of the
  // binned pose by using the raw bits of the floating point number.
  if (w == -0.0)
  {
    w = std::copysign(w, 0.0);
  }
  for (unsigned int i=0; i<3; ++i)
  {
    if (v(i) == -0.0)
    {
      v(i) = std::copysign(v(i), 0.0);
    }
  }
  EXPECT_TRUE(std::isfinite(w));
  EXPECT_TRUE(std::isfinite(v(0)));
  EXPECT_TRUE(std::isfinite(v(1)));
  EXPECT_TRUE(std::isfinite(v(2)));
  assert(std::isfinite(w));
  assert(std::isfinite(v(0)));
  assert(std::isfinite(v(1)));
  assert(std::isfinite(v(2)));

  rv.orientation.w = w;
  rv.orientation.x = v[0];
  rv.orientation.y = v[1];
  rv.orientation.z = v[2];
  return rv;
}

inline geometry_msgs::Pose binPose3(const geometry_msgs::Pose& pose,
                                   int bins_per_meter,
                                   int bins_per_rotation)
{
  geometry_msgs::Pose rv;
  // std::round is slow on AVX2, but floor is fast, and where the
  // quantization happens is not important
  rv.position.x = std::floor(pose.position.x * bins_per_meter) / bins_per_meter;
  rv.position.y = std::floor(pose.position.y * bins_per_meter) / bins_per_meter;
  rv.position.z = std::floor(pose.position.z * bins_per_meter) / bins_per_meter;

  // Decompose uniformly from SO(3) to the volume of a cube by solving this
  // uniform distribution in reverse:
  //
  // w = std::sqrt(1.0 - u0) * sin(2*M_PI*u1)
  // x = std::sqrt(u0) * cos(2*M_PI*u2)
  // y = std::sqrt(u0) * sin(2*M_PI*u2)
  // z = std::sqrt(1.0 - u0) * cos(2*M_PI*u1)
  //
  // Note w/x/y/z can be in any order and the distribution is still evenly
  // spaced in SO(3), so the choice of which way to assign w/x/y/z is arbitrary
  // but needs to remain consistent. The order above is choosen so rotations
  // around the z axis are binned evenly on rotation angle (but x/y axes are
  // not). This is a good choice for navigtion, where 2D navigation will always
  // use rotations only about the z-axis, and 3D navigation may use any
  // orientation, but would often have rotation axes near the z-axis as well.
  // Therefore:
  //
  // w / z = tan(2*M_PI*u1)
  // y / x = tan(2*M_PI*u2)
  //
  // Once u1/u2 are known, solve for u0 by using the coeffcient with the
  // maximum absolute value.
  Eigen::Array4d q(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
  // Go ahead and map any rotation in the "lower" hyper-hemisphere to its
  // equivalent antipode rotation. This prevents us from having to map antipode
  // rotations to the same bins later on, which would be complicated due to
  // rounding leaving the numbers of matching rotations slightly different.
  if (q(0) < 0.0 || q(0) == 0.0 && (
        q(1) < 0.0 || q(1) == 0.0 && (
          q(2) < 0.0 || q(2) == 0.0 && (
            q(3) < 0.0))))
  {
    q = -q;
  }

  // Use atan2 to avoid singularities
  double angle1 = std::atan2(q(0), q(3));
  double angle2 = std::atan2(q(2), q(1));
  Eigen::Array3d u;
  u(1) = angle1 / (2.0 * M_PI);
  u(2) = angle2 / (2.0 * M_PI);
  int index;
  // When calculating u0, use quaternion coordinates with the largest absolute
  // value.  This helps avoid numerical issues around zero. If the coordinate
  // value is far from zero, then both parts of the multiplication to calculate
  // that value must not be near zero. This gives the maximum precision to the
  // value of u0.
  q.abs().maxCoeff(&index);
  switch (index)
  {
    default:
    case 0:
    {
      // w = std::sqrt(1.0 - u0) * sin(2*M_PI*u1)
      u(0) = q(0) / std::sin(angle1);
      u(0) = 1.0 - u(0) * u(0);
      break;
    }
    case 1:
    {
      // x = std::sqrt(u0) * cos(2*M_PI*u2)
      u(0) = q(1) / std::cos(angle2);
      u(0) = u(0) * u(0);
      break;
    }
    case 2:
    {
      // y = std::sqrt(u0) * sin(2*M_PI*u2)
      u(0) = q(2) / std::sin(angle2);
      u(0) = u(0) * u(0);
      break;
    }
    case 3:
    {
      // z = std::sqrt(1.0 - u0) * cos(2*M_PI*u1)
      u(0) = q(3) / std::cos(angle1);
      u(0) = 1.0 - u(0) * u(0);
      break;
    }
  }
  // Its possible the trig above gets us slightly out of range on u(0), as
  // sin(angle1) may be slightly more than the original angle, meaning when
  // we divide by the coordinate (esp. near 1) we get a value just slightly
  // more than 1, when mathematically it should be 1 or slightly less than 1.
  // Fix this by clamping to the correct range.
  u(0) = std::min(std::max(u(0), 0.0), 1.0);
  assert(u(1) >= -0.5);
  assert(u(1) <= 0.5);
  assert(u(2) >= -0.5);
  assert(u(2) <= 0.5);

  // Need to double the number of buckets to get the correct size in rotations.
  // This is because a value of one of the u values of .5 represents an entire
  // rotation.
  const double bucket_factor = bins_per_rotation * 2;
  u *= bucket_factor;
  // Multiply by a value just smaller than 1.0 to prevent from very narrow
  // bucket when at end points. IOW, avoid getting `bucket_factor` as an input
  // to the floor() operation below, as that would result in a very narrow
  // bucket. This also lets us simply add .5 to the output of floor to avoid
  // the endpoints of the range completely (avoiding multiplying by zero
  // later).
  constexpr double avoid_ends_factor = 1.0 - 4.0 * std::numeric_limits<double>::epsilon();
  u *= avoid_ends_factor;
  // Avoid zero values for the binned u values, as that would create huge
  // buckets at zero (since multiplying by zero destroys the significance of
  // the other variable).
  u = (u.array().floor() + Eigen::Array3d(0.5, 0.5, 0.5)) / bucket_factor;
  assert(u(0) > 0.0);
  assert(u(0) < 1.0);
  assert(u(1) > -0.5);
  assert(u(1) < 0.5);
  assert(u(2) > -0.5);
  assert(u(2) < 0.5);
  double angle_u1 = 2 * M_PI * u(1);
  double angle_u2 = 2 * M_PI * u(2);
  double sin_angle_u1 = std::sin(angle_u1);
  double cos_angle_u1 = std::cos(angle_u1);
  double sin_angle_u2 = std::sin(angle_u2);
  double cos_angle_u2 = std::cos(angle_u2);
  q(0) = std::sqrt(1.0 - u(0)) * sin_angle_u1;
  q(1) = std::sqrt(u(0)) * cos_angle_u2;
  q(2) = std::sqrt(u(0)) * sin_angle_u2;
  q(3) = std::sqrt(1.0 - u(0)) * cos_angle_u1;

  assert(std::isfinite(q(0)));
  assert(std::isfinite(q(1)));
  assert(std::isfinite(q(2)));
  assert(std::isfinite(q(3)));
  assert(q(0) > 0.0);
  assert(q(1) != 0.0);
  assert(q(2) != 0.0);
  assert(q(3) != 0.0);

  rv.orientation.w = q(0);
  rv.orientation.x = q(1);
  rv.orientation.y = q(2);
  rv.orientation.z = q(3);

  return rv;
}

// Returns maximum angular distance for two rotations that hit the same bin
// (from furthest corner to furthest corner). If the desired error is to the
// center of the bin (the binned pose value), simply call with double the
// bins_per_rotation value (to get the max bin corner to the center bin
// position).
//
// Note: if running in 2D mode, the poses bin completely fairly when the
// rotation axis is the z-axis. In such cases, the true maximum angular error
// is just 2 * M_PI / bins_per_rotation.
inline double binPose3AngularDistanceLimit(int bins_per_rotation)
{
  // The largest bins are those at u0 = 1 or u0 = 0. To find the maximum
  // angular bin size (error) simply measure the angular distance along the
  // diagonal of the bin.
  //
  // So compare the corresponding quaternions at u0 = u1 = u2 = 0.0, and the
  // cooresponding corner u0 = u1 = u2 = 1 / (2 * bins_per_rotation)
  //
  // So q0 (at the corner) would be:
  //
  // w = std::sqrt(1.0 - u0) * sin(2*M_PI*u1)
  // x = std::sqrt(u0) * cos(2*M_PI*u2)
  // y = std::sqrt(u0) * sin(2*M_PI*u2)
  // z = std::sqrt(1.0 - u0) * cos(2*M_PI*u1)
  // w = 0
  // x = 0
  // y = 0
  // z = 1
  //
  // Because the angular bin size is found using the dot product of the
  // corners, and the corner being examined has zero cooefficients at w/x/y,
  // only the z coefficient is relevant.
  //
  // So z at the corresponding corner:
  // z_corner = std::sqrt(1 - .5 / bins_per_rotation) * cos(M_PI / bins_per_rotation)
  //
  // The angular distance from the original quaternion to the binned quaternion is therefore:
  // 2 * acos(q dot binned_q)
  // 2 * acos(z_corner)
  // so:
  // 2 * acos(std::sqrt(1 - .5 / bins_per_rotation) * cos(M_PI / bins_per_rotation))
  //
  return 2 * acos(std::sqrt(1.0 - .5 / bins_per_rotation) * cos(M_PI / bins_per_rotation));
}

template <bool tesseract_stretch=false>
inline geometry_msgs::Pose binPoseSimple(const geometry_msgs::Pose& pose,
                                   int bins_per_meter,
                                   int bins_per_rotation)
{
  geometry_msgs::Pose rv;
  // std::round is slow on AVX2, but floor is fast, and where the
  // quantization happens is not important
  rv.position.x = std::floor(pose.position.x * bins_per_meter) / bins_per_meter;
  rv.position.y = std::floor(pose.position.y * bins_per_meter) / bins_per_meter;
  rv.position.z = std::floor(pose.position.z * bins_per_meter) / bins_per_meter;

  Eigen::Array4d q(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
  EXPECT_NEAR(q.matrix().norm(), 1.0, 1e-6);
  if (tesseract_stretch)
  {
    double abs_max_coord = q.abs().maxCoeff();
    double stretch_factor = 1.0 / abs_max_coord;
    q *= stretch_factor;
  }
  int bin_factor = bins_per_rotation / 4;
  q = (q * bin_factor).floor();
//  q = (q * bin_factor + 0.5).floor();
//  q = (q * bin_factor + 0.5).floor() / bin_factor;
//  double destretch_factor = q.abs().maxCoeff();
//  q *= destretch_factor;
  q.matrix().normalize();

  if (q(0) < 0.0 || q(0) == 0.0 && (
        q(1) < 0.0 || q(1) == 0.0 && (
          q(2) < 0.0 || q(2) == 0.0 && (
            q(3) < 0.0))))
  {
    q = -q;
  }

  EXPECT_NEAR(q.matrix().norm(), 1.0, 1e-6);

  rv.orientation.w = q(0);
  rv.orientation.x = q(1);
  rv.orientation.y = q(2);
  rv.orientation.z = q(3);

  return rv;
}

inline geometry_msgs::Pose binPoseAxisAngle(const geometry_msgs::Pose& pose,
                                   int bins_per_meter,
                                   int bins_per_rotation)
{
  geometry_msgs::Pose rv;
  // std::round is slow on AVX2, but floor is fast, and where the
  // quantization happens is not important
  rv.position.x = std::floor(pose.position.x * bins_per_meter) / bins_per_meter;
  rv.position.y = std::floor(pose.position.y * bins_per_meter) / bins_per_meter;
  rv.position.z = std::floor(pose.position.z * bins_per_meter) / bins_per_meter;

  Eigen::Quaterniond q(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
  // rotation represents the amount of rotation represented in the axis/angle
  // representation of the versor (quaternion). Bin rotation directly, and bin
  // the axis as-is by projecting the solid ball that the vector part of the
  // versor (quaternion) into a cubic solid.
  double rotation = atan2(q.vec().norm(), q.w()) / M_PI;
  q.w() = std::cos(2 * M_PI * (std::floor(rotation * bins_per_rotation) / bins_per_rotation));
  int max_index;
  double abs_max_coord = q.vec().cwiseAbs().maxCoeff(&max_index);
  double stretch_factor = 1.0 / abs_max_coord;
  q.vec() *= stretch_factor;
  int bin_factor = bins_per_rotation / 2;
  q.vec() = (q.vec() * bin_factor).array().floor() / bin_factor;
  double destretch_factor = std::abs(q.vec()[max_index]);
  q.vec() *= destretch_factor;
  // XXX Should we do this????
  q.normalize();

  // There are two versors (quaternions) that represent each rotation. To bin
  // properly, map the negative hemisphere to their positive equivalents.
  if (q.w() < 0.0 || q.w() == 0.0 && (
        q.x() < 0.0 || q.x() == 0.0 && (
          q.y() < 0.0 || q.y() == 0.0 && (
            q.z() < 0.0))))
  {
    q.coeffs() *= -1.0;
  }

  EXPECT_NEAR(q.norm(), 1.0, 1e-6);

  rv.orientation.w = q.w();
  rv.orientation.x = q.x();
  rv.orientation.y = q.y();
  rv.orientation.z = q.z();

  return rv;
}


#define EXPECT_POSE_NEAR(p1, p2) \
  EXPECT_NEAR(p1.position.x, p2.position.x, 1e-6); \
  EXPECT_NEAR(p1.position.y, p2.position.y, 1e-6); \
  EXPECT_NEAR(p1.position.z, p2.position.z, 1e-6); \
  EXPECT_NEAR(p1.orientation.w, p2.orientation.w, 1e-6); \
  EXPECT_NEAR(p1.orientation.x, p2.orientation.x, 1e-6); \
  EXPECT_NEAR(p1.orientation.y, p2.orientation.y, 1e-6); \
  EXPECT_NEAR(p1.orientation.z, p2.orientation.z, 1e-6); \

#define EXPECT_POSE_NOT_NEAR(p1, p2) \
  EXPECT_TRUE( \
      std::abs(p1.position.x - p2.position.x) > 1e-6 || \
      std::abs(p1.position.y - p2.position.y) > 1e-6 || \
      std::abs(p1.position.z - p2.position.z) > 1e-6 || \
      std::abs(p1.orientation.w - p2.orientation.w) > 1e-6 || \
      std::abs(p1.orientation.x - p2.orientation.x) > 1e-6 || \
      std::abs(p1.orientation.y - p2.orientation.y) > 1e-6 || \
      std::abs(p1.orientation.z - p2.orientation.z) > 1e-6)

class PoseBinKey
{
public:
  PoseBinKey(const geometry_msgs::Pose& pose)
  {
    binned_pose_ = pose;
    hash_ = hash_value();
  }

  // Directly store the hash value in a public location for speed.
  size_t hash_;

  bool operator==(const PoseBinKey& rhs) const
  {
    return binned_pose_.orientation.x == rhs.binned_pose_.orientation.x &&
           binned_pose_.orientation.y == rhs.binned_pose_.orientation.y &&
           binned_pose_.orientation.z == rhs.binned_pose_.orientation.z &&
           binned_pose_.orientation.w == rhs.binned_pose_.orientation.w &&
           binned_pose_.position.x == rhs.binned_pose_.position.x &&
           binned_pose_.position.y == rhs.binned_pose_.position.y &&
           binned_pose_.position.z == rhs.binned_pose_.position.z;
  }

  const geometry_msgs::Pose& getBinnedPose() const { return binned_pose_; } 
protected:
  geometry_msgs::Pose binned_pose_;

  size_t hash_value() const
  {
    // Compute the hash off the raw bits by treating the doubles as if they
    // were unsigned 64-bit integers, multiplying them by medium sized
    // consecutive primes and summing them up. This operation is SIMD
    // friendly and much faster than std::hash, and works well for the types
    // of floating point coordinates encountered in Costmap queries.
    union {double d; uint64_t uint;} u[7] = {
        binned_pose_.orientation.x,
        binned_pose_.orientation.y,
        binned_pose_.orientation.z,
        binned_pose_.orientation.w,
        binned_pose_.position.x,
        binned_pose_.position.y,
        binned_pose_.position.z,
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
    for (unsigned i=0; i<7; ++i)
    {
      rv += u[i].uint * primes[i];
    }
    return static_cast<size_t>(rv);
  }
};

struct PoseBinKeyHash
{
  size_t operator()(const PoseBinKey& key) const
  {
    return key.hash_;
  }
};

struct PoseBinKeyEqual
{
  bool operator()(const PoseBinKey& lhs, const PoseBinKey& rhs) const
  {
    return lhs == rhs;
  }
};

using PoseBinEntry = std::pair<size_t, double>;
using PoseBinMap = std::unordered_map<PoseBinKey, PoseBinEntry, PoseBinKeyHash, PoseBinKeyEqual>;
  
void test_pose_binning_impl(int bins_per_meter, int bins_per_rotation)
{
  geometry_msgs::Pose in, out, expected;
  double half_rotation = std::floor(0.5 * bins_per_rotation + 0.5) / bins_per_rotation;
  double quarter_rotation = std::floor(0.25 * bins_per_rotation + 0.5) / bins_per_rotation;
  double opposite_quarter_rotation = half_rotation - quarter_rotation;
  in.position.x = 0;
  in.position.y = 0;
  in.position.z = 0;
  in.orientation.w = 1;
  in.orientation.x = 0;
  in.orientation.y = 0;
  in.orientation.z = 0;
  out = binPose(in, bins_per_meter, bins_per_rotation);
  EXPECT_POSE_NEAR(in, out);
  out = binPose2(in, bins_per_meter, bins_per_rotation);
  expected.position.x = 0;
  expected.position.y = 0;
  expected.position.z = 0;
  expected.orientation.w = 0;
  expected.orientation.x = 0;
  expected.orientation.y = quarter_rotation;
  expected.orientation.z = quarter_rotation;
  EXPECT_POSE_NEAR(out, expected);
  // The rest of these fixed tests only make sense when bins_per_rotation is > 1.
  if (bins_per_rotation > 1)
  {
    in.orientation.w = 0;
    in.orientation.x = 0;
    in.orientation.y = 0;
    in.orientation.z = 1;
    out = binPose(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(in, out);
    out = binPose2(in, bins_per_meter, bins_per_rotation);
    expected.orientation.w = half_rotation;
    expected.orientation.x = quarter_rotation;
    expected.orientation.y = quarter_rotation;
    expected.orientation.z = 0;
    EXPECT_POSE_NEAR(out, expected);
    in.orientation.z = -1;
    out = binPose(in, bins_per_meter, bins_per_rotation);
    EXPECT_NEAR(out.orientation.z, 1.0, 1e-6);
    out = binPose2(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(out, expected);
    in.orientation.z = -1.000001;
    out = binPose2(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(out, expected);
    in.orientation.z = 0.999999;
    in.orientation.w = 0.000001;
    out = binPose2(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(out, expected);
    in.orientation.z = -0.999999;
    in.orientation.w = 0.000001;
    expected.orientation.x = opposite_quarter_rotation;
    expected.orientation.y = opposite_quarter_rotation;
    out = binPose2(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(out, expected);
    double alpha = M_PI * ((static_cast<double>(bins_per_rotation-1) / bins_per_rotation) + 1e-9);
    in.orientation.w = cos(alpha/2);
    in.orientation.z = sin(alpha/2);
    expected.orientation.x = quarter_rotation;
    expected.orientation.y = quarter_rotation;
    out = binPose2(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(out, expected);
    in.orientation.z = -sin(alpha/2);
    expected.orientation.x = opposite_quarter_rotation;
    expected.orientation.y = opposite_quarter_rotation;
    out = binPose2(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(out, expected);
    in.orientation.w = -cos(alpha/2);
    expected.orientation.x = quarter_rotation;
    expected.orientation.y = quarter_rotation;
    out = binPose2(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(out, expected);
    // Incline the rotation axis as much as possible while staying in the bin.
    in.orientation.z = -sin(alpha/2) * cos(M_PI * (1.0 / bins_per_rotation) - 1e-9);
    out = binPose2(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(out, expected);
    if (bins_per_rotation > 2)
    {
      // Incline just a bit too far. This test only makes sense when
      // bins_per_rotation > 2.
      in.orientation.z = -sin(alpha/2) * cos(M_PI * (1.0 / bins_per_rotation) + 1e-9);
      out = binPose2(in, bins_per_meter, bins_per_rotation);
      EXPECT_POSE_NOT_NEAR(out, expected);
    }
    // Push the rotation angle just past the current bin.
    alpha = M_PI * ((static_cast<double>(bins_per_rotation-1) / bins_per_rotation) - 1e-9);
    in.orientation.w = cos(alpha/2);
    in.orientation.z = sin(alpha/2);
    out = binPose2(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NOT_NEAR(out, expected);
  }

  constexpr size_t n = 100000;
  std::vector<geometry_msgs::Pose> pose_arr, binned_poses;
  pose_arr.resize(n);
  binned_poses.resize(n);
  std::mt19937 gen(1);
  std::uniform_real_distribution<> distr(-1.0, 1.0);

  double max_angular_distance = 0.0;
  const double angular_distance_limit = binPose3AngularDistanceLimit(2 * bins_per_rotation);
  for (unsigned i=0; i<n; ++i)
  {
    pose_arr[i].position.x = distr(gen);
    pose_arr[i].position.y = distr(gen);
    pose_arr[i].position.z = distr(gen);
    Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
    pose_arr[i].orientation.x = q.x();
    pose_arr[i].orientation.y = q.y();
    pose_arr[i].orientation.z = q.z();
    pose_arr[i].orientation.w = q.w();
#if 0
    binned_poses[i] = binPose2(pose_arr[i], bins_per_meter, bins_per_rotation);
    // reconstruct q representation of the binned pose
    double half_alpha = M_PI * binned_poses[i].orientation.w;
    double cos_half_alpha = std::cos(half_alpha);
    double sin_half_alpha = std::sin(half_alpha);
    Eigen::Quaterniond binned_q(
        cos_half_alpha,
        sin_half_alpha * std::cos(2 * M_PI * binned_poses[i].orientation.x),
        sin_half_alpha * std::cos(2 * M_PI * binned_poses[i].orientation.y),
        sin_half_alpha * std::cos(2 * M_PI * binned_poses[i].orientation.z));
#endif
    binned_poses[i] = binPose3(pose_arr[i], bins_per_meter, bins_per_rotation);
    // Verify a binned pose maps back to itself
    geometry_msgs::Pose check_pose = binPose3(binned_poses[i], bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(check_pose, binned_poses[i]);
    Eigen::Quaterniond binned_q(
        binned_poses[i].orientation.w,
        binned_poses[i].orientation.x,
        binned_poses[i].orientation.y,
        binned_poses[i].orientation.z);
    // The distance between two quaternions is simply the dot-product.
    // To get into radians solve the equaion:
    // cos (angular_distance/2) = |q1 dot q2|
    double abs_prod = std::abs(q.dot(binned_q.normalized()));
    double angular_distance = 2 * acos(abs_prod <= 1 ? abs_prod : 0);
    max_angular_distance = std::max(angular_distance, max_angular_distance);
    if (angular_distance > angular_distance_limit)
    {
      std::cout << "angular distance: " << angular_distance
        << " original q.w: " << q.w()
        << " original q.vec: " << q.vec()
        << " binned pose: " << binned_poses[i]
        << " binned q.w: " << binned_q.w()
        << " binned q.vec: " << binned_q.vec()
        << std::endl;
    }
  }

  std::cout << "bins_per_rotation: " << bins_per_rotation
    << " max_angular_distance: " << max_angular_distance
    << " angular_distance_limit: " << angular_distance_limit
    << " max_angular_distance ratio: " << max_angular_distance / angular_distance_limit
    << std::endl;
#if 0
  std::cout << "first pose: " << pose_arr[0] << std::endl;
  std::cout << "first bin pose: " << binPose(pose_arr[0], bins_per_meter, bins_per_rotation) << std::endl;
  std::cout << "first bin2 pose: " << binPose2(pose_arr[0], bins_per_meter, bins_per_rotation) << std::endl;

  for (unsigned i=0; i<360; ++i)
  {
    double half_cos = cos(i * M_PI / 360.0);
    double half_sin = sin(i * M_PI / 360.0);
    pose_arr[i].orientation.x = half_sin * cos(i * M_PI / (180.0 * bins_per_rotation));
    pose_arr[i].orientation.y = half_sin * cos(i * M_PI / (180.0 * bins_per_rotation / 2));
    pose_arr[i].orientation.z = half_sin * cos(i * M_PI / (180.0 * bins_per_rotation / 4));
    pose_arr[i].orientation.w = half_cos;
  }
  for (unsigned i=0; i<360; ++i)
  {
    binned_poses[i] = binPose(pose_arr[i], bins_per_meter, bins_per_rotation);
    std::cout << i << ": "
      << binned_poses[i].orientation.x << " "
      << binned_poses[i].orientation.y << " "
      << binned_poses[i].orientation.z << " "
      << binned_poses[i].orientation.w << std::endl;
  }
  for (unsigned i=0; i<360; ++i)
  {
    binned_poses[i] = binPoseLog(pose_arr[i], bins_per_meter, bins_per_rotation);
    std::cout << i << ": [log] "
      << binned_poses[i].orientation.x << " "
      << binned_poses[i].orientation.y << " "
      << binned_poses[i].orientation.z << " "
      << binned_poses[i].orientation.w << std::endl;
  }
  for (unsigned i=0; i<360; ++i)
  {
    binned_poses[i] = binPose2(pose_arr[i], bins_per_meter, bins_per_rotation);
    std::cout << i << ": [2] "
      << binned_poses[i].orientation.x << " "
      << binned_poses[i].orientation.y << " "
      << binned_poses[i].orientation.z << " "
      << binned_poses[i].orientation.w << std::endl;
  }
#endif

  std::chrono::high_resolution_clock::time_point start_time;
  std::chrono::high_resolution_clock::duration bin_pose_time;

  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned i=0; i<n; ++i)
  {
    binned_poses[i] = binPose(pose_arr[i], bins_per_meter, bins_per_rotation);
  }
  bin_pose_time = std::chrono::high_resolution_clock::now() - start_time;
  std::cout << "Average time to bin random pose: " <<
      std::chrono::duration_cast<std::chrono::nanoseconds>(bin_pose_time).count() / n <<
      "ns" << std::endl;

  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned i=0; i<n; ++i)
  {
    binned_poses[i] = binPose2(pose_arr[i], bins_per_meter, bins_per_rotation);
  }
  bin_pose_time = std::chrono::high_resolution_clock::now() - start_time;
  std::cout << "Average time to bin2 random pose: " <<
      std::chrono::duration_cast<std::chrono::nanoseconds>(bin_pose_time).count() / n <<
      "ns" << std::endl;

  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned i=0; i<n; ++i)
  {
    binned_poses[i] = binPose3(pose_arr[i], bins_per_meter, bins_per_rotation);
  }
  bin_pose_time = std::chrono::high_resolution_clock::now() - start_time;
  std::cout << "Average time to bin3 random pose: " <<
      std::chrono::duration_cast<std::chrono::nanoseconds>(bin_pose_time).count() / n <<
      "ns" << std::endl;

#if 0
  PoseBinMap pose_bin_map;
  for (double i=0; i<360.0; i+=.25)
  {
    double half_cos = cos(i * M_PI / 360.0);
    double half_sin = sin(i * M_PI / 360.0);
    in.orientation.x = 0.0;
    in.orientation.y = 0.0;
    in.orientation.z = half_sin;
    in.orientation.w = half_cos;
    out = binPose3(in, bins_per_meter, bins_per_rotation);
    PoseBinKey pose_bin_key(out);
    Eigen::Quaterniond q(
        in.orientation.w,
        in.orientation.x,
        in.orientation.y,
        in.orientation.z);
    Eigen::Quaterniond binned_q(
        out.orientation.w,
        out.orientation.x,
        out.orientation.y,
        out.orientation.z);
    double angular_distance = 2 * acos(std::abs(q.normalized().dot(binned_q.normalized())));
    auto& pose_bin = pose_bin_map[pose_bin_key];
    ++pose_bin.first;
    pose_bin.second = std::max(pose_bin.second, angular_distance);
  }
  for (const auto& bin : pose_bin_map)
  {
    std::cout << "bin key: " << bin.first.getBinnedPose()
      << " size: " << bin.second.first
      << " angular size: " << bin.second.second << std::endl;
  }
#endif
}

using BinPoseFunction = std::function<geometry_msgs::Pose(const geometry_msgs::Pose&, int, int)>;

void eval_pose_binning(BinPoseFunction bin_func, int bins_per_meter, int bins_per_rotation)
{
  // Measure bin_func fairness by sampling a higher resolution uniform grid on SO(3)
  PoseBinMap pose_bin_map;
  int factor = 8;
  int nb = factor * bins_per_rotation;
  std::mt19937 gen(1);
  std::uniform_real_distribution<> distr(-.499, .499);
  for (unsigned int u1 = 0; u1 < nb; ++u1)
  {
    for (unsigned int u2 = 0; u2 < nb; ++u2)
    {
      for (unsigned int u3 = 0; u3 < nb; ++u3)
      {
        const double d1 = (static_cast<double>(u1) + 0.5 + distr(gen)) / nb;
        const double d1_c0 = std::sqrt(1.0 - d1);
        const double d1_c1 = std::sqrt(d1);
        const double d2 = (static_cast<double>(u2) + 0.5 + distr(gen)) / nb;
        const double cosd2 = std::cos(2 * M_PI * d2);
        const double sind2 = std::sin(2 * M_PI * d2);
        const double d3 = (static_cast<double>(u3) + 0.5 + distr(gen)) / nb;
        const double cosd3 = std::cos(2 * M_PI * d3);
        const double sind3 = std::sin(2 * M_PI * d3);
        geometry_msgs::Pose pose;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        Eigen::Quaterniond q(
            d1_c0 * sind2,
            d1_c1 * cosd3,
            d1_c1 * sind3,
            d1_c0 * cosd2);
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose = bin_func(pose, bins_per_meter, bins_per_rotation);
        Eigen::Quaterniond binned_q(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z);
        PoseBinKey pose_bin_key(pose);
        double angular_distance = 2 * acos(std::abs(q.normalized().dot(binned_q.normalized())));
        auto& pose_bin = pose_bin_map[pose_bin_key];
        ++pose_bin.first;
        pose_bin.second = std::max(pose_bin.second, angular_distance);
      }
    }
  }
  size_t total_bins = 0, min_bins = std::numeric_limits<size_t>::max(), max_bins = 0;
  double total_angle = 0, min_angle = std::numeric_limits<double>::max(), max_angle = 0;
  for (const auto& bin : pose_bin_map)
  {
    size_t bin_count = bin.second.first;
    double angular_size = bin.second.second;
    total_bins += bin_count;
    total_angle += angular_size;
    min_bins = std::min(min_bins, bin_count);
    max_bins = std::max(max_bins, bin_count);
    min_angle = std::min(min_angle, angular_size);
    max_angle = std::max(max_angle, angular_size);
  }
  double mean_bins = static_cast<double>(total_bins) / pose_bin_map.size();
  double mean_angle = total_angle / pose_bin_map.size();
  double bins_sum_of_sq_diffs = 0;
  double angle_sum_of_sq_diffs = 0;
  for (const auto& bin : pose_bin_map)
  {
    size_t bin_count = bin.second.first;
    double angular_size = bin.second.second;
    double diff = bin_count - mean_bins;
    bins_sum_of_sq_diffs += diff * diff;
    diff = angular_size - mean_angle;
    angle_sum_of_sq_diffs += diff * diff;
    if (bin_count == max_bins || bin_count == min_bins)
    {
      std::cout << "bin key: " << bin.first.getBinnedPose()
        << " size: " << bin.second.first
        << " angular size: " << bin.second.second << std::endl;
    }
  }
  double stdev_bins = std::sqrt(bins_sum_of_sq_diffs / (pose_bin_map.size() - 1));
  double stdev_angle = std::sqrt(angle_sum_of_sq_diffs / (pose_bin_map.size() - 1));
  std::cout << "number of bins: " << pose_bin_map.size() << std::endl;
  std::cout << "mean size of bin: " << mean_bins << std::endl;
  std::cout << "min size of bin: " << min_bins << std::endl;
  std::cout << "max size of bin: " << max_bins << std::endl;
  std::cout << "stdev: " << stdev_bins << std::endl;
  std::cout << "mean max-angle of bin: " << mean_angle << std::endl;
  std::cout << "min max-angle of bin: " << min_angle << std::endl;
  std::cout << "max max-angle of bin: " << max_angle << std::endl;
  std::cout << "stdev: " << stdev_angle << std::endl;
}

Eigen::Quaterniond getPose3Quaternion(double d1, double d2, double d3)
{
  const double d1_c0 = std::sqrt(1.0 - d1);
  const double d1_c1 = std::sqrt(d1);
  const double cosd2 = std::cos(2 * M_PI * d2);
  const double sind2 = std::sin(2 * M_PI * d2);
  const double cosd3 = std::cos(2 * M_PI * d3);
  const double sind3 = std::sin(2 * M_PI * d3);
  return Eigen::Quaterniond(
      d1_c0 * sind2,
      d1_c1 * cosd3,
      d1_c1 * sind3,
      d1_c0 * cosd2);
}

TEST(test_octree_solver, test_pose_binning)
{
  constexpr int bins_per_rotation_to_test[] = {1, 2, 4, 6, 8, 10, 16, 20, 32, 50, 100, 128, 256, 500, 512, 1000, 1024};

  for (int bins_per_rotation : bins_per_rotation_to_test)
  {
    test_pose_binning_impl(16, bins_per_rotation);
  }

  std::cout << "Stats for binPose" << std::endl;
  eval_pose_binning(binPose, 16, 16);
  std::cout << "Stats for binPose2" << std::endl;
  eval_pose_binning(binPose2, 16, 16);
  std::cout << "Stats for binPoseLog" << std::endl;
  eval_pose_binning(binPoseLog, 16, 16);
  std::cout << "Stats for binPoseLog cube stretch" << std::endl;
  eval_pose_binning(binPoseLog<true>, 16, 16);
  std::cout << "Stats for binPose3" << std::endl;
  eval_pose_binning(binPose3, 16, 16);
  std::cout << "Stats for binPoseSimple" << std::endl;
  eval_pose_binning(binPoseSimple, 16, 16);
  std::cout << "Stats for binPoseSimple tesseract stretch" << std::endl;
  eval_pose_binning(binPoseSimple<true>, 16, 16);
  std::cout << "Stats for binPoseAxisAngle" << std::endl;
  eval_pose_binning(binPoseSimple<true>, 16, 16);
  geometry_msgs::Pose in, out, expected;
  in.position.x = 0;
  in.position.y = 0;
  in.position.z = 0;
  in.orientation.w = 0;
  in.orientation.x = 0;
  in.orientation.y = 0;
  in.orientation.z = 1;
  out = binPose3(in, 16, 16);
  in.orientation.z = -1;
  expected = binPose3(in, 16, 16);
  EXPECT_POSE_NEAR(out, expected);
#if 0
  int bins = 16;
  PoseBinMap pose_bin_map;
  for (unsigned i=0; i<360; ++i)
  {
    double half_cos = cos(i * M_PI / 360.0);
    double half_sin = sin(i * M_PI / 360.0);
    in.orientation.x = 0.0;
    in.orientation.y = 0.0;
    in.orientation.z = half_sin;
    in.orientation.w = half_cos;
//    out = binPoseLog<true>(in, bins, bins);
    out = binPose3(in, bins, bins);
    PoseBinKey pose_bin_key(out);
    Eigen::Quaterniond q(
        in.orientation.w,
        in.orientation.x,
        in.orientation.y,
        in.orientation.z);
    Eigen::Quaterniond binned_q(
        out.orientation.w,
        out.orientation.x,
        out.orientation.y,
        out.orientation.z);
    double angular_distance = 2 * acos(std::abs(q.normalized().dot(binned_q.normalized())));
    auto& pose_bin = pose_bin_map[pose_bin_key];
    ++pose_bin.first;
    pose_bin.second = std::max(pose_bin.second, angular_distance);
#if 0
    std::cout << i << ": "
      << out.orientation.x << " "
      << out.orientation.y << " "
      << out.orientation.z << " "
      << out.orientation.w << std::endl;
#endif
  }
  for (const auto& bin : pose_bin_map)
  {
    std::cout << "bin key: " << bin.first.getBinnedPose()
      << " size: " << bin.second.first
      << " angular size: " << bin.second.second << std::endl;
  }
#endif
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
