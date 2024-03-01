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
#include <costmap_3d/bin_pose.h>
#include <costmap_3d/costmap_3d_query.h>
#include <costmap_3d/octree_solver.h>

static std::string make_package_url(const char* path)
{
  return std::string("package://costmap_3d/") + path;
}

template <typename Generator, typename S>
void generateRandomTransforms(
    Generator& gen,
    S extents[6],
    fcl::aligned_vector<fcl::Transform3<S>>& transforms,
    std::size_t n);

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
  size_t n = 10000;
  std::chrono::high_resolution_clock::time_point start_time;
  fcl::aligned_vector<fcl::Transform3<double>> transforms;
  double extents[6] = {-10.0, -10.0, -10.0, 10.0, 10.0, 10.0};
  std::mt19937 gen(1);
  generateRandomTransforms(gen, extents, transforms, n);
  for (unsigned i=0; i < n; ++i)
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
  for (unsigned i=0; i < n; ++i)
  {
    const fcl::Transform3<double>& obbrss_tf = transforms[i];
    total_distance += costmap_3d::distanceOctomapOBB(radius, aabb_center, obbrss.obb, obbrss_tf);
  }
  std::cout << "Branch-free implementation: "
    << std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now() - start_time).count() / n
    << " ns, total distance: " << total_distance << std::endl;
  total_distance = 0;
  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned i=0; i < n; ++i)
  {
    const fcl::Transform3<double>& obbrss_tf = transforms[i];
    total_distance += sphereOBBSignedDistance(radius, aabb_center, obbrss.obb, obbrss_tf);
  }
  std::cout << "FCL-based implementation: "
    << std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now() - start_time).count() / n
    << " ns, total distance: " << total_distance << std::endl;
  // Time Eigen's implementation as a reference as well.
  total_distance = 0;
  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned i=0; i < n; ++i)
  {
    const fcl::Transform3<double>& obbrss_tf = transforms[i];
    total_distance += sphereOBBSignedDistanceEigen(radius, aabb_center, obbrss.obb, obbrss_tf);
  }
  std::cout << "Eigen-based implementation: "
    << std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now() - start_time).count() / n <<
    " ns, total distance: " << total_distance << std::endl;
}

void octree_solver_test(
    std::size_t n,
    bool negative_x_roi = false,
    bool non_negative_x_roi = false,
    bool skip_check = false);

TEST(test_octree_solver, test_against_fcl)
{
  octree_solver_test(15, false, false);
  octree_solver_test(15, true, false);
  octree_solver_test(15, false, true);
  std::chrono::high_resolution_clock::time_point start_time;
}

template <typename Generator>
double rand_interval(Generator& gen, double rmin, double rmax)
{
  std::uniform_real_distribution<> d_distr(rmin, rmax);
  return d_distr(gen);
}

template <typename Generator>
Eigen::Quaterniond getRandomVersor(Generator& gen)
{
  const double u1 = rand_interval(gen, 0, 1);
  const double u2 = rand_interval(gen, 0, 2 * M_PI);
  const double u3 = rand_interval(gen, 0, 2 * M_PI);
  const double a = std::sqrt(1.0 - u1);
  const double b = std::sqrt(u1);
  return Eigen::Quaterniond(a * std::sin(u2), a * std::cos(u2), b * std::sin(u3), b * std::cos(u3));
}

template <typename Generator, typename S>
void generateRandomTransforms(
    Generator& gen,
    S extents[6],
    fcl::aligned_vector<fcl::Transform3<S>>& transforms,
    std::size_t n)
{
  transforms.resize(n);
  for (std::size_t i=0; i < n; ++i)
  {
    auto x = rand_interval(gen, extents[0], extents[3]);
    auto y = rand_interval(gen, extents[1], extents[4]);
    auto z = rand_interval(gen, extents[2], extents[5]);
    fcl::Vector3<S> T(x, y, z);
    transforms[i].setIdentity();
    transforms[i].linear() = getRandomVersor(gen).matrix();
    transforms[i].translation() = T;
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

  for (std::size_t i=0; i < tree_boxes.size(); ++i)
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

  if (cdata->done) { dist = result.min_distance; return true; }

  fcl::distance(o1, o2, request, result);

  dist = result.min_distance;

  if (dist <= 0) return true;  // in collision or in touch

  return cdata->done;
}

void octree_solver_test(std::size_t n, bool negative_x_roi, bool non_negative_x_roi, bool skip_check)
{
  using S = costmap_3d::Costmap3DQuery::FCLFloat;
  costmap_3d::Costmap3DPtr octree(new costmap_3d::Costmap3D(
          costmap_3d::Costmap3DQuery::getFileNameFromPackageURL(make_package_url("test/aisles.bt"))));
  // Ensure occupancy threshold is setup properly.
  octree->setOccupancyThres(0.5);
  std::shared_ptr<fcl::OcTree<S>> tree_ptr(new fcl::OcTree<S>(octree));

  // Use Costmap3DQuery to get BVH for test mesh
  costmap_3d::Costmap3DQuery query(octree, make_package_url("test/test_robot.stl"));
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

  std::mt19937 gen(1);
  generateRandomTransforms(gen, extents, transforms, n);
  // Be sure to test identity
  transforms[0] = fcl::Transform3<S>::Identity();

  std::chrono::high_resolution_clock::duration total_time(0);
  std::chrono::high_resolution_clock::time_point start_time;
  for (std::size_t i=0; i < n; ++i)
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
    std::cout << " octree iteration " << i << ": "
      << std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::high_resolution_clock::now() -start_time).count()
      << "ns" << std::endl;
    total_time += std::chrono::high_resolution_clock::now() - start_time;

    // Check the result against FCL's broadphase distance
    std::vector<std::shared_ptr<fcl::CollisionObject<S>>> boxes;
    generateBoxesFromOctomap<S>(*tree_ptr, &boxes, non_negative_x_roi, negative_x_roi);
    for (std::size_t j=0; j < boxes.size(); ++j)
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
  std::cout << "Average time per octree solve: "
    << std::chrono::duration_cast<std::chrono::nanoseconds>(total_time).count() / n
    << "ns" << std::endl;
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
      bool more_permutations = true;
      while (more_permutations)
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
        more_permutations = std::next_permutation(permutation, permutation + n);
      }
      if (n > 1)
      {
        // Now test sorting 0/1 sequences of every combination to ensure
        // sorting works properly with equal entries. Use a bitset to count the
        // number of set bits in a fairly standard way (popcount isn't added
        // until C++20).
        std::bitset<8> bits;
        for (unsigned p=0; p < (1 << n); ++p)
        {
          bits.reset();
          for (unsigned bit=0; bit < n; ++bit)
          {
            if (((1 << bit) & p) == 0)
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
  constexpr size_t n = 10000;
  std::array<std::array<double, 8>, n> darrs;
  std::array<unsigned, n> ns;
  std::mt19937 gen(1);
  std::uniform_int_distribution<> n_distr(2, 8);
  std::uniform_real_distribution<> d_distr(-1e6, 1e6);

  for (unsigned a=0; a < n; ++a)
  {
    ns[a] = n_distr(gen);
    for (unsigned i=0; i < ns[a]; ++i)
    {
      darrs[a][i] = d_distr(gen);
    }
  }
  // Check to make sure that ArgSortUpTo8 actually sorts these random numbers
  // within the acceptable tolerance (as the internal packing step does
  // introduce some absolute error at the nano-meter scale)
  for (unsigned a=0; a < n; ++a)
  {
    unsigned indices[8];
    costmap_3d::ArgSortUpTo8(ns[a], indices, darrs[a].data());
    for (unsigned i=0; i < ns[a] - 1; ++i)
    {
      EXPECT_LT(darrs[a][indices[i]], darrs[a][indices[i + 1]] + 1e-7);
    }
  }
  // Now time the same sorts, and compare to std::sort on the same data.
  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned a=0; a < n; ++a)
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
  for (unsigned a=0; a < n; ++a)
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
  explicit PoseBinKey(const geometry_msgs::Pose& pose)
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
    for (unsigned i=0; i < 7; ++i)
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
using costmap_3d::binPose;
using costmap_3d::binPoseAngularDistanceLimit;

void test_pose_binning_impl(int bins_per_meter, int bins_per_rotation, bool verbose = false)
{
  geometry_msgs::Pose in, out, expected;
  in.position.x = 0;
  in.position.y = 0;
  in.position.z = 0;
  in.orientation.w = 1;
  in.orientation.x = 0;
  in.orientation.y = 0;
  in.orientation.z = 0;
  expected = binPose(in, bins_per_meter, bins_per_rotation);
  if (bins_per_rotation > 1)
  {
    in.orientation.w = .999999;
    in.orientation.x = 0;
    in.orientation.y = 0;
    in.orientation.z = -std::sqrt(1-.999999*.999999);
    out = binPose(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(out, expected);
    in.orientation.w *= -1;
    in.orientation.z *= -1;
    out = binPose(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(out, expected);
    // Make the angle as big as possible while staying in the bin.
    double u0 = 0 + (0.5 / bins_per_rotation) - std::numeric_limits<double>::epsilon();
    double u1 = 0 + (0.5 / bins_per_rotation) - std::numeric_limits<double>::epsilon();
    double u2 = 0.25 + (0.5 / bins_per_rotation) - std::numeric_limits<double>::epsilon();
    in.orientation.x = std::sqrt(u0) * std::cos(2 * M_PI * u1);
    in.orientation.y = std::sqrt(u0) * std::sin(2 * M_PI * u1);
    in.orientation.z = std::sqrt(1.0 - u0) * std::cos(2 * M_PI * u2);
    in.orientation.w = std::sqrt(1.0 - u0) * std::sin(2 * M_PI * u2);
    out = binPose(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(out, expected);
    // Go just too far.
    u2 = 0.25 + (0.5 / bins_per_rotation) + std::numeric_limits<double>::epsilon();
    in.orientation.z = std::sqrt(1.0 - u0) * std::cos(2 * M_PI * u2);
    in.orientation.w = std::sqrt(1.0 - u0) * std::sin(2 * M_PI * u2);
    out = binPose(in, bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NOT_NEAR(out, expected);
  }
  in.position.x = 0;
  in.position.y = 0;
  in.position.z = 0;
  in.orientation.w = 0;
  in.orientation.x = 0;
  in.orientation.y = 0;
  in.orientation.z = 1;
  out = binPose(in, bins_per_meter, bins_per_rotation);
  in.orientation.z = -1;
  expected = binPose(in, bins_per_meter, bins_per_rotation);
  EXPECT_POSE_NEAR(out, expected);
  in.orientation.w = -0;
  expected = binPose(in, bins_per_meter, bins_per_rotation);
  EXPECT_POSE_NEAR(out, expected);
  in.orientation.x = -0;
  expected = binPose(in, bins_per_meter, bins_per_rotation);
  EXPECT_POSE_NEAR(out, expected);
  in.orientation.x = 0;
  in.orientation.y = -0;
  expected = binPose(in, bins_per_meter, bins_per_rotation);
  EXPECT_POSE_NEAR(out, expected);
  in.orientation.w = sin(.001);
  in.orientation.z = cos(.001);
  expected = binPose(in, bins_per_meter, bins_per_rotation);
  EXPECT_POSE_NEAR(out, expected);
  in.orientation.w *= -1.0;
  in.orientation.z *= -1.0;
  expected = binPose(in, bins_per_meter, bins_per_rotation);
  EXPECT_POSE_NEAR(out, expected);
  in.orientation.z *= -1.0;
  expected = binPose(in, bins_per_meter, bins_per_rotation);
  if (bins_per_rotation > 1)
  {
    EXPECT_POSE_NOT_NEAR(out, expected);
  }
  else
  {
    // There is only one bin...
    EXPECT_POSE_NEAR(out, expected);
  }
  in.orientation.w *= -1.0;
  in.orientation.z *= -1.0;
  expected = binPose(in, bins_per_meter, bins_per_rotation);
  if (bins_per_rotation > 1)
  {
    EXPECT_POSE_NOT_NEAR(out, expected);
  }
  else
  {
    // There is only one bin...
    EXPECT_POSE_NEAR(out, expected);
  }

  constexpr size_t n = 10000;
  std::vector<geometry_msgs::Pose> pose_arr, binned_poses;
  pose_arr.resize(n);
  binned_poses.resize(n);
  std::mt19937 gen(1);
  std::uniform_real_distribution<> distr(-1.0, 1.0);

  double max_angular_distance = 0.0;
  // Because the below code measures to the center of the bin, use the limit of
  // double the bins_per_rotation to get the maximum distance from the center
  // of a bin to one of its corners.
  const double angular_distance_limit = binPoseAngularDistanceLimit(2 * bins_per_rotation);
  for (unsigned i=0; i < n; ++i)
  {
    pose_arr[i].position.x = distr(gen);
    pose_arr[i].position.y = distr(gen);
    pose_arr[i].position.z = distr(gen);
    Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
    pose_arr[i].orientation.x = q.x();
    pose_arr[i].orientation.y = q.y();
    pose_arr[i].orientation.z = q.z();
    pose_arr[i].orientation.w = q.w();
    binned_poses[i] = binPose(pose_arr[i], bins_per_meter, bins_per_rotation);
    // Verify a binned pose maps back to itself
    geometry_msgs::Pose check_pose = binPose(binned_poses[i], bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(check_pose, binned_poses[i]);
    Eigen::Quaterniond binned_q(
        binned_poses[i].orientation.w,
        binned_poses[i].orientation.x,
        binned_poses[i].orientation.y,
        binned_poses[i].orientation.z);
    // The distance between two quaternions is simply the dot-product.
    // To get into radians solve the equation:
    // cos (angular_distance/2) = |q1 dot q2|
    double abs_prod = std::abs(q.dot(binned_q.normalized()));
    EXPECT_LE(abs_prod, 1.0);
    double angular_distance = 2 * acos(abs_prod);
    max_angular_distance = std::max(angular_distance, max_angular_distance);
    EXPECT_LE(angular_distance, angular_distance_limit);
  }

  if (verbose)
  {
    std::cout << "bins_per_rotation: " << bins_per_rotation
      << " max_angular_distance: " << max_angular_distance
      << " angular_distance_limit: " << angular_distance_limit
      << " max_angular_distance ratio: " << max_angular_distance / angular_distance_limit
      << std::endl;
  }

  std::chrono::high_resolution_clock::time_point start_time;
  std::chrono::high_resolution_clock::duration bin_pose_time;

  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned i=0; i < n; ++i)
  {
    binned_poses[i] = binPose(pose_arr[i], bins_per_meter, bins_per_rotation);
  }
  bin_pose_time = std::chrono::high_resolution_clock::now() - start_time;
  if (verbose)
  {
    std::cout << "Average time to bin random 3D pose: " <<
        std::chrono::duration_cast<std::chrono::nanoseconds>(bin_pose_time).count() / n <<
        "ns" << std::endl;
  }

  for (unsigned i=0; i < n; ++i)
  {
    pose_arr[i].position.x = distr(gen);
    pose_arr[i].position.y = distr(gen);
    pose_arr[i].position.z = 0.0;
    double alpha = 2 * M_PI * distr(gen);
    Eigen::Quaterniond q(cos(alpha / 2), 0, 0, sin(alpha / 2));
    pose_arr[i].orientation.x = q.x();
    pose_arr[i].orientation.y = q.y();
    pose_arr[i].orientation.z = q.z();
    pose_arr[i].orientation.w = q.w();
    binned_poses[i] = binPose(pose_arr[i], bins_per_meter, bins_per_rotation);
    // Verify a binned pose maps back to itself
    geometry_msgs::Pose check_pose = binPose(binned_poses[i], bins_per_meter, bins_per_rotation);
    EXPECT_POSE_NEAR(check_pose, binned_poses[i]);
    Eigen::Quaterniond binned_q(
        binned_poses[i].orientation.w,
        binned_poses[i].orientation.x,
        binned_poses[i].orientation.y,
        binned_poses[i].orientation.z);
    // The distance between two quaternions is simply the dot-product.
    // To get into radians solve the equation:
    // cos (angular_distance/2) = |q1 dot q2|
    double abs_prod = std::abs(q.dot(binned_q.normalized()));
    EXPECT_LE(abs_prod, 1.0);
    double angular_distance = 2 * acos(abs_prod);
    max_angular_distance = std::max(angular_distance, max_angular_distance);
    EXPECT_LE(angular_distance, angular_distance_limit);
  }

  start_time = std::chrono::high_resolution_clock::now();
  for (unsigned i=0; i < n; ++i)
  {
    binned_poses[i] = binPose(pose_arr[i], bins_per_meter, bins_per_rotation);
  }
  bin_pose_time = std::chrono::high_resolution_clock::now() - start_time;
  if (verbose)
  {
    std::cout << "Average time to bin random 2D pose: " <<
        std::chrono::duration_cast<std::chrono::nanoseconds>(bin_pose_time).count() / n <<
        "ns" << std::endl;
  }

  // Verify that rotations about the z axis all bin equally
  PoseBinMap pose_bin_map;
  for (double i=.125 / bins_per_rotation; i < 1.0; i+=(.25 / bins_per_rotation))
  {
    double half_cos = cos(M_PI * i);
    double half_sin = sin(M_PI * i);
    in.orientation.x = 0.0;
    in.orientation.y = 0.0;
    in.orientation.z = half_sin;
    in.orientation.w = half_cos;
    out = binPose(in, bins_per_meter, bins_per_rotation);
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
    EXPECT_EQ(bin.second.first, 4);
    if (bin.second.first != 4)
    {
      std::cout << "bin key: " << bin.first.getBinnedPose()
        << " size: " << bin.second.first
        << " angular size: " << bin.second.second << std::endl;
    }
  }
}

using BinPoseFunction = std::function<geometry_msgs::Pose(const geometry_msgs::Pose&, int, int)>;

void eval_pose_binning_fairness(BinPoseFunction bin_func, int bins_per_meter, int bins_per_rotation)
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
    if (max_bins != min_bins && (bin_count == max_bins || bin_count == min_bins))
    {
      std::cout << "bin key: " << bin.first.getBinnedPose()
        << " size: " << bin.second.first
        << " angular size: " << bin.second.second << std::endl;
    }
  }
  double stdev_bins = std::sqrt(bins_sum_of_sq_diffs / (pose_bin_map.size() - 1));
  double stdev_angle = std::sqrt(angle_sum_of_sq_diffs / (pose_bin_map.size() - 1));
  EXPECT_EQ(stdev_bins, 0.0);
  if (stdev_bins != 0.0)
  {
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
}

TEST(test_octree_solver, test_pose_binning)
{
  constexpr int bins_per_rotation_to_test[] = {1, 2, 4, 6, 8, 10, 16, 20, 32, 50, 100, 128, 256, 500, 512, 1000, 1024};

  for (int bins_per_rotation : bins_per_rotation_to_test)
  {
    test_pose_binning_impl(16, bins_per_rotation);
  }
  test_pose_binning_impl(16, 2000, true);

  // Testing fairness is expensive, only do it for a few bins per rotation
  // values.
  eval_pose_binning_fairness(binPose, 8, 8);
  eval_pose_binning_fairness(binPose, 16, 16);
  eval_pose_binning_fairness(binPose, 20, 20);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
