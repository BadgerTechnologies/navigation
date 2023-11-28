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
#include <random>

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
  size_t n = 100000;
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
  std::shared_ptr<fcl::OcTree<S>> tree_ptr(new fcl::OcTree<S>(octree));
  tree_ptr->setFreeThres(0.01);
  tree_ptr->setOccupancyThres(0.5);

  // Use Costmap3DQuery to get BVH for test mesh
  costmap_3d::Costmap3DQuery query(octree, PACKAGE_URL + "test/test_robot.stl");
  costmap_3d::Costmap3DQuery::FCLRobotModelConstPtr m1 = query.getFCLRobotModel();
  std::shared_ptr<const fcl::CollisionGeometry<S>> m1_ptr(m1);

  if (negative_x_roi)
  {
    fcl::Vector3<S> normal(1.0, 0.0, 0.0);
    fcl::Halfspace<S> negative_x(normal, 0);
    tree_ptr->addToRegionOfInterest(negative_x);
  }
  if (non_negative_x_roi)
  {
    fcl::Vector3<S> normal(-1.0, 0.0, 0.0);
    fcl::Halfspace<S> non_negative_x(normal, 0);
    tree_ptr->addToRegionOfInterest(non_negative_x);
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
    fcl::DistanceRequest<S> request;
    fcl::DistanceResult<S> result;
    request.abs_err = 0.0;
    request.rel_err = 0.0;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;
    result.min_distance = std::numeric_limits<S>::max();
    start_time = std::chrono::high_resolution_clock::now();
    octree_solver.distance(
        tree_ptr.get(),
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

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
