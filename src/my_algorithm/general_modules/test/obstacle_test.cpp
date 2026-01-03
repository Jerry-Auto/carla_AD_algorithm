#include <cmath>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <limits>
#include <random>
#include <string>
#include <vector>
#include <algorithm>
#include <memory>

#include "general_modules/common_types.h"
#include "general_modules/collision_detection.h"
#include "general_modules/geometry.h"
#include "general_modules/math_tool.h"

using AD_algorithm::general::Obstacle;
using AD_algorithm::general::TrajectoryPoint;
using AD_algorithm::general::VehicleParams;
using AD_algorithm::general::Vec2d;
using AD_algorithm::general::Polygon2d;
using AD_algorithm::general::CollisionDetection;

static constexpr double kEps = 1e-7;

static bool approx_eq(double a, double b, double eps = 1e-6) {
  return std::fabs(a - b) <= eps * (1.0 + std::max(std::fabs(a), std::fabs(b)));
}

struct Stats {
  int failures = 0;
  int checks = 0;
};

static void expect_true(Stats& st, bool cond, const std::string& msg) {
  ++st.checks;
  if (!cond) {
    ++st.failures;
    std::cerr << "FAIL: " << msg << "\n";
  }
}

static void expect_near(Stats& st, double got, double expected, double eps, const std::string& msg) {
  ++st.checks;
  if (!approx_eq(got, expected, eps)) {
    ++st.failures;
    std::cerr << "FAIL: " << msg
              << " expected=" << std::setprecision(17) << expected
              << " got=" << got
              << " diff=" << std::fabs(got - expected) << "\n";
  }
}

static double randu(std::mt19937_64& rng, double lo, double hi) {
  std::uniform_real_distribution<double> dist(lo, hi);
  return dist(rng);
}

static double rand_heading(std::mt19937_64& rng) {
  return randu(rng, -M_PI, M_PI);
}

static Obstacle make_obs(double x, double y, double heading, double L, double W) {
  Obstacle o;
  o.x = x;
  o.y = y;
  o.heading = heading;
  o.length = L;
  o.width = W;
  return o;
}

static void check_point_api_consistency(Stats& st, const Obstacle& o, const Vec2d& p, double margin) {
  // Basic equivalence by definition
  double d = o.minDistanceTo(p.x, p.y);
  bool c = o.contains(p.x, p.y, margin);
  expect_true(st, (d <= margin) == c, "contains(p,margin) should equal (minDistanceTo(p) <= margin)");

  // Compare against CollisionDetection with bounding box built from current obstacle state
  auto poly = CollisionDetection::get_bounding_box(o);
  double d_ref = CollisionDetection::distance_to(poly, p);
  expect_near(st, d, d_ref, 1e-6, "minDistanceTo(point) should match CollisionDetection::distance_to(box, point)");
}

static void check_obs_obs_api_consistency(Stats& st, const Obstacle& a, const Obstacle& b) {
  double d1 = a.minDistanceTo(b);
  double d2 = b.minDistanceTo(a);

  bool ov1 = a.hasOverlap(b);
  bool ov2 = b.hasOverlap(a);

  // Symmetry
  expect_near(st, d1, d2, 1e-9, "minDistanceTo(obs) should be symmetric");
  expect_true(st, ov1 == ov2, "hasOverlap(obs) should be symmetric");

  // Compare to CollisionDetection reference using fresh boxes
  auto pa = CollisionDetection::get_bounding_box(a);
  auto pb = CollisionDetection::get_bounding_box(b);

  double d_ref = CollisionDetection::distance_to(pa, pb);
  bool ov_ref = CollisionDetection::has_overlap(pa, pb);

  expect_near(st, d1, d_ref, 1e-6, "Obstacle::minDistanceTo(obs) should match CollisionDetection distance");
  expect_true(st, ov1 == ov_ref, "Obstacle::hasOverlap(obs) should match CollisionDetection overlap");

  // Invariants: overlap => distance ~0; distance ~0 => overlap (touch counts semantics)
  if (ov1) {
    expect_true(st, d1 <= 1e-6, "overlap => distance approx 0");
  }
  if (d1 <= 1e-9) {
    expect_true(st, ov1, "distance approx 0 => overlap (touch counts)");
  }
}

static void check_trj_distance(Stats& st, const Obstacle& obs, const TrajectoryPoint& trj, const VehicleParams& vp) {
  // Obstacle::minDistanceTo(TrajectoryPoint) uses default VehicleParams in your current implementation
  // so we compare against that same default box to avoid false failures.
  auto obox = CollisionDetection::get_bounding_box(obs);
  auto ego_default = CollisionDetection::get_bounding_box(trj); // default VehicleParams
  double d_ref = CollisionDetection::distance_to(obox, ego_default);

  double d = obs.minDistanceTo(trj);
  expect_near(st, d, d_ref, 1e-6, "Obstacle::minDistanceTo(trj) should match CollisionDetection with default VehicleParams");
}

static void run_constructed_cases(Stats& st) {
  // Touching boxes (edge touch): distance 0, overlap true
  {
    Obstacle a = make_obs(0, 0, 0, 2, 2);
    Obstacle b = make_obs(2, 0, 0, 2, 2);
    check_obs_obs_api_consistency(st, a, b);
  }

  // Tiny gap vs tiny penetration
  {
    Obstacle a = make_obs(0, 0, 0, 2, 2);
    Obstacle b_gap = make_obs(2 + 1e-6, 0, 0, 2, 2);
    expect_true(st, !a.hasOverlap(b_gap), "tiny gap should not overlap");
    expect_true(st, a.minDistanceTo(b_gap) > 0.0, "tiny gap distance should be > 0");

    Obstacle b_pen = make_obs(2 - 1e-6, 0, 0, 2, 2);
    expect_true(st, a.hasOverlap(b_pen), "tiny penetration should overlap");
    expect_true(st, a.minDistanceTo(b_pen) <= 1e-6, "tiny penetration distance should be ~0");
  }

  // Cache-expiration regression: mutate fields after construction, results must reflect new state
  {
    Obstacle o;
    o.x = 0; o.y = 0; o.heading = 0; o.length = 2; o.width = 2;

    Vec2d p_far{10, 0};
    double d1 = o.minDistanceTo(p_far.x, p_far.y);

    // Move obstacle close to point
    o.x = 9.5; o.y = 0;
    double d2 = o.minDistanceTo(p_far.x, p_far.y);

    expect_true(st, d2 < d1, "after modifying obstacle pose, minDistanceTo should change accordingly (no stale cache)");
  }
}

static void run_random_tests(Stats& st, uint64_t seed, int N) {
  std::mt19937_64 rng(seed);

  // scale scan (similar to collision extreme test)
  std::vector<double> scales = {1e-6, 1.0, 1e6};

  for (double S : scales) {
    for (int i = 0; i < N; ++i) {
      Obstacle a = make_obs(randu(rng, -20, 20) * S,
                            randu(rng, -20, 20) * S,
                            rand_heading(rng),
                            randu(rng, 0.5, 8.0) * S,
                            randu(rng, 0.5, 4.0) * S);

      Obstacle b = make_obs(randu(rng, -20, 20) * S,
                            randu(rng, -20, 20) * S,
                            rand_heading(rng),
                            randu(rng, 0.5, 8.0) * S,
                            randu(rng, 0.5, 4.0) * S);

      // Point tests
      Vec2d p{randu(rng, -25, 25) * S, randu(rng, -25, 25) * S};
      double margin = randu(rng, 0.0, 2.0) * S;

      check_point_api_consistency(st, a, p, margin);

      // Obstacle-obstacle tests
      check_obs_obs_api_consistency(st, a, b);

      // Trajectory test (random pose)
      TrajectoryPoint trj;
      trj.x = randu(rng, -20, 20) * S;
      trj.y = randu(rng, -20, 20) * S;
      trj.heading = rand_heading(rng);

      VehicleParams vp;
      check_trj_distance(st, a, trj, vp);
    }
  }
}

int main(int argc, char** argv) {
  uint64_t seed = 20260103ULL;
  int N = 20000;
  if (argc >= 2) seed = static_cast<uint64_t>(std::stoull(argv[1]));
  if (argc >= 3) N = std::stoi(argv[2]);

  std::cout << "Running strict Obstacle API tests...\n";
  std::cout << "seed=" << seed << " N=" << N << "\n";

  Stats st;
  run_constructed_cases(st);
  run_random_tests(st, seed, N);

  std::cout << "Checks: " << st.checks << "\n";
  if (st.failures == 0) {
    std::cout << "ALL OBSTACLE TESTS PASS\n";
    return 0;
  }
  std::cerr << st.failures << " OBSTACLE TESTS FAILED\n";
  return 1;
}