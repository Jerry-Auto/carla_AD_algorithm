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

#include "general_modules/collision_detection.h"
#include "general_modules/geometry.h"
#include "general_modules/math_tool.h"
#include "general_modules/common_types.h"

using AD_algorithm::general::CollisionDetection;
using AD_algorithm::general::Vec2d;
using AD_algorithm::general::Polygon2d;
using AD_algorithm::general::LineSegment2d;
using AD_algorithm::general::Box2d;
using AD_algorithm::general::Triangle2d;

static constexpr bool kTouchIsOverlap = true;
static constexpr double kEps = 1e-9;

static bool approx_eq(double a, double b, double eps = 1e-9) {
  return std::fabs(a - b) <= eps * (1.0 + std::max(std::fabs(a), std::fabs(b)));
}

static std::ostream& operator<<(std::ostream& os, const Vec2d& p) {
  os << "(" << std::setprecision(17) << p.x << "," << p.y << ")";
  return os;
}

struct TestStats {
  int failures = 0;
  int checks = 0;
};

static void expect_true(TestStats& st, bool cond, const std::string& msg) {
  ++st.checks;
  if (!cond) {
    ++st.failures;
    std::cerr << "FAIL: " << msg << "\n";
  }
}

static void expect_near(TestStats& st, double a, double b, double eps, const std::string& msg) {
  ++st.checks;
  if (!approx_eq(a, b, eps)) {
    ++st.failures;
    std::cerr << "FAIL: " << msg << " expected=" << std::setprecision(17) << b
              << " got=" << a << " diff=" << std::fabs(a - b) << "\n";
  }
}

static std::shared_ptr<LineSegment2d> seg(const Vec2d& a, const Vec2d& b) {
  return std::make_shared<LineSegment2d>(a, b);
}

static std::shared_ptr<Polygon2d> box(double cx, double cy, double L, double W, double h) {
  return std::make_shared<Box2d>(Vec2d{cx, cy}, L, W, h);
}

static std::shared_ptr<Polygon2d> tri(const Vec2d& a, const Vec2d& b, const Vec2d& c) {
  return std::make_shared<Triangle2d>(a, b, c);
}

// Custom convex polygon type to force generic Polygon2d path (NOT Box2d/Triangle2d)
struct ConvexPoly : public Polygon2d {
  ConvexPoly(const std::vector<Vec2d>& points) : Polygon2d(points) {}
  bool is_convex() const override { return true; }
};

static std::shared_ptr<Polygon2d> convex_poly(const std::vector<Vec2d>& pts) {
  return std::make_shared<ConvexPoly>(pts);
}

static void check_invariants_poly_poly(TestStats& st,
                                       const std::shared_ptr<Polygon2d>& A,
                                       const std::shared_ptr<Polygon2d>& B,
                                       const std::string& tag) {
  double dAB = CollisionDetection::distance_to(A, B);
  double dBA = CollisionDetection::distance_to(B, A);
  bool ovAB = CollisionDetection::has_overlap(A, B);
  bool ovBA = CollisionDetection::has_overlap(B, A);

  expect_true(st, std::isfinite(dAB), tag + " distance AB finite");
  expect_true(st, std::isfinite(dBA), tag + " distance BA finite");
  expect_true(st, dAB >= -1e-12, tag + " distance AB non-negative");
  expect_true(st, dBA >= -1e-12, tag + " distance BA non-negative");
  expect_near(st, dAB, dBA, 1e-9, tag + " symmetry distance(A,B)=distance(B,A)");
  expect_true(st, ovAB == ovBA, tag + " symmetry overlap(A,B)=overlap(B,A)");

  if (ovAB) {
    expect_true(st, dAB <= 1e-6, tag + " overlap => distance ~0");
  }
  if (kTouchIsOverlap && dAB <= 1e-9) {
    // For touch policy, distance==0 should mean overlap (within tolerance)
    expect_true(st, ovAB, tag + " distance ~0 => overlap (touch counts)");
  }
}

static void run_constructed_edge_cases(TestStats& st) {
  // 1) Box-box: tiny gap vs touch vs tiny penetration
  {
    auto A = box(0, 0, 2, 2, 0);
    // Touch at x=1
    auto B_touch = box(2, 0, 2, 2, 0);
    check_invariants_poly_poly(st, A, B_touch, "box-box touch");

    // Tiny gap
    auto B_gap = box(2 + 1e-9, 0, 2, 2, 0);
    double d = CollisionDetection::distance_to(A, B_gap);
    expect_true(st, d > 0.0, "box-box tiny gap distance > 0");
    expect_true(st, !CollisionDetection::has_overlap(A, B_gap), "box-box tiny gap should not overlap");

    // Tiny penetration
    auto B_pen = box(2 - 1e-9, 0, 2, 2, 0);
    expect_true(st, CollisionDetection::has_overlap(A, B_pen), "box-box tiny penetration should overlap");
    expect_near(st, CollisionDetection::distance_to(A, B_pen), 0.0, 1e-6, "box-box tiny penetration distance ~0");
  }

  // 2) Box-segment: collinear with edge, partial overlap range
  // (This should be distance 0; overlap depends on your has_overlap(Box2d, LineSegment2d) which is SAT-based)
  {
    auto A = box(0, 0, 2, 2, 0);
    auto s = seg(Vec2d{-0.5, 1.0}, Vec2d{0.5, 1.0}); // lies on top edge
    double d = CollisionDetection::distance_to(A, s);
    expect_near(st, d, 0.0, 1e-12, "box-seg collinear-on-edge distance 0");
    if (kTouchIsOverlap) {
      expect_true(st, CollisionDetection::has_overlap(A, s), "box-seg collinear-on-edge overlap true");
    }
  }

  // 3) Generic polygon-segment: force the generic polygon path that uses segments_intersect
  // Build a square as ConvexPoly (NOT Box2d) so CollisionDetection::has_overlap(poly, seg) will call
  // has_overlap_aabb + has_overlap_polygon_line_segment + segments_intersect(...)
  {
    auto P = convex_poly({{-1,-1},{1,-1},{1,1},{-1,1}});

    // (a) Segment touches at a vertex
    auto s_vertex_touch = seg(Vec2d{1, 1}, Vec2d{2, 2});
    double d1 = CollisionDetection::distance_to(P, s_vertex_touch);
    expect_near(st, d1, 0.0, 1e-12, "generic poly - vertex touch segment distance 0");
    if (kTouchIsOverlap) {
      bool ov = CollisionDetection::has_overlap(P, s_vertex_touch);
      expect_true(st, ov, "generic poly - vertex touch overlap should be true (touch counts)");
    }

    // (b) Segment collinear with an edge and overlapping
    auto s_collinear = seg(Vec2d{-0.5, 1.0}, Vec2d{0.5, 1.0});
    double d2 = CollisionDetection::distance_to(P, s_collinear);
    expect_near(st, d2, 0.0, 1e-12, "generic poly - collinear overlap distance 0");
    if (kTouchIsOverlap) {
      bool ov = CollisionDetection::has_overlap(P, s_collinear);
      expect_true(st, ov, "generic poly - collinear overlap should be true (touch counts)");
    }

    // (c) Segment tangent (touching but not crossing)
    auto s_tangent = seg(Vec2d{-3, 1.0}, Vec2d{3, 1.0});
    double d3 = CollisionDetection::distance_to(P, s_tangent);
    expect_near(st, d3, 0.0, 1e-12, "generic poly - tangent distance 0");
    if (kTouchIsOverlap) {
      bool ov = CollisionDetection::has_overlap(P, s_tangent);
      expect_true(st, ov, "generic poly - tangent overlap should be true (touch counts)");
    }
  }

  // 4) Triangle degeneracy: almost collinear triangle + point query (should be finite)
  {
    auto T = tri(Vec2d{0,0}, Vec2d{1e-12, 0}, Vec2d{2e-12, 1e-18});
    Vec2d p{0,0};
    double d = CollisionDetection::distance_to(T, p);
    expect_true(st, std::isfinite(d), "degenerate-ish triangle point distance should be finite");
  }
}

static double randu(std::mt19937_64& rng, double lo, double hi) {
  std::uniform_real_distribution<double> dist(lo, hi);
  return dist(rng);
}

static double rand_heading(std::mt19937_64& rng) {
  return randu(rng, -M_PI, M_PI);
}

static void run_scale_scan_random(TestStats& st, uint64_t seed, int N) {
  std::mt19937_64 rng(seed);

  const std::vector<double> scales = {1e-6, 1.0, 1e6};

  for (double S : scales) {
    for (int i = 0; i < N; ++i) {
      double ax = randu(rng, -20, 20) * S;
      double ay = randu(rng, -20, 20) * S;
      double bx = randu(rng, -20, 20) * S;
      double by = randu(rng, -20, 20) * S;

      double aL = randu(rng, 0.5, 8.0) * S;
      double aW = randu(rng, 0.5, 4.0) * S;
      double bL = randu(rng, 0.5, 8.0) * S;
      double bW = randu(rng, 0.5, 4.0) * S;

      double ah = rand_heading(rng);
      double bh = rand_heading(rng);

      auto A = box(ax, ay, aL, aW, ah);
      auto B = box(bx, by, bL, bW, bh);

      // Invariants
      check_invariants_poly_poly(st, A, B, "scale=" + std::to_string(S) + " rand box-box");

      // Poly-point
      Vec2d p{randu(rng, -25, 25) * S, randu(rng, -25, 25) * S};
      double dp = CollisionDetection::distance_to(A, p);
      expect_true(st, std::isfinite(dp), "poly-point distance finite (scale=" + std::to_string(S) + ")");
      expect_true(st, dp >= -1e-12, "poly-point distance non-negative");

      // Poly-seg
      Vec2d s1{randu(rng, -25, 25) * S, randu(rng, -25, 25) * S};
      Vec2d s2{randu(rng, -25, 25) * S, randu(rng, -25, 25) * S};
      auto sg = seg(s1, s2);
      double ds = CollisionDetection::distance_to(A, sg);
      expect_true(st, std::isfinite(ds), "poly-seg distance finite (scale=" + std::to_string(S) + ")");
      expect_true(st, ds >= -1e-12, "poly-seg distance non-negative");

      // If overlap => segment distance must be 0
      if (CollisionDetection::has_overlap(A, sg)) {
        expect_true(st, ds <= 1e-6 * S + 1e-9, "overlap poly-seg => distance ~0");
      }
    }
  }
}

int main(int argc, char** argv) {
  uint64_t seed = 20260103ULL;
  int N = 50000;
  if (argc >= 2) seed = static_cast<uint64_t>(std::stoull(argv[1]));
  if (argc >= 3) N = std::stoi(argv[2]);

  TestStats st;
  std::cout << "Running EXTREME CollisionDetection tests...\n";
  std::cout << "touch_is_overlap=" << (kTouchIsOverlap ? "true" : "false")
            << " seed=" << seed << " N=" << N << "\n";

  run_constructed_edge_cases(st);
  run_scale_scan_random(st, seed, N);

  std::cout << "Checks: " << st.checks << "\n";
  if (st.failures == 0) {
    std::cout << "ALL EXTREME TESTS PASS\n";
    return 0;
  }
  std::cerr << st.failures << " EXTREME TESTS FAILED\n";
  return 1;
}