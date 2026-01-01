#include "gtest/gtest.h"
#include "lattice/trajectory_evaluator.h"
#include "lattice/trajectory1d_generator.h"

using namespace AD_algorithm::planner;

TEST(TrajectoryEvaluatorTest, JointEvaluateProducesReasonableCost) {
  PlannerParams params;
  TrajectoryEvaluator evaluator(params);

  // 构造 lon：线性 s(t) ~ v*t
  Trajectory1DGenerator::LonCandidate lon;
  lon.T = 2.0;
  lon.curve = std::make_shared<AD_algorithm::general::PolynomialCurve>();
  lon.curve->curve_fitting(0.0, 0.0, 5.0, 0.0, 2.0, 10.0, 0.0, 0.0);

  // 构造 lat：小偏移与大偏移两种
  Trajectory1DGenerator::LatCandidate lat_small;
  lat_small.param_s = 5.0;
  lat_small.curve = std::make_shared<AD_algorithm::general::PolynomialCurve>();
  lat_small.curve->curve_fitting(0.0, 0.0, 0.0, 0.0, 5.0, 0.1, 0.0, 0.0);

  Trajectory1DGenerator::LatCandidate lat_large;
  lat_large.param_s = 5.0;
  lat_large.curve = std::make_shared<AD_algorithm::general::PolynomialCurve>();
  lat_large.curve->curve_fitting(0.0, 0.0, 0.0, 0.0, 5.0, 1.0, 0.0, 0.0);

  double c_small = evaluator.EvaluatePair(lon, lat_small, 5.0);
  double c_large = evaluator.EvaluatePair(lon, lat_large, 5.0);

  EXPECT_GT(c_large, c_small);
}
