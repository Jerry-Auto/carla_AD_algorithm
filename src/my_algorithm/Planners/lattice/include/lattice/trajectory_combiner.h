#ifndef LATTICE_TRAJECTORY_COMBINER_H_
#define LATTICE_TRAJECTORY_COMBINER_H_

#include <vector>
#include "lattice/lattice_types.h"
#include "lattice/trajectory1d_generator.h"
#include "general_modules/FrenetFrame.h"

namespace AD_algorithm {
namespace planner {

class TrajectoryCombiner {
 public:
  TrajectoryCombiner() = default;

  /**
   * @brief 将一维纵向与横向候选组合为二维笛卡尔轨迹。
   *
   * 使用 `general::FrenetFrame`（基于 `ref_path`）进行 Frenet <-> Cartesian 的转换。
   * 函数沿纵向候选的时间区间以固定时间步长（默认 0.02s）进行采样，计算必要的空间导数（dl/ds, d2l/ds2），
   * 然后调用 `frenet_to_cartesian` 完成坐标转换。
   *
   * @param lon 纵向候选（time->s）
   * @param lat 横向候选（time->d）
   * @param ref_path 用于构建 FrenetFrame 的参考路径（笛卡尔点序列）
   * @return 离散的 `TrajectoryPoint` 向量（笛卡尔坐标）
   */
  std::vector<AD_algorithm::general::TrajectoryPoint> Combine(
      const Trajectory1DGenerator::LonCandidate& lon,
      const Trajectory1DGenerator::LatCandidate& lat,
      const std::vector<PathPoint>& ref_path,
      double s_offset = 0.0) const;
};

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_TRAJECTORY_COMBINER_H_
