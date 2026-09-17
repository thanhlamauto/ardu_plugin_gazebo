#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <random>
#include <vector>

#include "uav_navigation_core/mppi/cost_evaluator.hpp"
#include "uav_navigation_core/trajectory_safety_checker.hpp"

namespace uav_navigation_core::mppi {

struct MppiOptimizerConfig {
  double lambda{1.0};
  std::array<double, 4> noise_sigma{{0.8, 0.8, 0.3, 0.3}};
  std::array<double, 4> minimum{{-2.0, -2.0, -1.0, -0.6}};
  std::array<double, 4> maximum{{2.0, 2.0, 1.0, 0.6}};
};

struct MppiOptimizationResult {
  ControlSequence shifted_nominal{};
  ControlBatch perturbed_actions{};
  ControlBatch effective_noise{};
  std::vector<MppiTrajectory> trajectories{};
  std::vector<CostBreakdown> cost_breakdowns{};
  std::vector<double> rollout_costs{};
  std::vector<double> perturbation_costs{};
  std::vector<double> total_costs{};
  std::vector<double> weights{};
  std::vector<bool> safe{};
  ControlSequence updated_nominal{};
  Control first_command{};
};

class GaussianNoiseSampler {
public:
  GaussianNoiseSampler(std::array<double, 4> standard_deviation,
                       std::uint64_t seed);
  ControlBatch Sample(std::size_t samples, std::size_t horizon);

private:
  std::array<double, 4> standard_deviation_;
  std::mt19937_64 engine_;
  std::normal_distribution<double> standard_normal_{0.0, 1.0};
};

class MppiOptimizer {
public:
  explicit MppiOptimizer(MppiOptimizerConfig config = {});

  MppiOptimizationResult OptimizeInjected(
      const IMotionModel &model, const CostEvaluator &cost_evaluator,
      const CostContext &cost_context, const MppiState &initial_state,
      const ControlSequence &nominal_before_shift, const Control &tail_control,
      const ControlBatch &injected_noise, double dt_s,
      const ControlBatch &specific_actions = {},
      const TrajectorySafetyChecker *safety_checker = nullptr,
      const ObstacleMap *safety_obstacles = nullptr) const;

  const MppiOptimizerConfig &config() const { return config_; }

private:
  MppiOptimizerConfig config_;
};

struct RecoveryProposalConfig {
  std::size_t horizon{0};
  std::size_t max_samples{0};
  double dt_s{0.1};
  double reference_speed_m_s{10.0};
  std::array<double, 4> minimum{{-10.0, -10.0, -0.6, -0.6}};
  std::array<double, 4> maximum{{10.0, 10.0, 0.6, 0.6}};
  bool proactive{false};
  bool rejection_recovery{false};
};

ControlBatch BuildRecoveryProposals(const PathReference *reference_path,
                                    double path_progress_m,
                                    double initial_horizontal_speed_m_s,
                                    const RecoveryProposalConfig &config);

} // namespace uav_navigation_core::mppi
