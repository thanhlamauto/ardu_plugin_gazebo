#include "uav_navigation_core/mppi/optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace uav_navigation_core::mppi {
namespace {
std::array<double, 4> Components(const Control &control) {
  return {control.velocity_enu_m_s.x, control.velocity_enu_m_s.y,
          control.velocity_enu_m_s.z, control.yaw_rate_enu_rad_s};
}
Control FromComponents(const std::array<double, 4> &values) {
  return {{values[0], values[1], values[2]}, values[3]};
}
Control Add(const Control &a, const Control &b) {
  const auto av = Components(a);
  const auto bv = Components(b);
  std::array<double, 4> result{};
  for (std::size_t i = 0; i < 4; ++i)
    result[i] = av[i] + bv[i];
  return FromComponents(result);
}
Control Sub(const Control &a, const Control &b) {
  const auto av = Components(a);
  const auto bv = Components(b);
  std::array<double, 4> result{};
  for (std::size_t i = 0; i < 4; ++i)
    result[i] = av[i] - bv[i];
  return FromComponents(result);
}
Control Clamp(const Control &control, const std::array<double, 4> &minimum,
              const std::array<double, 4> &maximum) {
  auto values = Components(control);
  for (std::size_t i = 0; i < 4; ++i)
    values[i] = std::clamp(values[i], minimum[i], maximum[i]);
  return FromComponents(values);
}
bool Finite(const Control &control) {
  const auto values = Components(control);
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}
Trajectory ToSafetyTrajectory(const MppiTrajectory &input) {
  Trajectory output;
  output.points.reserve(input.points.size());
  for (const auto &point : input.points) {
    TrajectoryPoint converted;
    converted.time_from_start_s = point.time_from_start_s;
    converted.state.stamp_ns = point.state.stamp_ns;
    converted.state.position_enu_m = point.state.position_enu_m;
    converted.state.velocity_enu_m_s = point.state.velocity_enu_m_s;
    converted.state.acceleration_enu_m_s2 =
        point.state.acceleration_memory_enu_m_s2;
    converted.state.yaw_enu_rad = point.state.yaw_enu_rad;
    converted.control = point.requested_control;
    output.points.push_back(converted);
  }
  return output;
}
State ToSafetyState(const MppiState &input) {
  State output;
  output.stamp_ns = input.stamp_ns;
  output.position_enu_m = input.position_enu_m;
  output.velocity_enu_m_s = input.velocity_enu_m_s;
  output.acceleration_enu_m_s2 = input.acceleration_memory_enu_m_s2;
  output.yaw_enu_rad = input.yaw_enu_rad;
  return output;
}
// Maps the safety checker verdict onto the diagnostic rejection categories.
// The checker already reports the first failing predicate, so this mapping
// preserves the deterministic "first failure" policy and never double counts.
RejectionReason ClassifyRejection(const SafetyResult &safety) {
  if (safety.safe || safety.reason == "clear_trajectory")
    return RejectionReason::kSafe;
  if (safety.reason == "swept_collision")
    return RejectionReason::kSweptCollision;
  if (safety.reason == "known_map_collision")
    return RejectionReason::kStaticCollision;
  if (safety.reason == "predicted_stopping_clearance")
    return RejectionReason::kStoppingDistance;
  if (safety.reason.rfind("invalid", 0) == 0)
    return RejectionReason::kNonFinite;
  return RejectionReason::kOther;
}
void AccumulateRejection(RejectionCounts &counts, RejectionReason reason) {
  counts.total += 1;
  switch (reason) {
  case RejectionReason::kSafe:
    counts.safe += 1;
    break;
  case RejectionReason::kNonFinite:
    counts.reject_non_finite += 1;
    break;
  case RejectionReason::kSweptCollision:
    counts.reject_swept_collision += 1;
    break;
  case RejectionReason::kStaticCollision:
    counts.reject_static_collision += 1;
    break;
  case RejectionReason::kDynamicCollision:
    counts.reject_dynamic_collision += 1;
    break;
  case RejectionReason::kStoppingDistance:
    counts.reject_stopping_distance += 1;
    break;
  case RejectionReason::kOther:
    counts.reject_other += 1;
    break;
  }
}
} // namespace

GaussianNoiseSampler::GaussianNoiseSampler(
    std::array<double, 4> standard_deviation, std::uint64_t seed)
    : standard_deviation_(standard_deviation), engine_(seed) {
  for (double value : standard_deviation_)
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument(
          "MPPI noise deviations must be finite and nonnegative");
}

ControlBatch GaussianNoiseSampler::Sample(std::size_t samples,
                                          std::size_t horizon) {
  ControlBatch result(samples, ControlSequence(horizon));
  for (auto &sequence : result)
    for (auto &control : sequence) {
      std::array<double, 4> values{};
      for (std::size_t i = 0; i < 4; ++i)
        values[i] = standard_deviation_[i] * standard_normal_(engine_);
      control = FromComponents(values);
    }
  return result;
}

MppiOptimizer::MppiOptimizer(MppiOptimizerConfig config)
    : config_(std::move(config)) {
  if (!std::isfinite(config_.lambda) || config_.lambda <= 0.0)
    throw std::invalid_argument("MPPI lambda must be finite and positive");
  for (std::size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(config_.noise_sigma[i]) ||
        config_.noise_sigma[i] <= 0.0 || !std::isfinite(config_.minimum[i]) ||
        !std::isfinite(config_.maximum[i]) ||
        config_.minimum[i] > config_.maximum[i])
      throw std::invalid_argument("MPPI optimizer limits are invalid");
  }
}

MppiOptimizationResult MppiOptimizer::OptimizeInjected(
    const IMotionModel &model, const CostEvaluator &cost_evaluator,
    const CostContext &cost_context, const MppiState &initial_state,
    const ControlSequence &nominal_before_shift, const Control &tail_control,
    const ControlBatch &injected_noise, double dt_s,
    const ControlBatch &specific_actions,
    const TrajectorySafetyChecker *safety_checker,
    const ObstacleMap *safety_obstacles,
    const std::vector<ProposalSource> *proposal_sources) const {
  if (nominal_before_shift.empty() || injected_noise.empty())
    throw std::invalid_argument("MPPI optimizer needs a horizon and samples");
  const std::size_t horizon = nominal_before_shift.size();
  for (const auto &control : nominal_before_shift)
    if (!Finite(control))
      throw std::invalid_argument("MPPI nominal controls must be finite");
  if (!Finite(tail_control))
    throw std::invalid_argument("MPPI tail control must be finite");
  for (const auto &sequence : injected_noise) {
    if (sequence.size() != horizon)
      throw std::invalid_argument("MPPI noise shape does not match horizon");
    for (const auto &control : sequence)
      if (!Finite(control))
        throw std::invalid_argument("MPPI noise must be finite");
  }
  if (specific_actions.size() > injected_noise.size())
    throw std::invalid_argument("MPPI specific proposals exceed sample count");
  for (const auto &sequence : specific_actions)
    if (sequence.size() != horizon)
      throw std::invalid_argument("MPPI proposal shape does not match horizon");
  if (proposal_sources != nullptr &&
      proposal_sources->size() != specific_actions.size())
    throw std::invalid_argument(
        "MPPI proposal source labels do not match proposals");
  if ((safety_checker == nullptr) != (safety_obstacles == nullptr))
    throw std::invalid_argument(
        "MPPI safety checker and obstacle map must be paired");

  MppiOptimizationResult result;
  result.shifted_nominal.reserve(horizon);
  for (std::size_t t = 1; t < horizon; ++t)
    result.shifted_nominal.push_back(nominal_before_shift[t]);
  result.shifted_nominal.push_back(tail_control);
  result.perturbed_actions.resize(injected_noise.size());
  result.effective_noise.resize(injected_noise.size());
  for (std::size_t sample = 0; sample < injected_noise.size(); ++sample) {
    auto &actions = result.perturbed_actions[sample];
    actions.resize(horizon);
    if (sample < specific_actions.size()) {
      actions = specific_actions[sample];
    } else {
      for (std::size_t t = 0; t < horizon; ++t)
        actions[t] = Add(result.shifted_nominal[t], injected_noise[sample][t]);
    }
    for (std::size_t t = 0; t < horizon; ++t)
      actions[t] = Clamp(actions[t], config_.minimum, config_.maximum);
    auto &effective = result.effective_noise[sample];
    effective.resize(horizon);
    for (std::size_t t = 0; t < horizon; ++t)
      effective[t] = Sub(actions[t], result.shifted_nominal[t]);
  }

  result.sample_sources.resize(injected_noise.size(), ProposalSource::kGaussian);
  for (std::size_t sample = 0; sample < injected_noise.size(); ++sample) {
    if (sample < specific_actions.size()) {
      result.sample_sources[sample] =
          proposal_sources != nullptr
              ? (*proposal_sources)[sample]
              : ProposalSource::kSpecificAction;
    }
  }

  const auto rollout_started = std::chrono::steady_clock::now();
  result.trajectories =
      RolloutBatch(model, initial_state, result.perturbed_actions, dt_s);
  result.rollout_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - rollout_started)
          .count();
  const auto cost_started = std::chrono::steady_clock::now();
  result.cost_breakdowns.reserve(result.trajectories.size());
  result.rollout_costs.reserve(result.trajectories.size());
  result.perturbation_costs.reserve(result.trajectories.size());
  result.total_costs.reserve(result.trajectories.size());
  result.safe.assign(result.trajectories.size(), true);
  result.rejection_reasons.assign(result.trajectories.size(),
                                  RejectionReason::kSafe);
  // The obstacle cloud and known-geometry filtering are independent of any one
  // candidate trajectory, so they are prepared once for the whole solve. This
  // is semantically identical to the per-trajectory preparation it replaces.
  std::optional<PreparedCollisionEnvironment> prepared_environment;
  if (safety_checker) {
    const auto prepare_started = std::chrono::steady_clock::now();
    prepared_environment = safety_checker->Prepare(*safety_obstacles);
    result.safety_time_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - prepare_started)
            .count();
  }
  for (std::size_t sample = 0; sample < result.trajectories.size(); ++sample) {
    const auto breakdown = cost_evaluator.EvaluateTrajectory(
        result.trajectories[sample], result.perturbed_actions[sample],
        cost_context);
    result.cost_breakdowns.push_back(breakdown);
    result.rollout_costs.push_back(breakdown.Total());
    double perturbation = 0.0;
    for (std::size_t t = 0; t < horizon; ++t) {
      const auto nominal = Components(result.shifted_nominal[t]);
      const auto noise = Components(result.effective_noise[sample][t]);
      for (std::size_t i = 0; i < 4; ++i)
        perturbation += nominal[i] * config_.lambda * noise[i] /
                        (config_.noise_sigma[i] * config_.noise_sigma[i]);
    }
    result.perturbation_costs.push_back(perturbation);
    result.total_costs.push_back(breakdown.Total() + perturbation);
    if (safety_checker) {
      const auto safety_started = std::chrono::steady_clock::now();
      const auto safety = safety_checker->Evaluate(
          ToSafetyState(initial_state),
          ToSafetyTrajectory(result.trajectories[sample]),
          *prepared_environment);
      result.safe[sample] = safety.safe;
      const auto reason = ClassifyRejection(safety);
      result.rejection_reasons[sample] = reason;
      AccumulateRejection(result.rejection_counts, reason);
      result.safety_time_ms +=
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - safety_started)
              .count();
    }
  }
  if (!safety_checker) {
    result.rejection_counts.total = result.trajectories.size();
    result.rejection_counts.safe = result.trajectories.size();
  }
  for (std::size_t sample = 0; sample < result.trajectories.size(); ++sample) {
    auto &stats = result.proposal_source_stats[static_cast<std::size_t>(
        result.sample_sources[sample])];
    stats.generated += 1;
    if (result.safe[sample]) {
      stats.safe += 1;
      stats.best_cost = std::min(stats.best_cost, result.total_costs[sample]);
    }
  }
  result.cost_time_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - cost_started)
                            .count() -
                        result.safety_time_ms;

  result.weights.assign(result.total_costs.size(), 0.0);
  bool any_safe = safety_checker == nullptr;
  if (safety_checker)
    any_safe = std::any_of(result.safe.begin(), result.safe.end(),
                           [](bool value) { return value; });
  if (any_safe) {
    double beta = std::numeric_limits<double>::infinity();
    for (std::size_t sample = 0; sample < result.total_costs.size(); ++sample)
      if (result.safe[sample])
        beta = std::min(beta, result.total_costs[sample]);
    double eta = 0.0;
    for (std::size_t sample = 0; sample < result.total_costs.size(); ++sample) {
      if (result.safe[sample]) {
        result.weights[sample] =
            std::exp(-(result.total_costs[sample] - beta) / config_.lambda);
        eta += result.weights[sample];
      }
    }
    if (eta > 0.0)
      for (double &weight : result.weights)
        weight /= eta;
  }

  result.updated_nominal = result.shifted_nominal;
  for (std::size_t t = 0; t < horizon; ++t) {
    auto updated = Components(result.updated_nominal[t]);
    for (std::size_t sample = 0; sample < result.weights.size(); ++sample) {
      const auto noise = Components(result.effective_noise[sample][t]);
      for (std::size_t i = 0; i < 4; ++i)
        updated[i] += result.weights[sample] * noise[i];
    }
    result.updated_nominal[t] = FromComponents(updated);
  }
  result.first_command = result.updated_nominal.front();
  return result;
}

RecoveryProposals BuildRecoveryProposalsWithSources(
    const PathReference *reference_path, double path_progress_m,
    double initial_horizontal_speed_m_s, const RecoveryProposalConfig &config) {
  if (config.horizon == 0 || config.max_samples == 0 ||
      !std::isfinite(config.dt_s) || config.dt_s <= 0.0 ||
      !std::isfinite(config.reference_speed_m_s) ||
      config.reference_speed_m_s <= 0.0 || !std::isfinite(path_progress_m) ||
      !std::isfinite(initial_horizontal_speed_m_s) ||
      initial_horizontal_speed_m_s < 0.0)
    throw std::invalid_argument(
        "MPPI recovery proposal configuration is invalid");
  for (std::size_t i = 0; i < 4; ++i)
    if (!std::isfinite(config.minimum[i]) ||
        !std::isfinite(config.maximum[i]) ||
        config.minimum[i] > config.maximum[i])
      throw std::invalid_argument("MPPI recovery proposal bounds are invalid");
  if (!config.proactive && !config.rejection_recovery)
    return {};
  RecoveryProposals output;
  // The first sequence is the deliberate zero/hold specific action. It is
  // classified as a specific action because it is not derived from the path.
  output.proposals.push_back(ControlSequence(config.horizon));
  output.sources.push_back(ProposalSource::kSpecificAction);
  const auto append_path_speeds = [&](const std::vector<double> &speeds,
                                      ProposalSource source) {
    ControlSequence actions(config.horizon);
    double progress = path_progress_m;
    Vec3 previous = reference_path->Sample(progress);
    for (std::size_t t = 0; t < config.horizon; ++t) {
      progress += speeds[t] * config.dt_s;
      const Vec3 current = reference_path->Sample(progress);
      actions[t].velocity_enu_m_s = {(current.x - previous.x) / config.dt_s,
                                     (current.y - previous.y) / config.dt_s,
                                     (current.z - previous.z) / config.dt_s};
      previous = current;
      actions[t] = Clamp(actions[t], config.minimum, config.maximum);
    }
    output.proposals.push_back(std::move(actions));
    output.sources.push_back(source);
  };
  if (reference_path) {
    for (double speed : {0.5, 1.0, 2.0, 4.0})
      append_path_speeds(std::vector<double>(config.horizon, speed),
                         ProposalSource::kReferenceProposal);
  }
  if (config.proactive && reference_path) {
    for (double target : {1.0, 2.0, 4.0, 6.0, config.reference_speed_m_s})
      for (double deceleration : {1.5, 3.0}) {
        std::vector<double> speeds;
        speeds.reserve(config.horizon);
        double speed = initial_horizontal_speed_m_s;
        for (std::size_t t = 0; t < config.horizon; ++t) {
          speed += std::clamp(target - speed, -deceleration * config.dt_s,
                              deceleration * config.dt_s);
          speeds.push_back(speed);
        }
        const ProposalSource source =
            target < initial_horizontal_speed_m_s
                ? ProposalSource::kBrakingProposal
                : ProposalSource::kRecoveryProposal;
        append_path_speeds(speeds, source);
      }
  }
  if (output.proposals.size() > config.max_samples) {
    output.proposals.resize(config.max_samples);
    output.sources.resize(config.max_samples);
  }
  return output;
}

ControlBatch BuildRecoveryProposals(const PathReference *reference_path,
                                    double path_progress_m,
                                    double initial_horizontal_speed_m_s,
                                    const RecoveryProposalConfig &config) {
  return BuildRecoveryProposalsWithSources(reference_path, path_progress_m,
                                           initial_horizontal_speed_m_s, config)
      .proposals;
}

} // namespace uav_navigation_core::mppi
