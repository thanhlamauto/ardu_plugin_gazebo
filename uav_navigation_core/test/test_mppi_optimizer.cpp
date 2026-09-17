#include "uav_navigation_core/mppi/optimizer.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "m4_fixture.hpp"

namespace core = uav_navigation_core;
namespace mppi = uav_navigation_core::mppi;
namespace {
int failures = 0;
void Check(bool condition, const std::string &message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}
void CheckVector(const std::vector<double> &actual,
                 const std::vector<double> &expected, double tolerance,
                 const std::string &message) {
  Check(actual.size() == expected.size(), message + " size");
  const auto count = std::min(actual.size(), expected.size());
  for (std::size_t i = 0; i < count; ++i)
    Check(std::abs(actual[i] - expected[i]) < tolerance,
          message + " at " + std::to_string(i));
}
std::vector<double> Flatten(const mppi::ControlBatch &batch) {
  std::vector<double> result;
  for (const auto &sequence : batch) {
    const auto flat = m4_test::Flatten(sequence);
    result.insert(result.end(), flat.begin(), flat.end());
  }
  return result;
}

void GoldenOptimizer(const std::string &name) {
  const auto fixture = m4_test::Load(name);
  const auto &json = fixture.json;
  const auto nominal =
      m4_test::ControlArray(Numbers(json, "nominal_before_shift_flat"));
  const auto tail_values = Numbers(json, "tail_control_flat");
  const auto tail = mppi_test::ControlAt(tail_values, 0);
  const auto noise_flat = Numbers(json, "injected_noise_flat");
  const auto expected_weights = Numbers(json, "expected_weights_flat");
  const std::size_t samples = expected_weights.size();
  const std::size_t horizon = nominal.size();
  mppi::ControlBatch noise(samples);
  for (std::size_t sample = 0; sample < samples; ++sample)
    for (std::size_t t = 0; t < horizon; ++t)
      noise[sample].push_back(
          mppi_test::ControlAt(noise_flat, (sample * horizon + t) * 4));

  mppi::MppiOptimizerConfig config;
  config.lambda = Number(json, "lambda_");
  const auto sigma = Numbers(json, "noise_sigma");
  for (std::size_t i = 0; i < 4; ++i)
    config.noise_sigma[i] = sigma[i];
  config.minimum = {{-fixture.dynamics.vmax_m_s, -fixture.dynamics.vmax_m_s,
                     -fixture.dynamics.vzmax_m_s,
                     -fixture.dynamics.yaw_rate_max_rad_s}};
  config.maximum = {{fixture.dynamics.vmax_m_s, fixture.dynamics.vmax_m_s,
                     fixture.dynamics.vzmax_m_s,
                     fixture.dynamics.yaw_rate_max_rad_s}};
  const mppi::MultirotorMotionModel model(fixture.dynamics);
  const mppi::CostEvaluator evaluator(fixture.cost);
  const mppi::MppiOptimizer optimizer(config);
  const auto actual = optimizer.OptimizeInjected(
      model, evaluator, fixture.context, fixture.initial, nominal, tail, noise,
      fixture.dt_s);

  CheckVector(m4_test::Flatten(actual.shifted_nominal),
              Numbers(json, "expected_shifted_nominal_flat"), 1e-12,
              name + " shifted nominal");
  CheckVector(Flatten(actual.perturbed_actions),
              Numbers(json, "expected_perturbed_actions_flat"), 1e-12,
              name + " bounded actions");
  CheckVector(Flatten(actual.effective_noise),
              Numbers(json, "expected_effective_noise_flat"), 1e-12,
              name + " effective noise");
  std::vector<double> states;
  for (const auto &trajectory : actual.trajectories)
    for (std::size_t t = 1; t < trajectory.points.size(); ++t) {
      const auto flat = mppi_test::Flatten(trajectory.points[t].state, true);
      states.insert(states.end(), flat.begin(), flat.end());
    }
  CheckVector(states, Numbers(json, "expected_states_flat"), 1e-9,
              name + " rollout states");
  CheckVector(actual.rollout_costs,
              Numbers(json, "expected_rollout_costs_flat"), 1e-8,
              name + " rollout costs");
  CheckVector(actual.perturbation_costs,
              Numbers(json, "expected_perturbation_costs_flat"), 1e-9,
              name + " perturbation costs");
  CheckVector(actual.total_costs, Numbers(json, "expected_total_costs_flat"),
              1e-8, name + " total costs");
  CheckVector(actual.weights, expected_weights, 1e-9,
              name + " importance weights");
  CheckVector(m4_test::Flatten(actual.updated_nominal),
              Numbers(json, "expected_updated_nominal_flat"), 1e-9,
              name + " updated nominal");
  CheckVector(m4_test::Flatten({actual.first_command}),
              Numbers(json, "expected_first_command_flat"), 1e-9,
              name + " first command");
}

void NativeSampler() {
  const std::array<double, 4> sigma{{.8, .5, .3, .2}};
  mppi::GaussianNoiseSampler first(sigma, 42), second(sigma, 42),
      other(sigma, 43);
  const auto a = first.Sample(20000, 1);
  const auto b = second.Sample(20000, 1);
  const auto c = other.Sample(1, 1);
  Check(Flatten(a) == Flatten(b),
        "native sampler repeats exactly for one C++ seed");
  Check(Flatten(mppi::ControlBatch{a.front()}) != Flatten(c),
        "native sampler changes for another seed");
  std::array<double, 4> sum{}, square{};
  for (const auto &sequence : a) {
    const auto values = m4_test::Flatten(sequence);
    for (std::size_t i = 0; i < 4; ++i) {
      sum[i] += values[i];
      square[i] += values[i] * values[i];
      Check(std::isfinite(values[i]), "native sampler output is finite");
    }
  }
  for (std::size_t i = 0; i < 4; ++i) {
    const double mean = sum[i] / a.size();
    const double deviation = std::sqrt(square[i] / a.size() - mean * mean);
    Check(std::abs(mean) < .03, "native sampler mean is near zero");
    Check(std::abs(deviation - sigma[i]) < .03,
          "native sampler standard deviation matches configuration");
  }
}

class DirectMotionModel final : public mppi::IMotionModel {
public:
  mppi::MppiState Step(const mppi::MppiState &state,
                       const core::Control &control,
                       double dt_s) const override {
    mppi::MppiState next = state;
    next.stamp_ns += static_cast<core::TimeNs>(std::round(dt_s * 1e9));
    next.velocity_enu_m_s = control.velocity_enu_m_s;
    next.position_enu_m.x += control.velocity_enu_m_s.x * dt_s;
    next.position_enu_m.y += control.velocity_enu_m_s.y * dt_s;
    next.position_enu_m.z += control.velocity_enu_m_s.z * dt_s;
    next.yaw_enu_rad += control.yaw_rate_enu_rad_s * dt_s;
    next.applied_control = control;
    return next;
  }
};

void SafetyWeighting() {
  mppi::MppiOptimizerConfig optimizer_config;
  optimizer_config.minimum = {{-2, -2, -1, -1}};
  optimizer_config.maximum = {{2, 2, 1, 1}};
  mppi::MppiCostConfig cost_config;
  cost_config.w_goal = 0;
  cost_config.w_terminal = 0;
  cost_config.w_obstacle = 0;
  cost_config.w_effort = 0;
  cost_config.w_smoothness = 0;
  cost_config.w_yaw = 0;
  const mppi::MppiOptimizer optimizer(optimizer_config);
  const mppi::CostEvaluator evaluator(cost_config);
  DirectMotionModel model;
  mppi::MppiState initial;
  mppi::CostContext context;
  context.initial_state = initial;
  mppi::ControlSequence nominal(2), noise_positive(2), noise_negative(2);
  for (auto &control : noise_positive)
    control.velocity_enu_m_s.x = 1.0;
  for (auto &control : noise_negative)
    control.velocity_enu_m_s.x = -1.0;
  core::TrajectorySafetyConfig safety_config;
  safety_config.collision_radius_m = .1;
  safety_config.braking_acceleration_m_s2 = 10.;
  safety_config.stopping_delay_s = 0.;
  safety_config.stopping_clearance_m = .1;
  core::TrajectorySafetyChecker checker(safety_config);
  core::ObstacleMap obstacles;
  obstacles.points_enu_m.push_back({1., 0., 0.});
  auto result = optimizer.OptimizeInjected(
      model, evaluator, context, initial, nominal, {},
      {noise_positive, noise_negative}, 1., {}, &checker, &obstacles);
  Check(!result.safe[0] && result.safe[1],
        "M2 safety predicate masks only unsafe MPPI samples");
  Check(result.weights[0] == 0. && std::abs(result.weights[1] - 1.) < 1e-12,
        "all weight mass remains on feasible samples");
  result = optimizer.OptimizeInjected(
      model, evaluator, context, initial, nominal, {},
      {noise_positive, noise_positive}, 1., {}, &checker, &obstacles);
  Check(result.weights[0] == 0. && result.weights[1] == 0.,
        "zero-safe-sample case returns all-zero weights");
}

void RecoveryProposals() {
  const auto json = ReadFixture("mppi_m4/recovery_proposals.json");
  const auto path_points = m4_test::Vec3Array(Numbers(json, "path_flat"));
  const mppi::PathReference path(path_points);
  mppi::RecoveryProposalConfig config;
  config.horizon = static_cast<std::size_t>(Number(json, "horizon"));
  config.max_samples = static_cast<std::size_t>(Number(json, "samples"));
  config.dt_s = Number(json, "dt");
  config.reference_speed_m_s = Number(json, "reference_speed_m_s");
  const double vmax = Number(json, "vmax");
  const double vzmax = Number(json, "vzmax");
  const double yawmax = Number(json, "yaw_rate_max");
  config.minimum = {{-vmax, -vmax, -vzmax, -yawmax}};
  config.maximum = {{vmax, vmax, vzmax, yawmax}};
  config.proactive = true;
  const auto actual = mppi::BuildRecoveryProposals(
      &path, Number(json, "path_progress_m"),
      Number(json, "initial_horizontal_speed_m_s"), config);
  Check(actual.size() ==
            static_cast<std::size_t>(Number(json, "expected_count")),
        "recovery proposal count matches Python");
  CheckVector(Flatten(actual), Numbers(json, "expected_proposals_flat"), 1e-9,
              "recovery proposals match Python");
}
} // namespace

int main() {
  GoldenOptimizer("optimizer_project");
  GoldenOptimizer("optimizer_paper");
  NativeSampler();
  SafetyWeighting();
  RecoveryProposals();
  return failures == 0 ? 0 : 1;
}
