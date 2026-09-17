#include "uav_navigation_core/mppi/cost_evaluator.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "m4_fixture.hpp"

namespace mppi = uav_navigation_core::mppi;
namespace {
int failures = 0;
void Check(bool condition, const std::string &message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

void Golden() {
  const std::vector<std::string> names{
      "cost_project_open_space",    "cost_project_obstacle",
      "cost_project_path_tracking", "cost_paper_open_space",
      "cost_paper_collision",       "cost_paper_reference_tracking",
      "cost_paper_input_change",    "cost_progress_objective",
      "cost_stopping_cost",         "cost_speed_limit"};
  for (const auto &name : names) {
    const auto fixture = m4_test::Load(name);
    const mppi::MultirotorMotionModel model(fixture.dynamics);
    const auto trajectory =
        mppi::Rollout(model, fixture.initial, fixture.controls, fixture.dt_s);
    const mppi::CostEvaluator evaluator(fixture.cost);
    const auto actual = evaluator.EvaluateTrajectory(
        trajectory, fixture.controls, fixture.context);
    const auto expected = Numbers(fixture.json, "expected_terms_flat");
    const auto terms = m4_test::Terms(actual);
    for (std::size_t i = 0; i < terms.size(); ++i)
      Check(std::abs(terms[i] - expected[i]) < 1e-9,
            name + " cost component " + std::to_string(i));
    Check(std::abs(actual.Total() - Number(fixture.json, "expected_total")) <
              1e-9,
          name + " total cost parity");
  }
}

void EdgeCases() {
  auto fixture = m4_test::Load("cost_project_open_space");
  const mppi::MultirotorMotionModel model(fixture.dynamics);
  const auto trajectory =
      mppi::Rollout(model, fixture.initial, fixture.controls, fixture.dt_s);
  mppi::CostEvaluator evaluator(fixture.cost);
  auto short_actions = fixture.controls;
  short_actions.pop_back();
  bool threw = false;
  try {
    evaluator.EvaluateTrajectory(trajectory, short_actions, fixture.context);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  Check(threw, "cost evaluator rejects inconsistent state/action lengths");
  auto invalid_context = fixture.context;
  invalid_context.goal_enu_m.x = std::numeric_limits<double>::quiet_NaN();
  threw = false;
  try {
    evaluator.EvaluateTrajectory(trajectory, fixture.controls, invalid_context);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  Check(threw, "cost evaluator rejects non-finite context");

  const mppi::PathReference path({{0, 0, 0}, {2, 0, 0}, {2, 2, 0}});
  Check(std::abs(path.Distance({1, 1, 0}) - 1.0) < 1e-12,
        "path distance projects onto segments");
  Check(std::abs(path.Progress({2, 1, 0}) - 3.0) < 1e-12,
        "path progress uses polyline arc length");
  Check(std::abs(path.Sample(3.0).y - 1.0) < 1e-12,
        "path sampling uses arc length");
}
} // namespace

int main() {
  Golden();
  EdgeCases();
  return failures == 0 ? 0 : 1;
}
