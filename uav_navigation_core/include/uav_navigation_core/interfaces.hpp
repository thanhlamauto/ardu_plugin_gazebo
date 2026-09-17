#pragma once

#include "uav_navigation_core/types.hpp"

namespace uav_navigation_core {

class IGlobalPlanner {
 public:
  virtual ~IGlobalPlanner() = default;
  virtual GlobalPlan Plan(const State& start, const Goal& goal,
                          const CostGrid2D& map) = 0;
};

class ILocalPlanner {
 public:
  virtual ~ILocalPlanner() = default;
  virtual LocalPlan Compute(const State& state, const Path& global_path,
                            const ObstacleMap& obstacles) = 0;
  virtual void Reset() = 0;
};

class ISafetyChecker {
 public:
  virtual ~ISafetyChecker() = default;
  virtual SafetyResult Evaluate(const State& initial_state,
                                const Trajectory& trajectory,
                                const ObstacleMap& obstacles) const = 0;
};

class ICommandConditioner {
 public:
  virtual ~ICommandConditioner() = default;
  virtual Control Apply(const State& measured_state,
                        const Control& requested_control) = 0;
  virtual void Reset() = 0;
};

}  // namespace uav_navigation_core
