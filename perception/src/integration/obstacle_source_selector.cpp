#include "perception/integration/obstacle_source_selector.h"

#include <stdexcept>

namespace perception::integration {
namespace {

// Which phase owns each unimplemented mode. Named in the error so the message tells whoever
// flipped the config what they are actually waiting for.
const char* OwningPhase(ObstacleSourceMode mode) {
  switch (mode) {
    case ObstacleSourceMode::kOracle:            return "P3 (implemented)";
    case ObstacleSourceMode::kShadow:            return "P12 (pipeline orchestration)";
    case ObstacleSourceMode::kCompare:           return "P11/P12 (paired evaluator)";
    case ObstacleSourceMode::kEstimatedFallback: return "P15 (closed loop with fallback)";
    case ObstacleSourceMode::kEstimated:         return "P16 (promotion)";
  }
  return "an unassigned phase";
}

}  // namespace

const char* ToString(ObstacleSourceMode mode) {
  switch (mode) {
    case ObstacleSourceMode::kOracle:            return "oracle";
    case ObstacleSourceMode::kShadow:            return "shadow";
    case ObstacleSourceMode::kCompare:           return "compare";
    case ObstacleSourceMode::kEstimatedFallback: return "estimated_fallback";
    case ObstacleSourceMode::kEstimated:         return "estimated";
  }
  return "unknown";
}

bool ParseObstacleSourceMode(const std::string& name, ObstacleSourceMode& out,
                             std::string& error) {
  if (name == "oracle")             { out = ObstacleSourceMode::kOracle;            return true; }
  if (name == "shadow")             { out = ObstacleSourceMode::kShadow;            return true; }
  if (name == "compare")            { out = ObstacleSourceMode::kCompare;           return true; }
  if (name == "estimated_fallback") { out = ObstacleSourceMode::kEstimatedFallback; return true; }
  if (name == "estimated")          { out = ObstacleSourceMode::kEstimated;         return true; }
  error = "unrecognised perception.mode.name '" + name +
          "'. ObstacleSourceSelector knows oracle|shadow|compare|estimated_fallback|"
          "estimated. If the config loader has gained a mode this selector has not, teach "
          "this function rather than letting the new mode fall through to the oracle path.";
  return false;
}

bool ObstacleSourceSelector::Bind(const PerceptionConfig& config,
                                  const ::dpcbf::DynamicObstacleManager* manager,
                                  std::string& error) {
  bound_ = false;
  oracle_states_.clear();
  last_emitted_count_ = 0;

  if (!ParseObstacleSourceMode(config.mode.name, mode_, error)) {
    return false;
  }
  if (mode_ != ObstacleSourceMode::kOracle) {
    error = std::string("perception.mode.name is '") + ToString(mode_) +
            "', but only 'oracle' is implemented at this phase. " + ToString(mode_) +
            " is owned by " + OwningPhase(mode_) +
            ". Refusing to start rather than silently running the oracle path under an "
            "estimated-path mode name.";
    return false;
  }
  if (manager == nullptr) {
    error = "oracle mode requires a DynamicObstacleManager; got nullptr";
    return false;
  }

  frames_ = config.frames;
  oracle_.BindObstacleSource(manager);
  // Sized for the shipped 90-obstacle arena; grows if a config asks for more.
  oracle_states_.reserve(128);
  bound_ = true;
  return true;
}

bool ObstacleSourceSelector::BindModel(const mjModel* model, std::string& error) {
  if (!bound_) {
    error = "ObstacleSourceSelector::BindModel called before Bind";
    return false;
  }
  return oracle_.BindModel(model, frames_, error);
}

std::vector<::dpcbf::ObstacleState> ObstacleSourceSelector::GetObstacleStates(double now_s) {
  if (!bound_) {
    // Unreachable through the sanctioned startup order (Bind is called before any thread that
    // could reach here). Throwing rather than returning an empty vector is the point: an
    // empty obstacle list is a perfectly plausible "the arena is clear" answer, and DPCBF
    // would happily drive straight through 90 invisible cylinders on it.
    throw std::runtime_error(
        "ObstacleSourceSelector::GetObstacleStates called before Bind. Returning no obstacles "
        "would read as an empty arena to DpcbfSafetyFilter, so this fails loudly instead.");
  }
  if (mode_ != ObstacleSourceMode::kOracle) {
    throw std::runtime_error(std::string("ObstacleSourceSelector: mode '") + ToString(mode_) +
                             "' is not implemented (owned by " + OwningPhase(mode_) + ")");
  }

  oracle_.SampleObstacles(now_s, oracle_states_);

  std::vector<::dpcbf::ObstacleState> states;
  adapters::dpcbf::ToDpcbfObstacleStates(oracle_states_, states);
  last_emitted_count_ = states.size();
  return states;
}

}  // namespace perception::integration
