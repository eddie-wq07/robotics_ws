#pragma once

#include <cmath>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace flexiv_hardware {
enum class ArmCommandMode { None, Position, PositionVelocity, Velocity, Torque, Cartesian };

inline std::optional<ArmCommandMode> select_arm_mode(
    const std::set<std::string>& requested, const std::set<std::string>& position,
    const std::set<std::string>& velocity, const std::set<std::string>& torque,
    const std::set<std::string>& cartesian, bool has_cartesian) {
  auto both = position;
  both.insert(velocity.begin(), velocity.end());
  if (requested.empty()) return ArmCommandMode::None;
  if (requested == position) return ArmCommandMode::Position;
  if (requested == both) return ArmCommandMode::PositionVelocity;
  if (requested == velocity) return ArmCommandMode::Velocity;
  if (requested == torque) return ArmCommandMode::Torque;
  if (has_cartesian && requested == cartesian) return ArmCommandMode::Cartesian;
  return std::nullopt;
}

// Maps into preallocated SDK arrays. Validate first, so rejected commands do
// not partially update the outgoing command. No allocation in hardware write().
inline bool map_position_command(const std::vector<double>& positions,
    const std::vector<double>& velocities, const std::vector<size_t>& mapping,
    bool with_velocity, std::vector<double>& target_positions,
    std::vector<double>& target_velocities) {
  if (mapping.size() != target_positions.size() || mapping.size() != target_velocities.size())
    return false;
  for (auto i : mapping) {
    if (i >= positions.size() || !std::isfinite(positions[i]) ||
        (with_velocity && (i >= velocities.size() || !std::isfinite(velocities[i])))) return false;
  }
  for (size_t i=0; i<mapping.size(); ++i) {
    target_positions[i] = positions[mapping[i]];
    target_velocities[i] = with_velocity ? velocities[mapping[i]] : 0.;
  }
  return true;
}
}  // namespace flexiv_hardware
