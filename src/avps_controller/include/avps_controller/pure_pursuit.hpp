#pragma once

#include <vector>
#include <cmath>
#include <optional>

namespace avps
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class PurePursuit
{
public:
  struct Params
  {
    double lookahead_dist{0.6};
    double linear_velocity{0.3};
    double max_angular_vel{1.5};
    double goal_tolerance{0.25};
  };

  struct Command
  {
    double linear{0.0};
    double angular{0.0};
    bool goal_reached{false};
  };

  explicit PurePursuit(const Params & params = Params{});

  void set_path(const std::vector<Pose2D> & path);
  Command compute(const Pose2D & current_pose);
  bool has_path() const { return !path_.empty(); }
  void clear_path() { path_.clear(); closest_idx_ = 0; }

private:
  Params params_;
  std::vector<Pose2D> path_;
  std::size_t closest_idx_{0};

  std::size_t find_closest_index(const Pose2D & pose) const;
  std::optional<Pose2D> find_lookahead_point(
    const Pose2D & pose, std::size_t start_idx) const;
  static double distance(const Pose2D & a, const Pose2D & b);
  static double normalize_angle(double angle);
};

}  // namespace avps