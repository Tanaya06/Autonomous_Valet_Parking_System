#include "avps_controller/pure_pursuit.hpp"


#include <algorithm>
#include <limits>


namespace avps
{


PurePursuit::PurePursuit(const Params & params)
: params_(params)
{}


void PurePursuit::set_path(const std::vector<Pose2D> & path)
{
  path_ = path;
  closest_idx_ = 0;
}


PurePursuit::Command PurePursuit::compute(const Pose2D & current_pose)
{
  Command cmd;


  if (path_.empty()) {
    return cmd;
  }


  // Check if goal is reached
  if (distance(current_pose, path_.back()) < params_.goal_tolerance) {
    cmd.goal_reached = true;
    return cmd;
  }


  closest_idx_ = find_closest_index(current_pose);


  auto lookahead = find_lookahead_point(current_pose, closest_idx_);


  if (!lookahead) {
    // Past the end of path — head toward last point
    lookahead = path_.back();
  }


  // Transform lookahead point into vehicle frame
  double dx = lookahead->x - current_pose.x;
  double dy = lookahead->y - current_pose.y;


  double cos_yaw = std::cos(current_pose.yaw);
  double sin_yaw = std::sin(current_pose.yaw);


  double local_x =  cos_yaw * dx + sin_yaw * dy;
  double local_y = -sin_yaw * dx + cos_yaw * dy;


  double chord = std::hypot(local_x, local_y);


  double curvature = 0.0;
  if (chord > 1e-6) {
    curvature = 2.0 * local_y / (chord * chord);
  }


  cmd.linear  = params_.linear_velocity;
  cmd.angular = std::clamp(
    params_.linear_velocity * curvature,
    -params_.max_angular_vel,
     params_.max_angular_vel);


  return cmd;
}


std::size_t PurePursuit::find_closest_index(const Pose2D & pose) const
{
  std::size_t best = closest_idx_;
  double best_dist = std::numeric_limits<double>::max();


  for (std::size_t i = closest_idx_; i < path_.size(); ++i) {
    double d = distance(pose, path_[i]);
    if (d < best_dist) {
      best_dist = d;
      best = i;
    }
  }
  return best;
}


std::optional<Pose2D> PurePursuit::find_lookahead_point(
  const Pose2D & pose, std::size_t start_idx) const
{
  for (std::size_t i = start_idx; i + 1 < path_.size(); ++i) {
    const Pose2D & a = path_[i];
    const Pose2D & b = path_[i + 1];


    // Segment vector
    double seg_dx = b.x - a.x;
    double seg_dy = b.y - a.y;
    double seg_len = std::hypot(seg_dx, seg_dy);


    if (seg_len < 1e-9) {
      continue;
    }


    // Vector from a to pose
    double to_pose_x = pose.x - a.x;
    double to_pose_y = pose.y - a.y;


    // Project pose onto segment
    double t = (to_pose_x * seg_dx + to_pose_y * seg_dy) / (seg_len * seg_len);
    t = std::clamp(t, 0.0, 1.0);


    // Closest point on segment to pose
    double closest_x = a.x + t * seg_dx;
    double closest_y = a.y + t * seg_dy;


    double dist_to_seg = std::hypot(pose.x - closest_x, pose.y - closest_y);


    // Use the lookahead circle intersection heuristic
    double remaining = std::sqrt(
      std::max(0.0, params_.lookahead_dist * params_.lookahead_dist -
                    dist_to_seg * dist_to_seg));


    double along = t * seg_len + remaining;
    along = std::clamp(along, 0.0, seg_len);


    double frac = along / seg_len;
    Pose2D lp;
    lp.x   = a.x + frac * seg_dx;
    lp.y   = a.y + frac * seg_dy;
    lp.yaw = std::atan2(seg_dy, seg_dx);


    // Accept if the point is roughly lookahead_dist away
    if (distance(pose, lp) >= params_.lookahead_dist * 0.5 || i + 2 >= path_.size()) {
      return lp;
    }
  }
  return std::nullopt;
}


double PurePursuit::distance(const Pose2D & a, const Pose2D & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}


double PurePursuit::normalize_angle(double angle)
{
  while (angle >  M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}


}  // namespace avps
