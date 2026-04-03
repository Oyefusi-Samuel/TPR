#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <queue>
#include <vector>
#include <cmath>

using std::placeholders::_1;

struct GridPose
{
  int x{0};
  int y{0};

  GridPose() = default;
  GridPose(int x_, int y_) : x(x_), y(y_) {}
};

class AStarSmoother : public rclcpp::Node
{
public:
  AStarSmoother()
  : Node("a_star_smoother")
  {
    declare_parameter<int>("iterations", 2);
    declare_parameter<int>("cost_limit", 20);

    iterations_  = get_parameter("iterations").as_int();
    cost_limit_  = get_parameter("cost_limit").as_int();

    rclcpp::QoS map_qos(10);
    map_qos.transient_local();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", map_qos,
      std::bind(&AStarSmoother::mapCallback, this, _1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/a_star/path", 10,
      std::bind(&AStarSmoother::pathCallback, this, _1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/a_star/path/smooth", 10);

    RCLCPP_INFO(get_logger(), "A* LOS smoother started");
  }

private:
  /* ===================== Callbacks ===================== */

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!map_) {
      RCLCPP_ERROR(get_logger(), "No costmap received");
      return;
    }

    startTime = std::chrono::system_clock::now();
    
    auto smoothed = smoothenPath(*msg);
    path_pub_->publish(smoothed);

    endTime = std::chrono::system_clock::now();

    testDuration = endTime - startTime;

    RCLCPP_INFO_STREAM(get_logger(), "Smoothed path published in " << testDuration.count() << " sec" );

    // RCLCPP_INFO(get_logger(), "Smoothed path published");
  }

  /* ===================== Core Logic ===================== */

  nav_msgs::msg::Path smoothenPath(const nav_msgs::msg::Path & path)
  {
    std::vector<GridPose> grid_path;
    grid_path.reserve(path.poses.size());

    for (const auto & p : path.poses) {
      grid_path.push_back(poseToGrid(p.pose));
    }

    for (int i = 0; i < iterations_; ++i) {
      grid_path = smoothenGridPath(grid_path);
    }

    nav_msgs::msg::Path out;
    out.header.frame_id = map_->header.frame_id;

    for (const auto & gp : grid_path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = out.header.frame_id;
      ps.pose = gridToPose(gp);
      out.poses.push_back(ps);
    }

    return out;
  }

  std::vector<GridPose> smoothenGridPath(const std::vector<GridPose> & path)
  {
    if (path.size() <= 1) {
      return path;
    }

    std::queue<GridPose> pose_queue;
    std::vector<GridPose> sparse_path;

    GridPose start = path.front();
    GridPose goal  = path.back();

    pose_queue.push(start);
    sparse_path.push_back(start);

    size_t i = 1;

    while (i < path.size()) {
      GridPose next = path[i];
      pose_queue.push(next);

      auto line = bresenhamLine(start, next);

      if (lineCrossesObstacle(line)) {
        GridPose last_safe = pose_queue.front();
        pose_queue.pop();

        sparse_path.push_back(last_safe);
        start = last_safe;

        while (!pose_queue.empty()) {
          pose_queue.pop();
        }
        pose_queue.push(start);
      } else {
        pose_queue.pop();
        ++i;
      }
    }

    sparse_path.push_back(goal);

    /* Re-densify */
    std::vector<GridPose> dense_path;
    dense_path.push_back(sparse_path.front());

    for (size_t k = 1; k < sparse_path.size(); ++k) {
      auto line = bresenhamLine(sparse_path[k - 1], sparse_path[k]);
      dense_path.insert(dense_path.end(), line.begin() + 1, line.end());
    }

    return dense_path;
  }

  /* ===================== Geometry ===================== */

  GridPose poseToGrid(const geometry_msgs::msg::Pose & pose) const
  {
    int gx = static_cast<int>(
      (pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
    int gy = static_cast<int>(
      (pose.position.y - map_->info.origin.position.y) / map_->info.resolution);

    return GridPose(gx, gy);
  }

  geometry_msgs::msg::Pose gridToPose(const GridPose & gp) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x =
      gp.x * map_->info.resolution + map_->info.origin.position.x;
    pose.position.y =
      gp.y * map_->info.resolution + map_->info.origin.position.y;
    pose.orientation.w = 1.0;
    return pose;
  }

  inline int gridToCell(const GridPose & gp) const
  {
    return gp.y * map_->info.width + gp.x;
  }

  bool lineCrossesObstacle(const std::vector<GridPose> & line) const
  {
    const auto & data = map_->data;

    for (const auto & gp : line) {
      int idx = gridToCell(gp);
      if (idx < 0 || idx >= static_cast<int>(data.size())) {
        return true;
      }
      if (data[idx] > cost_limit_) {
        return true;
      }
    }
    return false;
  }

  std::vector<GridPose> bresenhamLine(const GridPose & start,
                                     const GridPose & end) const
  {
    std::vector<GridPose> line;

    int x0 = start.x, y0 = start.y;
    int x1 = end.x,   y1 = end.y;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (true) {
      line.emplace_back(x0, y0);

      if (x0 == x1 && y0 == y1)
        break;

      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }

    return line;
  }

  /* ===================== Members ===================== */

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  int iterations_{2};
  int cost_limit_{20};

  std::chrono::duration<double> testDuration;
  std::chrono::_V2::system_clock::time_point startTime;
  std::chrono::_V2::system_clock::time_point endTime;
};

/* ===================== main ===================== */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarSmoother>());
  rclcpp::shutdown();
  return 0;
}
