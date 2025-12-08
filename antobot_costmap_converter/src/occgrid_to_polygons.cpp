#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <cmath>
#include <algorithm>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <costmap_converter/costmap_converter_interface.h>

class OccGridToPolygons : public rclcpp::Node {
public:
  OccGridToPolygons() : rclcpp::Node("occgrid_to_polygons") {
    declare_parameter<int>("occupied_min_value", 60);
    declare_parameter<int>("min_area_cells", 3);
    declare_parameter<int>("dilate_kernel", 3);
    declare_parameter<double>("approx_epsilon_cells", 1.0);
    declare_parameter<double>("cluster_max_distance", 0.4);
    declare_parameter<int>("cluster_min_pts", 3);
    declare_parameter<int>("cluster_max_pts", 30);
    declare_parameter<double>("convex_hull_min_pt_separation", 0.0);
    declare_parameter<bool>("use_costmap2d", true);
    declare_parameter<bool>("use_plugin_core", false);
    declare_parameter<std::string>("converter_plugin", "costmap_converter::CostmapToPolygonsDBSMCCH");
    declare_parameter<std::string>("obstacles_topic", "/costmap_converter/obstacles_custom");
    declare_parameter<std::string>("markers_topic", "/costmap_converter/polygon_markers_custom");
    get_parameter("occupied_min_value", occupied_min_value_);
    get_parameter("min_area_cells", min_area_cells_);
    get_parameter("dilate_kernel", dilate_kernel_);
    get_parameter("approx_epsilon_cells", approx_eps_);
    get_parameter("cluster_max_distance", cluster_eps_);
    get_parameter("cluster_min_pts", cluster_min_pts_);
    get_parameter("cluster_max_pts", cluster_max_pts_);
    get_parameter("convex_hull_min_pt_separation", convex_hull_min_sep_);
    get_parameter("use_costmap2d", use_costmap2d_);
    get_parameter("use_plugin_core", use_plugin_core_);
    get_parameter("converter_plugin", plugin_name_);
    get_parameter("obstacles_topic", obstacles_topic_);
    get_parameter("markers_topic", markers_topic_);

    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap/costmap", rclcpp::QoS(10),
      std::bind(&OccGridToPolygons::cb, this, std::placeholders::_1));
    pub_obs_ = create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>(obstacles_topic_, 10);
    pub_marker_ = create_publisher<visualization_msgs::msg::Marker>(markers_topic_, 10);
    pub_time_ = create_publisher<std_msgs::msg::Float64>("/costmap_converter/custom_compute_ms", 10);

    loader_ = std::make_unique<pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons>>("costmap_converter", "costmap_converter::BaseCostmapToPolygons");
    try {
      converter_ = loader_->createSharedInstance(plugin_name_);
      plugin_nh_ = std::make_shared<rclcpp::Node>("occgrid_converter_plugin_custom", "costmap_converter");
      plugin_nh_->declare_parameter<double>("cluster_max_distance", cluster_eps_);
      plugin_nh_->declare_parameter<int>("cluster_min_pts", cluster_min_pts_);
      plugin_nh_->declare_parameter<int>("cluster_max_pts", cluster_max_pts_);
      plugin_nh_->declare_parameter<double>("convex_hull_min_pt_separation", convex_hull_min_sep_);
      converter_->initialize(plugin_nh_);
    } catch (...) {}
  }

private:
  void cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    auto t0 = std::chrono::steady_clock::now();
    int w = msg->info.width;
    int h = msg->info.height;
    double res = msg->info.resolution;
    double ox = msg->info.origin.position.x;
    double oy = msg->info.origin.position.y;
    std::vector<cv::Point2f> pts;
    pts.reserve(w * h / 8);
    if (use_costmap2d_) {
      nav2_costmap_2d::Costmap2D costmap(w, h, res, ox, oy);
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          int8_t v = msg->data[y * w + x];
          unsigned char c;
          if (v < 0) c = nav2_costmap_2d::NO_INFORMATION;
          else if (v >= occupied_min_value_) c = nav2_costmap_2d::LETHAL_OBSTACLE;
          else c = nav2_costmap_2d::FREE_SPACE;
          costmap.setCost(x, y, c);
        }
      }
      if (use_plugin_core_ && converter_) {

        converter_->setCostmap2D(&costmap);
        converter_->compute();
        auto obstacles_ptr = converter_->getObstacles();
        visualization_msgs::msg::Marker line_list;
        line_list.header = msg->header;
        line_list.ns = "occgrid_polygons_custom";
        line_list.action = visualization_msgs::msg::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 0;
        line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
        line_list.scale.x = res;
        line_list.color.r = 1.0;
        line_list.color.g = 0.0;
        line_list.color.b = 0.0;
        line_list.color.a = 1.0;
        if (obstacles_ptr) {
          auto obstacles = *obstacles_ptr;
          obstacles.header = msg->header;
          RCLCPP_INFO(get_logger(), "plugin_core obstacles: %zu", obstacles.obstacles.size());
          pub_obs_->publish(obstacles);

          for (const auto &obs : obstacles.obstacles) {
            for (size_t i = 0; i + 1 < obs.polygon.points.size(); ++i) {
              geometry_msgs::msg::Point s, e;
              s.x = obs.polygon.points[i].x; s.y = obs.polygon.points[i].y; s.z = 0.02;
              e.x = obs.polygon.points[i+1].x; e.y = obs.polygon.points[i+1].y; e.z = 0.02;
              line_list.points.push_back(s);
              line_list.points.push_back(e);
            }
            if (obs.polygon.points.size() > 2) {
              geometry_msgs::msg::Point s, e;
              s.x = obs.polygon.points.back().x; s.y = obs.polygon.points.back().y; s.z = 0.02;
              e.x = obs.polygon.points.front().x; e.y = obs.polygon.points.front().y; e.z = 0.02;
              line_list.points.push_back(s);
              line_list.points.push_back(e);
            }
          }
        }
        pub_marker_->publish(line_list);
        auto t1 = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> dt = t1 - t0;
        std_msgs::msg::Float64 ms; ms.data = dt.count();
        pub_time_->publish(ms);
        return;
      } else {

        cv::Mat mask(h, w, CV_8UC1);
        for (int y = 0; y < h; ++y) {
          for (int x = 0; x < w; ++x) {
            int value = costmap.getCost(x, y);
            mask.at<uint8_t>(y, x) = (value >= nav2_costmap_2d::LETHAL_OBSTACLE) ? 255 : 0;
          }
        }
        int kr = std::max(1, (int)std::lround(cluster_eps_ / res));
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * kr + 1, 2 * kr + 1));
        cv::Mat closed;
        cv::morphologyEx(mask, closed, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(closed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        costmap_converter_msgs::msg::ObstacleArrayMsg obstacles;
        obstacles.header = msg->header;
        visualization_msgs::msg::Marker line_list;
        line_list.header = msg->header;
        line_list.ns = "occgrid_polygons_custom";
        line_list.action = visualization_msgs::msg::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 0;
        line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
        line_list.scale.x = res;
        line_list.color.r = 1.0;
        line_list.color.g = 0.0;
        line_list.color.b = 0.0;
        line_list.color.a = 1.0;

        for (auto &contour : contours) {

          if ((int)contour.size() < cluster_min_pts_) continue;
          if (cv::contourArea(contour) < (double)min_area_cells_) continue;
          std::vector<int> hullIdx;
          cv::convexHull(contour, hullIdx);
          geometry_msgs::msg::Polygon poly;
          for (int idx : hullIdx) {
            const auto &p = contour[idx];
            geometry_msgs::msg::Point32 pt;
            pt.x = (float)(ox + (p.x + 0.5) * res);
            pt.y = (float)(oy + (p.y + 0.5) * res);
            poly.points.push_back(pt);
          }
          if (poly.points.size() >= 3) poly.points.push_back(poly.points.front());

          simplifyPolygon(poly);
          costmap_converter_msgs::msg::ObstacleMsg obs; obs.polygon = poly; obstacles.obstacles.push_back(obs);

          for (size_t i = 0; i + 1 < poly.points.size(); ++i) {
            geometry_msgs::msg::Point s, e;
            s.x = poly.points[i].x; s.y = poly.points[i].y; s.z = 0.02;
            e.x = poly.points[i+1].x; e.y = poly.points[i+1].y; e.z = 0.02;
            line_list.points.push_back(s);
            line_list.points.push_back(e);
          }
          if (poly.points.size() > 2) {
            geometry_msgs::msg::Point s, e;
            s.x = poly.points.back().x; s.y = poly.points.back().y; s.z = 0.02;
            e.x = poly.points.front().x; e.y = poly.points.front().y; e.z = 0.02;
            line_list.points.push_back(s);
            line_list.points.push_back(e);
          }
        }

        pub_obs_->publish(obstacles);
        pub_marker_->publish(line_list);
        auto t1 = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> dt = t1 - t0;
        std_msgs::msg::Float64 ms; ms.data = dt.count();
        pub_time_->publish(ms);
        return;
      }
    } else {

      cv::Mat mask(h, w, CV_8UC1);
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          int8_t v = msg->data[y * w + x];
          mask.at<uint8_t>(y, x) = (v >= occupied_min_value_) ? 255 : 0;
        }
      }
      if (dilate_kernel_ > 1) {
        cv::Mat k = cv::Mat::ones(dilate_kernel_, dilate_kernel_, CV_8UC1);
        cv::dilate(mask, mask, k, cv::Point(-1,-1), 1);
      }
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          if (mask.at<uint8_t>(y, x)) {
            pts.emplace_back(ox + (x + 0.5) * res, oy + (y + 0.5) * res);
          }
        }
      }
    }

    double eps_world = cluster_eps_;

    buildBuckets(pts, ox, oy, w, h, res, eps_world);
    auto clusters = dbscan(pts, eps_world, cluster_min_pts_);

    costmap_converter_msgs::msg::ObstacleArrayMsg obstacles;
    obstacles.header = msg->header;

    visualization_msgs::msg::Marker line_list;
    line_list.header = msg->header;
    line_list.ns = "occgrid_polygons_custom";
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.scale.x = res;
    line_list.color.r = 1.0;
    line_list.color.g = 0.0;
    line_list.color.b = 0.0;
    line_list.color.a = 1.0;

    for (size_t ci = 0; ci < clusters.size(); ++ci) {
      auto &idxs = clusters[ci];
      if ((int)idxs.size() < cluster_min_pts_) continue;
      std::vector<int> sub = idxs;
      geometry_msgs::msg::Polygon poly;
      if (sub.size() >= 3) {

        auto hullpts = monotoneChainHullIdx(pts, sub);
        for (auto &p : hullpts) poly.points.push_back(p);
        simplifyPolygon(poly);
      } else {

        for (int i : sub) { geometry_msgs::msg::Point32 pt; pt.x = pts[i].x; pt.y = pts[i].y; poly.points.push_back(pt); }
      }
      
      costmap_converter_msgs::msg::ObstacleMsg obs;
      obs.polygon = poly;
      obstacles.obstacles.push_back(obs);

      for (size_t i = 0; i + 1 < poly.points.size(); ++i) {
        geometry_msgs::msg::Point s, e;
        s.x = poly.points[i].x; s.y = poly.points[i].y; s.z = 0.02;
        e.x = poly.points[i+1].x; e.y = poly.points[i+1].y; e.z = 0.02;
        line_list.points.push_back(s);
        line_list.points.push_back(e);
      }
      if (poly.points.size() > 2) {
        geometry_msgs::msg::Point s, e;
        s.x = poly.points.back().x; s.y = poly.points.back().y; s.z = 0.02;
        e.x = poly.points.front().x; e.y = poly.points.front().y; e.z = 0.02;
        line_list.points.push_back(s);
        line_list.points.push_back(e);
      }
    }
    
    pub_obs_->publish(obstacles);
    pub_marker_->publish(line_list);
    auto t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> dt = t1 - t0;
    std_msgs::msg::Float64 ms; ms.data = dt.count();
    pub_time_->publish(ms);
  }


  std::vector<std::vector<int>> dbscan(const std::vector<cv::Point2f>& pts, double eps, int min_pts) {
    int n = (int)pts.size();
    const int UNCLASSIFIED = 0;
    const int NOISE = -1;
    std::vector<int> labels(n, UNCLASSIFIED);
    std::vector<bool> visited(n, false);
    std::vector<std::vector<int>> clusters;
    int cid = 0;
    for (int i = 0; i < n; ++i) {
      if (visited[i]) continue;
      visited[i] = true;
      auto neigh = regionQuery(pts, i, eps);
      if ((int)neigh.size() < min_pts) {
        labels[i] = NOISE;
        noise_indices_.push_back(i);
      } else {
        ++cid;
        clusters.emplace_back();
        labels[i] = cid;
        clusters.back().push_back(i);
        std::vector<int> queue = neigh;
        for (size_t k = 0; k < queue.size(); ++k) {
          int j = queue[k];
          if (!visited[j]) {
            visited[j] = true;
            auto neigh2 = regionQuery(pts, j, eps);
            if ((int)neigh2.size() >= min_pts) {
              queue.insert(queue.end(), neigh2.begin(), neigh2.end());
            }
          }
          if (labels[j] == UNCLASSIFIED || labels[j] == NOISE) {
            labels[j] = cid;
            clusters.back().push_back(j);
          }
        }
      }
    }
    return clusters;
  }


  std::vector<int> regionQuery(const std::vector<cv::Point2f>& pts, int idx, double eps) {
    std::vector<int> neigh;
    int cx = int((pts[idx].x - base_x_) / eps);
    int cy = int((pts[idx].y - base_y_) / eps);
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        int nx = cx + dx;
        int ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= cells_x_ || ny >= cells_y_) continue;
        auto &bucket = neighbor_lookup_[(ny * cells_x_) + nx];
        for (int j : bucket) {
          float dxp = pts[j].x - pts[idx].x;
          float dyp = pts[j].y - pts[idx].y;
          if (dxp*dxp + dyp*dyp <= eps*eps) neigh.push_back(j);
        }
      }
    }
    return neigh;
  }


  void buildBuckets(const std::vector<cv::Point2f>& pts, double ox, double oy, int w, int h, double res, double grid_eps) {
    offset_x_ = ox;
    offset_y_ = oy;
    base_x_ = std::floor(ox / grid_eps) * grid_eps;
    base_y_ = std::floor(oy / grid_eps) * grid_eps;
    double size_x_m = w * res;
    double size_y_m = h * res;
    cells_x_ = int(size_x_m / grid_eps) + 1;
    cells_y_ = int(size_y_m / grid_eps) + 1;
    if ((int)neighbor_lookup_.size() != cells_x_ * cells_y_) {
      neighbor_lookup_.clear();
      neighbor_lookup_.resize(cells_x_ * cells_y_);
    } else {
      for (auto &b : neighbor_lookup_) b.clear();
    }
    for (int i = 0; i < (int)pts.size(); ++i) {
      int cx = int((pts[i].x - base_x_) / grid_eps);
      int cy = int((pts[i].y - base_y_) / grid_eps);
      if (cx < 0 || cy < 0 || cx >= cells_x_ || cy >= cells_y_) continue;
      neighbor_lookup_[(cy * cells_x_) + cx].push_back(i);
    }
  }


  std::vector<geometry_msgs::msg::Point32> monotoneChainHullIdx(const std::vector<cv::Point2f>& pts, const std::vector<int>& idxs) {
    std::vector<int> P = idxs;
    std::sort(P.begin(), P.end(), [&](int a, int b){ if (pts[a].x == pts[b].x) return pts[a].y < pts[b].y; return pts[a].x < pts[b].x; });
    std::vector<int> H;
    auto cross = [&](int O, int A, int B){ return (pts[A].x - pts[O].x)*(pts[B].y - pts[O].y) - (pts[A].y - pts[O].y)*(pts[B].x - pts[O].x); };
    for (int i : P) {
      while (H.size() >= 2 && cross(H[H.size()-2], H.back(), i) <= 0) H.pop_back();
      H.push_back(i);
    }
    size_t t = H.size()+1;
    for (int k = (int)P.size()-2; k >= 0; --k) {
      int i = P[k];
      while (H.size() >= t && cross(H[H.size()-2], H.back(), i) <= 0) H.pop_back();
      H.push_back(i);
    }
    if (!H.empty()) H.pop_back();
    std::vector<geometry_msgs::msg::Point32> out; out.reserve(H.size());
    for (int j : H) { geometry_msgs::msg::Point32 pt; pt.x = pts[j].x; pt.y = pts[j].y; out.push_back(pt); }
    return out;
  }


  void simplifyPolygon(geometry_msgs::msg::Polygon& polygon) {
    size_t triangleThreshold = 3;
    if (polygon.points.size() > 1
        && std::abs(polygon.points.front().x - polygon.points.back().x) < 1e-5
        && std::abs(polygon.points.front().y - polygon.points.back().y) < 1e-5) {
      triangleThreshold = 4;
    }
    if (polygon.points.size() <= triangleThreshold) return;
    auto dp = [&](auto &&self, const std::vector<geometry_msgs::msg::Point32>& pts) -> std::vector<geometry_msgs::msg::Point32> {
      if (pts.size() <= 2) return pts;
      double maxd = -1.0; size_t index = 0;
      auto dist = [&](const geometry_msgs::msg::Point32& a, const geometry_msgs::msg::Point32& b, const geometry_msgs::msg::Point32& p){
        double A = p.x - a.x, B = p.y - a.y, C = b.x - a.x, D = b.y - a.y;
        double dot = A*C + B*D; double len_sq = C*C + D*D; double t = len_sq>0? dot/len_sq : 0.0;
        double ex = a.x + t*C, ey = a.y + t*D; double dx = p.x - ex, dy = p.y - ey; return std::sqrt(dx*dx + dy*dy);
      };
      for (size_t i = 1; i + 1 < pts.size(); ++i) {
        double d = dist(pts.front(), pts.back(), pts[i]);
        if (d > maxd) { maxd = d; index = i; }
      }
      if (maxd < convex_hull_min_sep_) return std::vector<geometry_msgs::msg::Point32>{pts.front(), pts.back()};
      std::vector<geometry_msgs::msg::Point32> left(pts.begin(), pts.begin()+index+1);
      std::vector<geometry_msgs::msg::Point32> right(pts.begin()+index, pts.end());
      auto l = self(self, left);
      auto r = self(self, right);
      l.pop_back(); l.insert(l.end(), r.begin(), r.end());
      return l;
    };
    polygon.points = dp(dp, polygon.points);
  }


  int occupied_min_value_;
  int min_area_cells_;
  int dilate_kernel_;
  double approx_eps_;
  double cluster_eps_;
  int cluster_min_pts_;
  int cluster_max_pts_;
  double convex_hull_min_sep_;
  bool use_costmap2d_;
  bool use_plugin_core_;
  std::string plugin_name_;
  std::unique_ptr<pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons>> loader_;
  std::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter_;
  rclcpp::Node::SharedPtr plugin_nh_;
  std::vector<std::vector<int>> neighbor_lookup_;
  int cells_x_;
  int cells_y_;
  double offset_x_;
  double offset_y_;
  double base_x_;
  double base_y_;
  std::vector<int> noise_indices_;
  std::string obstacles_topic_;
  std::string markers_topic_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr pub_obs_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_time_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccGridToPolygons>());
  rclcpp::shutdown();
  return 0;
}
