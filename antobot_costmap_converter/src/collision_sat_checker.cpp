#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <optional>
#include <limits>
#include <cmath>
#include <string>

class CollisionSATChecker : public rclcpp::Node {
public:
  CollisionSATChecker() : rclcpp::Node("collision_sat_checker") {

    declare_parameter<std::string>("footprint_topic", "/costmap/published_footprint");
    declare_parameter<double>("inflation_factor", 2.0);
    declare_parameter<std::string>("inflated_topic", "/costmap/inflated_footprint");
    declare_parameter<std::string>("clusters_topic", "/costmap_converter/obstacles_custom");
    get_parameter("footprint_topic", footprint_topic_);
    get_parameter("inflation_factor", inflation_factor_);
    get_parameter("inflated_topic", inflated_topic_);
    get_parameter("clusters_topic", clusters_topic_);

    sub_footprint_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      footprint_topic_, rclcpp::QoS(10), std::bind(&CollisionSATChecker::cbFootprint, this, std::placeholders::_1));
    sub_clusters_ = create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
      clusters_topic_, rclcpp::QoS(10), std::bind(&CollisionSATChecker::cbClusters, this, std::placeholders::_1));

    pub_inflated_ = create_publisher<geometry_msgs::msg::PolygonStamped>(inflated_topic_, 10);
    pub_collision_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/costmap/collision_region", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&CollisionSATChecker::tick, this));
    RCLCPP_INFO(get_logger(), "collision_sat_checker started");
  }

private:
  void cbFootprint(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) { last_footprint_ = *msg; }
  void cbClusters(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg) { last_clusters_ = *msg; }

  struct Vec2 { double x, y; };

  static std::vector<Vec2> poly2vec(const geometry_msgs::msg::Polygon &poly) {
    std::vector<Vec2> out; out.reserve(poly.points.size());
    for (const auto &p : poly.points) out.push_back({(double)p.x, (double)p.y});
    return out;
  }


  static std::vector<Vec2> inflate(const std::vector<Vec2> &poly, double factor) {
    if (poly.empty()) return poly;
    double cx = 0.0, cy = 0.0;
    for (const auto &v : poly) { cx += v.x; cy += v.y; }
    cx /= (double)poly.size(); cy /= (double)poly.size();
    std::vector<Vec2> out; out.reserve(poly.size());
    for (const auto &v : poly) {
      double dx = v.x - cx, dy = v.y - cy;
      out.push_back({cx + factor * dx, cy + factor * dy});
    }
    return out;
  }

  static double cross(const Vec2 &a, const Vec2 &b) { return a.x*b.y - a.y*b.x; }
  static Vec2 sub(const Vec2 &a, const Vec2 &b) { return {a.x - b.x, a.y - b.y}; }

  static double signedArea(const std::vector<Vec2> &poly) {
    double A = 0.0; if (poly.size() < 2) return 0.0;
    for (size_t i = 0, j = (poly.size() ? poly.size()-1 : 0); i < poly.size(); j = i++) {
      A += (poly[j].x * poly[i].y - poly[i].x * poly[j].y);
    }
    return 0.5 * A;
  }

  static void normalizeCCW(std::vector<Vec2> &poly) {
    if (signedArea(poly) < 0.0) std::reverse(poly.begin(), poly.end());
    if (!poly.empty()) {
      if (std::hypot(poly.front().x - poly.back().x, poly.front().y - poly.back().y) < 1e-6) poly.pop_back();
    }
  }

  static bool inside(const Vec2 &p, const Vec2 &A, const Vec2 &B) {
    Vec2 AB = sub(B, A); Vec2 AP = sub(p, A);
    return cross(AB, AP) >= -1e-9;
  }

  static bool intersectLineSeg(const Vec2 &A, const Vec2 &B, const Vec2 &S, const Vec2 &E, Vec2 &I) {
    Vec2 dClip = sub(B, A); Vec2 dSeg = sub(E, S);
    double denom = cross(dSeg, dClip);
    if (std::abs(denom) < 1e-12) return false;
    double t = cross(sub(A, S), dClip) / denom;
    I = {S.x + t * dSeg.x, S.y + t * dSeg.y};
    return true;
  }

  static std::vector<Vec2> intersectConvex(const std::vector<Vec2> &subject_in, const std::vector<Vec2> &clip_in) {
    std::vector<Vec2> subject = subject_in;
    std::vector<Vec2> clip = clip_in;
    normalizeCCW(subject);
    normalizeCCW(clip);
    std::vector<Vec2> output = subject;
    for (size_t i = 0; i < clip.size(); ++i) {
      Vec2 A = clip[i]; Vec2 B = clip[(i + 1) % clip.size()];
      std::vector<Vec2> input = output; output.clear();
      if (input.empty()) break;
      for (size_t j = 0; j < input.size(); ++j) {
        Vec2 S = input[j]; Vec2 E = input[(j + 1) % input.size()];
        bool Ein = inside(E, A, B);
        bool Sin = inside(S, A, B);
        if (Ein) {
          if (!Sin) { Vec2 I; if (intersectLineSeg(A, B, S, E, I)) output.push_back(I); }
          output.push_back(E);
        } else if (Sin) {
          Vec2 I; if (intersectLineSeg(A, B, S, E, I)) output.push_back(I);
        }
      }
    }
    return output;
  }

  static Vec2 edgeNormal(const Vec2 &a, const Vec2 &b) {
    double ex = b.x - a.x, ey = b.y - a.y; // edge
    // normal (perp) vector
    return {-ey, ex};
  }


  static void project(const std::vector<Vec2> &poly, const Vec2 &axis, double &minp, double &maxp) {
    minp = std::numeric_limits<double>::infinity();
    maxp = -std::numeric_limits<double>::infinity();
    double ax = axis.x, ay = axis.y;
    double len = std::sqrt(ax*ax + ay*ay);
    double nx = len > 0 ? ax/len : 0.0;
    double ny = len > 0 ? ay/len : 0.0;
    for (const auto &v : poly) {
      double d = v.x*nx + v.y*ny;
      if (d < minp) minp = d;
      if (d > maxp) maxp = d;
    }
  }


  static bool satOverlap(const std::vector<Vec2> &A, const std::vector<Vec2> &B) {
    auto checkAxes = [&](const std::vector<Vec2> &P, const std::vector<Vec2> &Q) {
      if (P.size() < 2 || Q.size() < 2) return false;
      for (size_t i = 0; i < P.size(); ++i) {
        const Vec2 &p0 = P[i];
        const Vec2 &p1 = P[(i + 1) % P.size()];
        Vec2 n = edgeNormal(p0, p1);
        if (std::abs(n.x) < 1e-9 && std::abs(n.y) < 1e-9) continue;
        double a0, a1, b0, b1; project(P, n, a0, a1); project(Q, n, b0, b1);
        if (a1 < b0 || b1 < a0) return false;
      }
      return true;
    };
    return checkAxes(A, B) && checkAxes(B, A);
  }


  void tick() {
    if (!last_footprint_.has_value() || !last_clusters_.has_value()) {
      RCLCPP_INFO(get_logger(), "waiting footprint & clusters...");
      return;
    }
    const auto &foot = last_footprint_.value().polygon;
    auto fp = poly2vec(foot);
    auto fpinflated = inflate(fp, inflation_factor_);
    geometry_msgs::msg::PolygonStamped out;
    out.header = last_footprint_.value().header;
    for (const auto &v : fpinflated) {
      geometry_msgs::msg::Point32 p; p.x = (float)v.x; p.y = (float)v.y; out.polygon.points.push_back(p);
    }
    pub_inflated_->publish(out);
    RCLCPP_INFO(get_logger(), "inflated footprint published: %zu pts (factor %.2f)", out.polygon.points.size(), inflation_factor_);
    bool collided = false;
    size_t collided_idx = (size_t)-1;
    std::vector<Vec2> collision_poly;
    for (size_t i = 0; i < last_clusters_->obstacles.size(); ++i) {
      const auto &poly = last_clusters_->obstacles[i].polygon;
      auto obs = poly2vec(poly);
      if (obs.size() < 2 || fp.size() < 2) continue;
      if (satOverlap(fpinflated, obs)) {
        collided = true; collided_idx = i;
        collision_poly = intersectConvex(fpinflated, obs);
        break;
      }
    }
    std::string suf;
    if (collided) suf = std::string(" (obstacle ") + std::to_string(collided_idx) + ")";
    RCLCPP_INFO(get_logger(), "SAT collision: %s%s", collided ? "YES" : "NO", suf.c_str());

    if (collided && !collision_poly.empty()) {
      geometry_msgs::msg::PolygonStamped coll;
      coll.header = last_footprint_.value().header;
      for (const auto &v : collision_poly) { geometry_msgs::msg::Point32 p; p.x = (float)v.x; p.y = (float)v.y; coll.polygon.points.push_back(p); }
      pub_collision_->publish(coll);
      RCLCPP_INFO(get_logger(), "collision region published: %zu pts", coll.polygon.points.size());
    }
  }

  std::string footprint_topic_;
  std::string clusters_topic_;
  std::string inflated_topic_;
  double inflation_factor_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_footprint_;
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_clusters_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_inflated_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_collision_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<geometry_msgs::msg::PolygonStamped> last_footprint_;
  std::optional<costmap_converter_msgs::msg::ObstacleArrayMsg> last_clusters_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionSATChecker>());
  rclcpp::shutdown();
  return 0;
}
