#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <optional>
#include <limits>
#include <cmath>
#include <string>
#include <algorithm>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

class CollisionChecker : public rclcpp::Node {
public:
  CollisionChecker() : rclcpp::Node("collision_sat_checker"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {

    // Parameters
    declare_parameter<std::string>("footprint_topic", "/costmap/footprint");
    declare_parameter<double>("inflation_factor_1", 1.5);
    declare_parameter<double>("inflation_factor_2", 2.0);
    declare_parameter<std::string>("inflated_topic", "/costmap/footprint_2");
    declare_parameter<std::string>("inflated_topic_1", "/costmap/footprint_1");
    declare_parameter<std::string>("clusters_topic", "/costmap_converter/obstacles");
    declare_parameter<std::string>("target_frame", "base_link");
    declare_parameter<bool>("use_tf", false);
    declare_parameter<double>("footprint_line_width", 0.06);
    declare_parameter<double>("collision_line_width_factor", 2.0);
    declare_parameter<bool>("publish_collision_polygon", true);
    declare_parameter<bool>("publish_collision_marker", false);

    get_parameter("footprint_topic", footprint_topic_);
    get_parameter("inflation_factor_1", inflation_factor_1);
    get_parameter("inflation_factor_2", inflation_factor_2);
    get_parameter("clusters_topic", clusters_topic_);
    get_parameter("target_frame", target_frame_);
    get_parameter("use_tf", use_tf_);
    get_parameter("footprint_line_width", footprint_line_width_);
    get_parameter("collision_line_width_factor", collision_line_width_factor_);
    get_parameter("publish_collision_polygon", publish_collision_polygon_);
    get_parameter("publish_collision_marker", publish_collision_marker_);

    // Topic
    sub_footprint_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      footprint_topic_, rclcpp::QoS(10), std::bind(&CollisionChecker::cbFootprint, this, std::placeholders::_1));
    sub_clusters_ = create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
      clusters_topic_, rclcpp::QoS(10), std::bind(&CollisionChecker::cbClusters, this, std::placeholders::_1));

      
    pub_inflated1_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/collision_checker/footprint_1", 10);
    pub_inflated2_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/collision_checker/footprint_2", 10);
    pub_collision_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/collision_checker/collision_region", 10);
    pub_collision_level_ = create_publisher<std_msgs::msg::Int32>("/collision_checker/collision_level", 10);
    pub_time_ = create_publisher<std_msgs::msg::Float64>("/collision_checker/compute_ms", 10);

    // Timer
    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&CollisionChecker::tick, this));
    RCLCPP_INFO(get_logger(), "collision_checker started");
  }

private:
  struct Vec2 { double x, y; };
  void cbFootprint(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) { last_footprint_ = *msg; }
  void cbClusters(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg) { last_clusters_ = *msg; }

  bool transformPoint(const geometry_msgs::msg::Point32 &in,
                      const std::string &from_frame,
                      const std::string &to_frame,
                      geometry_msgs::msg::Point32 &out) {
    try {
      geometry_msgs::msg::PointStamped ps, ts;
      ps.header.frame_id = from_frame;
      ps.point.x = in.x; ps.point.y = in.y; ps.point.z = 0.0;
      ts = tf_buffer_.transform(ps, to_frame, tf2::durationFromSec(0.05));
      out.x = (float)ts.point.x; out.y = (float)ts.point.y; out.z = 0.0f;
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "TF transform failed: %s", ex.what());
      return false;
    }
  }

  // convert from PolygonStamped to vector
  std::vector<Vec2> polyStampedToVec(const geometry_msgs::msg::PolygonStamped &poly_stamped,
                                     const std::string &to_frame) {
    std::vector<Vec2> out;
    const auto &poly = poly_stamped.polygon;
    if (!use_tf_ || poly_stamped.header.frame_id == to_frame) {
      out.reserve(poly.points.size());
      for (const auto &p : poly.points) 
        out.push_back({(double)p.x, (double)p.y});
      return out;
    }
    out.reserve(poly.points.size());
    for (const auto &p : poly.points) {
      geometry_msgs::msg::Point32 tp;
      if (transformPoint(p, poly_stamped.header.frame_id, to_frame, tp)) {
        out.push_back({(double)tp.x, (double)tp.y});
      }
    }
    return out;
  }

  std::vector<Vec2> obstaclePolyToVec(const geometry_msgs::msg::Polygon &poly,
                                      const std::string &from_frame,
                                      const std::string &to_frame) {
    std::vector<Vec2> out;
    if (!use_tf_ || from_frame == to_frame) {
      out.reserve(poly.points.size());
      for (const auto &p : poly.points) out.push_back({(double)p.x, (double)p.y});
      return out;
    }
    out.reserve(poly.points.size());
    for (const auto &p : poly.points) {
      geometry_msgs::msg::Point32 tp;
      if (transformPoint(p, from_frame, to_frame, tp)) {
        out.push_back({(double)tp.x, (double)tp.y});
      }
    }
    return out;
  }


  static std::vector<Vec2> poly2vec(const geometry_msgs::msg::Polygon &poly) {
    std::vector<Vec2> out; out.reserve(poly.points.size());
    for (const auto &p : poly.points) out.push_back({(double)p.x, (double)p.y});
    return out;
  }


  // inflate the poly
  static std::vector<Vec2> inflate(const std::vector<Vec2> &poly, double factor) {
    if (poly.empty()) return poly;
    double cx = 0.0, cy = 0.0;
    for (const auto &v : poly) { 
      cx += v.x; 
      cy += v.y; 
    }

    cx /= (double)poly.size(); 
    cy /= (double)poly.size();

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
    auto t0 = std::chrono::steady_clock::now();
    if (!last_footprint_.has_value() || !last_clusters_.has_value()) {
      RCLCPP_INFO(get_logger(), "waiting footprint & clusters...");
      return;
    }
    // Transform footprint and publish inflated variants
    auto f0 = polyStampedToVec(last_footprint_.value(), target_frame_);
    auto f1 = inflate(f0, inflation_factor_1);
    auto f2 = inflate(f0, inflation_factor_2);
    geometry_msgs::msg::PolygonStamped out1;
    out1.header = last_footprint_.value().header;
    out1.header.frame_id = target_frame_;
    for (const auto &v : f1) { 
      geometry_msgs::msg::Point32 p; 
      p.x = (float)v.x; 
      p.y = (float)v.y; 
      out1.polygon.points.push_back(p); 
    }
    pub_inflated2_->publish(out1);

    geometry_msgs::msg::PolygonStamped out2;
    out2.header = last_footprint_.value().header;
    out2.header.frame_id = target_frame_;
    for (const auto &v : f2) { 
      geometry_msgs::msg::Point32 p; 
      p.x = (float)v.x; p.y = (float)v.y; 
      out2.polygon.points.push_back(p); 
    }
    pub_inflated1_->publish(out2);
    
    bool collided0 = false, collided1 = false, collided2 = false;
    size_t idx0 = (size_t)-1, idx1 = (size_t)-1, idx2 = (size_t)-1;
    std::vector<Vec2> coll_poly0, coll_poly1, coll_poly2;
    const std::string clusters_frame = last_clusters_->header.frame_id;
    
    for (size_t i = 0; i < last_clusters_->obstacles.size(); ++i) {
      const auto &poly = last_clusters_->obstacles[i].polygon;
      auto obs = obstaclePolyToVec(poly, clusters_frame, target_frame_);
      if (obs.size() < 2 || f0.size() < 2) continue;
      if (!collided2 && satOverlap(f2, obs)) { collided2 = true; idx2 = i; coll_poly2 = intersectConvex(f2, obs); }
      if (!collided1 && satOverlap(f1, obs)) { collided1 = true; idx1 = i; coll_poly1 = intersectConvex(f1, obs); }
      if (!collided0 && satOverlap(f0, obs)) { collided0 = true; idx0 = i; coll_poly0 = intersectConvex(f0, obs); }
    }
    int level = -1;
    if (collided0) level = 0; else if (collided1) level = 1; else if (collided2) level = 2;
    std_msgs::msg::Int32 lvl; lvl.data = level; pub_collision_level_->publish(lvl);

    if (publish_collision_polygon_ && level >= 0) {
      const auto &chosen = (level == 2 ? coll_poly2 : (level == 1 ? coll_poly1 : coll_poly0));
      geometry_msgs::msg::PolygonStamped coll;
      coll.header = last_footprint_.value().header;
      coll.header.frame_id = target_frame_;
      for (const auto &v : chosen) { geometry_msgs::msg::Point32 p; p.x = (float)v.x; p.y = (float)v.y; coll.polygon.points.push_back(p); }
      pub_collision_->publish(coll);
      // RCLCPP_INFO(get_logger(), "collision region published: %zu pts", coll.polygon.points.size());
    } else {
      geometry_msgs::msg::PolygonStamped clear;
      clear.header = last_footprint_.value().header;
      clear.header.frame_id = target_frame_;
      pub_collision_->publish(clear);
    }

    auto t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> dt = t1 - t0;
    std_msgs::msg::Float64 ms;
    ms.data = dt.count();
    pub_time_->publish(ms);
    RCLCPP_INFO(get_logger(), "collision tick compute: %.2f ms; collision level: %d", ms.data, level);
  }

  std::string footprint_topic_;
  std::string clusters_topic_;
  std::string inflated_topic_;
  std::string inflated_topic_1_;
  std::string target_frame_;
  bool use_tf_;
  double inflation_factor_1, inflation_factor_2;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_footprint_;
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_clusters_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_inflated1_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_inflated2_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_collision_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_time_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_collision_level_;
  double footprint_line_width_;
  double collision_line_width_factor_;
  bool publish_collision_polygon_;
  bool publish_collision_marker_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<geometry_msgs::msg::PolygonStamped> last_footprint_;
  std::optional<costmap_converter_msgs::msg::ObstacleArrayMsg> last_clusters_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionChecker>());
  rclcpp::shutdown();
  return 0;
}
