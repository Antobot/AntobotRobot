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
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include "antobot_move_msgs/msg/collision_data.hpp"

class CollisionChecker : public rclcpp::Node {
public:
  CollisionChecker() : rclcpp::Node("collision_sat_checker") {

    // Parameters
    declare_parameter<std::string>("footprint_topic", "/costmap/footprint");
    declare_parameter<double>("inflation_factor_1", 1.2);
    declare_parameter<double>("inflation_factor_2", 1.5);
    declare_parameter<std::string>("clusters_topic", "/costmap_converter/obstacles");
    declare_parameter<std::string>("target_frame", "base_link");
    declare_parameter<double>("footprint_line_width", 0.06);
    declare_parameter<bool>("publish_collision_polygon", true);


    get_parameter("footprint_topic", footprint_topic_);
    get_parameter("inflation_factor_1", inflation_factor_1);
    get_parameter("inflation_factor_2", inflation_factor_2);
    get_parameter("clusters_topic", clusters_topic_);
    get_parameter("target_frame", target_frame_);
    get_parameter("footprint_line_width", footprint_line_width_);
    get_parameter("publish_collision_polygon", publish_collision_polygon_);

    // Topic
    sub_footprint_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      footprint_topic_, rclcpp::QoS(10), std::bind(&CollisionChecker::cbFootprint, this, std::placeholders::_1));
    sub_clusters_ = create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
      clusters_topic_, rclcpp::QoS(10), std::bind(&CollisionChecker::cbClusters, this, std::placeholders::_1));
    
      
    pub_collision_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/collision_checker/collision_region", 10);
    pub_collision_level_ = create_publisher<antobot_move_msgs::msg::CollisionData>("/collision_checker/collision_level", 10);
    // pub_collision_position_ = create_publisher<std_msgs::msg::Int16>("/collision_checker/collision_position", 10); // 0: ; 1: front; 2: back; 3: front and back
    pub_region_marker_ = create_publisher<visualization_msgs::msg::Marker>("/collision_checker/robot_region", 10);
    // pub_level1_region_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/collision_checker/level1_region", 10);
    // pub_level2_region_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/collision_checker/level2_region", 10);

    // Timer
    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&CollisionChecker::tick, this));
    RCLCPP_INFO(get_logger(), "collision_checker started");
  }

private:
  struct Vec2 { double x, y; };
  void cbFootprint(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) { 
    last_footprint_ = *msg; 
  }

  void cbClusters(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg) { 
    last_clusters_ = *msg; 
  }
  

  // convert from PolygonStamped to vector
  std::vector<Vec2> polyStampedToVec(const geometry_msgs::msg::PolygonStamped &poly_stamped) {
    std::vector<Vec2> out;
    const auto &poly = poly_stamped.polygon;
    out.reserve(poly.points.size());
    for (const auto &p : poly.points) out.push_back({(double)p.x, (double)p.y});
    return out;
  }

  std::vector<Vec2> obstaclePolyToVec(const geometry_msgs::msg::Polygon &poly) {
    std::vector<Vec2> out;
    out.reserve(poly.points.size());
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
      out.push_back({cx + factor * dx, cy + dy});
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("SS"), "cx: " << cx << "; cy: " << cy << "; cx + factor * dx: " << (cx + factor * dx) << "; cy + factor * dy: " << (cy + factor * dy));
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
    auto f0 = polyStampedToVec(last_footprint_.value());
    auto f1 = inflate(f0, inflation_factor_1);
    auto f2 = inflate(f0, inflation_factor_2);

    {
      visualization_msgs::msg::Marker m1;
      m1.header = last_footprint_.value().header;
      m1.header.frame_id = target_frame_;
      m1.ns = "inflate_1";
      m1.id = 11;
      m1.type = visualization_msgs::msg::Marker::LINE_LIST;
      m1.action = visualization_msgs::msg::Marker::ADD;
      m1.pose.orientation.w = 1.0;
      m1.scale.x = std::max(0.01, footprint_line_width_);
      m1.color.r = 1.0; 
      m1.color.g = 0.65; 
      m1.color.b = 0.0; 
      m1.color.a = 1.0;
      m1.frame_locked = true;
      if (f1.size() >= 2) {
        for (size_t i = 0; i + 1 < f1.size(); ++i) {
          geometry_msgs::msg::Point s, e;
          s.x = f1[i].x; s.y = f1[i].y; s.z = 0.02;
          e.x = f1[i+1].x; e.y = f1[i+1].y; e.z = 0.02;
          m1.points.push_back(s);
          m1.points.push_back(e);
        }
        geometry_msgs::msg::Point s, e;
        s.x = f1.back().x; s.y = f1.back().y; s.z = 0.02;
        e.x = f1.front().x; e.y = f1.front().y; e.z = 0.02;
        m1.points.push_back(s);
        m1.points.push_back(e);
      }
      pub_region_marker_->publish(m1);

      visualization_msgs::msg::Marker m2;
      m2.header = m1.header;
      m2.header.frame_id = target_frame_;
      m2.ns = "inflate_2";
      m2.id = 12;
      m2.type = visualization_msgs::msg::Marker::LINE_LIST;
      m2.action = visualization_msgs::msg::Marker::ADD;
      m2.pose.orientation.w = 1.0;
      m2.scale.x = m1.scale.x;
      m2.color.r = 1.0; m2.color.g = 0.0; m2.color.b = 0.0; m2.color.a = 1.0;
      m2.frame_locked = true;
      if (f2.size() >= 2) {
        for (size_t i = 0; i + 1 < f2.size(); ++i) {
          geometry_msgs::msg::Point s, e;
          s.x = f2[i].x; s.y = f2[i].y; s.z = 0.02;
          e.x = f2[i+1].x; e.y = f2[i+1].y; e.z = 0.02;
          m2.points.push_back(s);
          m2.points.push_back(e);
        }
        geometry_msgs::msg::Point s, e;
        s.x = f2.back().x; s.y = f2.back().y; s.z = 0.02;
        e.x = f2.front().x; e.y = f2.front().y; e.z = 0.02;
        m2.points.push_back(s);
        m2.points.push_back(e);
      }
      pub_region_marker_->publish(m2);
    }
  
    
    bool collided1 = false, collided2 = false;
    bool collided11 = false, collided12 = false, collided21 = false, collided22 = false;
    // removed unused obstacle indices
    std::vector<Vec2> coll_poly11, coll_poly12, coll_poly21, coll_poly22;
    for (size_t i = 0; i < last_clusters_->obstacles.size(); ++i) {
      const auto &poly = last_clusters_->obstacles[i].polygon;
      auto obs = obstaclePolyToVec(poly);
      if (obs.size() < 2 || f0.size() < 2) 
        continue;

      if (!collided21 || !collided22) {

        bool hit2 = satOverlap(f2, obs);
        if (hit2) {
          auto tmp2 = intersectConvex(f2, obs); 
          if (!tmp2.empty()) {
              double avg_x = 0.0;
              for (const auto& p : tmp2) {
                  avg_x += p.x;
              }
              avg_x /= tmp2.size();

              if (avg_x > 0) {
                  collided21 = true;
                  coll_poly21 = tmp2;
              } else {
                  collided22 = true;
                  coll_poly22 = tmp2;
              }
          }
        }
      }
      
      if (!collided11 || !collided12) {

        bool hit1 = satOverlap(f1, obs);
        if (hit1) {
          auto tmp1 = intersectConvex(f1, obs);
          if (!tmp1.empty()) {
              double avg_x = 0.0;
              for (const auto& p : tmp1) {
                  avg_x += p.x;
              }
              avg_x /= tmp1.size();

              if (avg_x > 0) {
                  collided11 = true;
                  coll_poly11 = tmp1;
              } else {
                  collided12 = true;
                  coll_poly12 = tmp1;
              }
          }
        }
      }
    }


    collided1 = collided11 || collided12;
    collided2 = collided21 || collided22;
    // RCLCPP_INFO_STREAM(get_logger(), "collided11:" << collided11 << " collided12:" << collided12 << "; collided21:" << collided21 << " collided22:" << collided22);
    

    geometry_msgs::msg::PolygonStamped coll1_front;
    geometry_msgs::msg::PolygonStamped coll1_back;
    coll1_front.header = last_footprint_.value().header;
    coll1_back.header = last_footprint_.value().header;
    coll1_front.header.frame_id = target_frame_;
    coll1_back.header.frame_id = target_frame_;
    if (collided1) {
      for (const auto &v : coll_poly11) { 
        geometry_msgs::msg::Point32 p; 
        p.x = (float)v.x; 
        p.y = (float)v.y; 
        coll1_front.polygon.points.push_back(p); 
      }
      for (const auto &v : coll_poly12) { 
        geometry_msgs::msg::Point32 p; 
        p.x = (float)v.x; 
        p.y = (float)v.y; 
        coll1_back.polygon.points.push_back(p); 
      }
      pub_collision_->publish(coll1_front);
      pub_collision_->publish(coll1_back);
    }

    geometry_msgs::msg::PolygonStamped coll2_front;
    geometry_msgs::msg::PolygonStamped coll2_back;
    coll2_front.header = last_footprint_.value().header;
    coll2_back.header = last_footprint_.value().header;
    coll2_front.header.frame_id = target_frame_;
    coll2_back.header.frame_id = target_frame_;
    if (collided2) {
      for (const auto &v : coll_poly21) { 
        geometry_msgs::msg::Point32 p; 
        p.x = (float)v.x; 
        p.y = (float)v.y; 
        coll2_front.polygon.points.push_back(p); 
      }
      for (const auto &v : coll_poly22) { 
        geometry_msgs::msg::Point32 p; 
        p.x = (float)v.x; 
        p.y = (float)v.y; 
        coll2_back.polygon.points.push_back(p); 
      }
      pub_collision_->publish(coll2_front);
      pub_collision_->publish(coll2_back);
    }
    // pub_level2_region_->publish(coll2);

    // RCLCPP_INFO_STREAM(get_logger(), "positionx:" << position);

    int level = 0;
    if (collided1) 
      level = 2; 
    else if (collided2) 
      level = 1;

    int position = 0; // 0: none; 1: front; 2: back; 3: front and back
    bool front_hit = collided11 || collided21;
    bool back_hit  = collided12 || collided22;
    if (front_hit && back_hit) position = 3;
    else if (front_hit) position = 1;
    else if (back_hit) position = 2;

    antobot_move_msgs::msg::CollisionData CollisionData;
    CollisionData.collision_level = level; 
    CollisionData.collision_position = position;
    pub_collision_level_->publish(CollisionData);

    if (publish_collision_polygon_) {
      if (!(collided1 || collided2)) {
        geometry_msgs::msg::PolygonStamped clear;
        clear.header = last_footprint_.value().header;
        clear.header.frame_id = target_frame_;
        pub_collision_->publish(clear);
      }
    }

    auto t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> dt = t1 - t0;
    std_msgs::msg::Float64 ms;
    ms.data = dt.count();
    // RCLCPP_INFO(get_logger(), "collision tick compute: %.2f ms; collision level: %d", ms.data, level);
  }

  std::string footprint_topic_;
  std::string clusters_topic_;
  std::string target_frame_;
  double inflation_factor_1, inflation_factor_2;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_footprint_;
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_clusters_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_collision_;
  rclcpp::Publisher<antobot_move_msgs::msg::CollisionData>::SharedPtr pub_collision_level_; // TODO: Level; Positionc
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_region_marker_;
  // rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_level1_region_;
  // rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_level2_region_;
  bool publish_collision_polygon_;
  double footprint_line_width_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<geometry_msgs::msg::PolygonStamped> last_footprint_;
  std::optional<costmap_converter_msgs::msg::ObstacleArrayMsg> last_clusters_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionChecker>());
  rclcpp::shutdown();
  return 0;
}
