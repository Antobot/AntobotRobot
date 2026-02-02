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
#include <geometry_msgs/msg/twist.hpp>

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
    declare_parameter<double>("collision_line_width_factor", 2.0);
    declare_parameter<bool>("publish_collision_polygon", true);
    declare_parameter<bool>("use_swept_volume", true);
    declare_parameter<double>("predict_horizon", 1.0);
    declare_parameter<int>("num_steps", 10);
    declare_parameter<bool>("publish_swept_marker", true);
    declare_parameter<double>("swept_line_width", 0.02);
    declare_parameter<std::string>("collision_strategy", "inflate");

    get_parameter("footprint_topic", footprint_topic_);
    get_parameter("inflation_factor_1", inflation_factor_1);
    get_parameter("inflation_factor_2", inflation_factor_2);
    get_parameter("clusters_topic", clusters_topic_);
    get_parameter("target_frame", target_frame_);
    get_parameter("publish_collision_polygon", publish_collision_polygon_);
    get_parameter("predict_horizon", predict_horizon_);
    get_parameter("num_steps", num_steps_);
    get_parameter("publish_swept_marker", publish_swept_marker_);
    get_parameter("swept_line_width", swept_line_width_);
    get_parameter("collision_strategy", collision_strategy_);

    // Topic
    sub_footprint_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      footprint_topic_, rclcpp::QoS(10), std::bind(&CollisionChecker::cbFootprint, this, std::placeholders::_1));
    sub_clusters_ = create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
      clusters_topic_, rclcpp::QoS(10), std::bind(&CollisionChecker::cbClusters, this, std::placeholders::_1));
    
    // TODO
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "/antobot/nav/cmd_vel_test", rclcpp::QoS(10), std::bind(&CollisionChecker::cbCmdVel, this, std::placeholders::_1));

      
    pub_collision_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/collision_checker/collision_region", 10);
    pub_collision_level_ = create_publisher<std_msgs::msg::Int16>("/collision_checker/collision_level", 10);
    pub_swept_marker_ = create_publisher<visualization_msgs::msg::Marker>("/collision_checker/robot_region", 10);
    // pub_level1_region_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/collision_checker/level1_region", 10);
    // pub_level2_region_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/collision_checker/level2_region", 10);

    // Timer
    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&CollisionChecker::tick, this));
    RCLCPP_INFO(get_logger(), "collision_checker started");
  }

private:
  struct Vec2 { double x, y; };
  void cbFootprint(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) { last_footprint_ = *msg; }
  void cbClusters(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg) { 
    last_clusters_ = *msg; 
  }
  void cbCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
     v_ = msg->linear.x; 
     omega_ = msg->angular.z; 
    //  RCLCPP_INFO_STREAM(this->get_logger(), "v_:" << v_ << "; omega_:" << omega_);
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

  static std::vector<Vec2> transformPoly(const std::vector<Vec2> &poly, double x, double y, double th) {
    double c = std::cos(th), s = std::sin(th);
    std::vector<Vec2> out; out.reserve(poly.size());
    for (const auto &p : poly) out.push_back({x + c * p.x - s * p.y, y + s * p.x + c * p.y});
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

  static std::vector<Vec2> convexHull(std::vector<Vec2> pts) {
    std::sort(pts.begin(), pts.end(), [](const Vec2 &a, const Vec2 &b){ return a.x < b.x || (a.x == b.x && a.y < b.y); });
    std::vector<Vec2> H;
    for (const auto &p : pts) {
      while (H.size() >= 2 && cross({H.back().x - H[H.size()-2].x, H.back().y - H[H.size()-2].y}, {p.x - H.back().x, p.y - H.back().y}) <= 0) H.pop_back();
      H.push_back(p);
    }
    size_t lower = H.size();
    for (int i = (int)pts.size()-2; i >= 0; --i) {
      const auto &p = pts[i];
      while (H.size() > lower && cross({H.back().x - H[H.size()-2].x, H.back().y - H[H.size()-2].y}, {p.x - H.back().x, p.y - H.back().y}) <= 0) H.pop_back();
      H.push_back(p);
    }
    if (!H.empty()) H.pop_back();
    return H;
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

    swept_hull_computed_ = false;
    // Transform footprint and publish inflated variants
    auto f0 = polyStampedToVec(last_footprint_.value());
    auto f1 = inflate(f0, inflation_factor_1);
    auto f2 = inflate(f0, inflation_factor_2);
    std::vector<Vec2> swept0, swept1, swept2;
    if (collision_strategy_ == "swept"){
      double dt = predict_horizon_ / std::max(1, num_steps_);
      double x = 0.0, y = 0.0, th = 0.0;
      for (int k = 0; k <= num_steps_; ++k) {
        auto tfp0 = transformPoly(f0, x, y, th);
        auto tfp1 = transformPoly(f1, x, y, th);
        auto tfp2 = transformPoly(f2, x, y, th);
        swept0.insert(swept0.end(), tfp0.begin(), tfp0.end());
        swept1.insert(swept1.end(), tfp1.begin(), tfp1.end());
        swept2.insert(swept2.end(), tfp2.begin(), tfp2.end());
        double v = v_, w = omega_;
        x += v * std::cos(th) * dt;
        y += v * std::sin(th) * dt;
        th += w * dt;
      }
    }
    std::vector<Vec2> hull0, hull1, hull2;
    if (collision_strategy_ == "swept"){
      if (!swept0.empty()) hull0 = convexHull(swept0);
      if (!swept1.empty()) hull1 = convexHull(swept1);
      if (!swept2.empty()) hull2 = convexHull(swept2);
      // keep legacy marker publisher using member swept_hull_
      swept_hull_ = hull0;
      swept_hull_computed_ = true;
    }

    if (collision_strategy_ == "inflate") {
      visualization_msgs::msg::Marker m1;
      m1.header = last_footprint_.value().header;
      m1.header.frame_id = target_frame_;
      m1.ns = "inflate_1";
      m1.id = 11;
      m1.type = visualization_msgs::msg::Marker::LINE_LIST;
      m1.action = visualization_msgs::msg::Marker::ADD;
      m1.pose.orientation.w = 1.0;
      m1.scale.x = std::max(0.01, swept_line_width_);
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
      pub_swept_marker_->publish(m1);

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
      pub_swept_marker_->publish(m2);
    }
  
    
    bool collided0 = false, collided1 = false, collided2 = false;
    // removed unused obstacle indices
    std::vector<Vec2> coll_poly1, coll_poly2;
    for (size_t i = 0; i < last_clusters_->obstacles.size(); ++i) {
      const auto &poly = last_clusters_->obstacles[i].polygon;
      auto obs = obstaclePolyToVec(poly);
      if (obs.size() < 2 || f0.size() < 2) continue;
      bool use_inflate = (collision_strategy_ == "inflate");
      bool use_swept = (collision_strategy_ == "swept") ;
      if (!collided2) {
        bool hit2 = (use_inflate && satOverlap(f2, obs)) || (use_swept && !hull2.empty() && satOverlap(hull2, obs));
        if (hit2) { collided2 = true; const auto &shape = (use_swept && !hull2.empty()) ? hull2 : f2; coll_poly2 = intersectConvex(shape, obs); }
      }
      if (!collided1) {
        bool hit1 = (use_inflate && satOverlap(f1, obs)) || (use_swept && !hull1.empty() && satOverlap(hull1, obs));
        if (hit1) { collided1 = true; const auto &shape = (use_swept && !hull1.empty()) ? hull1 : f1; coll_poly1 = intersectConvex(shape, obs); }
      }
      // level 0 removed
    }
    if ((collision_strategy_ == "swept") && publish_swept_marker_) {
      if (!swept_hull_computed_ && !swept0.empty()) { 
        swept_hull_ = convexHull(swept0); 
      }
      visualization_msgs::msg::Marker m;
      m.header = last_footprint_.value().header;
      m.header.frame_id = target_frame_;
      m.ns = "swept_volume";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::LINE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = std::max(0.01, swept_line_width_);
      m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
      m.frame_locked = true;
      if (swept_hull_.size() >= 2) {
        for (size_t i = 0; i + 1 < swept_hull_.size(); ++i) {
          geometry_msgs::msg::Point s, e;
          s.x = swept_hull_[i].x; s.y = swept_hull_[i].y; s.z = 0.02;
          e.x = swept_hull_[i+1].x; e.y = swept_hull_[i+1].y; e.z = 0.02;
          m.points.push_back(s);
          m.points.push_back(e);
        }
        geometry_msgs::msg::Point s, e;
        s.x = swept_hull_.back().x; s.y = swept_hull_.back().y; s.z = 0.02;
        e.x = swept_hull_.front().x; e.y = swept_hull_.front().y; e.z = 0.02;
        m.points.push_back(s);
        m.points.push_back(e);
      }
      pub_swept_marker_->publish(m);

      visualization_msgs::msg::Marker m1;
      m1.header = m.header;
      m1.header.frame_id = target_frame_;
      m1.ns = "swept_volume_1";
      m1.id = 1;
      m1.type = visualization_msgs::msg::Marker::LINE_LIST;
      m1.action = visualization_msgs::msg::Marker::ADD;
      m1.pose.orientation.w = 1.0;
      m1.scale.x = m.scale.x;
      m1.color.r = 1.0; m1.color.g = 0.65; m1.color.b = 0.0; m1.color.a = 1.0;
      m1.frame_locked = true;
      if (hull1.size() >= 2) {
        for (size_t i = 0; i + 1 < hull1.size(); ++i) {
          geometry_msgs::msg::Point s, e;
          s.x = hull1[i].x; s.y = hull1[i].y; s.z = 0.02;
          e.x = hull1[i+1].x; e.y = hull1[i+1].y; e.z = 0.02;
          m1.points.push_back(s);
          m1.points.push_back(e);
        }
        geometry_msgs::msg::Point s, e;
        s.x = hull1.back().x; s.y = hull1.back().y; s.z = 0.02;
        e.x = hull1.front().x; e.y = hull1.front().y; e.z = 0.02;
        m1.points.push_back(s);
        m1.points.push_back(e);
      }
      pub_swept_marker_->publish(m1);

      visualization_msgs::msg::Marker m2;
      m2.header = m.header;
      m2.header.frame_id = target_frame_;
      m2.ns = "swept_volume_2";
      m2.id = 2;
      m2.type = visualization_msgs::msg::Marker::LINE_LIST;
      m2.action = visualization_msgs::msg::Marker::ADD;
      m2.pose.orientation.w = 1.0;
      m2.scale.x = m.scale.x;
      m2.color.r = 1.0; m2.color.g = 0.0; m2.color.b = 0.0; m2.color.a = 1.0;
      m2.frame_locked = true;
      if (hull2.size() >= 2) {
        for (size_t i = 0; i + 1 < hull2.size(); ++i) {
          geometry_msgs::msg::Point s, e;
          s.x = hull2[i].x; s.y = hull2[i].y; s.z = 0.02;
          e.x = hull2[i+1].x; e.y = hull2[i+1].y; e.z = 0.02;
          m2.points.push_back(s);
          m2.points.push_back(e);
        }
        geometry_msgs::msg::Point s, e;
        s.x = hull2.back().x; s.y = hull2.back().y; s.z = 0.02;
        e.x = hull2.front().x; e.y = hull2.front().y; e.z = 0.02;
        m2.points.push_back(s);
        m2.points.push_back(e);
      }
      pub_swept_marker_->publish(m2);
    }


    geometry_msgs::msg::PolygonStamped coll1;
    coll1.header = last_footprint_.value().header;
    coll1.header.frame_id = target_frame_;
    if (collided1) {
      for (const auto &v : coll_poly1) { geometry_msgs::msg::Point32 p; p.x = (float)v.x; p.y = (float)v.y; coll1.polygon.points.push_back(p); }
    }
    // pub_level1_region_->publish(coll1);

    geometry_msgs::msg::PolygonStamped coll2;
    coll2.header = last_footprint_.value().header;
    coll2.header.frame_id = target_frame_;
    if (collided2) {
      for (const auto &v : coll_poly2) { geometry_msgs::msg::Point32 p; p.x = (float)v.x; p.y = (float)v.y; coll2.polygon.points.push_back(p); }
    }
    // pub_level2_region_->publish(coll2);

    int level = 0;
    if (collided1) 
      level = 2; 
    else if (collided2) 
      level = 1;

    std_msgs::msg::Int16 lvl; 
    lvl.data = level; 
    pub_collision_level_->publish(lvl);

    if (publish_collision_polygon_ && level >= 0) {
      const auto &chosen = (level == 2 ? coll_poly1 : coll_poly2);
      geometry_msgs::msg::PolygonStamped coll;
      coll.header = last_footprint_.value().header;
      coll.header.frame_id = target_frame_;
      for (const auto &v : chosen) { 
        geometry_msgs::msg::Point32 p; 
        p.x = (float)v.x; 
        p.y = (float)v.y; 
        coll.polygon.points.push_back(p); 
      }
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
    // RCLCPP_INFO(get_logger(), "collision tick compute: %.2f ms; collision level: %d", ms.data, level);
  }

  std::string footprint_topic_;
  std::string clusters_topic_;
  std::string target_frame_;
  double inflation_factor_1, inflation_factor_2;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_footprint_;
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_clusters_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_collision_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_collision_level_; // TODO: Level; Positionc
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_swept_marker_;
  // rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_level1_region_;
  // rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_level2_region_;
  bool publish_collision_polygon_;
  double predict_horizon_;
  int num_steps_;
  double v_ = 0.0, omega_ = 0.0;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  std::vector<Vec2> swept_hull_;
  bool swept_hull_computed_ = false;
  bool publish_swept_marker_;
  double swept_line_width_;
  std::string collision_strategy_;
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
