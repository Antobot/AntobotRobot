#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64.hpp>
#include <costmap_converter/costmap_converter_interface.h>
#include <pluginlib/class_loader.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>

class OccGridConverterBridge : public rclcpp::Node {
public:
  OccGridConverterBridge() : rclcpp::Node("occgrid_converter_bridge"),
    loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons")
  {
    declare_parameter<std::string>("converter_plugin", "costmap_converter::CostmapToPolygonsDBSMCCH");
    declare_parameter<int>("occupied_min_value", 60);
    declare_parameter<double>("cluster_max_distance", 0.3);
    declare_parameter<int>("cluster_min_pts", 3);
    declare_parameter<int>("cluster_max_pts", 30);
    declare_parameter<double>("convex_hull_min_pt_separation", 0.0);
    get_parameter("converter_plugin", plugin_name_);
    get_parameter("occupied_min_value", occupied_min_value_);
    get_parameter("cluster_max_distance", cluster_max_distance_);
    get_parameter("cluster_min_pts", cluster_min_pts_);
    get_parameter("cluster_max_pts", cluster_max_pts_);
    get_parameter("convex_hull_min_pt_separation", convex_hull_min_pt_sep_);

    converter_ = loader_.createSharedInstance(plugin_name_);
    plugin_nh_ = std::make_shared<rclcpp::Node>("occgrid_converter_plugin", "costmap_converter");
    plugin_nh_->declare_parameter<double>("cluster_max_distance", cluster_max_distance_);
    plugin_nh_->declare_parameter<int>("cluster_min_pts", cluster_min_pts_);
    plugin_nh_->declare_parameter<int>("cluster_max_pts", cluster_max_pts_);
    plugin_nh_->declare_parameter<double>("convex_hull_min_pt_separation", convex_hull_min_pt_sep_);
    converter_->initialize(plugin_nh_);

    pub_obs_ = create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("/costmap_converter/obstacles", 10);
    pub_marker_ = create_publisher<visualization_msgs::msg::Marker>("/costmap_converter/polygon_markers", 10);
    pub_time_ = create_publisher<std_msgs::msg::Float64>("/costmap_converter/plugin_compute_ms", 10);

    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap/costmap", rclcpp::QoS(10),
      std::bind(&OccGridConverterBridge::cb, this, std::placeholders::_1));
  }

private:
  void cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    auto t0 = std::chrono::steady_clock::now();
    int w = msg->info.width;
    int h = msg->info.height;
    double res = msg->info.resolution;

    nav2_costmap_2d::Costmap2D costmap(w, h, res, msg->info.origin.position.x, msg->info.origin.position.y);
    unsigned char * data = costmap.getCharMap();

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

    converter_->setCostmap2D(&costmap);
    converter_->compute();
    auto obstacles = converter_->getObstacles();
    if (obstacles) {
      auto out = *obstacles;
      out.header = msg->header;
      pub_obs_->publish(out);

      visualization_msgs::msg::Marker line_list;
      line_list.header = msg->header;
      line_list.ns = "occgrid_polygons";
      line_list.action = visualization_msgs::msg::Marker::ADD;
      line_list.pose.orientation.w = 1.0;
      line_list.id = 0;
      line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
      line_list.scale.x = res;
      line_list.color.g = 1.0;
      line_list.color.a = 1.0;

      for (const auto &obs : out.obstacles) {
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
      pub_marker_->publish(line_list);
    }
    auto t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> dt = t1 - t0;
    std_msgs::msg::Float64 ms; ms.data = dt.count();
    pub_time_->publish(ms);
  }

  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> loader_;
  std::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter_;
  rclcpp::Node::SharedPtr plugin_nh_;
  std::string plugin_name_;
  int occupied_min_value_;
  double cluster_max_distance_;
  int cluster_min_pts_;
  int cluster_max_pts_;
  double convex_hull_min_pt_sep_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr pub_obs_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_time_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccGridConverterBridge>());
  rclcpp::shutdown();
  return 0;
}
