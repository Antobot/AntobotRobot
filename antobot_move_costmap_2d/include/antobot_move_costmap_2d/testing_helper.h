#ifndef AM_COSTMAP_2D_TESTING_HELPER_H
#define AM_COSTMAP_2D_TESTING_HELPER_H

#include<antobot_move_costmap_2d/cost_values.h>
#include<antobot_move_costmap_2d/costmap_2d.h>
#include <antobot_move_costmap_2d/static_layer.h>
#include <antobot_move_costmap_2d/obstacle_layer.h>
#include <antobot_move_costmap_2d/inflation_layer.h>

#include <sensor_msgs/point_cloud2_iterator.h>

const double MAX_Z(1.0);

char printableCost(unsigned char cost)
{
  switch (cost)
  {
  case am_costmap_2d::NO_INFORMATION: return '?';
  case am_costmap_2d::LETHAL_OBSTACLE: return 'L';
  case am_costmap_2d::INSCRIBED_INFLATED_OBSTACLE: return 'I';
  case am_costmap_2d::FREE_SPACE: return '.';
  default: return '0' + (unsigned char) (10 * cost / 255);
  }
}

void printMap(am_costmap_2d::Costmap2D& costmap)
{
  printf("map:\n");
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      printf("%4d", int(costmap.getCost(j, i)));
    }
    printf("\n\n");
  }
}

unsigned int countValues(am_costmap_2d::Costmap2D& costmap, unsigned char value, bool equal = true)
{
  unsigned int count = 0;
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      unsigned char c = costmap.getCost(j, i);
      if ((equal && c == value) || (!equal && c != value))
      {
        count+=1;
      }
    }
  }
  return count;
}

void addStaticLayer(am_costmap_2d::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  am_costmap_2d::StaticLayer* slayer = new am_costmap_2d::StaticLayer();
  layers.addPlugin(boost::shared_ptr<am_costmap_2d::Layer>(slayer));
  slayer->initialize(&layers, "static", &tf);
}

am_costmap_2d::am_ObstacleLayer* addam_ObstacleLayer(am_costmap_2d::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  am_costmap_2d::am_ObstacleLayer* olayer = new am_costmap_2d::am_ObstacleLayer();
  olayer->initialize(&layers, "obstacles", &tf);
  layers.addPlugin(boost::shared_ptr<am_costmap_2d::Layer>(olayer));
  return olayer;
}

void addObservation(am_costmap_2d::am_ObstacleLayer* olayer, double x, double y, double z = 0.0,
                    double ox = 0.0, double oy = 0.0, double oz = MAX_Z){
  sensor_msgs::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  *iter_x = x;
  *iter_y = y;
  *iter_z = z;

  geometry_msgs::Point p;
  p.x = ox;
  p.y = oy;
  p.z = oz;

  am_costmap_2d::Observation obs(p, cloud, 100.0, 100.0);  // obstacle range = raytrace range = 100.0
  olayer->addStaticObservation(obs, true, true);
}

am_costmap_2d::InflationLayer* addInflationLayer(am_costmap_2d::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  am_costmap_2d::InflationLayer* ilayer = new am_costmap_2d::InflationLayer();
  ilayer->initialize(&layers, "inflation", &tf);
  boost::shared_ptr<am_costmap_2d::Layer> ipointer(ilayer);
  layers.addPlugin(ipointer);
  return ilayer;
}


#endif  // AM_COSTMAP_2D_TESTING_HELPER_H
