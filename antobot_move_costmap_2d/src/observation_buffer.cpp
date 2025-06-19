/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 * 
 * For faster PointCloud2/laserScan callback (BufferCloud), opengl is 
 * used for transfroming the data in to global frame
 *********************************************************************/
#include <antobot_move_costmap_2d/observation_buffer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

using namespace glm;

using namespace std;
using namespace tf2;

namespace am_costmap_2d
{
ObservationBuffer::ObservationBuffer(string topic_name, double observation_keep_time, double expected_update_rate,
                                     double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                                     double raytrace_range, tf2_ros::Buffer& tf2_buffer, string global_frame,
                                     string sensor_frame, double tf_tolerance) :
    tf2_buffer_(tf2_buffer), observation_keep_time_(observation_keep_time), expected_update_rate_(expected_update_rate),
    last_updated_(ros::Time::now()), global_frame_(global_frame), sensor_frame_(sensor_frame), topic_name_(topic_name),
    min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
    obstacle_range_(obstacle_range), raytrace_range_(raytrace_range), tf_tolerance_(tf_tolerance)
{
}

ObservationBuffer::~ObservationBuffer()
{
}

bool ObservationBuffer::setGlobalFrame(const std::string new_global_frame)
{
  ros::Time transform_time = ros::Time::now();
  std::string tf_error;

  geometry_msgs::TransformStamped transformStamped;
  if (!tf2_buffer_.canTransform(new_global_frame, global_frame_, transform_time, ros::Duration(tf_tolerance_), &tf_error))
  {
    ROS_ERROR("Transform between %s and %s with tolerance %.2f failed: %s.", new_global_frame.c_str(),
              global_frame_.c_str(), tf_tolerance_, tf_error.c_str());
    return false;
  }

  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    try
    {
      Observation& obs = *obs_it;

      geometry_msgs::PointStamped origin;
      origin.header.frame_id = global_frame_;
      origin.header.stamp = transform_time;
      origin.point = obs.origin_;

      // we need to transform the origin of the observation to the new global frame
      tf2_buffer_.transform(origin, origin, new_global_frame);
      obs.origin_ = origin.point;

      // we also need to transform the cloud of the observation to the new global frame
      tf2_buffer_.transform(*(obs.cloud_), *(obs.cloud_), new_global_frame);
    }
    catch (TransformException& ex)
    {
      ROS_ERROR("TF Error attempting to transform an observation from %s to %s: %s", global_frame_.c_str(),
                new_global_frame.c_str(), ex.what());
      return false;
    }
  }

  // now we need to update our global_frame member
  global_frame_ = new_global_frame;
  return true;
}

void ObservationBuffer::bufferCloud(const sensor_msgs::PointCloud2& cloud)
{
  // To Calculate elapsed time
  // struct timeval start, end;
  // double start_t, end_t, t_diff;
  // gettimeofday(&start, NULL);
    
  geometry_msgs::PointStamped global_origin;

  // create a new observation on the list to be populated
  observation_list_.push_front(Observation());

  // check whether the origin frame has been set explicitly or whether we should get it from the cloud
  string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  try
  {
    //ROS_WARN("bufferCloud origin frame : %s",origin_frame.c_str()); // zed 
    // given these observations come from sensors... we'll need to store the origin pt of the sensor
    geometry_msgs::PointStamped local_origin;
    local_origin.header.stamp = cloud.header.stamp;
    local_origin.header.frame_id = origin_frame;
    local_origin.point.x = 0;
    local_origin.point.y = 0;
    local_origin.point.z = 0;
    //ROS_WARN("global_frame 1 :  %s",global_frame_.c_str());
    tf2_buffer_.transform(local_origin, global_origin, global_frame_);
    tf2::convert(global_origin.point, observation_list_.front().origin_);

    // make sure to pass on the raytrace/obstacle range of the observation buffer to the observations
    observation_list_.front().raytrace_range_ = raytrace_range_;
    observation_list_.front().obstacle_range_ = obstacle_range_;
    observation_list_.front().min_obstacle_height_ = min_obstacle_height_;
    observation_list_.front().max_obstacle_height_ = max_obstacle_height_;

    sensor_msgs::PointCloud2 global_frame_cloud;
    geometry_msgs::TransformStamped t_in;

    t_in = tf2_buffer_.lookupTransform(
          global_frame_,
          sensor_frame_,
          ros::Time(0.0),
          ros::Duration(0.1));

    
    // Original version
    //tf2::doTransform(cloud, global_frame_cloud, transform);
    
    global_frame_cloud = cloud;
    global_frame_cloud.header = t_in.header;
    // Eigen::Transform<float,3,Eigen::Isometry> t = Eigen::Translation3f(t_in.transform.translation.x, t_in.transform.translation.y,
    //                                                                   t_in.transform.translation.z) * Eigen::Quaternion<float>(
    //                                                                   t_in.transform.rotation.w, t_in.transform.rotation.x,
    //                                                                   t_in.transform.rotation.y, t_in.transform.rotation.z);

    sensor_msgs::PointCloud2ConstIterator<float> x_in(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_in(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_in(cloud, "z");

    sensor_msgs::PointCloud2Iterator<float> x_out(global_frame_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_out(global_frame_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_out(global_frame_cloud, "z");
 
    // Get Translation Matrix
    glm::mat4 TranslationMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(t_in.transform.translation.x, t_in.transform.translation.y, t_in.transform.translation.z));
    //ROS_WARN(" translate %s",glm::to_string(myMatrix).c_str());

    // Get Rotation Matrix
    quat my_quaternion = quat(t_in.transform.rotation.w,t_in.transform.rotation.x,t_in.transform.rotation.y,t_in.transform.rotation.z);
    glm::mat4 RotationMatrix = glm::toMat4(my_quaternion);
    //ROS_WARN("Rotation %s",glm::to_string(RotationMatrix).c_str());
    
    // Get final Transfrom Matrix
    glm::mat4 myModelMatrix = TranslationMatrix * RotationMatrix;
    //ROS_WARN("final %s",glm::to_string(myModelMatrix).c_str());

    //Eigen::Vector3f point;
    for(; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out) {
      glm::vec4 myVector(*x_in, *y_in, *z_in, 1.0f); // 1.0f for 4th value makes this vector a point in 3D space. Otherwise it becomes direction vector
      glm::vec4 myTransformedVector = myModelMatrix * myVector;
      *x_out = myTransformedVector.x;
      *y_out = myTransformedVector.y;
      *z_out = myTransformedVector.z;
      // Original version with Eigen
      // point = t * Eigen::Vector3f(*x_in, *y_in, *z_in);
      // *x_out = point.x();
      // *y_out = point.y();
      // *z_out = point.z();
      //ROS_WARN("%f %f",myTransformedVector.x,point.x());
    }

    

    // transform the point cloud
    //tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_); // this method takes 2 seconds depending on the pointcloud size
    //ROS_WARN("global_frame 2 :  %s",global_frame_.c_str());
    global_frame_cloud.header.stamp = cloud.header.stamp;

    // now we need to remove observations from the cloud that are below or above our height thresholds
    sensor_msgs::PointCloud2& observation_cloud = *(observation_list_.front().cloud_);
    observation_cloud.height = global_frame_cloud.height;
    observation_cloud.width = global_frame_cloud.width;
    observation_cloud.fields = global_frame_cloud.fields;
    observation_cloud.is_bigendian = global_frame_cloud.is_bigendian;
    observation_cloud.point_step = global_frame_cloud.point_step;
    observation_cloud.row_step = global_frame_cloud.row_step;
    observation_cloud.is_dense = global_frame_cloud.is_dense;

    unsigned int cloud_size = global_frame_cloud.height*global_frame_cloud.width;
    sensor_msgs::PointCloud2Modifier modifier(observation_cloud);
    modifier.resize(cloud_size);
    unsigned int point_count = 0;

    // copy over the points that are within our height bounds
    sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");
    std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin(), iter_global_end = global_frame_cloud.data.end();
    std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();
    for (; iter_global != iter_global_end; ++iter_z, iter_global += global_frame_cloud.point_step)
    {
      // applying max&min obstacle height will be done in  obstacle_layer.cpp updateBounds
      std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
      iter_obs += global_frame_cloud.point_step;
      ++point_count;
      // if ((*iter_z) <= max_obstacle_height_
      //     && (*iter_z) >= min_obstacle_height_)
      // {
      //   std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
      //   iter_obs += global_frame_cloud.point_step;
      //   ++point_count;
      // }
    }

    // resize the cloud for the number of legal points
    modifier.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
  }
  catch (TransformException& ex)
  {
    // if an exception occurs, we need to remove the empty observation from the list
    observation_list_.pop_front();
    ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(),
              cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  last_updated_ = ros::Time::now();
  // gettimeofday(&end, NULL);
  // start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  // end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  // t_diff = end_t - start_t;
  // ROS_WARN("Sensor callback time: %.9f", t_diff);

  // we'll also remove any stale observations from the list
  purgeStaleObservations();
}

// returns a copy of the observations
void ObservationBuffer::getObservations(vector<Observation>& observations)
{
  // first... let's make sure that we don't have any stale observations
  purgeStaleObservations();

  // now we'll just copy the observations for the caller
  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  { 
    //ROS_INFO("observation push back");
    observations.push_back(*obs_it);
  }
}


void ObservationBuffer::purgeAllObservations()
{
  if (!observation_list_.empty())
  {
      ROS_INFO("Purge All observation");
      list<Observation>::iterator obs_it = observation_list_.begin();
      observation_list_.clear();
      return;


  }
}
void ObservationBuffer::purgeStaleObservations()
{
  if (!observation_list_.empty())
  {
    list<Observation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == ros::Duration(0.0))
    { 
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      Observation& obs = *obs_it;
      // check if the observation is out of date... and if it is, remove it and those that follow from the list
      if ((last_updated_ - obs.cloud_->header.stamp) > observation_keep_time_)
      {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

bool ObservationBuffer::isCurrent() const
{
  if (expected_update_rate_ == ros::Duration(0.0))
    return true;

  bool current = (ros::Time::now() - last_updated_).toSec() <= expected_update_rate_.toSec();
  if (!current)
  {
    ROS_WARN(
        "The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
        topic_name_.c_str(), (ros::Time::now() - last_updated_).toSec(), expected_update_rate_.toSec());
  }
  return current;
}

void ObservationBuffer::resetLastUpdated()
{
  last_updated_ = ros::Time::now();
}
}  // namespace am_costmap_2d

