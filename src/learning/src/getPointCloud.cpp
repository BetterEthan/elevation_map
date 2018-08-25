
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
using namespace grid_map;

  pcl::visualization::CloudViewer viewer("11");

class SubscribeAndPublish
{
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  sensor_msgs::PointCloud2 image_data;
  pcl::PointCloud< pcl::PointXYZ >  output;
  ros::Publisher publisher;
  ros::Publisher occupancyGridPub;
  GridMap map;

public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::PointCloud2>("cloud2", 50);
 
    //Topic you want to subscribe
    sub_ = n_.subscribe("/kinect2/sd/points", 1, &SubscribeAndPublish::callback, this);

    //发布grid_map
    publisher = n_.advertise<grid_map_msgs::GridMap>("grid_mapaa", 1, true);

    occupancyGridPub = n_.advertise<nav_msgs::OccupancyGrid>("occupancy", 1, true);

    map.add("elevation");
    map.add("sum");
    map.add("count",Matrix::Zero(map.getSize()(0),map.getSize()(1)));
    map.setFrameId("map");
    map.setGeometry(Length(5, 5), 0.03);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  }

  void callback(const sensor_msgs::PointCloud2& msg)
  {
     map.clear("sum");
     map.clear("elevation");
     map.clear("count");
     
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud11 (new pcl::PointCloud<pcl::PointXYZRGB>);
      
     image_data = msg;

     pcl::fromROSMsg(msg,*cloud11);
     
     //ROS_INFO("%d\t%d\t%d",(*cloud11).width,cloud11->height,cloud11->points.size());

    
    pcl::PointXYZ current_point2;

    for (size_t i = 0; i < cloud11->points.size(); ++i)
    {
        current_point2.x = cloud11->points[i].x + cloud11->points[i].y;
        current_point2.y = -(0.866f * cloud11->points[i].y + 0.5f * cloud11->points[i].z - 0.81f);
        current_point2.z = -0.5f * cloud11->points[i].y + 0.866f * cloud11->points[i].z;
        if(current_point2.z < 13.2f && current_point2.y > 0.03f)
        {
          cloud11->points[i].r = 255;
          cloud11->points[i].g = 0;
          cloud11->points[i].b = 0;
        }
        
        if(current_point2.x  > -2.5f && current_point2.x <2.5f && current_point2.z > -2.5f && current_point2.z < 2.5f)
        {
            //寻找该位置所对应的网格
            Index it11;
            Position position1;
            position1.x() = current_point2.x;
            position1.y() = current_point2.z;
            map.getIndex( position1,it11);

            //记录值  如果没有访问过则将初始值填入
            if(!map.isValid(it11,"sum"))
            {
                map.at("sum",it11) = current_point2.y;
                 map.at("count",it11) = 1;
                
            }
            //如果访问过则比较新点，进行更新
            else if(map.at("sum",it11) < current_point2.y)
            {
                map.at("sum",it11) = current_point2.y;
                //map.at("count",it11) += 1;
                //map.at("elevation",it11) = map.at("sum",it11)/map.at("count",it11);
               // std::cout << "ok" << endl;
            }
            
            //std::cout << map.at("count",it11) << endl;
            //std::cout << "aaa:\n" << it11 << std::endl;
        }

      //map.getIndex(position1,it11);

    }
    

     //viewer.setCameraPosition(0, 0, -2, 0, -1, 0, 0);
     viewer.showCloud(cloud11);

     //sensor_msgs::PointCloud2 outMsg;

     //pcl::toROSMsg(*cloud11,outMsg);

     pub_.publish(msg);

     //发布高程图
    // Publish grid map.
    ros::Time time = ros::Time::now();
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    //发布占据地图
    nav_msgs::OccupancyGrid occupancyGrid;
    GridMapRosConverter::toOccupancyGrid(map, "sum",-1,0.05, occupancyGrid);
    occupancyGridPub.publish(occupancyGrid);
  }
 

 
};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");
  
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;
 
  ros::spin();
 
  return 0;
}