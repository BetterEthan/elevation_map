#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <string.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud11 (new pcl::PointCloud<pcl::PointXYZRGBA>);
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

struct callback_args{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};



class SubscribeAndPublish
{
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  sensor_msgs::PointCloud2 image_data;
  pcl::PointCloud< pcl::PointXYZ >  output;

  

public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::PointCloud2>("cloud2", 50);
 
    //Topic you want to subscribe
    sub_ = n_.subscribe("/kinect2/sd/points", 1, &SubscribeAndPublish::callback, this);
  }


  void callback(const sensor_msgs::PointCloud2& msg)
  {
     

      
     image_data = msg;

     pcl::fromROSMsg(msg,*cloud11);
     
     //ROS_INFO("2::%d\t%d\t%d",(*cloud11).width,cloud11->height,cloud11->points.size());





     //viewer.showCloud(cloud11);

     pub_.publish(msg);
  }
 

 
};//End of class SubscribeAndPublish
 
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
  {
      struct callback_args* data = (struct callback_args *)args;
      if (event.getPointIndex() == -1)
          return;
      PointT current_point;
      event.getPoint(current_point.x, current_point.y, current_point.z);
      data->clicked_points_3d->points.push_back(current_point);
      // Draw clicked points in red:
      pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
      data->viewerPtr->removePointCloud("clicked_points");
      data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
      data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");

      PointT current_point2;

      current_point2.x = current_point.x + current_point.y;
      current_point2.y = -(0.866f * current_point.y + 0.5f * current_point.z - 0.81f);
      current_point2.z = -0.5f * current_point.y + 0.866f * current_point.z;

      std::cout << current_point2.x << " " << current_point2.y << " " << current_point2.z << std::endl;
  }




int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");
  ros::Time::init();
   ros::Rate loop_rate(1000);
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;
 


  while (ros::ok())
  {
    //ROS_INFO("1");
    ros::spinOnce();
  }

    //ROS_INFO("%d\t%d\t%d",(*cloud11).width,cloud11->height,cloud11->points.size());
    viewer->addPointCloud(cloud11, "bunny");

    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    std::vector<pcl::visualization::Camera> cam; 
    //Save the position of the camera           
    viewer->getCameras(cam); 

    //Print recorded points on the screen: 
    cout << "Cam: " << endl 
             << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl 
             << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl 
             << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;
    // Add point picking callback to viewer:
    static struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
     cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

    // Spin until 'Q' is pressed:
    viewer->spin();

  ros::spin();
 //ROS_INFO("2");
  return 0;
}
