#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
ros::Time current_time;
int main(int argc, char *argv[])
{
    std::string s;
    /* code for main function */
    ros::init(argc, argv, "learning_yaml");
    ros::NodeHandle n;
    tf::tfMessage aa;
    geometry_msgs::PoseWithCovarianceStamped pose;
    current_time = ros::Time::now();

    n.param<std::string>("/kincet2",s,"dd");

    ros::Publisher pub = n.advertise<tf::tfMessage>("tf", 1000);
    ros::Publisher pub2 = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("xp_pose", 1000);
    
    pose.header.stamp = current_time;
    pose.header.frame_id = "aa";
    pose.pose.pose.position.x = 0;
    pose.pose.pose.position.y = 0;
    pose.pose.pose.position.z = 0;
    pose.pose.pose.orientation.x = 0.01;
    pose.pose.pose.orientation.y = 0.12;
    pose.pose.pose.orientation.z = 0.31;
    pose.pose.pose.orientation.w = 0.31;


    aa.transforms.resize(1);
    aa.transforms[0].header.frame_id = "xx";
    aa.transforms[0].header.stamp = current_time;
    aa.transforms[0].child_frame_id = "11";
    aa.transforms[0].transform.translation.x = 1;
    aa.transforms[0].transform.translation.y = 0;
    aa.transforms[0].transform.translation.z = 0;
    aa.transforms[0].transform.rotation.x = 1.31;
    aa.transforms[0].transform.rotation.y = 3.41;
    aa.transforms[0].transform.rotation.z = 3.21;
    aa.transforms[0].transform.rotation.w = 1.06;
	//设置循环的频率为1Hz
	ros::Rate loop_rate(100);	
 
	while(ros::ok())
	{	
        ROS_INFO("string_param_init: %s", s.c_str());
        pub.publish(aa);
        pub2.publish(pose);
        loop_rate.sleep();
    }
    
    return 0;
}