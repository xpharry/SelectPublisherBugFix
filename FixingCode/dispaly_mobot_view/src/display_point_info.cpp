#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <geometry_msgs/PointStamped.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

sensor_msgs::PointCloud2 old_selected_cloud;
sensor_msgs::PointCloud2 new_selected_cloud;

void callback(const geometry_msgs::PointStamped& point) {
	ROS_INFO("ENTER callbackForClick");
    cout << "frame_id: " << point.header.frame_id << endl
    	<< "  x: " << point.point.x << endl
    	<< "  y: " << point.point.y << endl
    	<< "  z: " << point.point.z << endl;
}

void callbackForOld(const sensor_msgs::PointCloud2& cloud) {
	ROS_INFO("ENTER callbackForOld");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clr_ptr(new PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_pcl_clr_ptr(new PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud, *pcl_clr_ptr);
    ROS_INFO("got cloud with %d * %d points", (int) pcl_clr_ptr->width, (int) pcl_clr_ptr->height);
    int npts = pcl_clr_ptr->points.size();
    cout<<"color pts size = "<<npts<<endl;
    if(npts == 0) return;

	geometry_msgs::PointStamped the_point;
	pcl::PointXYZRGB the_clr_point;

    Eigen::Vector3f centroid;
    Eigen::Vector3f cloud_pt;
    Eigen::Vector3i aver_clr;
    Eigen::Vector3i curr_clr;      
    centroid<<0,0,0;
    aver_clr<<0,0,0;
    for (int ipt = 0; ipt < npts; ipt++) {
        cloud_pt = pcl_clr_ptr->points[ipt].getVector3fMap();
        centroid += cloud_pt; //add all the column vectors together
        curr_clr = pcl_clr_ptr->points[ipt].getRGBVector3i();
        // cout << curr_clr.transpose() << endl;
        aver_clr += curr_clr;
    }
    centroid /= npts; 
    aver_clr /= npts;

    the_point.header.frame_id = cloud.header.frame_id;
    the_point.point.x = centroid[0];
    the_point.point.y = centroid[1];
    the_point.point.z = centroid[2];

    the_clr_point.x = centroid[0];
    the_clr_point.y = centroid[1];
    the_clr_point.z = centroid[2];
    the_clr_point.r = aver_clr[0];
    the_clr_point.g = aver_clr[1];
    the_clr_point.b = aver_clr[2];

    processed_pcl_clr_ptr->points.push_back(the_clr_point);
    processed_pcl_clr_ptr->width = 1;
    processed_pcl_clr_ptr->height = 1;
    processed_pcl_clr_ptr->header.frame_id = cloud.header.frame_id;    

    pcl::toROSMsg(*processed_pcl_clr_ptr, old_selected_cloud);

    cout << "frame_id: " << cloud.header.frame_id << endl
    	<< "  x: " << centroid[0] << endl
    	<< "  y: " << centroid[1] << endl
    	<< "  z: " << centroid[2] << endl
    	<< "  r: " << aver_clr[0] << endl
    	<< "  g: " << aver_clr[1] << endl
    	<< "  b: " << aver_clr[2] << endl;
}

void callbackForNew(const sensor_msgs::PointCloud2& cloud) {
	ROS_INFO("ENTER callbackForNew");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clr_ptr(new PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_pcl_clr_ptr(new PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud, *pcl_clr_ptr);
    ROS_INFO("got cloud with %d * %d points", (int) pcl_clr_ptr->width, (int) pcl_clr_ptr->height);
    int npts = pcl_clr_ptr->points.size();
    cout<<"color pts size = "<<npts<<endl;
	if(npts == 0) return;

	geometry_msgs::PointStamped the_point;
	pcl::PointXYZRGB the_clr_point;

    Eigen::Vector3f centroid;
    Eigen::Vector3f cloud_pt;
    Eigen::Vector3i aver_clr;
    Eigen::Vector3i curr_clr;      
    centroid<<0,0,0;
    aver_clr<<0,0,0;
    for (int ipt = 0; ipt < npts; ipt++) {
        cloud_pt = pcl_clr_ptr->points[ipt].getVector3fMap();
        centroid += cloud_pt; //add all the column vectors together
        curr_clr = pcl_clr_ptr->points[ipt].getRGBVector3i();
        // cout << curr_clr.transpose() << endl;
        aver_clr += curr_clr;
    }
    centroid /= npts; 
    aver_clr /= npts;

    the_point.header.frame_id = cloud.header.frame_id;
    the_point.point.x = centroid[0];
    the_point.point.y = centroid[1];
    the_point.point.z = centroid[2];

    the_clr_point.x = centroid[0];
    the_clr_point.y = centroid[1];
    the_clr_point.z = centroid[2];
    the_clr_point.r = aver_clr[0];
    the_clr_point.g = aver_clr[1];
    the_clr_point.b = aver_clr[2];

    processed_pcl_clr_ptr->points.push_back(the_clr_point);
    processed_pcl_clr_ptr->width = 1;
    processed_pcl_clr_ptr->height = 1;
    processed_pcl_clr_ptr->header.frame_id = cloud.header.frame_id;    

    pcl::toROSMsg(*processed_pcl_clr_ptr, new_selected_cloud);

    cout << "frame_id: " << cloud.header.frame_id << endl
    	<< "  x: " << centroid[0] << endl
    	<< "  y: " << centroid[1] << endl
    	<< "  z: " << centroid[2] << endl
    	<< "  r: " << aver_clr[0] << endl
    	<< "  g: " << aver_clr[1] << endl
    	<< "  b: " << aver_clr[2] << endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "display_point_info");
	ros::NodeHandle nh;

	// initialize subscriber
	ros::Subscriber clicked_point_subscriber = nh.subscribe("/clicked_point", 1, callback);
    ros::Subscriber old_selected_points_subscriber = nh.subscribe("/selected_points", 1, callbackForOld);
    ros::Subscriber new_selected_points_subscriber = nh.subscribe("/selected_patch", 1, callbackForNew);

    // initialize publisher
	ros::Publisher old_selected_info_pub = nh.advertise<sensor_msgs::PointCloud2>("old_selected", 1);
	ros::Publisher new_selected_info_pub = nh.advertise<sensor_msgs::PointCloud2>("new_selected", 1);

	while (ros::ok()) {
        old_selected_info_pub.publish(old_selected_cloud);
        new_selected_info_pub.publish(new_selected_cloud);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
	}

	return 0;
}