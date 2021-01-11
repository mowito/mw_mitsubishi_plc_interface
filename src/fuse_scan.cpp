#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_geometry/laser_geometry.h>
#include <vector>
#include <algorithm>

class FusedScan {

  public:
        FusedScan(ros::NodeHandle nh) : nh_(nh), 
                                        scan_front_(nh_, "sick_safetyscanners/scan_front", 20),
                                        scan_back_(nh_, "sick_safetyscanners/scan_back", 20),
                                        sync_(MySyncPolicy(20), scan_front_, scan_back_)
        {
            ROS_INFO("I'm in Constructor");
           
            //coordinate callback for both laser scan message and a non_leg_clusters message
            sync_.registerCallback(boost::bind(&FusedScan::fused_scan_callback, this, _1, _2));
            
            //publish fused_scan to scan topic
            fused_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud> ("/fuse_scan_pcl", 20);
            scan_front_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/scan_front_pcl", 20);
            scan_back_pub_  = nh_.advertise<sensor_msgs::PointCloud>("/scan_back_pcl", 20);

        }

    private:

        ros::NodeHandle nh_;
        tf::TransformListener tflistener_;
        laser_geometry::LaserProjection projector_;

        message_filters::Subscriber <sensor_msgs::LaserScan> scan_front_;
        message_filters::Subscriber <sensor_msgs::LaserScan> scan_back_;
        //message_filters::TimeSynchronizer <sensor_msgs::LaserScan, sensor_msgs::LaserScan> sync_;

        typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync_;    
        ros::Publisher fused_scan_pub_;
        ros::Publisher scan_front_pub_;
        ros::Publisher scan_back_pub_;

        sensor_msgs::PointCloud cloud_fuse;

        //callback function
        void fused_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_front, 
                                 const sensor_msgs::LaserScan::ConstPtr& scan_back)
        {
            ROS_INFO("angle_min : %f", scan_front->angle_min);
            ROS_INFO("angle_max : %f", scan_front->angle_max);
            ROS_INFO("angle_increment : %f", scan_front->angle_increment);
            if (!tflistener_.waitForTransform(scan_front->header.frame_id, "base_link", 
                scan_front->header.stamp + ros::Duration().fromSec(scan_front->ranges.size()*scan_front->time_increment), 
                ros::Duration(3.0))) {
                    return;
            }
            
            if (!tflistener_.waitForTransform(scan_back->header.frame_id, "base_link", 
                scan_back->header.stamp + ros::Duration().fromSec(scan_back->ranges.size()*scan_back->time_increment), 
                ros::Duration(3.0))) {
                    return;
            }
            //convert scan_front to pointcloud
            sensor_msgs::PointCloud cloud_front;
            projector_.transformLaserScanToPointCloud("base_link", *scan_front, cloud_front, tflistener_);
            
            //convert scan_Back to pointcloud    
            sensor_msgs::PointCloud cloud_back;
            projector_.transformLaserScanToPointCloud("base_link", *scan_back, cloud_back, tflistener_);

            //merge point clouds
            sensor_msgs::PointCloud cloud_fuse;

            cloud_fuse.header.frame_id = cloud_front.header.frame_id;
            cloud_fuse.header.seq      = cloud_front.header.seq;
            cloud_fuse.header.stamp    = cloud_front.header.stamp;

            ROS_INFO("cloud_front size : %ld", cloud_front.points.size());

            // copy the cloud_back from index 570 to 1650 in cloud_fuse points
            for (int i = 0; i < 1080; i++)
                cloud_fuse.points.push_back(cloud_back.points[i+140]);
            
            for (int i = 0; i < 1080; i++)
                cloud_fuse.points.push_back(cloud_front.points[i+140]);

            ROS_INFO("Cloud fuse size: %ld", cloud_fuse.points.size());

            fused_scan_pub_.publish(cloud_fuse);
        }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "fused_scan");
    ros::NodeHandle nh;
    FusedScan fs(nh);
    ros::spin();
}



