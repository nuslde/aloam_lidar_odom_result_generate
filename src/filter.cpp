#include <iostream>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
using namespace std;
class clip_box
{
public:
    clip_box()
    {   pub = nh.advertise<sensor_msgs::PointCloud2>("/clip_boxed_pointcloud", 1);
        sub = nh.subscribe("/lidar_top",3,&clip_box::call_back,this);
        min_point<<-1.8,-0.8,0.0,1.0;//cube的两个对角点
        max_point<<-10,10,5,5;
 
    }
 
    void call_back(const sensor_msgs::PointCloud2ConstPtr input)
    {
        cout<<"I RECEIVED INPUT !"<<endl;
        pcl::fromROSMsg(*input,*cloud_in);
 
        nh.getParam("x1",x1);
        nh.getParam("y1",y1);
        nh.getParam("z1",z1);
        nh.getParam("x2",x2);
        nh.getParam("y2",y2);
        nh.getParam("z2",z2);
        nh.getParam("negative",negative);
        min_point<<x1,y1,z1,1.0;//cube的两个对角点
        max_point<<x2,y2,z2,1.0;
 
        clipper.setMin(min_point);
        clipper.setMax(max_point);
        clipper.setInputCloud(cloud_in);
        clipper.filter(*cloud_final);
        clipper.setNegative(negative);//默认为false
 
        pcl::toROSMsg(*cloud_final,cloud_out);
        cloud_out.header.stamp = ros::Time::now();
        cloud_out.header.frame_id = "/lidar";
        pub.publish(cloud_out);
        cout<<"I PROCESSED IT !"<<endl;
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};//转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final {new pcl::PointCloud<pcl::PointXYZ>};//加工后的pcl格式
    sensor_msgs::PointCloud2 cloud_out;//转换为ros格式
    //clip
    pcl::CropBox<pcl::PointXYZ> clipper;//要包含的头文件，在官方文档里
    Eigen::Vector4f min_point;
    Eigen::Vector4f max_point;
    float x1,y1,z1;
    float x2,y2,z2;
    bool negative;
};
int
main(int argc,char **argv)
{
    ros::init(argc,argv,"clip_box");
    clip_box clip_box;
    ros::spin();
}
