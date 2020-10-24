/*This script is used to verify lidar data that processed byhough transform receive lidar data
* 1 reveive lidar data from rostopic
* 2 process lidar data
*/

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include<iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>


ros::Publisher cur_cloud_pub;

void velodyne_pointsCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //convert sensor_msg to pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);


    //remove invalid point
    std::vector<int> indices;
    pcl::fromROSMsg(*msg, *input);
    pcl::removeNaNFromPointCloud(*input,*input, indices);
    pcl::PointCloud<pcl::PointXYZ>::iterator it = input->points.begin();
    while (it != input->points.end())
    {
        float x, y, z;
        x = it->x;
        y = it->y;
        z = it->z;
        if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
        {
            it = input->points.erase(it);
        }
        else
            ++it;
    }


    //hough_transform
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> deg_cloud;

    Eigen::Matrix4d A1;
    Eigen::Vector4d b1;
    Eigen::Vector4d x;
    int rand1,rand2,rand3;
    int point_size = input->points.size();

    pcl::KdTreeFLANN<pcl::PointXYZ> original_kd_tree;
    original_kd_tree.setInputCloud(input);
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    for(int i = 0;i<10000;i++)
    {
        rand1 = rand() % point_size;
        rand2 = rand() % point_size;
        rand3 = rand() % point_size;

        if(original_kd_tree.radiusSearch(input->points[rand1],5,pointIdxNKNSearch, pointNKNSquaredDistance)>0)
        {
            if(pointIdxNKNSearch.size()<5)
                continue;
            int np2,np1;
            np1 = rand()%(pointNKNSquaredDistance.size()-1)+1;
            np2 = rand()%(pointNKNSquaredDistance.size()-1)+1;
            if(np1 = np2)
                np2 = sqrt(np2)+1;
            rand2 = pointIdxNKNSearch[np1];
            rand3 = pointIdxNKNSearch[np2];
            pointIdxNKNSearch.clear();
            pointNKNSquaredDistance.clear();
        }
        else
        {
            continue;
        }

        A1(0,0) = input->points[rand1].x;
        A1(0,1) = input->points[rand1].y;
        A1(0,2) = input->points[rand1].z;
        A1(0,3) = 1;

        A1(1,0) = input->points[rand2].x;
        A1(1,1) = input->points[rand2].y;
        A1(1,2) = input->points[rand2].z;
        A1(1,3) = 1;

        A1(2,0) = input->points[rand3].x;
        A1(2,1) = input->points[rand3].y;
        A1(2,2) = input->points[rand3].z;
        A1(2,3) = 1;

        A1(3,0) = 1;
        A1(3,1) = 1;
        A1(3,2) = 1;
        A1(3,3) = 1;

        b1(0) = 0;
        b1(1) = 0;
        b1(2) = 0;
        b1(3) = 1;

        x = A1.inverse()*b1;

        float norm = sqrt(x(0)*x(0)+x(1)*x(1)+x(2)*x(2));
        pcl::PointXYZ p(x(0),x(1),x(2));
        if(p.z>0)
        {
            p.x = p.x/norm;
            p.y = p.y/norm;
            p.z = p.z/norm;
        }
        else
        {
            p.x = -1*p.x/norm;
            p.y = -1*p.y/norm;
            p.z = -1*p.z/norm;
        }


        cloud.points.push_back(p);

        float theta = atan2(p.y,p.x);
        float alpha = asin(p.z);
        //        std::cout<<"theta = "<<theta<<", alpha = "<<alpha<<std::endl;

        //        float rou = 10*pow(cos(alpha),5);
        float rou = cos(alpha);

        p.x = rou*cos(theta);
        p.y = rou*sin(theta);
        p.z = 0;
        deg_cloud.points.push_back(p);
    }

//    for(int i = 0;i<10000;i++)
//    {
//        pcl::PointXYZ p;
//        float rou = 1+((rand()%100)/1000.0);
//        //        float rou = 1;
//        p.x = rou*cos((i%360)*M_PI/180);
//        p.y = rou*sin((i%360)*M_PI/180);
//        p.z = 0;
//        deg_cloud.points.push_back(p);
//    }

    //output data
    cloud.width = cloud.points.size();
    cloud.height = 1;

    deg_cloud.width = deg_cloud.points.size();
    deg_cloud.height = 1;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(deg_cloud, output);
    output.header.frame_id = "velodyne";
    cur_cloud_pub.publish(output);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Hough_transform");
    ros::NodeHandle nh;

    cur_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("parameter_space", 1);
    ros::Subscriber sub = nh.subscribe("velodyne_points", 2000, velodyne_pointsCallBack);
    ros::spin();

}
