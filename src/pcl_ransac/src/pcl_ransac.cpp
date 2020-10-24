// Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/main.cpp

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <PointCloud.h>
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include<iostream>

ros::Publisher cur_cloud_pub;
void velodyne_pointsCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> pointcloud,processed;
    pcl::fromROSMsg(*msg, pointcloud);
    PointCloud pc;

    for(int i = 0;i<pointcloud.points.size();i++)
    {
        pc.push_back(Point(Vec3f(pointcloud[i].x,pointcloud[i].y,pointcloud[i].z)));
    }

    // set the bbox in pc
    pc.setBBox(Vec3f(-150,-150,-150), Vec3f(150,150,150));
    //void calcNormals( float radius, unsigned int kNN = 20, unsigned int maxTries = 100 );
    pc.calcNormals(3);


    std::cout << "added " << pc.size() << " points" << std::endl;

    RansacShapeDetector::Options ransacOptions;
    ransacOptions.m_epsilon = .2f * pc.getScale(); // set distance threshold to .01f of bounding box width
    // NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
    ransacOptions.m_bitmapEpsilon = .02f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
    // NOTE: This threshold is NOT multiplied internally!
    ransacOptions.m_normalThresh = .9f; // this is the cos of the maximal normal deviation
    ransacOptions.m_minSupport = 10; // this is the minimal numer of points required for a primitive
    ransacOptions.m_probability = .001f; // this is the "probability" with which a primitive is overlooked

    RansacShapeDetector detector(ransacOptions); // the detector object

    // set which primitives are to be detected by adding the respective constructors
    detector.Add(new PlanePrimitiveShapeConstructor());
    detector.Add(new CylinderPrimitiveShapeConstructor());

    MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes
    size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection

    for(int j = remaining;j<pc.size();j++)
    {
        pcl::PointXYZ p(pc[j].pos[0],pc[j].pos[1],pc[j].pos[2]);
        processed.points.push_back(p);
    }

    processed.width = processed.points.size();
    processed.height = 1;
    std::cout<<"classified = "<<processed.points.size()<<" , remaining = "<<remaining<<std::endl;
    pcl::toROSMsg(processed, output);
    output.header.frame_id = "velodyne";
    cur_cloud_pub.publish(output);


}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"Combine_Traj_PointCloud");
    ros::NodeHandle nh;

    cur_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("cur_cloud", 10);
    ros::Subscriber sub = nh.subscribe("velodyne_points", 2000, velodyne_pointsCallBack);
    ros::spin();

}
