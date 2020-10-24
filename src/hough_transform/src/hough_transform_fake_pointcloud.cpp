/*This script is used to Generate Fake point cloud data
 * 1 Generate Fake point cloud data
 * 2 Publish this data to rostopic
*/

#include "ros/ros.h"
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv)
{
    //init ros
    ros::init(argc,argv,"Hough_transform_fake_data");
    ros::NodeHandle nh;

    ros::Publisher cur_cloud_pub;
    cur_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points", 10);
    pcl::PointCloud<pcl::PointXYZ>::Ptr original (new pcl::PointCloud<pcl::PointXYZ>);




    //generate fake data
    srand((int)time(0));

    ros::Rate r(1);
    while(ros::ok())
    {
        original->clear();
        float a,b,c;
        for(int i=0;i<100;i++)
        {
            for(int j=0;j<100;j++)
            {
                pcl::PointXYZ p(i,j,(rand()%100)/1000.0);
                original->points.push_back(p);
            }
        }

        for(int k = 1;k<10;k++)
        {
            a = (rand()%100)/100.0-0.5;
            b = (rand()%100)/100.0-0.5;
            c = (rand()%100)/100.0-0.5;
            int is = rand()%1000;
            int js = rand()%1000;
            for(int i=is;i<is+100;i++)
            {
                for(int j=js;j<js+100;j++)
                {
                    pcl::PointXYZ p(i+(rand()%100)/1000.0,j+(rand()%100)/1000.0, (-a*i-b*j)/c+(rand()%100)/1000.0);
                    original->points.push_back(p);
                }
            }
        }

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*original,*original, indices);

        original->width = original->points.size();
        original->height = 1;

        //publish fake point cloud
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*original, output);
        output.header.frame_id = "velodyne";
        cur_cloud_pub.publish(output);
        r.sleep();
    }
}
