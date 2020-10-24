#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>


struct EIGEN_ALIGN16 PointAllFields {
  PCL_ADD_POINT4D;
  float time_offset_us;
  uint16_t reflectivity;
  uint16_t intensity;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(PointAllFields,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, time_offset_us,time_offset_us)
                                  (uint16_t, reflectivity,reflectivity)
                                  (uint16_t, intensity,intensity)
                                  (uint8_t, ring,ring))

typedef pcl::PointCloud<PointAllFields> LoaderPointcloud;

void PointCloudStampedCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    LoaderPointcloud pointcloud;
    bool has_timing = false;
    bool has_intensity = false;
    for (const sensor_msgs::PointField& field : msg->fields)
    {
        if (field.name == "time_offset_us")
        {
            has_timing = true;
        } else if (field.name == "intensity")
        {
            has_intensity = true;
        }
    }

    if (has_timing)
    {
        pcl::fromROSMsg(*msg, pointcloud);
        std::cout<<"has_timing"<<std::endl;
    }
    else
    {
        std::cout<<"without timing"<<std::endl;
    }

    for(int i = 0;i<pointcloud.points.size();i+=10000)
    {
        std::cout<<"timestamp = "<<std::fixed<<std::setprecision(6)<<pointcloud.points[i].time_offset_us<<std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"PointCloud_Listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("lslidar_point_cloud", 10, PointCloudStampedCallback);
    ros::spin();

}
