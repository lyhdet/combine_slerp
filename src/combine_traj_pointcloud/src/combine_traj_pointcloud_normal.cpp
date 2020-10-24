#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "geometry_msgs/PoseStamped.h"
#include "iostream"
#include <iomanip>
#include <vector>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>


int start_time;
int end_time;

#define PAUSE printf("Press Enter key to continue..."); fgetc(stdin);

struct EIGEN_ALIGN16 PointAllFields {
    PCL_ADD_POINT4D;
    int32_t time_offset_us;
    uint16_t reflectivity;
    uint16_t intensity;
    uint8_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(PointAllFields,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (int32_t, time_offset_us,time_offset_us)
                                  (uint16_t, reflectivity,reflectivity)
                                  (uint16_t, intensity,intensity)
                                  (uint8_t, ring,ring))

typedef pcl::PointCloud<PointAllFields> LoaderPointcloud;

struct POS
{
    double x, y, z, h, p, r;

    POS(double _x,double _y,double _z, double _h, double _p, double _r)
    {
        x = _x;
        y = _y;
        z = _z;
        h = _h;
        p = _p;
        r = _r;
    }

    std::string print()
    {
        return "x = " + std::to_string(x) + " y = " + std::to_string(y) + "z = " + std::to_string(z) + " h = " + std::to_string(h) + "p = " + std::to_string(p) + " r = " + std::to_string(r);
    }
};
std::vector<std::vector<POS> > pos_table;
ros::Publisher cur_cloud_pub;

std::vector<std::string> split(const char *s, const char *delim)
{
    std::vector<std::string> result;
    if (s && strlen(s))
    {
        int len = strlen(s);
        char *src = new char[len + 1];
        strcpy(src, s);
        src[len] = '\0';
        char *tokenptr = strtok(src, delim);
        while (tokenptr != NULL)
        {
            std::string tk = tokenptr;
            result.emplace_back(tk);
            tokenptr = strtok(NULL, delim);
        }
        delete[] src;
    }
    return result;
}


void PointCloudStampedCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    LoaderPointcloud pointcloud;
    pcl::PointCloud<pcl::PointXYZ> cur_cloud;
    static pcl::PointCloud<pcl::PointXYZ> tot_cloud;
    sensor_msgs::PointCloud2 output;

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

    double base_time = pointcloud.header.stamp;
    double time_offset = 1759.13;

    //    std::cout<<"base_time = "<<base_time<<std::endl;

    //    PAUSE;
    //initialize the transform from imu to lidar
    Eigen::Matrix3d tf_r;
    /*1225 bd
    tf_r<<-0.0550001,-0.997242,-0.049827,0.339994,-0.0656251,0.938135,-0.938818,0.0346566,0.342666;

    Eigen::Isometry3d Tf = Eigen::Isometry3d::Identity();
    Tf.rotate(tf_r);
    Tf.pretranslate(Eigen::Vector3d(0.00288644,0.0117165,0.000833863));
*/

    tf_r<<-0.0521543,-0.997754,-0.0420328,0.338205,-0.05725,0.93933,-0.939626,0.0347745,0.340431;

    Eigen::Isometry3d Tf = Eigen::Isometry3d::Identity();
    Tf.rotate(tf_r);
    Tf.pretranslate(Eigen::Vector3d(0.0134603,0.00202759,-9.90725e-05));


    std::cout<<Tf.matrix()<<std::endl;
    for(int i = 0;i< pointcloud.points.size();i+=1)
    {
        double timestamp = (pointcloud.points[i].time_offset_us+time_offset+base_time)*1e-6 - start_time;
        //extract the pose transform of each point
        POS cur_pos = pos_table[int(timestamp)][int(timestamp*1000)%1000];
        //        POS cur_pos = pos_table[0][0];

        //        std::cout<<"pointcloud.points[i].time_offset_us = "<<pointcloud.points[i].time_offset_us
        //                <<" , timestamp = "<<timestamp
        //               <<" , second = "<<int(timestamp)<<" , msecond = "<<int(timestamp*1000)%1000<<std::endl;

        //        PAUSE;

        Eigen::Matrix3d R_M = Eigen::AngleAxisd(-1*cur_pos.h*M_PI/180.0,Eigen::Vector3d(0,0,1)).matrix()*Eigen::AngleAxisd(1*cur_pos.p*M_PI/180.0,Eigen::Vector3d(1,0,0)).matrix()*Eigen::AngleAxisd(cur_pos.r*M_PI/180.0,Eigen::Vector3d(0,1,0)).matrix();
        Eigen::Isometry3d Tw = Eigen::Isometry3d::Identity();
        Tw.rotate(R_M);
        Tw.pretranslate(Eigen::Vector3d(cur_pos.x,cur_pos.y,cur_pos.z));

        //compute the whole transform(Transform combine) Tc = Tw*Tf
        Eigen::Isometry3d Tc = Tw*Tf;

        //transform each point form lidar coordinate to world coordinate
        Eigen::Vector3d cur_point_eigen(pointcloud.points[i].x,pointcloud.points[i].y,pointcloud.points[i].z);
        cur_point_eigen = Tc*cur_point_eigen;

        pcl::PointXYZ cur_point_pcl;
        cur_point_pcl.x = cur_point_eigen(0);
        cur_point_pcl.y = cur_point_eigen(1);
        cur_point_pcl.z = cur_point_eigen(2);

        cur_cloud.points.push_back(cur_point_pcl);
        tot_cloud.points.push_back(cur_point_pcl);


    }

    //save_point_cloud
    if(tot_cloud.points.size()>1000000)
    {
        tot_cloud.height = 1;
        tot_cloud.width =  tot_cloud.points.size();
        tot_cloud.is_dense = false;

        static int partcnt = 0;
        std::string filename = "single_4_5-5_part_";
        filename = filename + std::to_string(partcnt)+".pcd";
        std::cout<<filename<<std::endl;
        pcl::io::savePCDFile(filename,tot_cloud);
        tot_cloud.points.clear();
        partcnt++;

    }

    tot_cloud.height = 1;
    tot_cloud.width =  tot_cloud.points.size();

    //publish the combined point cloud
    pcl::toROSMsg(tot_cloud, output);
    output.header.frame_id = "odom";
    cur_cloud_pub.publish(output);



}

bool init_start_end_time(const std::string& bag_path)
{
    rosbag::Bag bag;
    try {
        bag.open(bag_path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return false;
    }

    std::vector<std::string> types;
    types.push_back(std::string("sensor_msgs/PointCloud2"));
    rosbag::View view(bag, rosbag::TypeQuery(types));

    int scan_num = 0;
    int base_time = 0;
    start_time = 0;

//    std::vector<int> time;

    for (const rosbag::MessageInstance& m : view)
    {
        std::cout << " Loading scan: \e[1m" << scan_num++ << "\e[0m from ros bag"
                  << '\r' << std::flush;

        LoaderPointcloud pointcloud;
        pcl::PointCloud<pcl::PointXYZ> cur_cloud;
        sensor_msgs::PointCloud2 output;
        sensor_msgs::PointCloud2 msg = *(m.instantiate<sensor_msgs::PointCloud2>());

        pcl::fromROSMsg(msg, pointcloud);

        base_time = pointcloud.header.stamp*1e-6;
        if(base_time != 0)
        {
            end_time = base_time + 10;
            if(start_time == 0)
            {
                start_time = base_time - 10;
            }
        }
    }

    return true;
}


bool load_lidar_and_combine(const std::string& bag_path,const std::string& output_save_path,int save_per_point,const std::string& rot_trans_str,double time_offset)
{
    rosbag::Bag bag;
    pcl::PointCloud<pcl::PointXYZ> tot_cloud;
    //    double time_offset = 1759.13;
    Eigen::Matrix3d tf_r;
    Eigen::Vector3d tf_t;

    std::vector<std::string> rot_trans;
    rot_trans = split(rot_trans_str.c_str(),",");
    tf_r<<atof(rot_trans[0].c_str()),atof(rot_trans[1].c_str()),atof(rot_trans[2].c_str()),atof(rot_trans[4].c_str()),atof(rot_trans[5].c_str()),atof(rot_trans[6].c_str()),atof(rot_trans[8].c_str()),atof(rot_trans[9].c_str()),atof(rot_trans[10].c_str());
    tf_t<<atof(rot_trans[3].c_str()),atof(rot_trans[7].c_str()),atof(rot_trans[11].c_str());
    Eigen::Isometry3d Tf = Eigen::Isometry3d::Identity();
    Tf.rotate(tf_r);
    Tf.pretranslate(tf_t);
    std::cout<<Tf.matrix()<<std::endl;

    try {
        bag.open(bag_path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return false;
    }

    std::vector<std::string> types;
    types.push_back(std::string("sensor_msgs/PointCloud2"));
    rosbag::View view(bag, rosbag::TypeQuery(types));

    size_t scan_num = 0;
    for (const rosbag::MessageInstance& m : view)
    {
        std::cout << " Loading scan: \e[1m" << scan_num++ << "\e[0m from ros bag"
                  << '\r' << std::flush;

        LoaderPointcloud pointcloud;
        pcl::PointCloud<pcl::PointXYZ> cur_cloud;
        sensor_msgs::PointCloud2 output;
        sensor_msgs::PointCloud2 msg = *(m.instantiate<sensor_msgs::PointCloud2>());

        bool has_timing = false;
        bool has_intensity = false;
        for (const sensor_msgs::PointField& field : msg.fields)
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
            pcl::fromROSMsg(msg, pointcloud);
            std::cout<<"has_timing"<<std::endl;
        }
        else
        {
            std::cout<<"without timing"<<std::endl;
            return false;
        }

        for(int i = 0;i<pointcloud.points.size();i+=10000)
        {
            std::cout<<"timestamp = "<<std::fixed<<std::setprecision(6)<<pointcloud.points[i].time_offset_us<<std::endl;
        }

        double base_time = pointcloud.header.stamp;
        for(int i = 0;i< pointcloud.points.size();i+=1)
        {
            double timestamp = (pointcloud.points[i].time_offset_us+time_offset+base_time)*1e-6 - start_time;
            //extract the pose transform of each point
            POS cur_pos = pos_table[int(timestamp)][int(timestamp*1000)%1000];

            Eigen::Matrix3d R_M = Eigen::AngleAxisd(-1*cur_pos.h*M_PI/180.0,Eigen::Vector3d(0,0,1)).matrix()*Eigen::AngleAxisd(1*cur_pos.p*M_PI/180.0,Eigen::Vector3d(1,0,0)).matrix()*Eigen::AngleAxisd(cur_pos.r*M_PI/180.0,Eigen::Vector3d(0,1,0)).matrix();
            Eigen::Isometry3d Tw = Eigen::Isometry3d::Identity();
            Tw.rotate(R_M);
            Tw.pretranslate(Eigen::Vector3d(cur_pos.x,cur_pos.y,cur_pos.z));

            //compute the whole transform(Transform combine) Tc = Tw*Tf
            Eigen::Isometry3d Tc = Tw*Tf;

            //transform each point form lidar coordinate to world coordinate
            Eigen::Vector3d cur_point_eigen(pointcloud.points[i].x,pointcloud.points[i].y,pointcloud.points[i].z);
            cur_point_eigen = Tc*cur_point_eigen;

            pcl::PointXYZ cur_point_pcl;
            cur_point_pcl.x = cur_point_eigen(0);
            cur_point_pcl.y = cur_point_eigen(1);
            cur_point_pcl.z = cur_point_eigen(2);

            cur_cloud.points.push_back(cur_point_pcl);
            tot_cloud.points.push_back(cur_point_pcl);


        }

        //save_point_cloud
        if(tot_cloud.points.size()>save_per_point)
        {
            tot_cloud.height = 1;
            tot_cloud.width =  tot_cloud.points.size();
            tot_cloud.is_dense = false;

            static int partcnt = 0;
            std::string filename = output_save_path + "/single_4_5-5_part_";
            filename = filename + std::to_string(partcnt)+".pcd";
            std::cout<<filename<<std::endl;
            pcl::io::savePCDFile(filename,tot_cloud);
            tot_cloud.points.clear();
            partcnt++;

        }

        tot_cloud.height = 1;
        tot_cloud.width =  tot_cloud.points.size();

        //publish the combined point cloud
        pcl::toROSMsg(tot_cloud, output);
        output.header.frame_id = "odom";
        cur_cloud_pub.publish(output);
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Combine_Traj_PointCloud");
    ros::NodeHandle nh("~");
    std::string input_traj_file;
    std::string input_lidar_bag;
    std::string output_save_path;
    int save_per_point;
    std::string rot_trans;
    double time_offset;

    nh.param("input_traj_file",input_traj_file,std::string("input_traj_file"));
    nh.param("input_lidar_bag",input_lidar_bag,std::string("input_lidar_bag"));
    nh.param("output_save_path",output_save_path,std::string("output_save_path"));
    nh.param("save_per_point",save_per_point,1000000);
    nh.param("rot_trans",rot_trans,std::string("0,0,0,0,0,0,0,0,0,0,0,0"));
    nh.param("time_offset",time_offset,1759.13);

    init_start_end_time(input_lidar_bag);

    std::cout<<"start_time = "<<start_time<<" , end_time = "<<end_time<<std::endl;

    std::ifstream tf(input_traj_file,std::ios::in);

    bool is_relative = true;

    int last_time = end_time-start_time;
    pos_table.resize(last_time);
    char s[1000] = {0};

    std::string _date = "";
    std::string _second = "";
    std::string _sx = "";
    std::string _sy = "";
    std::string _sz = "";
    std::string _spitch = "";
    std::string _sroll = "";
    std::string _syaw = "";

    int line_cnt = 0;
    double base_east,base_north,base_up;

    while(!tf.eof())
    {
        tf.getline(s,sizeof(s));
        //        std::cout<<strlen(s)<<std::endl;
        //        std::cout<<s<<std::endl;
        std::stringstream word(s);
        //remove time data
        word>>_date;
        word>>_second;
        //parse useful data
        word>>_sx;
        word>>_sy;
        word>>_sz;
        word>>_syaw;
        word>>_spitch;
        word>>_sroll;

        double dx,dy,dz,droll,dpitch,dyaw,dsecond;

        if(line_cnt == 0)
        {
            if(is_relative)
            {
                dsecond = atof(_second.c_str());
                dx = atof(_sx.c_str());
                dy = atof(_sy.c_str());
                dz = atof(_sz.c_str());
                droll = atof(_sroll.c_str());
                dpitch = atof(_spitch.c_str());
                dyaw = atof(_syaw.c_str());
                base_east = dx;
                base_north = dy;
                base_up = dz;
                dx -= base_east;
                dy -= base_north;
                dz -= base_up;
            }
            else
            {
                dsecond = atof(_second.c_str());
                dx = atof(_sx.c_str());
                dy = atof(_sy.c_str());
                dz = atof(_sz.c_str());
                droll = atof(_sroll.c_str());
                dpitch = atof(_spitch.c_str());
                dyaw = atof(_syaw.c_str());
                base_east = 0;
                base_north = 0;
                base_up = 0;
                dx -= base_east;
                dy -= base_north;
                dz -= base_up;
            }

        }else
        {
            dsecond = atof(_second.c_str());
            dx = atof(_sx.c_str()) - base_east;
            dy = atof(_sy.c_str()) - base_north;
            dz = atof(_sz.c_str()) - base_up;
            droll = atof(_sroll.c_str());
            dpitch = atof(_spitch.c_str());
            dyaw = atof(_syaw.c_str());
        }


        if(dsecond<start_time)
            continue;


        if(dsecond>=end_time)
            continue;

        //std::cout<<s<<std::endl;
        POS cur_pos(dx, dy, dz, dyaw, dpitch, droll);
        std::cout<<"dsecond = "<<dsecond<<"line_cnt = "<<line_cnt<<std::endl;
        pos_table[int(dsecond - start_time)].push_back(cur_pos);
        line_cnt++;
    }

    tf.close();

    std::ofstream offset(output_save_path + "/offset.txt",std::ios::app);

    if(!offset.is_open())
    {
        std::cout<<"failed to save offset"<<std::endl;
        return 0;
    }

    offset<<std::fixed<<std::setprecision(5)<<"x = "<<base_east<<", y = "<<base_north<<"z = "<<base_up<<std::endl;

    offset.close();

    for(int i = 0;i<last_time;i++)
    {
        std::cout<<pos_table[i].size()<<std::endl;
    }


    //    std::cout<<"dx = "<<base_east<<" dy = "<<base_north<<" dz = "<<base_up<<std::endl;

    cur_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("cur_cloud", 10);

    load_lidar_and_combine(input_lidar_bag,output_save_path,save_per_point,rot_trans,time_offset);
    printf("x = %10.5lf, y = %10.5lf, z = %10.5lf",base_east,base_north,base_up);
    //    ros::Subscriber sub = nh.subscribe("lslidar_point_cloud", 4000, PointCloudStampedCallback);
    //    ros::spin();

    //    std::cout<<pos_table.size()<<std::endl;
    return 0;
}
