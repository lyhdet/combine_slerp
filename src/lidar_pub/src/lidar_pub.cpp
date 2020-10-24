/*
 *parse hexadecimal data to lidar real data
*/
#include <vector>
#include <fstream>
#include <iostream>
#include <cstring>
#include <math.h>
#include <cmath>
#include <iomanip>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

//ROS
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>


const double PI = 3.1415926535;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


struct Point
{
    double x, y, z, r, g, b, timestamp;
};
std::vector<float> lidar_angle;
std::vector<std::vector<int> > lidar_dist;
std::vector<std::vector<int> > lidar_inst;
std::vector<std::vector<double> > tstamp_list;
std::vector<Point> cur_lidar_pointcloud;

ros::Publisher pcl_pub;
ros::Publisher pose_pub;
PointCloud Cloud;
sensor_msgs::PointCloud2 output;

bool jumpflag = true;
bool finishflag = false;


int start_time = 19421;
//int start_time = 19000;
int last_time = 560;

void LidarParsing()
{
    if(tstamp_list[0][0]<start_time)
    {
        jumpflag = true;
        return;
    }

    if(tstamp_list[0][0]>start_time+last_time)
        finishflag = true;

    jumpflag = false;

    static int frame_count = 0;
    cur_lidar_pointcloud.clear();
    double cosTheta[16] = { 0 };
    double sinTheta[16] = { 0 };
    for (int i = 0; i < 16; i++)
    {
        int theta = i;
        if (i % 2 == 0)
            theta = i - 15;
        cosTheta[i] = cos(theta * PI / 180.f);
        sinTheta[i] = sin(theta * PI / 180.f);
    }

    std::vector<double> sinAngle;
    std::vector<double> cosAngle;

    for (int i = 0; i < (int)lidar_angle.size(); i++)
    {
        sinAngle.push_back(sin(lidar_angle[i] * PI / 180.f));
        cosAngle.push_back(cos(lidar_angle[i] * PI / 180.f));
    }


    Cloud.clear();
    for (int j = 0; j < (int)lidar_angle.size(); j++)
    {
        for (int i = 0; i < 16; i++)
        {
            PointT temp1;
            Point temp;
            temp1.x = temp.x = -1*lidar_dist[i][j] * cosTheta[i] * sinAngle[j] / 100.f;
            temp1.y = temp.y = -1*lidar_dist[i][j] * cosTheta[i] * cosAngle[j] / 100.f;
            temp1.z = temp.z = lidar_dist[i][j] * sinTheta[i] / 100.f;
            temp.r = 255;
            temp.g = 255;
            temp.b = 255;
            temp.timestamp = tstamp_list[i][j];

            cur_lidar_pointcloud.push_back(temp);
            Cloud.points.push_back(temp1);
        }
    }

    Eigen::Matrix3d tf_r = Eigen::AngleAxisd(4*M_PI/180.0,Eigen::Vector3d(0,1,0)).matrix()*Eigen::AngleAxisd(-1*M_PI/2,Eigen::Vector3d(0,0,1)).matrix()*Eigen::AngleAxisd(-1*M_PI/2,Eigen::Vector3d(0,1,0)).matrix();

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.rotate(tf_r);
    tf.pretranslate(Eigen::Vector3d(0,0,0));

    pcl::transformPointCloud(Cloud,Cloud,tf.matrix());



    for (int i = 0; i < cur_lidar_pointcloud.size(); i++)
    {

        geometry_msgs::PoseStamped PointStamp;
        PointStamp.pose.position.x = Cloud.points[i].x;
        PointStamp.pose.position.y = Cloud.points[i].y;
        PointStamp.pose.position.z = Cloud.points[i].z;
        PointStamp.header.frame_id = "odom";
        PointStamp.header.stamp.sec = int(cur_lidar_pointcloud[i].timestamp);
        PointStamp.header.stamp.nsec = int(cur_lidar_pointcloud[i].timestamp*100)%100;
        pose_pub.publish(PointStamp);



    }

    static int tot_point_size = 0;
    tot_point_size += cur_lidar_pointcloud.size();
    std::cout<<"Point Cloud Size = "<<tot_point_size<<std::endl;
//    Cloud.is_dense = false;
//    PointCloud transformed;

//    Eigen::Matrix3d tf_r = Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,1,0)).matrix()*Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,0,1));
//    Eigen::Matrix3d tf_r = Eigen::AngleAxisd(-1*M_PI/2,Eigen::Vector3d(0,0,1)).matrix()*Eigen::AngleAxisd(-1*M_PI/2,Eigen::Vector3d(0,1,0)).matrix();

//    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
//    tf.rotate(tf_r);
//    tf.pretranslate(Eigen::Vector3d(0,0,0));
//    pcl::transformPointCloud(Cloud,transformed,tf.matrix());

    /*pcl::toROSMsg(Cloud, output);
    output.header.frame_id = "odom";
    pcl_pub.publish(output);*/

    ros::Rate loop_rate(0.4);
    loop_rate.sleep();
//    std::cout<<"one frame"<<std::endl;
    frame_count++;
}

void LidarReceive(char* recvBuf)
{
    //char recvBuf[1206] = { 0 };
    int recvLen = 1206;
    static std::vector<float>angle;
    static std::vector<std::vector<int> >distance;
    static std::vector<std::vector<int> >intensity;
    static std::vector<std::vector<double> >timestamp;
    static int cnt = 0;
    static double hhmmss_timestamp = 0;
    if (cnt == 0)
    {
        distance.resize(16);
        intensity.resize(16);
        timestamp.resize(16);
    }
    cnt++;

    std::vector<unsigned char> data;
    for (int i = 0; i < 1206; i++)
    {
        data.push_back(recvBuf[i]);
    }

    if (data[0] == 165 && data[1] == 255 && data[2] == 0 && data[3] == 90)
    {
        printf("%04d/%02d/%02d  %02dh%02dm%02ds", 2000 + data[36], data[37], data[38], data[39], data[40], data[41]);
        printf("\n");
        hhmmss_timestamp = data[39] * 3600 + data[40] * 60 + data[41];
    }

    static int hehe = 0;
    if (hhmmss_timestamp > 0 && recvLen > 0 && data[0] == 255 && data[1] == 238)
    {
        hehe++;
        double cur_timestamp = (unsigned int)(data[1200] + data[1201] * 256 + data[1202] * 256 * 256 + data[1203] * 256 * 256 * 256);
        if (cur_timestamp < 1000000)
        {
            //printf("时间更新 cur_time_stamp = %.6lf\n", hhmmss_timestamp + cur_timestamp / 1000000);
            //std::cout << "时间更新 cur_time_stamp = " << hhmmss_timestamp + cur_timestamp/1000000 << std::endl;
        }
        cur_timestamp = hhmmss_timestamp + cur_timestamp / 1000000;

        for (int i = 0; i < 12; i += 2)
        {
            if ((data[3 + 100 * i] * 256 + data[2 + 100 * i]) / 100.f >= 360.0)
            {
                angle.push_back(0);
            }
            else
            {
                angle.push_back((data[3 + 100 * i] * 256 + data[2 + 100 * i]) / 100.f);
            }
            for (int j = 0; j < 16; j++)
            {
                distance[j].push_back((data[5 + 3 * j + 100 * i] * 256 + data[4 + 3 * j + 100 * i]));
                intensity[j].push_back(data[6 + 3 * j + 100 * i]);
                timestamp[j].push_back(cur_timestamp + (((i / 2) * 100 + j) * 3.125 - 596.875) / 1000000);
            }
            if (angle.size() >= 2 && i >= 2)
            {
                if (angle[angle.size() - 1] < 1.0 && angle[angle.size() - 2] > 359.0)
                {
                    if (angle[angle.size() - 1] + angle[angle.size() - 2] > 360.0)
                    {
                        angle.insert(angle.end() - 1, angle[angle.size() - 1] + angle[angle.size() - 2] - 360.0);
                    }
                    else
                    {
                        angle.insert(angle.end() - 1, angle[angle.size() - 2] + 0.18);
                    }
                }
                else
                {
                    angle.insert(angle.end() - 1, (angle[angle.size() - 1] + angle[angle.size() - 2]) / 2);
                }
                for (int j = 0; j < 16; j++)
                {
                    distance[j].insert(distance[j].end() - 1, (data[53 + 3 * j + 100 * (i - 2)] * 256 + data[52 + 3 * j + 100 * (i - 2)]));
                    intensity[j].insert(intensity[j].end() - 1, data[54 + 3 * j + 100 * (i - 2)]);
                    timestamp[j].insert(timestamp[j].end() - 1, cur_timestamp + ((((i - 2) / 2) * 100 + j + 16)*3.125 - 596.875) / 1000000);
                }
            }
            if (std::abs(angle.back() - angle.front()) <= 0.5 && angle.size() > 500)
            {
                //if (lidar_angle.empty())
                //{
                lidar_dist.clear();
                lidar_inst.clear();
                lidar_angle.clear();
                tstamp_list.clear();
                lidar_dist = distance;
                lidar_inst = intensity;
                lidar_angle = angle;
                tstamp_list = timestamp;
                //}

                angle.clear();
                distance.clear();
                distance.resize(16);
                intensity.clear();
                intensity.resize(16);
                timestamp.clear();
                timestamp.resize(16);
                LidarParsing();
            }
        }

        //std::cout << "pause" << std::endl;
    }
}




int main(int argc, char **argv)
{
    ros::init(argc,argv,"show_lidar");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("show_lidar", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("PointStamped", 2000000);
    visualization_msgs::Marker line_list_r,line_list_g,line_list_b;
    line_list_r.header.frame_id = line_list_g.header.frame_id = line_list_b.header.frame_id = "/odom";
    line_list_r.header.stamp = line_list_g.header.stamp = line_list_b.header.stamp = ros::Time::now();
    line_list_r.ns = line_list_g.ns = line_list_b.ns = "points_and_lines";
    line_list_r.action = line_list_g.action = line_list_b.action = visualization_msgs::Marker::ADD;
    line_list_r.pose.orientation.w = line_list_g.pose.orientation.w = line_list_b.pose.orientation.w = 1.0;
    line_list_r.id = 1;
    line_list_g.id = 2;
    line_list_b.id = 3;

    line_list_r.type = visualization_msgs::Marker::LINE_LIST;
    line_list_g.type = visualization_msgs::Marker::LINE_LIST;
    line_list_b.type = visualization_msgs::Marker::LINE_LIST;

    line_list_r.scale.x = 0.3;
    line_list_g.scale.x = 0.3;
    line_list_b.scale.x = 0.3;

    line_list_r.color.r = 1.0;
    line_list_r.color.a = 1.0;

    line_list_g.color.g = 1.0;
    line_list_g.color.a = 1.0;

    line_list_b.color.b = 1.0;
    line_list_b.color.a = 1.0;

    geometry_msgs::Point pp,p_x,p_y,p_z;
    pp.x = 0;
    pp.y = 0;
    pp.z = 0;
    p_x = p_y = p_z = pp;
    p_x.x = pp.x+100;
    p_y.y = pp.y+100;
    p_z.z = pp.z+100;

    line_list_r.points.push_back(pp);
    line_list_r.points.push_back(p_x);
    line_list_g.points.push_back(pp);
    line_list_g.points.push_back(p_y);
    line_list_b.points.push_back(pp);
    line_list_b.points.push_back(p_z);

    marker_pub.publish(line_list_r);
    marker_pub.publish(line_list_b);
    marker_pub.publish(line_list_g);


    std::ifstream in("/home/lyh/lab/libpcap_test/bin/saved_0829-2_lidar_data.csv");
    std::string line;

    char buf[4000];
    char recvBuf[1206];
    char hex[2];

    if (in)
    {
        while (std::getline(in, line))
        {
            if(finishflag)
                break;

            if(jumpflag)
            {
                for(int i = 0;i<20000;i++)
                {
                    std::getline(in, line);
                }
                jumpflag = false;
            }


            strcpy(buf, line.c_str());

            //std::cout << line << std::endl;
            for (int i = 0; i < 1206; i++)
            {
                strncpy(hex, buf + i * 3, 2);
                sscanf(hex, "%x", &recvBuf[i]);
            }
            LidarReceive(recvBuf);
            marker_pub.publish(line_list_r);
            marker_pub.publish(line_list_b);
            marker_pub.publish(line_list_g);
        }
    }
    else
    {
        std::cout << "no such file " << std::endl;
    }
    return 0;
}
