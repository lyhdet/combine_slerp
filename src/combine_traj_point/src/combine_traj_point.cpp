#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "iostream"
#include <iomanip>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int point_cnt = 0;
PointCloud Cloud;
PointCloud tot_cloud;
ros::Publisher pcl_pub, tot_pcl_pub;
sensor_msgs::PointCloud2 output;
ros::Publisher marker_pub;
visualization_msgs::Marker line_list_r,line_list_g,line_list_b;

int start_time = 19645;
int last_time = 360;


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


// 回调函数
void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(msg->header.stamp.sec<start_time||msg->header.stamp.sec-start_time>=last_time)
        return;
    PointT pp;
    pp.x = msg->pose.position.x;
    pp.y = msg->pose.position.y;
    pp.z = msg->pose.position.z;

    //    Cloud.points.push_back(pp);
    //    if(point_cnt%60000 == 0)
    //    {
    //        pcl::toROSMsg(Cloud, output);
    //        output.header.frame_id = "odom";
    //        pcl_pub.publish(output);
    ////        std::cout<<std::fixed<<std::setprecision(10)<<"point_cnt: "<<point_cnt<<" Sec: "<<msg->header.stamp.sec<<"."<<msg->header.stamp.nsec<<" x , y , z :"<<msg->pose.position.x<<" , "<<msg->pose.position.y<<" , "<<msg->pose.position.z<<std::endl;
    //        Cloud.clear();
    //    }

    if(point_cnt%10 == 0)
    {
        Eigen::Vector3d cur_p(pp.x,pp.y,pp.z);
//        POS cur_pos = pos_table[int(msg->header.stamp.sec-start_time)][msg->header.stamp.nsec];
        POS cur_pos = pos_table[int(msg->header.stamp.sec-start_time)][msg->header.stamp.nsec+5];
        Eigen::Matrix3d R_M = Eigen::AngleAxisd(-1*cur_pos.h*M_PI/180.0,Eigen::Vector3d(0,0,1)).matrix()*Eigen::AngleAxisd(1*cur_pos.p*M_PI/180.0,Eigen::Vector3d(1,0,0)).matrix()*Eigen::AngleAxisd(cur_pos.r*M_PI/180.0,Eigen::Vector3d(0,1,0)).matrix();
        //Eigen::Matrix3d R_M = Eigen::AngleAxisd(cur_pos.r*M_PI/180.0,Eigen::Vector3d(0,1,0)).matrix()*Eigen::AngleAxisd(-1*cur_pos.h*M_PI/180.0,Eigen::Vector3d(0,0,1)).matrix();
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(R_M);
        T.pretranslate(Eigen::Vector3d(cur_pos.x,cur_pos.y,cur_pos.z));
        //        T.pretranslate(Eigen::Vector3d(0,0,0));

        cur_p = T*cur_p;

        PointT cur_pp;
        cur_pp.x = cur_p(0);
        cur_pp.y = cur_p(1);
        cur_pp.z = cur_p(2);

        tot_cloud.points.push_back(cur_pp);
    }
    if(point_cnt%20000 == 0)
        std::cout<<"Point Cloud size = "<<tot_cloud.points.size()<<std::endl;

    if(point_cnt%100000 == 0)
    {
        POS cur_pos = pos_table[int(msg->header.stamp.sec-start_time)][msg->header.stamp.nsec];

        Eigen::Matrix3d R_M = Eigen::AngleAxisd(-1*cur_pos.h*M_PI/180.0,Eigen::Vector3d(0,0,1)).matrix()*Eigen::AngleAxisd(1*cur_pos.p*M_PI/180.0,Eigen::Vector3d(0,1,0)).matrix()*Eigen::AngleAxisd(cur_pos.r*M_PI/180.0,Eigen::Vector3d(0,0,1)).matrix();
        //Eigen::Matrix3d R_M = Eigen::AngleAxisd(cur_pos.r*M_PI/180.0,Eigen::Vector3d(0,1,0)).matrix()*Eigen::AngleAxisd(-1*cur_pos.h*M_PI/180.0,Eigen::Vector3d(0,0,1)).matrix();

        std::cout<<"yaw = "<<cur_pos.h<<" pitch = "<<cur_pos.p<<" roll = "<<cur_pos.r<<std::endl;
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(R_M);
        T.pretranslate(Eigen::Vector3d(cur_pos.x,cur_pos.y,cur_pos.z));
        Eigen::Vector3d px,py,pz,pp;
        pp = Eigen::Vector3d(0,0,0);
        px = py = pz = pp;
        px(0) = pp(0)+10;
        py(1) = pp(1)+10;
        pz(2) = pp(2)+10;
        pp = T*pp;
        px = T*px;
        py = T*py;
        pz = T*pz;


        geometry_msgs::Point ppp,p_x,p_y,p_z;
        ppp.x = pp(0);
        ppp.y = pp(1);
        ppp.z = pp(2);
        p_x.x = px(0);
        p_x.y = px(1);
        p_x.z = px(2);

        p_y.x = py(0);
        p_y.y = py(1);
        p_y.z = py(2);


        p_z.x = pz(0);
        p_z.y = pz(1);
        p_z.z = pz(2);


        line_list_r.points.push_back(ppp);
        line_list_r.points.push_back(p_x);
        line_list_g.points.push_back(ppp);
        line_list_g.points.push_back(p_y);
        line_list_b.points.push_back(ppp);
        line_list_b.points.push_back(p_z);
        marker_pub.publish(line_list_r);

        marker_pub.publish(line_list_b);

        marker_pub.publish(line_list_g);
        std::cout<<"tot_cloud size = "<<tot_cloud.points.size()<<std::endl;

        /*if(tot_cloud.points.size()>99999)
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
        }*/

        pcl::toROSMsg(tot_cloud, output);
        output.header.frame_id = "odom";
        tot_pcl_pub.publish(output);
    }



    point_cnt++;

}

int main(int argc, char **argv)
{
    std::ifstream tf("/home/lyh/lab/data/huigu_0829/0830_huigu_02_2.txt",std::ios::in);

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

        double base_east,base_north,base_up;
        if(line_cnt == 0)
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


        if(dsecond - start_time>=last_time)
            continue;

        //std::cout<<s<<std::endl;
        POS cur_pos(dx, dy, dz, dyaw, dpitch, droll);
        std::cout<<"dsecond = "<<dsecond<<"line_cnt = "<<line_cnt<<std::endl;
        pos_table[int(dsecond - start_time)].push_back(cur_pos);
        line_cnt++;
    }

    for(int i = 0;i<last_time;i++)
    {
        std::cout<<pos_table[i].size()<<std::endl;
    }



    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    marker_pub = n.advertise<visualization_msgs::Marker>("combine_marker", 10);
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


    pcl_pub = n.advertise<sensor_msgs::PointCloud2> ("single_lidar", 1);
    tot_pcl_pub = n.advertise<sensor_msgs::PointCloud2> ("combine_lidar", 1);

    // 告诉master需要订阅chatter topic消息
    ros::Subscriber sub = n.subscribe("PointStamped", 2000000, chatterCallback);
    ros::spin();


    /*ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
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

    ros::Rate r(10);
    while(ros::ok())
    {
        marker_pub.publish(line_list_r);
        marker_pub.publish(line_list_b);
        marker_pub.publish(line_list_g);
        ros::spinOnce();
        r.sleep();
    }*/



    return 0;
}
