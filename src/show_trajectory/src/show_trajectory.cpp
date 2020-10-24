#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    std::ifstream tf("/home/lyh/lab/data/huigu_1218/data2/huigu_1218_02.txt",std::ios::in);

    char s[1000] = {0};

    std::string _sx = "";
    std::string _sy = "";
    std::string _sz = "";
    std::string _spitch = "";
    std::string _sroll = "";
    std::string _syaw = "";


    ros::init(argc, argv, "points_and_lines");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate r(1000);

    int index = 0;

    double base_east,base_north,base_up;
    visualization_msgs::Marker points, line_list_r,line_list_g,line_list_b;
    while(!tf.eof())
    {
//        visualization_msgs::Marker points, line_list_r,line_list_g,line_list_b;
        //PLOT prepare
        points.header.frame_id = line_list_r.header.frame_id = line_list_g.header.frame_id = line_list_b.header.frame_id = "/odom";
        points.header.stamp = line_list_r.header.stamp = line_list_g.header.stamp = line_list_b.header.stamp = ros::Time::now();
        points.ns = line_list_r.ns = line_list_g.ns = line_list_b.ns = "points_and_lines";
        points.action = line_list_r.action = line_list_g.action = line_list_b.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_list_r.pose.orientation.w = line_list_g.pose.orientation.w = line_list_b.pose.orientation.w = 1.0;

        points.id = 0;
        line_list_r.id = 1;
        line_list_g.id = 2;
        line_list_b.id = 3;

        points.type = visualization_msgs::Marker::POINTS;
        line_list_r.type = visualization_msgs::Marker::LINE_LIST;
        line_list_g.type = visualization_msgs::Marker::LINE_LIST;
        line_list_b.type = visualization_msgs::Marker::LINE_LIST;

        points.scale.x = 0.2;
        points.scale.y = 0.2;

        line_list_r.scale.x = 0.3;
        line_list_g.scale.x = 0.3;
        line_list_b.scale.x = 0.3;

        points.color.g = 1.0f;
        points.color.a = 1.0;

        line_list_r.color.r = 1.0;
        line_list_r.color.a = 1.0;

        line_list_g.color.g = 1.0;
        line_list_g.color.a = 1.0;

        line_list_b.color.b = 1.0;
        line_list_b.color.a = 1.0;


        tf.getline(s,sizeof(s));
        //        std::cout<<strlen(s)<<std::endl;
        //        std::cout<<s<<std::endl;
        std::stringstream word(s);
        //remove time data
        word>>_sx;
        word>>_sx;
        //parse useful data
        word>>_sx;
        word>>_sy;
        word>>_sz;
        word>>_syaw;
        word>>_spitch;
        word>>_sroll;

        double dx,dy,dz,droll,dpitch,dyaw;
        if(index == 0)
        {
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
            dx = atof(_sx.c_str()) - base_east;
            dy = atof(_sy.c_str()) - base_north;
            dz = atof(_sz.c_str()) - base_up;
            droll = atof(_sroll.c_str());
            dpitch = atof(_spitch.c_str());
            dyaw = atof(_syaw.c_str());
        }


        //speed up
        if(index%1000!=0)
        {
            index++;
            continue;
        }

        //start time
//        if(index/1000<820||index/1000>1180)
//        {
//            index++;
//            continue;
//        }
        if(index/1000<670||index/1000>1118)
        {
            index++;
            continue;
        }


        std::cout<< std::fixed << std::setprecision(6)<<"east = "<<dx<<" , north = "<<dy<<" , up = "<<dz<<" , roll = "<<droll<<" , pitch = "<<dpitch<<" , dyaw = "<<dyaw<<std::endl;
        std::cout<<index/1000<<" s"<<std::endl;
        Eigen::Matrix3d R_M = Eigen::AngleAxisd(-dyaw*M_PI/180.0,Eigen::Vector3d(0,0,1)).matrix()*Eigen::AngleAxisd(dpitch*M_PI/180.0,Eigen::Vector3d(1,0,0)).matrix()*Eigen::AngleAxisd(droll*M_PI/180.0,Eigen::Vector3d(0,1,0)).matrix();
//        Eigen::Matrix3d R_M = Eigen::AngleAxisd(0*M_PI/180.0,Eigen::Vector3d(1,0,0)).matrix()*Eigen::AngleAxisd(0*M_PI/180.0,Eigen::Vector3d(0,1,0)).matrix()*Eigen::AngleAxisd(-1*dyaw*M_PI/180.0,Eigen::Vector3d(0,0,1)).matrix();

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(R_M);
        T.pretranslate(Eigen::Vector3d(dx,dy,dz));

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

        geometry_msgs::Point p,p_x,p_y,p_z;
        p.x = pp(0);
        p.y = pp(1);
        p.z = pp(2);
        p_x.x = px(0);
        p_x.y = px(1);
        p_x.z = px(2);

        p_y.x = py(0);
        p_y.y = py(1);
        p_y.z = py(2);


        p_z.x = pz(0);
        p_z.y = pz(1);
        p_z.z = pz(2);



        points.points.push_back(p);

        // The line list needs two points for each line
        line_list_r.points.push_back(p);
        line_list_r.points.push_back(p_x);
        line_list_g.points.push_back(p);
        line_list_g.points.push_back(p_y);
        line_list_b.points.push_back(p);
        line_list_b.points.push_back(p_z);

        // %EndTag(HELIX)%

        marker_pub.publish(points);
        marker_pub.publish(line_list_r);

        marker_pub.publish(line_list_b);

        marker_pub.publish(line_list_g);

        r.sleep();


        index += 1;
    }

    return 0;
}
