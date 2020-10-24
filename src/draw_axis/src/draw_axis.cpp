#include <vector>
#include <iostream>


//ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"draw_axis");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("draw_axis", 10);
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

    ros::Rate loop(1);
    while(ros::ok())
    {
        marker_pub.publish(line_list_r);
        marker_pub.publish(line_list_b);
        marker_pub.publish(line_list_g);
        loop.sleep();
    }

    return 0;
}
