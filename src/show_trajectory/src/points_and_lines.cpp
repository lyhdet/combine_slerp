/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "points_and_lines");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate r(100);

    float f = 0.0;
    int index = 0;
    while (ros::ok())
    {
        // %Tag(MARKER_INIT)%
        visualization_msgs::Marker points, line_list_r,line_list_g,line_list_b;
        points.header.frame_id = line_list_r.header.frame_id = line_list_g.header.frame_id = line_list_b.header.frame_id = "/odom";
        points.header.stamp = line_list_r.header.stamp = line_list_g.header.stamp = line_list_b.header.stamp = ros::Time::now();
        points.ns = line_list_r.ns = line_list_g.ns = line_list_b.ns = "points_and_lines";
        points.action = line_list_r.action = line_list_g.action = line_list_b.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_list_r.pose.orientation.w = line_list_g.pose.orientation.w = line_list_b.pose.orientation.w = 1.0;
        // %EndTag(MARKER_INIT)%

        // %Tag(ID)%
        points.id = 0;
        line_list_r.id = 1;
        line_list_g.id = 2;
        line_list_b.id = 3;
        // %EndTag(ID)%

        // %Tag(TYPE)%
        points.type = visualization_msgs::Marker::POINTS;
        line_list_r.type = visualization_msgs::Marker::LINE_LIST;
        line_list_g.type = visualization_msgs::Marker::LINE_LIST;
        line_list_b.type = visualization_msgs::Marker::LINE_LIST;

        // %EndTag(TYPE)%

        // %Tag(SCALE)%
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_list_r.scale.x = 0.1;
        line_list_g.scale.x = 0.1;
        line_list_b.scale.x = 0.1;
        // %EndTag(SCALE)%

        // %Tag(COLOR)%
        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line list is red
        line_list_r.color.r = 1.0;
        line_list_r.color.a = 1.0;

        line_list_g.color.g = 1.0;
        line_list_g.color.a = 1.0;

        line_list_b.color.b = 1.0;
        line_list_b.color.a = 1.0;
        // %EndTag(COLOR)%

        // %Tag(HELIX)%
        // Create the vertices for the points and lines


        Eigen::Matrix3d R_M = Eigen::AngleAxisd((index%360)*M_PI/180.0,Eigen::Vector3d(0,0,1)).matrix()*Eigen::AngleAxisd(0,Eigen::Vector3d(0,1,0)).matrix();

        std::cout<<R_M.matrix()<<std::endl;
        Eigen::Vector3d px,py,pz,pp;
        pp = Eigen::Vector3d(0,0,0);
        px = py = pz = pp;
        px(0) = pp(0)+1;
        py(1) = pp(1)+1;
        pz(2) = pp(2)+1;

        pp = R_M*pp;
        px = R_M*px;
        py = R_M*py;
        pz = R_M*pz;

        std::cout<<"px = "<<px<<std::endl;



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

        f += 0.04;

        index++;
    }
}
// %EndTag(FULLTEXT)%


