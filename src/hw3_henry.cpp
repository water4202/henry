#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <cmath>

Eigen::Vector3d w_0,w,a_0,a_g,a_b,v_0(0,0,0),v,g(0,0,9.81),s_0(0,0,0);
Eigen::Matrix3d B;
Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
double past=0,now=0,dt=0,sigma;

void Callback(const sensor_msgs::Imu::ConstPtr& msg)
 {
    Eigen::Vector3d a_1(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    Eigen::Vector3d w_1(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    now = msg->header.stamp.toSec();
    dt = now - past;

    if(past!=0)
    {
      w = 0.5*(w_0 + w_1);
      B <<     0     , -w.z()*dt ,  w.y()*dt ,
            w.z()*dt ,     0     , -w.x()*dt ,
           -w.y()*dt ,  w.x()*dt ,     0     ;

      sigma = (w*dt).norm();
      C = C * (I + (sin(sigma)/sigma)*B + ((1 - cos(sigma))/sigma/sigma)*B*B);

      a_b = 0.5*(a_0 + a_1);
      a_g = C * a_b;
      v = v_0 + (a_g - g)*dt;
      s_0 = s_0 + 0.5*(v + v_0)*dt;
    }

    past = now;
    w_0 = w_1;
    a_0 = a_1;
    v_0 = v;
    std::cout << s_0 << std::endl;
 }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hw3_henry");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/imu/data", 1000, Callback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Rate r(200);
  visualization_msgs::Marker line_strip;

  while (ros::ok())
  {
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "hw3_henry";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = s_0.x();
    p.y = s_0.y();
    p.z = s_0.z();
    line_strip.points.push_back(p);

    marker_pub.publish(line_strip);

    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
