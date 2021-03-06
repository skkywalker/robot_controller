#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include <cstdlib>
#include <math.h>

#define K_LIN 1.0
#define K_ANG 0.5

double get_model_info(ros::ServiceClient& client, char* model_name, char info) {
  gazebo_msgs::GetModelState srv;

  srv.request.model_name = model_name;
  client.call(srv);

  switch (info)
  {
  case 'x':
    return srv.response.pose.position.x;

  case 'y':
    return srv.response.pose.position.y;

  case 't':
    tf::Quaternion q(
      srv.response.pose.orientation.x,
      srv.response.pose.orientation.y,
      srv.response.pose.orientation.z,
      srv.response.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_controller");

  double linear_speed;
  double angular_speed;

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

  ros::Rate loop_rate(60);

  geometry_msgs::Twist data;

  while(ros::ok()) {
    double yaw = get_model_info(client, "/", 't');
    double robot_x = get_model_info(client, "/", 'x');
    double robot_y = get_model_info(client, "/", 'y');

    double box_x = get_model_info(client, "unit_box", 'x');
    double box_y = get_model_info(client, "unit_box", 'y');

    float distance = sqrt(
      pow(box_x - robot_x,2) +
      pow(box_y - robot_y,2)
    ) - 4;
  
    float bearing = atan2(
      box_y - robot_y,
      box_x - robot_x
    ) - yaw;
    if(abs(bearing) > 3.1415) bearing *= -1;
    ROS_INFO("yaw: %f, bearing: %f", yaw, bearing+yaw);

    linear_speed = distance * K_LIN;
    angular_speed = bearing * K_ANG;

    if(linear_speed > 1) linear_speed = 1;
    else if (linear_speed < 0) linear_speed = 0;

    if(angular_speed > 0.5) angular_speed = 0.5;
    else if (angular_speed < -0.5) angular_speed = -0.5;

    data.linear.x = linear_speed;
    data.angular.z =  angular_speed;

    pub.publish(data);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
