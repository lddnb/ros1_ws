#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle nh;

    //调用服务产生第二只乌龟
    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    srv.request.x = 1;
    srv.request.y = 1;
    srv.request.theta = 0;
    srv.request.name = "turtle2";
    add_turtle.call(srv);

    ros::Publisher turtle_vel = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    tf2_ros::Buffer tf_buff;
    tf2_ros::TransformListener lr(tf_buff);

    

    ros::Rate rate(10);

    while (ros::ok) {
        geometry_msgs::TransformStamped transform;
        try {
            ros::Time now = ros::Time::now();
            ros::Time past = now - ros::Duration(5.0);
            //tf2的坐标系之间转换前面不能带‘/’
            //transform = tf_buff.lookupTransform("turtle2", "turtle1", ros::Time::now(), ros::Duration(1.0));
            transform = tf_buff.lookupTransform("turtle2", now, "turtle1", past, "world", ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist vel_msg;

        vel_msg.angular.z = atan2(transform.transform.translation.y, transform.transform.translation.x);

        vel_msg.linear.x = sqrt(pow(transform.transform.translation.x, 2) + pow(transform.transform.translation.y, 2));
        turtle_vel.publish(vel_msg);
        rate.sleep();
    }

    return 0;
}