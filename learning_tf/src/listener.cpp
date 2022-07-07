#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle nh;

    //调用服务产生第二只乌龟
    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    add_turtle.call(srv);

    ros::Publisher turtle_vel = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    tf::TransformListener lr;

    ros::Rate rate(10);

    while (ros::ok) {
        tf::StampedTransform transform;
        try {
            ros::Time now = ros::Time::now();
            ros::Time past = now - ros::Duration(5.0);
            //lr.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(10.0));
            //lr.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
            lr.waitForTransform("/turtle2", now, "/turtle1", past, "world", ros::Duration(1.0));
            lr.lookupTransform("/turtle2", now, "/turtle1", past, "world", transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist vel_msg;

        vel_msg.angular.z = atan2(transform.getOrigin().y(), transform.getOrigin().x());

        vel_msg.linear.x = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
        turtle_vel.publish(vel_msg);
        rate.sleep();
    }

    return 0;
}