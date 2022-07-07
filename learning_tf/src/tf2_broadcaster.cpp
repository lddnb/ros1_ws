#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void PoseCallback(const turtlesim::PoseConstPtr& msg) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "world";
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = turtle_name;
    transform.transform.translation.x = msg->x;
    transform.transform.translation.y = msg->y;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, msg->theta);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    br.sendTransform(transform);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_tf2_broadcaster");
    turtle_name = argv[1];
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(turtle_name+"/pose", 10, &PoseCallback);

    tf2_ros::StaticTransformBroadcaster static_br;
    geometry_msgs::TransformStamped transform;

    transform.header.frame_id = "turtle1";
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = "carrot1";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 2.0;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    ros::Rate rate(10);
    while (ros::ok) {
        static_br.sendTransform(transform);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
