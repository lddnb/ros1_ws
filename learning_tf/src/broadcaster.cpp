#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void PoseCallback(const turtlesim::PoseConstPtr& msg) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
    transform.setRotation(tf::createQuaternionFromYaw(msg->theta));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_tf_broadcaster");
    turtle_name = argv[1];
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(turtle_name+"/pose", 10, &PoseCallback);

    tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(0.0, 2.0, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    
    ros::Rate rate(10);
    while (ros::ok) {
        ros::spinOnce();
        
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1"));
        rate.sleep();
    }
    return 0;
}