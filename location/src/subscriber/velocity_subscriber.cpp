#include "subscriber/velocity_subscriber.h"

namespace location {

VelocitySubscriber::VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
: nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::msg_callback, this);
}

void VelocitySubscriber::msg_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg_ptr)
{
    std::lock_guard<std::mutex> lock(buff_mutex_);
    VelocityData velocity_data;
    velocity_data.time = twist_msg_ptr->header.stamp.toSec();
    velocity_data.linear_velocity.x = twist_msg_ptr->twist.linear.x;
    velocity_data.linear_velocity.y = twist_msg_ptr->twist.linear.y;
    velocity_data.linear_velocity.z = twist_msg_ptr->twist.linear.z;

    velocity_data.angular_velocity.x = twist_msg_ptr->twist.angular.x;
    velocity_data.angular_velocity.y = twist_msg_ptr->twist.angular.y;
    velocity_data.angular_velocity.z = twist_msg_ptr->twist.angular.z;

    new_velocity_data_.emplace_back(velocity_data);
}

void VelocitySubscriber::ParseData(std::deque<VelocityData>& deque_velocity_data)
{
    if (new_velocity_data_.size() > 0) {
        std::lock_guard<std::mutex> lock(buff_mutex_);
        deque_velocity_data.insert(deque_velocity_data.end(), new_velocity_data_.begin(), new_velocity_data_.end());
        new_velocity_data_.clear();
    }
}

} // namespace location