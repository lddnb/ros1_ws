#include "sensor_data/imu_data.h"

namespace location {
Eigen::Matrix3f ImuData::GetOrientationMatrix() {
    Eigen::Quaterniond q(orientation.x, orientation.y, orientation.z, orientation.w);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();
    return matrix;
}

bool ImuData::SyncData(std::deque<ImuData>& unsynced_data, std::deque<ImuData>& synced_data, double sync_time) {
    //求消息队列中与同步时间最近的两个IMU数据
    //当左右相邻数据中有一个数据与同步时间差距较大，说明有数据丢失
    while (unsynced_data.size() >= 2) {
        if (unsynced_data.front().time > sync_time) {
            return false;
        }
        if (unsynced_data.at(1).time < sync_time) {
            unsynced_data.pop_front();
            continue;
        }
        if (sync_time - unsynced_data.front().time > 0.2) {
            unsynced_data.pop_front();
            return false;
        }
        if (unsynced_data.at(1).time - sync_time > 0.2) {
            return false;
        }
        break;
    }

    if (unsynced_data.size() < 2) {
        return false;
    }

    ImuData front_data = unsynced_data.front();
    ImuData back_data = unsynced_data.back();
    ImuData sync_data;
    //对两个IMU数据进行线性插值
    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    sync_data.time = sync_time;
    sync_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    sync_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    sync_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    sync_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    sync_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    sync_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

    sync_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    sync_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    sync_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    sync_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    //插值之后需要进行归一化
    sync_data.orientation.Normlize();
    synced_data.emplace_back(sync_data);   
    
    return true;
}


}