#include "sensor_data/gnss_data.h"
#include "glog/logging.h"

bool location::GnssData::origin_position_inited = false;
GeographicLib::LocalCartesian location::GnssData::geo_converter;

namespace location {
bool GnssData::SyncData(std::deque<GnssData>& unsynced_data, std::deque<GnssData>& synced_data, double sync_time) {
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
    GnssData front_data = unsynced_data.front();
    GnssData back_data = unsynced_data.back();
    GnssData sync_data;
    //对两个IMU数据进行线性插值
    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    sync_data.time = sync_time;
    sync_data.status = back_data.status;

    sync_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    sync_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    sync_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
    sync_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
    sync_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
    sync_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

    synced_data.emplace_back(sync_data);

    return true;
}

void GnssData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}

void GnssData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

}