#pragma once

#include <deque>

#include "Geocentric/LocalCartesian.hpp"

namespace location {
class GnssData {
    public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

    public:
    static bool SyncData(std::deque<GnssData>& unsynced_data, std::deque<GnssData>& synced_data, double sync_time);
    void InitOriginPosition();
    void UpdateXYZ();

    private:    
    static bool origin_position_inited;
    static GeographicLib::LocalCartesian geo_converter;
};

}
