#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <deque>

namespace location {
class VelocityData {
    public:
    struct LinearVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    
    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

    public:
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
    static bool SyncData(std::deque<VelocityData>& unsynced_data, std::deque<VelocityData>& synced_data, double sync_time);    

};

} // namespace location
