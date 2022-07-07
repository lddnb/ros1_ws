#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <deque>

namespace location {
class ImuData {
    public:
    struct LinearAcceleration {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    class Orientation {
        public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
        
        public:
        void Normlize() {
            double norm = sqrt(x*x + y*y + z*z + w*w);
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }
    };
    
    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;

    public:
    Eigen::Matrix3f GetOrientationMatrix();
    static bool SyncData(std::deque<ImuData>& unsynced_data, std::deque<ImuData>& synced_data, double sync_time);    

};

} // namespace location
