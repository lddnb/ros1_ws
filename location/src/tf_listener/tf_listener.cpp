#include "tf_listener/tf_listener.h"
#include <glog/logging.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Geometry>

namespace location {
TFListener::TFListener(ros::NodeHandle nh, std::string base_frame_id, std::string child_frame_id)
    :nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id), listener_(buffer_) {}

bool TFListener::LookupData(Eigen::Matrix4f & transform_matrix) {
    try {
        geometry_msgs::TransformStamped transform;
        transform = buffer_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0));
        TransformToMatrix(transform, transform_matrix);
        return true;
    }
    catch (tf2::TransformException &ex) {
        //LOG(WARNING) << "Look up transform matrix faild";
        return false;
    }
}

bool TFListener::TransformToMatrix(const geometry_msgs::TransformStamped& transform, Eigen::Matrix4f& transform_matrix) {
    Eigen::Translation3f tl_btol(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);

    Eigen::Quaternionf q(transform.transform.rotation.w,
                         transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z);

    transform_matrix = (tl_btol * q.toRotationMatrix()).matrix();

    return true;
}

}