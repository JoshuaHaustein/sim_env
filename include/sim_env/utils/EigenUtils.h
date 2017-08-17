//
// Created by joshua on 7/27/17.
//

#ifndef SIM_ENV_EIGENUTILS_H
#define SIM_ENV_EIGENUTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sim_env {
    namespace utils {
        namespace eigen {
            enum class ScalingResult {
                Scaled, NotScaled, Failure
            };
            ScalingResult scaleToLimits(Eigen::VectorXf& vector, const Eigen::ArrayX2f& limits);
            void eulerToQuaternion(float roll, float pitch, float yaw, Eigen::Quaternionf& output);
            void quaternionToEuler(const Eigen::Quaternionf& quaternion, Eigen::Vector3f& output);
        }
    }
}
#endif //SIM_ENV_EIGENUTILS_H
