//
// Created by joshua on 7/27/17.
//

#ifndef SIM_ENV_EIGENUTILS_H
#define SIM_ENV_EIGENUTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <type_traits>

// #define EIGEN_TYPE_NEEDS_ALIGNMENT(T) (std::is_same<T, Eigen::Vector4f> or std::is_same<T, Eigen::Matrix2f> or std::is_same<T, Eigen::Matrix4f> or std::is_same<T, Eigen::Affine3f>)

namespace sim_env {
    namespace utils {
        namespace eigen {
            enum class ScalingResult {
                Scaled, NotScaled, Failure
            };
            ScalingResult scaleToLimits(Eigen::VectorXf& vector, const Eigen::ArrayX2f& limits);
            void eulerToQuaternion(float roll, float pitch, float yaw, Eigen::Quaternionf& output);
            void quaternionToEuler(const Eigen::Quaternionf& quaternion, Eigen::Vector3f& output);
            void setValues(Eigen::VectorXf& dest, const Eigen::VectorXf& source, const Eigen::VectorXi& indices);
        }
    }
}
#endif //SIM_ENV_EIGENUTILS_H
