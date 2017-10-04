//
// Created by joshua on 7/27/17.
//
#include "sim_env/utils/EigenUtils.h"
#include <limits>
#include <algorithm>

using namespace sim_env::utils::eigen;

ScalingResult sim_env::utils::eigen::scaleToLimits(Eigen::VectorXf& vector, const Eigen::ArrayX2f& limits) {
    ScalingResult result = ScalingResult::NotScaled;
    float c = std::numeric_limits<float>::max();
    for (unsigned int i = 0; i < vector.size(); ++i) {
        float x_i = std::min(std::max(limits(i, 0), vector[i]), limits(i, 1));
        float c_i = vector[i] == 0.0f ? 1.0f : x_i / vector[i];
        c = std::min(c, c_i);
    }
    if (c < 0.0f) {
        result = ScalingResult::Failure;
    } else if (c < 1.0f) {
        result = ScalingResult::Scaled;
    }
    vector = c * vector;
    return result;
}

void sim_env::utils::eigen::eulerToQuaternion(float roll, float pitch, float yaw,
                                              Eigen::Quaternionf& output) {
    // based on a code snippet from
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    float t0 = std::cos(yaw * 0.5f);
    float t1 = std::sin(yaw * 0.5f);
    float t2 = std::cos(roll * 0.5f);
    float t3 = std::sin(roll * 0.5f);
    float t4 = std::cos(pitch * 0.5f);
    float t5 = std::sin(pitch * 0.5f);

    output.w() = t0 * t2 * t4 + t1 * t3 * t5;
    output.x() = t0 * t3 * t4 - t1 * t2 * t5;
    output.y() = t0 * t2 * t5 + t1 * t3 * t4;
    output.z() = t1 * t2 * t4 - t0 * t3 * t5;
}

void sim_env::utils::eigen::quaternionToEuler(const Eigen::Quaternionf& quaternion, Eigen::Vector3f& output) {
    // based on a code snippet from
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    float ysqr = quaternion.y() * quaternion.y();

    // roll (x-axis rotation)
    float t0 = +2.0f * (quaternion.w() * quaternion.x() + quaternion.y() * quaternion.z());
    float t1 = +1.0f - 2.0f * (quaternion.x() * quaternion.x() + ysqr);
    float roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    float t2 = +2.0f * (quaternion.w() * quaternion.y() - quaternion.z() * quaternion.x());
    t2 = ((t2 > 1.0f) ? 1.0f : t2);
    t2 = ((t2 < -1.0f) ? -1.0f : t2);
    float pitch = std::asin(t2);

    // yaw (z-axis rotation)
    float t3 = +2.0f * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y());
    float t4 = +1.0f - 2.0f * (ysqr + quaternion.z() * quaternion.z());
    float yaw = std::atan2(t3, t4);

    // save in output
    output.x() = roll;
    output.y() = pitch;
    output.z() = yaw;
}
