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
