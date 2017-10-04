//
// Created by joshua on 8/23/17.
//

#include <sim_env/utils/MathUtils.h>
#include <algorithm>

float sim_env::utils::math::clamp(float value, float low, float high) {
    return std::min(std::max(value, low), high);
}
