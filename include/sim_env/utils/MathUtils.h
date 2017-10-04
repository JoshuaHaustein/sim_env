//
// Created by joshua on 8/23/17.
//

#ifndef SIM_ENV_MATHUTILS_H
#define SIM_ENV_MATHUTILS_H

namespace sim_env {
    namespace utils {
        namespace math {
            /**
             * Clamps the given value to the interval [low, high].
             * @param value value to clamp
             * @param low - lower bound
             * @param high - upper bound
             * @return clamped value
             */
            float clamp(float value, float low, float high);
        }
    }
}

#endif //SIM_ENV_MATHUTILS_H
