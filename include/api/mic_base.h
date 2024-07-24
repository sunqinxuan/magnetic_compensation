/*
 * Magnetic Interference Compensation
 *
 * Copyright (C) 2024 Qinxuan Sun. All rights reserved.
 *
 *     Author : Qinxuan Sun
 *    Contact : sunqinxuan@outlook.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MIC_BASE
#define MIC_BASE

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mic
{
    using float64_t = double;
    using float32_t = float;
    using char_t = char;
    using vector_3f_t = Eigen::Vector3f;
    using quaternionf_t = Eigen::Quaternionf;

    enum MicBool
    {
        MIC_FALSE               = 0,
        MIC_TRUE                = 1
    };
    using bool_t = MicBool;
    #define true (mic::MIC_TRUE)
    #define false (mic::MIC_FALSE)

    enum class MicRet : uint8_t
    {
        MIC_RET_FAILED                = 0,
        MIC_RET_SUCCESSED             = 1
    };
    using ret_t = MicRet;

    struct MicMagFlux
    {
        float64_t time_stamp; // TODO
        vector_3f_t vector;
        float32_t confidence;
    };
    using mic_mag_flux_t = MicMagFlux;

    struct MicMagOP
    {
        float64_t time_stamp; // TODO
        float32_t value;
        float32_t confidence;
    };
    using mic_mag_op_t = MicMagOP;

    struct MicIMU
    {
        float64_t time_stamp; // TODO
        vector_3f_t gyr;
        vector_3f_t acc;
        vector_3f_t gyr_bias;
        vector_3f_t acc_bias;
    };
    using mic_imu_t = MicIMU;

    struct MicNavState
    {
        vector_3f_t position;
        quaternionf_t attitude;
        vector_3f_t velocity;
    };
    using mic_nav_state_t = MicNavState;
}
#endif
