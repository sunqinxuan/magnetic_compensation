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
    using vector_3f_t = Eigen::Vector3d;
    using quaternionf_t = Eigen::Quaterniond;
    using matrix_3f_t = Eigen::Matrix3d;
    using matrix_3_18f_t = Eigen::Matrix<double, 3, 18>;
    // using vector_18f_t = Eigen::Matrix<double, 18, 1>;
    using vector_xf_t = Eigen::VectorXd;
    using matrix_xf_t = Eigen::MatrixXd;

    enum MicBool
    {
        MIC_FALSE = 0,
        MIC_TRUE = 1
    };
    using bool_t = MicBool;

#define true (mic::MIC_TRUE)
#define false (mic::MIC_FALSE)

    enum class MicRet : uint8_t
    {
        MIC_RET_FAILED = 0,
        MIC_RET_SUCCESSED = 1
    };
    using ret_t = MicRet;

    /** \brief MicMag represents the base class in MIC storing
     * the magnetic field data at a time step.*/
    struct MicMag
    {
        /** \brief The timestamp of this magnetic field data. */
        float64_t time_stamp; // TODO
        /** \brief The triaxial components of the magnetic field vector. */
        vector_3f_t vector;
        /** \brief The value of the magnetic field intensity. */
        float64_t value;
    };
    using mic_mag_t = MicMag;

    /** \brief MicIMU represents the IMU measurement data at a time step.*/
    struct MicIMU
    {
        /** \brief The timestamp of this IMU data. */
        float64_t time_stamp; // TODO
        /** \brief The time interval between two measurements. */
        float64_t dt;
        /** \brief The incremental angle (rad). */
        vector_3f_t gyr;
        /** \brief The incremental velocity (m/s). */
        vector_3f_t acc;
    };
    using mic_imu_t = MicIMU;

    // struct MicIMUError
    // {
    //     vector_3f_t gyr_bias;
    //     vector_3f_t acc_bias;
    //     vector_3f_t gyr_scale;
    //     vector_3f_t acc_scale;
    // };
    // using mic_imu_error_t = MicIMUError;

    /** \brief MicGNSS represents the GNSS measurement data at a time step.*/
    struct MicGNSS
    {
        /** \brief The timestamp of this GNSS data. */
        float64_t time_stamp;
        /** \brief latitude, longitude and altitude (rad,rad,m). */
        vector_3f_t lat_lon_alt;
        /** \brief position std. presented in the north-east-down coordinates (m). */
        vector_3f_t pos_std;
    };
    using mic_gnss_t = MicGNSS;

    /** \brief MicNavState represents the navigation state of
     * a aircraft or vehicle at a time step.*/
    struct MicNavState
    {
        MicNavState()
        {
            time_stamp = -1;
        }
        /** \brief The timestamp of this navigation state. */
        float64_t time_stamp; 
         /** \brief position presented in the north-east-down coordinates (m). */
        vector_3f_t position;
        /** \brief attitude of the body frame [nose, right wing, down]
         * w.r.t. the navigation frame [north, east, down]. */
        quaternionf_t attitude;
        /** \brief velocity presented in the north-east-down coordinates (m). */
        vector_3f_t velocity;
        // mic_imu_error_t imu_error;
    };
    using mic_nav_state_t = MicNavState;
}

#endif