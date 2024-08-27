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

#ifndef MIC_MAG_COMPENSATOR
#define MIC_MAG_COMPENSATOR

#include "common/mic_prerequisite.h"
#include "common/mic_utils.h"
#include "data_storer/mic_data_storer.h"

MIC_NAMESPACE_START

using mic_mag_storer_t =
    mic_data_storer_t<mic_mag_t, mic_nav_state_t>;
// using mic_mag_nav_state_storer_t=mic_data_storer_t<mic_nav_state_t>;

class MicMagCompensator;
using mic_mag_compensator_t = MicMagCompensator;
using mic_mag_compensator_shared_ptr = std::shared_ptr<MicMagCompensator>;

enum class MicMagCompensatorState : uint8_t
{
    MIC_MAG_COMPENSATE_UNCALIBRATED = 0,
    MIC_MAG_COMPENSATE_CALIBRATED = 1,
};
using mic_state_t = MicMagCompensatorState;

 /** \brief @b MicMagCompensator provides a base implementation of the Iterative
  * Closest Point algorithm. The transformation is estimated based on Singular Value
  * Decomposition (SVD).
  *
  * The algorithm has several termination criteria:
  *
  * <ol>
  * <li>Number of iterations has reached the maximum user imposed number of iterations
  * (via \ref setMaximumIterations)</li> <li>The epsilon (difference) between the
  * previous transformation and the current estimated transformation is smaller than an
  * user imposed value (via \ref setTransformationEpsilon)</li> <li>The sum of Euclidean
  * squared errors is smaller than a user defined threshold (via \ref
  * setEuclideanFitnessEpsilon)</li>
  * </ol>
  *
  *
  * Usage example:
  * \code
  * IterativeClosestPoint<PointXYZ, PointXYZ> icp;
  * // Set the input source and target
  * icp.setInputSource (cloud_source);
  * icp.setInputTarget (cloud_target);
  *
  * // Set the max correspondence distance to 5cm (e.g., correspondences with higher
  * // distances will be ignored)
  * icp.setMaxCorrespondenceDistance (0.05);
  * // Set the maximum number of iterations (criterion 1)
  * icp.setMaximumIterations (50);
  * // Set the transformation epsilon (criterion 2)
  * icp.setTransformationEpsilon (1e-8);
  * // Set the euclidean distance difference epsilon (criterion 3)
  * icp.setEuclideanFitnessEpsilon (1);
  *
  * // Perform the alignment
  * icp.align (cloud_source_registered);
  *
  * // Obtain the transformation that aligned cloud_source to cloud_source_registered
  * Eigen::Matrix4f transformation = icp.getFinalTransformation ();
  * \endcode
  *
  * \author Radu B. Rusu, Michael Dixon
  * \ingroup registration
  */
class MicMagCompensator : public MicObservable<MicMagCompensator>
{
public:
    MicMagCompensator();
    ~MicMagCompensator();

    mic_mag_storer_t &get_data_storer();
    // mic_nav_state_estimator_t &get_nav_state_estimator();
    float64_t get_curr_time() { return _curr_time_stamp; }

    /**
     * @brief add data (magnetic measurements and navigation states of the aircraft) to the compensator
     * @param ts timestamp
     * @param mag_data magnetic measurement at time ts
     * @param nav_state (optional) navigation state at time ts
     * @return ret_t
     * @retval if the data is successfully added
     */
    ret_t add_data(
        const float64_t ts,
        const mic_mag_t &mag_data,
        const mic_nav_state_t &nav_state = mic_nav_state_t());

    ret_t add_data_truth(
        const float64_t ts,
        const mic_mag_t &mag_data,
        const mic_nav_state_t &nav_state = mic_nav_state_t());

    ret_t calibrate();
    ret_t compenste(const float64_t ts, mic_mag_t &out);

    ret_t load_model(const std::string filename);
    ret_t save_model(const std::string filename);

protected:
    virtual ret_t do_calibrate() = 0;
    virtual ret_t do_compenste(
        const float64_t ts, mic_mag_t &out) = 0;

    virtual ret_t serialize(json_t &node);
    virtual ret_t deserialize(json_t &node);

    // void init_nav_state_estimator();

    float64_t _curr_time_stamp;

    /* data storer*/
    mic_mag_storer_t _mag_measure_storer;
    mic_mag_storer_t _mag_truth_storer;
    // mic_mag_nav_state_storer_t _nav_state_storer;

    /* working state */
    mic_state_t _state;
    std::string _version;
};

MIC_NAMESPACE_END

#endif