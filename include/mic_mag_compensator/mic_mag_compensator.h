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

/** \brief @b MicMagCompensator provides a base interface of the magnetic interference
 * compensation algorithm.
 *
 * The compensation model is calibrated first using measured data captured in a calibration
 * flight. Then the real-time compensation is implemented via the saved model.
 *
 * \author Qinxuan Sun, Yansong Gong
 * \ingroup compensation
 */
class MicMagCompensator : public MicObservable<MicMagCompensator>
{
public:
    /** \brief Empty constructor. */
    MicMagCompensator();
    /** \brief destructor. */
    ~MicMagCompensator();

    /** \brief Get a pointer to the data storer used to store the magnetic
     * and the navigation state data. */
    mic_mag_storer_t &get_data_storer();

    /** \brief Get the current timestamp. */
    float64_t get_curr_time() { return _curr_time_stamp; }

    /** \brief Add data (magnetic measurements and navigation states of
     * the aircraft) to the compensator.
     * \param[in] ts the timestamp of the added data
     * \param[in] mag_data the magnetic measurements
     * (three-component representation of the magnetic vector
     * and the magnetic intensity value)
     * \param[in] nav_state (optional) the navigation state provided
     * by either the avionics or INS
     */
    ret_t add_data(
        const float64_t ts,
        const mic_mag_t &mag_data,
        const mic_nav_state_t &nav_state = mic_nav_state_t());

    /** \brief Add the baseline data for model calibration.
     * \param[in] ts the timestamp of the added data
     * \param[in] mag_data the magnetic measurements provided either
     * by the earth magnetic model or the sensors outside the cabin
     * \param[in] nav_state (optional) the navigation state is provided
     * if the mag_data is captured via sensors outside the cabin
     */
    ret_t add_data_truth(
        const float64_t ts,
        const mic_mag_t &mag_data,
        const mic_nav_state_t &nav_state = mic_nav_state_t());

    /** \brief Call the calibration algorithm which calibrates the compensation model. */
    ret_t calibrate();

    /** \brief Call the compensation algorithm to compensate the input magnetic data.
     * \param[in] ts the timestamp at which the compensation result is required
     * \param[out] out output the compensated result at time \a ts
     */
    ret_t compenste(const float64_t ts, mic_mag_t &out);

    /** \brief Load the saved model to the compensator.
     * \param[in] filename the file (.mdl) which saves the compensation model coefficients
     */
    ret_t load_model(const std::string filename);

    /** \brief Save the compensation model to file.
     * \param[in] filename the file (.mdl) to write the compensation model coefficients
     */
    ret_t save_model(const std::string filename);

protected:
    virtual ret_t do_calibrate() = 0;
    virtual ret_t do_compenste(
        const float64_t ts, mic_mag_t &out) = 0;

    virtual ret_t serialize(json_t &node);
    virtual ret_t deserialize(json_t &node);

    /** \brief The current timestamp of the compensator. */
    float64_t _curr_time_stamp;

    /** \brief The data storer for the measurement data. */
    mic_mag_storer_t _mag_measure_storer;

    /** \brief The data storer for the baseline data. */
    mic_mag_storer_t _mag_truth_storer;

    /** \brief The working state of the compensator. */
    mic_state_t _state;
    
    std::string _version;
};

MIC_NAMESPACE_END

#endif